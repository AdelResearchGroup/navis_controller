"""Online tests against a real Navis. Nothing here moves the mobile base.

Skipped unless ``NAVIS_ROBOT_IP`` is set, so CI and a plain ``colcon test`` never run them.

    NAVIS_ROBOT_IP=192.168.6.102 pixi run test-robot

Environment variables:

- ``NAVIS_ROBOT_IP``: Navis address (required to run these tests)
- ``NAVIS_HTTP_PORT`` / ``NAVIS_WS_PORT``: default 8880 / 9090
- ``NAVIS_PASSWORD``: only needed if the robot does not use the manual's default password
- ``NAVIS_TEST_MAP``: map to read; default is the map navigation is running on, else the first map
- ``NAVIS_ALLOW_WRITES=1``: also run tests that change data on the robot (a temporary task is
  created and deleted again). Existing maps and tasks are never modified.
- ``NAVIS_START_NAVIGATION=1``: if navigation is not running, start it on the test map for the
  navigation tests (``/navis/slam_operate``) and stop it again afterwards (``/navis/nav_input``).
  Starting navigation loads the map and localization; it does not move the base. Navigation that
  was already running is left alone.

What is never called: mapping and recording, running or pausing tasks, ``cmd_vel``, initial pose,
charging, obstacle-avoidance and charge-level settings, and map rename/delete/import. Navigation is
only started/stopped with ``NAVIS_START_NAVIGATION=1``.

Topics that depend on hardware or on the current mode (camera, GNSS, LiDAR point cloud,
localization in navigation, ...) are skipped, not failed, when nothing arrives.
"""

import math
import os
import struct
import threading
import zipfile

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from navis_msgs.msg import (
    DiskStates,
    GnssStatus,
    NavigationStatus,
    RobotStatus,
    SensorStatus,
    SlamStatus,
    Task,
    TaskPoint,
)
from navis_msgs.srv import (
    DeleteTask,
    ExportMap,
    GetCameraList,
    GetChargePoint,
    GetMapImage,
    GetMapInfo,
    GetMapList,
    GetTask,
    GetVersions,
    Login,
    MapToPixel,
    NavInput,
    PixelToMap,
    SetTask,
)
from navis_test_utils import Harness, wait_for
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Imu, LaserScan, PointCloud2

from navis_controller.navis_http import NavisHTTPClient
from navis_controller.navis_websocket import NavisWebsocketClient

ROBOT_IP = os.environ.get("NAVIS_ROBOT_IP", "")
HTTP_PORT = os.environ.get("NAVIS_HTTP_PORT", "8880")
WS_PORT = os.environ.get("NAVIS_WS_PORT", "9090")
ALLOW_WRITES = os.environ.get("NAVIS_ALLOW_WRITES") == "1"
START_NAVIGATION = os.environ.get("NAVIS_START_NAVIGATION") == "1"
NAVIGATION_TIMEOUT = 90.0  # starting navigation loads the 3D map, which can take a while

pytestmark = pytest.mark.skipif(not ROBOT_IP, reason="set NAVIS_ROBOT_IP to run tests against a real Navis")

TOPIC_TIMEOUT = 5.0


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_norm(q) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def assert_frame(frame_id: str):
    assert not frame_id.startswith("/"), f"frame_id {frame_id!r} still has the ROS 1 leading slash"


def assert_stamp(header):
    assert header.stamp.sec > 0 or header.stamp.nanosec > 0, "header stamp is zero"


class Robot(Harness):
    def __init__(self, client, http, ws):
        super().__init__(None, client)
        self.http = http
        self.ws = ws

    def first_message(self, msg_type, topic: str, timeout: float = TOPIC_TIMEOUT):
        sub, received = self.collect(msg_type, topic)
        try:
            wait_for(lambda: received, timeout)
            return received[0] if received else None
        finally:
            self.client.destroy_subscription(sub)


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    overrides_http = [
        Parameter("base_url", value=f"http://{ROBOT_IP}:{HTTP_PORT}"),
        Parameter("download_dir", value=str(tmp_path_factory.mktemp("downloads"))),
    ]
    if os.environ.get("NAVIS_PASSWORD"):
        overrides_http.append(Parameter("password", value=os.environ["NAVIS_PASSWORD"]))

    rclpy.init()
    http = NavisHTTPClient(parameter_overrides=overrides_http)
    ws = NavisWebsocketClient(parameter_overrides=[Parameter("ws_url", value=f"ws://{ROBOT_IP}:{WS_PORT}")])
    client = rclpy.create_node("navis_robot_test_client")
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (http, ws, client):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert wait_for(lambda: ws._ws is not None, timeout=10.0), f"no websocket connection to {ROBOT_IP}:{WS_PORT}"
        yield Robot(client, http, ws)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        for node in (client, ws, http):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture(scope="module")
def maps(robot):
    res = robot.call(GetMapList, "/navis/get_map_list")
    assert res.success, res.message
    return res


@pytest.fixture(scope="module")
def slam_status(robot):
    msg = robot.first_message(SlamStatus, "/navis/slam_status")
    assert msg is not None, "no /slam_status from Navis"
    return msg


@pytest.fixture(scope="module")
def map_name(maps, slam_status):
    names = [m.map_name for m in maps.maps]
    wanted = os.environ.get("NAVIS_TEST_MAP")
    if wanted:
        assert wanted in names, f"NAVIS_TEST_MAP={wanted!r} not in {names}"
        return wanted
    if slam_status.navigation.state and slam_status.navigation.name in names:
        return slam_status.navigation.name
    if not names:
        pytest.skip("the robot has no maps")
    return names[0]


@pytest.fixture(scope="module")
def map_info(robot, map_name):
    res = robot.call(GetMapInfo, "/navis/get_map_info", GetMapInfo.Request(map_name=map_name))
    assert res.success, res.message
    return res


# ----------------------------------------------------------------------------------------------
# Safety
# ----------------------------------------------------------------------------------------------


def test_cmd_vel_forwarding_is_disabled(robot):
    assert robot.ws.get_parameter("cmd_vel_enabled").value is False


# ----------------------------------------------------------------------------------------------
# HTTP node
# ----------------------------------------------------------------------------------------------


def test_login(robot):
    res = robot.call(Login, "/navis/login")
    assert res.success, res.message


def test_relogin_with_invalid_token(robot):
    # Checks that the node recognises Navis rejecting a bad token (HTTP 200 with code 1012 on the
    # real robot) and logs in again by itself.
    with robot.http._token_lock:
        robot.http._token = "invalid-token"
    try:
        res = robot.call(GetMapList, "/navis/get_map_list")
        assert res.success, f"no automatic re-login after an invalid token: {res.message}"
        assert robot.http._token != "invalid-token"
    finally:
        robot.http._invalidate_token()  # don't leave a bad token behind for the other tests


def test_map_list(maps):
    assert maps.total == len(maps.maps)
    for m in maps.maps:
        assert m.map_name
    assert len({m.map_name for m in maps.maps}) == len(maps.maps), "duplicate map names"


def test_map_info(map_info, maps, map_name):
    info = map_info.info
    assert info.width > 0 and info.height > 0
    assert 0.0 < info.resolution <= 1.0
    assert math.isfinite(info.origin.position.x) and math.isfinite(info.origin.position.y)
    listed = next(m for m in maps.maps if m.map_name == map_name)
    assert sorted(map_info.tasks) == sorted(listed.tasks)


def test_map_image_matches_map_info(robot, map_name, map_info, tmp_path):
    target = tmp_path / f"{map_name}.png"
    res = robot.call(
        GetMapImage, "/navis/get_map_image", GetMapImage.Request(map_name=map_name, save_path=str(target)), 60.0
    )
    assert res.success, res.message
    data = bytes(res.image.data)
    assert len(data) >= 24 and data[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG image"
    assert target.read_bytes() == data
    width, height = struct.unpack(">II", data[16:24])  # IHDR
    assert (width, height) == (map_info.info.width, map_info.info.height)


def pixel_to_map(robot, map_name, png_x, png_y):
    # float() matters: an int in a float64 field passes the Python setter but is serialized as 0.
    request = PixelToMap.Request(map_name=map_name, png_x=float(png_x), png_y=float(png_y))
    res = robot.call(PixelToMap, "/navis/pixel_to_map", request)
    assert res.success, res.message
    return res.position


def test_pixel_map_round_trip(robot, map_name, map_info):
    info = map_info.info
    png_x, png_y = info.width // 4, (info.height * 3) // 4
    position = pixel_to_map(robot, map_name, png_x, png_y)
    back = robot.call(MapToPixel, "/navis/map_to_pixel", MapToPixel.Request(map_name=map_name, position=position))
    assert back.success, back.message
    assert (back.png_x, back.png_y) == (pytest.approx(png_x, abs=1.0), pytest.approx(png_y, abs=1.0)), (
        f"pixel ({png_x}, {png_y}) -> map ({position.x:.4f}, {position.y:.4f}) -> pixel ({back.png_x}, {back.png_y})"
    )


def test_pixel_to_map_matches_manual_formula(robot, map_name, map_info):
    """The manual: x = png_x * resolution + originX, y = (gridHeight - png_y) * resolution + originY.

    Samples three pixels and fits x/y = a * png_x + b * png_y + c, so a failure shows the formula
    Navis really uses next to the map info.
    """
    info = map_info.info
    step = 100
    p0 = pixel_to_map(robot, map_name, 0, 0)
    px = pixel_to_map(robot, map_name, step, 0)
    py = pixel_to_map(robot, map_name, 0, step)
    fit_x = ((px.x - p0.x) / step, (py.x - p0.x) / step, p0.x)
    fit_y = ((px.y - p0.y) / step, (py.y - p0.y) / step, p0.y)
    r, ox, oy, h = info.resolution, info.origin.position.x, info.origin.position.y, info.height
    manual_x = (r, 0.0, ox)
    manual_y = (0.0, -r, h * r + oy)

    def fmt(name, a, b, c):
        return f"{name} = {a:+.6f} * png_x {b:+.6f} * png_y {c:+.6f}"

    report = (
        f"map {map_name!r}: width={info.width} height={info.height} resolution={r} origin=({ox}, {oy})\n"
        f"  Navis:  {fmt('x', *fit_x)};  {fmt('y', *fit_y)}\n"
        f"  manual: {fmt('x', *manual_x)};  {fmt('y', *manual_y)}"
    )
    tol = (1e-4, 1e-4, r)
    for got, want in ((fit_x, manual_x), (fit_y, manual_y)):
        assert all(abs(g - w) <= t for g, w, t in zip(got, want, tol)), report


def test_tasks(robot, map_name, map_info):
    if not map_info.tasks:
        pytest.skip(f"map {map_name!r} has no tasks")
    info = map_info.info
    x_min, y_min = info.origin.position.x, info.origin.position.y
    x_max, y_max = x_min + info.width * info.resolution, y_min + info.height * info.resolution
    for task_name in list(map_info.tasks)[:5]:
        res = robot.call(GetTask, "/navis/get_task", GetTask.Request(map_name=map_name, task_name=task_name))
        assert res.success, f"{task_name}: {res.message}"
        assert res.task.task_name == task_name and res.task.map_name == map_name
        assert res.task.mode in (Task.MODE_POINT, Task.MODE_PATH)
        for p in res.task.points:
            assert p.index >= 0, f"{task_name}: negative point index {p.index}"
            assert quat_norm(p.pose.orientation) == pytest.approx(1.0, abs=1e-6)
            assert x_min <= p.pose.position.x <= x_max and y_min <= p.pose.position.y <= y_max, (
                f"{task_name}: point {p.point_name} at ({p.pose.position.x}, {p.pose.position.y}) is outside the map"
            )


def test_export_map(robot, map_name, tmp_path):
    target = tmp_path / f"{map_name}.zip"
    res = robot.call(ExportMap, "/navis/export_map", ExportMap.Request(map_name=map_name, save_path=str(target)), 150.0)
    assert res.success, res.message
    assert zipfile.is_zipfile(res.saved_path)
    with zipfile.ZipFile(res.saved_path) as z:
        suffixes = {os.path.splitext(n)[1].lower() for n in z.namelist()}
    assert ".png" in suffixes and ".yaml" in suffixes, f"unexpected ZIP content: {suffixes}"


def test_versions(robot):
    for service in ("/navis/get_software_versions", "/navis/get_device_versions"):
        res = robot.call(GetVersions, service)
        assert res.success, f"{service}: {res.message}"
        assert res.versions, f"{service}: empty"


def test_camera_list(robot):
    res = robot.call(GetCameraList, "/navis/get_camera_list")
    assert res.success, res.message


@pytest.mark.skipif(not ALLOW_WRITES, reason="set NAVIS_ALLOW_WRITES=1 to create/delete a temporary task")
def test_task_round_trip(robot, map_name, map_info):
    """Create a temporary task, read it back, delete it. Existing tasks are not touched."""
    name = f"navis_test_{os.getpid()}"
    assert name not in map_info.tasks
    info = map_info.info

    task = Task(task_name=name, edited_name=name, map_name=map_name, mode=Task.MODE_POINT, speed=0.3)
    task.evadible = Task.OBSTACLE_STOP
    point = TaskPoint(point_name="A", index=1, is_new=True)
    point.pose.position = Point(
        x=info.origin.position.x + info.width * info.resolution / 2,
        y=info.origin.position.y + info.height * info.resolution / 2,
    )
    yaw = math.radians(30.0)
    point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    task.points = [point]

    res = robot.call(SetTask, "/navis/set_task", SetTask.Request(task=task))
    assert res.success, res.message
    try:
        got = robot.call(GetTask, "/navis/get_task", GetTask.Request(map_name=map_name, task_name=name))
        assert got.success, got.message
        assert len(got.task.points) == 1
        p = got.task.points[0]
        assert p.pose.position.x == pytest.approx(point.pose.position.x, abs=0.01)
        assert p.pose.position.y == pytest.approx(point.pose.position.y, abs=0.01)
        assert yaw_of(p.pose.orientation) == pytest.approx(yaw, abs=math.radians(0.5))
        assert p.index == 1
    finally:
        res = robot.call(DeleteTask, "/navis/delete_task", DeleteTask.Request(map_name=map_name, task_name=name))
        assert res.success, f"could not delete temporary task {name!r}: {res.message}"
    after = robot.call(GetMapInfo, "/navis/get_map_info", GetMapInfo.Request(map_name=map_name))
    assert name not in after.tasks


# ----------------------------------------------------------------------------------------------
# Websocket node
# ----------------------------------------------------------------------------------------------


def test_connection_survives_heartbeat_window(robot):
    """Navis drops clients that are silent for 3 s; the connection must stay up well beyond that."""
    ws = robot.ws._ws
    sub, received = robot.collect(SlamStatus, "/navis/slam_status")
    try:
        assert not wait_for(lambda: robot.ws._ws is not ws, timeout=8.0), "connection was dropped"
        count = len(received)
        assert wait_for(lambda: len(received) > count, timeout=3.0), "/slam_status stopped arriving"
    finally:
        robot.client.destroy_subscription(sub)


def test_slam_status(slam_status):
    assert_stamp(slam_status.header)
    if slam_status.navigation.state:
        assert slam_status.navigation.name


def check_robot_status(m):
    assert math.isfinite(m.linear_velocity) and math.isfinite(m.angular_velocity)


def check_battery(m):
    assert 0.0 <= m.percentage <= 1.0
    assert m.voltage > 0.0


def check_imu(m):
    assert_frame(m.header.frame_id)
    if m.orientation_covariance[0] != -1.0:
        assert quat_norm(m.orientation) == pytest.approx(1.0, abs=1e-3)


def check_odom(m):
    assert_frame(m.header.frame_id)
    assert_frame(m.child_frame_id)
    assert quat_norm(m.pose.pose.orientation) == pytest.approx(1.0, abs=1e-3)


def check_scan(m):
    assert_frame(m.header.frame_id)
    expected = round((m.angle_max - m.angle_min) / m.angle_increment) + 1
    assert abs(len(m.ranges) - expected) <= 1


def check_points(m):
    assert_frame(m.header.frame_id)
    assert m.fields and m.width * m.height > 0
    assert m.row_step == m.point_step * m.width
    assert len(m.data) == m.row_step * m.height


def check_pose(m):
    assert_frame(m.header.frame_id)
    assert quat_norm(m.pose.orientation) == pytest.approx(1.0, abs=1e-3)


def check_camera_info(m):
    assert m.width > 0 and m.height > 0
    assert len(m.k) == 9 and m.k[8] == pytest.approx(1.0)


def check_compressed(m):
    assert m.format and len(m.data) > 0


def no_check(m):
    pass


# (topic, type, required, check). Optional topics depend on hardware or on the current mode.
TOPICS = [
    ("/navis/interface_management/robot_status", RobotStatus, True, check_robot_status),
    ("/navis/interface_management/BMS_status", BatteryState, True, check_battery),
    ("/navis/sensor_status", SensorStatus, True, no_check),
    ("/navis/imu/data", Imu, True, check_imu),
    ("/navis/odom_raw", Odometry, True, check_odom),
    ("/navis/scan", LaserScan, False, check_scan),
    ("/navis/points_raw", PointCloud2, False, check_points),
    ("/navis/gnss_status", GnssStatus, False, no_check),
    ("/navis/disk_states", DiskStates, False, no_check),
    ("/navis/camera/color/camera_info", CameraInfo, False, check_camera_info),
    ("/navis/camera/color/image_raw/compressed", CompressedImage, False, check_compressed),
]


@pytest.mark.parametrize(("topic", "msg_type", "required", "check"), TOPICS, ids=[t[0] for t in TOPICS])
def test_topic(robot, topic, msg_type, required, check):
    msg = robot.first_message(msg_type, topic)
    if msg is None:
        if required:
            pytest.fail(f"nothing received on {topic} within {TOPIC_TIMEOUT} s")
        pytest.skip(f"nothing on {topic} (depends on hardware/mode)")
    check(msg)


def test_unsubscribes_when_unused(robot):
    topic = "/interface_management/BMS_status"
    sub, received = robot.collect(BatteryState, "/navis" + topic)
    try:
        assert wait_for(lambda: received, timeout=TOPIC_TIMEOUT)
        assert topic in robot.ws._active_subs
    finally:
        robot.client.destroy_subscription(sub)
    assert wait_for(lambda: topic not in robot.ws._active_subs, timeout=4.0)


# ----------------------------------------------------------------------------------------------
# Navigation (needs navigation running, or NAVIS_START_NAVIGATION=1). Kept last so navigation is
# started as late as possible; the fixture stops it again when the module finishes.
# ----------------------------------------------------------------------------------------------


def wait_for_navigation(robot, running: bool, timeout: float):
    """Wait until /slam_status reports navigation running (or stopped); returns (last status, ok)."""
    sub, received = robot.collect(SlamStatus, "/navis/slam_status")
    try:
        ok = wait_for(lambda: bool(received) and received[-1].navigation.state == running, timeout, period=0.2)
        return (received[-1] if received else None), ok
    finally:
        robot.client.destroy_subscription(sub)


def navis_input(robot, service: str, map_name: str, op_type: str):
    request = NavInput.Request(file_name=map_name, op_type=op_type, id_type=NavInput.Request.ID_NAVIGATION)
    return robot.call(NavInput, service, request, timeout=60.0)


@pytest.fixture(scope="module")
def navigation(robot, map_name):
    """Current /slam_status with navigation running; starts (and later stops) it if allowed."""
    status = robot.first_message(SlamStatus, "/navis/slam_status")
    assert status is not None, "no /slam_status from Navis"
    if status.navigation.state:
        yield status  # already running: leave it alone
        return
    if not START_NAVIGATION:
        pytest.skip("navigation is not running (set NAVIS_START_NAVIGATION=1 to start it for these tests)")
    busy = [
        n
        for n, m in (("3D mapping", status.mapping_3d), ("2D mapping", status.mapping_2d), ("recording", status.record))
        if m.state
    ]
    if busy:
        pytest.skip(f"not starting navigation while {', '.join(busy)} is running")

    start = navis_input(robot, "/navis/slam_operate", map_name, NavInput.Request.OP_START)
    try:
        status, ok = wait_for_navigation(robot, True, NAVIGATION_TIMEOUT)
        assert ok, (
            f"navigation on {map_name!r} did not start within {NAVIGATION_TIMEOUT:.0f} s "
            f"(slam_operate: success={start.success}, {start.message!r})"
        )
        assert start.success, f"navigation started, but slam_operate reported failure: {start.message!r}"
        assert status.navigation.name == map_name
        yield status
    finally:
        stop = navis_input(robot, "/navis/nav_input", map_name, NavInput.Request.OP_STOP)
        _, stopped = wait_for_navigation(robot, False, 60.0)
        if not stopped:  # try the HTTP service before giving up
            navis_input(robot, "/navis/slam_operate", map_name, NavInput.Request.OP_STOP)
            _, stopped = wait_for_navigation(robot, False, 60.0)
        assert stopped, f"could not stop navigation again (nav_input: {stop.message!r}); stop it in the web UI"
        assert stop.success, f"navigation stopped, but nav_input reported failure: {stop.message!r}"


def test_navigation_map_is_listed(maps, navigation):
    assert navigation.navigation.name in [m.map_name for m in maps.maps]


# (topic, type, required, check) while navigation is running
NAVIGATION_TOPICS = [
    ("/navis/odom", Odometry, True, check_odom),
    ("/navis/localizer_pose", PoseStamped, False, check_pose),  # needs successful localization
    ("/navis/run_management/global_status", NavigationStatus, False, no_check),
]


@pytest.mark.parametrize(
    ("topic", "msg_type", "required", "check"), NAVIGATION_TOPICS, ids=[t[0] for t in NAVIGATION_TOPICS]
)
def test_navigation_topic(robot, navigation, topic, msg_type, required, check):
    msg = robot.first_message(msg_type, topic)
    if msg is None:
        if required:
            pytest.fail(f"nothing received on {topic} within {TOPIC_TIMEOUT} s while navigating")
        pytest.skip(f"nothing on {topic} while navigating")
    check(msg)


def test_get_charge_point(robot, navigation):
    res = robot.call(GetChargePoint, "/navis/get_charge_point")
    if not res.success:
        pytest.skip(f"no charge point: {res.message}")
    assert res.charge_point.header.frame_id == "map"
    assert quat_norm(res.charge_point.pose.orientation) == pytest.approx(1.0, abs=1e-3)
