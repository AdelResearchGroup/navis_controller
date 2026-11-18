"""Run both bridge nodes against the fake Navis server (no robot needed).

All tests share one fake server, one pair of nodes and one test client node. Each test cleans up
the subscriptions and knobs it changes.
"""

import math
import threading
import time

import pytest
import rclpy
from fake_navis_server import FAKE_PNG, FakeNavis
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from navis_msgs.msg import SlamStatus, TaskPoint
from navis_msgs.srv import (
    GetChargePoint,
    GetMapImage,
    GetMapInfo,
    GetMapList,
    GetTask,
    GetVersions,
    NavInput,
    PixelToMap,
    RunRealtimeTask,
    SetChargeLevel,
    SetTask,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

from navis_controller.navis_http import NavisHTTPClient
from navis_controller.navis_websocket import NavisWebsocketClient


def wait_for(condition, timeout: float = 5.0, period: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(period)
    return bool(condition())


class Harness:
    def __init__(self, fake: FakeNavis, client):
        self.fake = fake
        self.client = client
        self._clients = {}

    def call_async(self, srv_type, name: str, request=None):
        key = (srv_type, name)
        if key not in self._clients:
            cli = self.client.create_client(srv_type, name)
            assert cli.wait_for_service(timeout_sec=5.0), f"service {name} not available"
            self._clients[key] = cli
        return self._clients[key].call_async(request if request is not None else srv_type.Request())

    def call(self, srv_type, name: str, request=None, timeout: float = 10.0):
        future = self.call_async(srv_type, name, request)
        assert wait_for(future.done, timeout), f"{name} did not respond"
        return future.result()

    def collect(self, msg_type, topic: str):
        """Subscribe on the test client; returns (subscription, list of received messages)."""
        received = []
        sub = self.client.create_subscription(msg_type, topic, received.append, 10)
        return sub, received


@pytest.fixture(scope="module")
def h():
    fake = FakeNavis(publish_period=0.05)
    fake.start()
    rclpy.init()
    http = NavisHTTPClient(parameter_overrides=[
        Parameter("base_url", value=f"http://127.0.0.1:{fake.http_port}"),
    ])
    ws = NavisWebsocketClient(parameter_overrides=[
        Parameter("ws_url", value=f"ws://127.0.0.1:{fake.ws_port}"),
        Parameter("reconnect_delay", value=0.2),
    ])
    client = rclpy.create_node("navis_test_client")
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (http, ws, client):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    assert wait_for(lambda: fake.connections >= 1), "websocket node never connected"
    try:
        yield Harness(fake, client)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        for node in (client, ws, http):
            node.destroy_node()
        rclpy.shutdown()
        fake.stop()


# ----------------------------------------------------------------------------------------------
# HTTP node
# ----------------------------------------------------------------------------------------------

def test_logs_in_at_startup(h):
    assert h.fake.logins >= 1


def test_get_map_list(h):
    res = h.call(GetMapList, "/navis/get_map_list")
    assert res.success, res.message
    assert res.total == 1
    assert res.maps[0].map_name == "office"
    assert res.maps[0].id == 1671087653
    assert list(res.maps[0].tasks) == ["p1"]
    _, _, query, _ = [r for r in h.fake.http_requests if r[1] == "/map_list"][-1]
    assert query["page"] == "1" and query["limit"] == "-1" and query["sortType"] == ""


def test_get_map_info(h):
    res = h.call(GetMapInfo, "/navis/get_map_info", GetMapInfo.Request(map_name="office"))
    assert res.success, res.message
    assert (res.info.width, res.info.height) == (1200, 1000)
    assert res.info.resolution == pytest.approx(0.05)
    assert res.info.origin.position.x == pytest.approx(-10.0)
    assert res.info.origin.position.y == pytest.approx(-20.0)


def test_navis_error_is_reported(h):
    res = h.call(GetMapInfo, "/navis/get_map_info", GetMapInfo.Request(map_name="missing"))
    assert not res.success
    assert "map not found" in res.message


def test_get_map_image_saves_file(h, tmp_path):
    target = tmp_path / "maps" / "office.png"
    res = h.call(GetMapImage, "/navis/get_map_image", GetMapImage.Request(map_name="office", save_path=str(target)))
    assert res.success, res.message
    assert res.image.format == "png"
    assert bytes(res.image.data) == FAKE_PNG
    assert target.read_bytes() == FAKE_PNG


def test_get_map_image_error_body(h):
    res = h.call(GetMapImage, "/navis/get_map_image", GetMapImage.Request(map_name="missing"))
    assert not res.success
    assert "map not found" in res.message


def test_pixel_to_map(h):
    res = h.call(PixelToMap, "/navis/pixel_to_map", PixelToMap.Request(map_name="office", png_x=200.0, png_y=600.0))
    assert res.success, res.message
    assert res.position.x == pytest.approx(200 * 0.05 - 10.0)
    assert res.position.y == pytest.approx((1000 - 600) * 0.05 - 20.0)


def test_get_and_set_task(h):
    res = h.call(GetTask, "/navis/get_task", GetTask.Request(map_name="office", task_name="p1"))
    assert res.success, res.message
    assert len(res.task.points) == 2
    assert res.task.points[0].index == 1
    q = res.task.points[0].pose.orientation
    assert math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z) == pytest.approx(math.radians(78.24))

    task = res.task
    task.edited_name = "p1_copy"
    res = h.call(SetTask, "/navis/set_task", SetTask.Request(task=task))
    assert res.success, res.message
    body = h.fake.http_bodies("POST", "/set_task")[-1]
    assert body["editedName"] == "p1_copy"
    assert body["points"][0]["position"]["theta"] == pytest.approx(78.24)
    assert ("office", "p1_copy") in h.fake.tasks


def test_run_realtime_task_payload(h):
    point = TaskPoint()
    point.pose.position.x = 1.0
    point.pose.position.y = 2.0
    point.pose.orientation.z = math.sin(math.pi / 4)
    point.pose.orientation.w = math.cos(math.pi / 4)
    request = RunRealtimeTask.Request(mode="point", loop_time=2, points=[point])
    res = h.call(RunRealtimeTask, "/navis/run_realtime_task", request)
    assert res.success, res.message
    body = h.fake.http_bodies("POST", "/realtime_task")[-1]
    assert body["mode"] == "point" and body["loopTime"] == 2
    assert body["points"][0]["position"] == pytest.approx({"x": 1.0, "y": 2.0, "theta": 90.0})


def test_empty_realtime_task_rejected(h):
    res = h.call(RunRealtimeTask, "/navis/run_realtime_task", RunRealtimeTask.Request(mode="point"))
    assert not res.success


def test_versions(h):
    res = h.call(GetVersions, "/navis/get_software_versions")
    assert res.success and [v.id for v in res.versions] == ["Navis Board", "Navis Brain", "Navis Bridge"]
    res = h.call(GetVersions, "/navis/get_device_versions")
    assert res.success and res.versions[0].type == "X-7010"


def test_relogin_after_token_expiry(h):
    before = h.fake.logins
    h.fake.expire_tokens()
    res = h.call(GetMapList, "/navis/get_map_list")
    assert res.success, res.message
    assert h.fake.logins == before + 1


def test_slam_operate(h):
    request = NavInput.Request(file_name="office", op_type="start", id_type="navigation")
    res = h.call(NavInput, "/navis/slam_operate", request)
    assert res.result and res.success, res.message
    body = h.fake.http_bodies("POST", "/slam_operate")[-1]
    assert body == {"opType": "start", "idType": "navigation", "fileName": "office", "mapOperate": False}


# ----------------------------------------------------------------------------------------------
# Websocket node
# ----------------------------------------------------------------------------------------------

def test_heartbeat(h):
    pings = h.fake.pings
    assert wait_for(lambda: h.fake.pings >= pings + 2, timeout=4.0)


def test_always_subscribe_slam_status(h):
    assert wait_for(lambda: "/slam_status" in h.fake.subscriptions)
    sub, received = h.collect(SlamStatus, "/navis/slam_status")
    try:
        assert wait_for(lambda: received)
        assert received[0].navigation.state and received[0].navigation.name == "office"
    finally:
        h.client.destroy_subscription(sub)


def test_lazy_subscription(h):
    assert "/odom" not in h.fake.subscriptions
    sub, received = h.collect(Odometry, "/navis/odom")
    try:
        assert wait_for(lambda: "/odom" in h.fake.subscriptions, timeout=4.0)
        assert wait_for(lambda: received)
        msg = received[0]
        assert msg.header.frame_id == "map"  # "/map" from Navis
        assert msg.child_frame_id == "base_link"
        assert msg.pose.pose.position.x == pytest.approx(1.5)
        assert msg.twist.twist.linear.x == pytest.approx(0.1)
    finally:
        h.client.destroy_subscription(sub)
    assert wait_for(lambda: "/odom" not in h.fake.subscriptions, timeout=4.0)


def test_battery_topic(h):
    sub, received = h.collect(BatteryState, "/navis/interface_management/BMS_status")
    try:
        assert wait_for(lambda: received, timeout=4.0)
        assert received[0].percentage == pytest.approx(0.76)
    finally:
        h.client.destroy_subscription(sub)


def test_nav_input(h):
    request = NavInput.Request(file_name="office", op_type="start", id_type="navigation")
    res = h.call(NavInput, "/navis/nav_input", request)
    assert res.result and res.success
    assert res.message == "start navigation success."
    assert h.fake.service_calls("/input/op")[-1]["args"] == {
        "file_name": "office", "op_type": "start", "id_type": "navigation",
    }


def test_concurrent_service_calls_get_their_own_response(h):
    # The "pause" call is answered after the "resume" call; each caller must still get its own answer.
    h.fake.service_delay = lambda service, args: 0.5 if args.get("pause") is True else 0.0
    try:
        pause = h.call_async(SetBool, "/navis/pause_task", SetBool.Request(data=True))
        resume = h.call_async(SetBool, "/navis/pause_task", SetBool.Request(data=False))
        assert wait_for(lambda: pause.done() and resume.done(), timeout=5.0)
        assert pause.result().message == "pause=True"
        assert resume.result().message == "pause=False"
    finally:
        h.fake.service_delay = lambda service, args: 0.0


def test_service_response_without_id(h):
    h.fake.echo_ids = False
    try:
        res = h.call(SetBool, "/navis/next_goal", SetBool.Request(data=True))
        assert res.success and res.message == "data=True"
    finally:
        h.fake.echo_ids = True


def test_charging_services(h):
    res = h.call(GetChargePoint, "/navis/get_charge_point")
    assert res.success
    assert res.charge_point.pose.position.x == pytest.approx(-6.903390407562256)
    res = h.call(SetChargeLevel, "/navis/set_charge_level",
                 SetChargeLevel.Request(min_charge_level=20, max_charge_level=90))
    assert res.success and (res.min_charge_level, res.max_charge_level) == (20, 90)
    assert h.call(Trigger, "/navis/set_charge_point").success


def test_cancel_task(h):
    res = h.call(Trigger, "/navis/cancel_task")
    assert res.success
    assert wait_for(lambda: h.fake.published("/run_management/navi_task/cancel"))


def test_cmd_vel_gated_and_clamped(h):
    pub = h.client.create_publisher(Twist, "/navis/cmd_vel", 10)
    try:
        assert wait_for(lambda: pub.get_subscription_count() > 0)
        fast = Twist()
        fast.linear.x = 3.0
        fast.angular.z = -0.5

        for _ in range(5):  # disabled by default: nothing reaches Navis
            pub.publish(fast)
            time.sleep(0.05)
        time.sleep(0.3)
        assert h.fake.published("/cmd_vel") == []

        assert h.call(SetBool, "/navis/enable_cmd_vel", SetBool.Request(data=True)).success

        def sent():
            pub.publish(fast)
            return h.fake.published("/cmd_vel")
        assert wait_for(sent)
        cmd = h.fake.published("/cmd_vel")[0]
        assert cmd["linear"]["x"] == pytest.approx(1.5)  # clamped
        assert cmd["angular"]["z"] == pytest.approx(-0.5)

        assert h.call(SetBool, "/navis/enable_cmd_vel", SetBool.Request(data=False)).success
        stop = h.fake.published("/cmd_vel")[-1]
        assert stop["linear"]["x"] == 0.0 and stop["angular"]["z"] == 0.0
    finally:
        h.client.destroy_publisher(pub)


def test_initialpose(h):
    pub = h.client.create_publisher(PoseWithCovarianceStamped, "/navis/initialpose", 10)
    try:
        assert wait_for(lambda: pub.get_subscription_count() > 0)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = 42
        msg.pose.pose.position.x = 3.0
        msg.pose.pose.orientation.w = 1.0
        pub.publish(msg)
        assert wait_for(lambda: h.fake.published("/initialpose"))
        sent = h.fake.published("/initialpose")[-1]
        assert sent["header"]["frame_id"] == "map"  # default when empty
        assert sent["header"]["stamp"]["secs"] == 42
        assert sent["pose"]["pose"]["position"]["x"] == 3.0
        assert len(sent["pose"]["covariance"]) == 36
    finally:
        h.client.destroy_publisher(pub)


def test_reconnect(h):
    before = h.fake.connections
    h.fake.drop_connections()
    assert wait_for(lambda: h.fake.connections > before, timeout=5.0)
    assert wait_for(lambda: "/slam_status" in h.fake.subscriptions, timeout=4.0)
    res = h.call(SetBool, "/navis/set_obstacle_avoidance", SetBool.Request(data=False))
    assert res.success and res.message == "data=False"
