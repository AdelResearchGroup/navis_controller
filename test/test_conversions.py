"""Offline tests for the Navis <-> ROS 2 conversions (no robot, no network, no rclpy.init)."""

import base64
import math
import struct
from collections import namedtuple

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from navis_msgs.msg import GnssStatus, Task, TaskPoint
from sensor_msgs.msg import BatteryState, CameraInfo, Imu, LaserScan, NavSatStatus

from navis_controller import navis_websocket as ws
from navis_controller.navis_http import NavisHTTPClient, _navis_ok, _to_float, _to_int

HEADER = {"stamp": {"secs": 1681805920, "nsecs": 64378373}, "frame_id": "/map", "seq": 12}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------------------------------------
# Generic ROS 1 JSON -> ROS 2
# ----------------------------------------------------------------------------------------------

def test_normalize_ros1_header():
    out = ws._normalize_ros1({"header": HEADER, "child_frame_id": "/base_link"})
    assert out["header"] == {"stamp": {"sec": 1681805920, "nanosec": 64378373}, "frame_id": "map"}
    assert out["child_frame_id"] == "base_link"


def test_odometry():
    msg = ws._generic(Odometry)({
        "header": HEADER,
        "child_frame_id": "base_link",
        "pose": {
            "pose": {"position": {"x": 1, "y": -2, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
            "covariance": [0] * 36,
        },
        "twist": {"twist": {"linear": {"x": 0.5}, "angular": {"z": -0.1}}, "covariance": [0] * 36},
    })
    assert msg.header.stamp.sec == 1681805920
    assert msg.header.stamp.nanosec == 64378373
    assert msg.header.frame_id == "map"
    assert msg.pose.pose.position.x == 1.0 and isinstance(msg.pose.pose.position.x, float)
    assert msg.pose.pose.position.y == -2.0
    assert msg.twist.twist.linear.x == 0.5
    assert msg.twist.twist.angular.z == pytest.approx(-0.1)
    assert len(msg.pose.covariance) == 36


def test_imu_fixed_arrays():
    msg = ws._generic(Imu)({
        "header": HEADER,
        "orientation": {"x": -0.03, "y": 0.02, "z": -0.47, "w": 0.88},
        "orientation_covariance": [0.0] * 9,
        "angular_velocity": {"x": 1e-5, "y": 0.0017, "z": 0.0005},
        "angular_velocity_covariance": [0.0] * 9,
        "linear_acceleration": {"x": -0.04, "y": -0.74, "z": 9.87},
        "linear_acceleration_covariance": [0.0] * 9,
    })
    assert msg.orientation.w == pytest.approx(0.88)
    assert msg.linear_acceleration.z == pytest.approx(9.87)
    assert list(msg.orientation_covariance) == [0.0] * 9


def test_laser_scan_nulls_become_nan():
    msg = ws._generic(LaserScan)({
        "header": HEADER, "angle_min": -3.14, "angle_max": 3.14, "angle_increment": 0.01,
        "range_min": 0.2, "range_max": 100.0, "ranges": [1.0, None, 2.5], "intensities": [10, 20, 30],
    })
    assert msg.ranges[0] == pytest.approx(1.0)
    assert math.isnan(msg.ranges[1])
    assert msg.ranges[2] == pytest.approx(2.5)
    assert list(msg.intensities) == [10.0, 20.0, 30.0]


def test_point_cloud_base64_and_row_step():
    points = struct.pack("<fff", 1.0, 2.0, 3.0) + struct.pack("<fff", 4.0, 5.0, 6.0)
    msg = ws._point_cloud({
        "header": HEADER, "height": 1, "width": 2, "point_step": 12, "row_step": 0,
        "is_bigendian": False, "is_dense": True,
        "fields": [
            {"name": "x", "offset": 0, "datatype": 7, "count": 1},
            {"name": "y", "offset": 4, "datatype": 7, "count": 1},
            {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        ],
        "data": base64.b64encode(points).decode(),
    })
    assert bytes(msg.data) == points
    assert [f.name for f in msg.fields] == ["x", "y", "z"]
    assert msg.fields[1].offset == 4
    assert msg.row_step == 24  # Navis sends 0


def test_camera_info_uppercase_fields():
    msg = ws._generic(CameraInfo)({
        "header": HEADER, "height": 480, "width": 640, "distortion_model": "plumb_bob",
        "D": [0.1, 0.0, 0.0, 0.0, 0.0], "K": [600, 0, 320, 0, 600, 240, 0, 0, 1],
        "R": [1, 0, 0, 0, 1, 0, 0, 0, 1], "P": [600, 0, 320, 0, 0, 600, 240, 0, 0, 0, 1, 0],
    })
    assert list(msg.d) == pytest.approx([0.1, 0, 0, 0, 0])
    assert msg.k[0] == 600.0 and msg.k[8] == 1.0
    assert msg.p[3] == 0.0 and len(msg.p) == 12


def test_path_nested_messages():
    pose = {"header": HEADER, "pose": {"position": {"x": 1, "y": 2, "z": 0},
                                       "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}
    msg = ws._generic(Path)({"header": HEADER, "poses": [pose, pose]})
    assert len(msg.poses) == 2
    assert isinstance(msg.poses[0], PoseStamped)
    assert msg.poses[1].pose.position.y == 2.0
    assert msg.poses[0].header.frame_id == "map"


def test_unknown_fields_are_ignored():
    msg = ws._generic(PoseStamped)({"header": HEADER, "pose": {"position": {"x": 1}}, "something_new": 5})
    assert msg.pose.position.x == 1.0


def test_cbor_typed_array_tag_hook():
    Tag = namedtuple("Tag", "tag value")
    floats = struct.pack("<ff", 1.5, -2.0)
    assert ws.NavisWebsocketClient._cbor_tag_hook(None, Tag(85, floats)) == [1.5, -2.0]
    assert ws.NavisWebsocketClient._cbor_tag_hook(None, Tag(64, b"\x01\x02")) == b"\x01\x02"


# ----------------------------------------------------------------------------------------------
# Navis-specific messages
# ----------------------------------------------------------------------------------------------

def test_slam_status():
    msg = ws._slam_status({
        "nav": {"state": True, "name": "office", "task": "p1"},
        "mapping_3d": {"state": False, "name": ""},
        "mapping_2d": {"state": False, "name": ""},
        "record": {"state": True, "name": "bag1"},
    })
    assert msg.navigation.state and msg.navigation.name == "office" and msg.navigation.task == "p1"
    assert not msg.mapping_3d.state
    assert msg.record.state and msg.record.name == "bag1"


def test_battery():
    msg = ws._battery({"batteryCurrent": -1.5, "SOH": 98, "SOC": 76,
                       "batteryTemperature": 25.0, "batteryVoltage": 26.4})
    assert isinstance(msg, BatteryState)
    assert msg.percentage == pytest.approx(0.76)
    assert msg.voltage == pytest.approx(26.4)
    assert msg.current == pytest.approx(-1.5)
    assert math.isnan(msg.capacity)


def test_sensor_status():
    msg = ws._sensor_status({"imu_status": "ON", "lidar_status": "on", "RTK_status": "OFF", "camera_status": ""})
    assert msg.imu_on and msg.lidar_on
    assert not msg.rtk_on and not msg.camera_on


def test_gnss_strings():
    msg = ws._gnss_status({"longitude": "-74.65", "latitude": "40.34", "altitude": "30.5",
                           "gps_qual": "4", "num_sats": "17"})
    assert msg.fix.latitude == pytest.approx(40.34)
    assert msg.fix.longitude == pytest.approx(-74.65)
    assert msg.gps_qual == GnssStatus.QUAL_RTK_FIXED
    assert msg.num_sats == 17
    assert msg.fix.status.status == NavSatStatus.STATUS_GBAS_FIX
    assert ws._gnss_status({"gps_qual": "0"}).fix.status.status == NavSatStatus.STATUS_NO_FIX


def test_log_levels():
    msg = ws._log({"stamp": {"secs": 10, "nsecs": 5}, "msg": "hello", "level": 4})
    assert msg.level == 30 and msg.msg == "hello" and msg.name == "navis"
    assert msg.stamp.sec == 10 and msg.stamp.nanosec == 5
    assert ws._log({"level": 16}).level == 50


def test_disk_states():
    msg = ws._disk_states({"disks": [{"disk_total_size": "225", "disk_free_size": "211",
                                      "disk_name": "disk system"}]})
    assert msg.disks[0].name == "disk system"
    assert msg.disks[0].total_size == 225.0 and msg.disks[0].free_size == 211.0


def test_task_status():
    msg = ws._task_status({"task_type": "MUTIPOINT", "total_looptimes": 2, "loop_index": 1, "task_index": 3,
                           "countdown": 0, "isGoalReach": True, "task_name": "p1", "total_tasks": 5})
    assert msg.is_goal_reached and msg.task_index == 3 and msg.total_tasks == 5
    assert msg.task_type == msg.TASK_TYPE_MULTIPOINT


def test_charge_point():
    msg = ws._charge_point_pose({"get_charge_point": True,
                                 "charge_point": {"x": -6.9, "y": 1.38, "theta": 1.76}}, "map")
    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == pytest.approx(-6.9)
    assert yaw_of(msg.pose.orientation) == pytest.approx(1.76)
    assert ws._charge_point_pose({"get_charge_point": False}, "map") is None


# ----------------------------------------------------------------------------------------------
# HTTP helpers and task conversion
# ----------------------------------------------------------------------------------------------

class _TaskConverter:
    """The task conversion methods of NavisHTTPClient without creating a ROS node."""

    def __init__(self, theta_in_degrees: bool):
        self._theta_deg = theta_in_degrees


for _name in ("_theta_to_yaw", "_yaw_to_theta", "_point_from_navis", "_point_to_navis",
              "_task_from_navis", "_task_to_navis"):
    setattr(_TaskConverter, _name, getattr(NavisHTTPClient, _name))

NAVIS_TASK = {
    "taskName": "p1", "gridItemIdx": 0, "mode": "point", "speed": "", "evadible": 1, "mapName": "office",
    "editedName": "p1", "remark": "", "personName": "me",
    "points": [{
        "isNew": True, "index": "point-1", "pointType": "navigation", "pointName": "A",
        "actions": [{"actionContent": "5", "actionType": "stop", "actionOrder": 0, "actionName": "wait"}],
        "position": {"x": -3.35, "y": 37.48, "theta": 78.24}, "cpx": 0, "cpy": 0,
    }],
}


def test_navis_ok():
    assert _navis_ok({"success": True, "code": 0})
    assert _navis_ok({"code": "0"})
    assert not _navis_ok({"success": True, "code": -1})  # code takes priority
    assert _navis_ok({"success": True})


def test_lenient_parsing():
    assert _to_int("point-7") == 7
    assert _to_int("") == 0
    assert _to_int(3.9) == 3
    assert _to_float("") == 0.0
    assert _to_float("1.25") == 1.25


def test_task_from_navis_degrees():
    task = _TaskConverter(True)._task_from_navis(NAVIS_TASK)
    assert isinstance(task, Task)
    assert task.task_name == "p1" and task.map_name == "office" and task.person_name == "me"
    assert task.speed == 0.0  # "" in the manual's example
    p = task.points[0]
    assert isinstance(p, TaskPoint)
    assert p.index == 1
    assert p.pose.position.x == pytest.approx(-3.35)
    assert yaw_of(p.pose.orientation) == pytest.approx(math.radians(78.24))
    assert p.actions[0].duration == 5.0 and p.actions[0].action_type == "stop"


def test_task_from_navis_radians():
    task = _TaskConverter(False)._task_from_navis(
        {**NAVIS_TASK, "points": [{"position": {"x": 0, "y": 0, "theta": 1.0}}]}
    )
    assert yaw_of(task.points[0].pose.orientation) == pytest.approx(1.0)


def test_task_round_trip():
    conv = _TaskConverter(True)
    out = conv._task_to_navis(conv._task_from_navis(NAVIS_TASK))
    assert out["taskName"] == "p1" and out["mapName"] == "office" and out["editedName"] == "p1"
    assert out["mode"] == "point" and out["evadible"] == 1
    p = out["points"][0]
    assert p["position"]["x"] == pytest.approx(-3.35)
    assert p["position"]["theta"] == pytest.approx(78.24)
    assert p["index"] == 1 and p["pointName"] == "A" and p["isNew"] is True
    assert p["actions"][0]["actionContent"] == "5"


def test_realtime_point_is_minimal():
    point = TaskPoint()
    point.pose.position.x = 2.0
    point.pose.orientation.z = math.sin(math.pi / 4)
    point.pose.orientation.w = math.cos(math.pi / 4)
    d = _TaskConverter(True)._point_to_navis(point, full=False)
    assert set(d) == {"position", "isNew", "cpx", "cpy"}
    assert d["position"]["theta"] == pytest.approx(90.0)
