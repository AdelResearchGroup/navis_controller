"""ROS 2 bridge for the AgileX Navis websocket API (user API v0.0.12).

The Navis websocket server (port 9090) speaks a rosbridge-like protocol carrying ROS 1 messages.
This node keeps one connection open (reconnecting as needed), sends the required 1 Hz ``ping``
heartbeat, and translates between Navis and the ROS 2 interfaces from ``navis_msgs`` and the
standard packages (see the navis_msgs README for the full table).

Navis topics are republished under ``navis/<navis topic>``, e.g. ``/odom`` -> ``navis/odom`` and
``/run_management/global_status`` -> ``navis/run_management/global_status``. To avoid streaming
everything all the time, a Navis topic is only subscribed while the ROS publisher has at least one
subscriber, or while it is listed in the ``always_subscribe`` parameter.

Commands (subscriptions):
    navis/cmd_vel        geometry_msgs/Twist                     -> publish /cmd_vel (gated, see navis/enable_cmd_vel)
    navis/light_command  navis_msgs/LightCommand                 -> publish /scout_light_control
    navis/initialpose    geometry_msgs/PoseWithCovarianceStamped -> publish /initialpose

Services:
    navis/nav_input               navis_msgs/srv/NavInput        -> call /input/op
    navis/cancel_task             std_srvs/srv/Trigger           -> publish /run_management/navi_task/cancel
    navis/pause_task              std_srvs/srv/SetBool           -> call /run_management/pause (true = pause)
    navis/next_goal               std_srvs/srv/SetBool           -> call /run_management/next_goal
    navis/set_obstacle_avoidance  std_srvs/srv/SetBool           -> call /run_management/switch_obstacle_avoid_model
    navis/set_charge_point        std_srvs/srv/Trigger           -> call /find_charger/set_charge_point
    navis/get_charge_point        navis_msgs/srv/GetChargePoint  -> call /find_charger/get_charge_point
    navis/set_charging            std_srvs/srv/SetBool           -> call /find_charger/set_charge
    navis/set_charge_level        navis_msgs/srv/SetChargeLevel  -> call /find_charger/set_charge_level
    navis/enable_cmd_vel          std_srvs/srv/SetBool           enable/disable forwarding of navis/cmd_vel
"""

import array
import asyncio
import base64
import itertools
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

import rclpy
import websockets
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_parser.definition import AbstractGenericString, AbstractNestedType, Array, BasicType, NamespacedType
from rosidl_runtime_py.import_message import import_message_from_namespaced_type

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Imu, LaserScan, NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import SetBool, Trigger
from navis_msgs.msg import (
    DiskState,
    DiskStates,
    GnssStatus,
    LightCommand,
    ModuleStatus,
    NavigationStatus,
    RobotStatus,
    SensorStatus,
    SlamStatus,
    TaskStatus,
)
from navis_msgs.srv import GetChargePoint, NavInput, SetChargeLevel

try:  # optional: only needed when use_cbor is enabled
    import cbor2
except ImportError:  # pragma: no cover
    cbor2 = None


class NavisError(Exception):
    """Navis could not be reached or returned an error."""


# ----------------------------------------------------------------------------------------------
# Generic ROS 1 JSON -> ROS 2 message conversion
# ----------------------------------------------------------------------------------------------

_FLOAT_TYPES = {"float": "f", "double": "d", "long double": "d"}
_BYTE_TYPES = {"uint8", "octet", "char"}

# RFC 8746 typed-array CBOR tags used by rosbridge (little-endian variants)
_CBOR_TYPED_ARRAYS = {
    64: None, 68: None,  # uint8 -> bytes
    69: "<u2", 70: "<u4", 71: "<u8",
    72: "i1", 77: "<i2", 78: "<i4", 79: "<i8",
    85: "<f4", 86: "<f8",
}


def _normalize_ros1(value: Any) -> Any:
    """Recursively convert ROS 1 JSON conventions to ROS 2 ones.

    - time/duration {secs, nsecs} -> {sec, nanosec}
    - drop header.seq (does not exist in ROS 2; unknown keys are ignored anyway)
    - strip the leading "/" of frame ids ("/map" -> "map")
    """
    if isinstance(value, dict):
        out = {}
        for key, v in value.items():
            if key == "secs":
                key = "sec"
            elif key == "nsecs":
                key = "nanosec"
            elif key == "seq":
                continue
            v = _normalize_ros1(v)
            if key in ("frame_id", "child_frame_id") and isinstance(v, str):
                v = v.lstrip("/")
            out[key] = v
        return out
    if isinstance(value, list):
        return [_normalize_ros1(v) for v in value]
    return value


def _float(v: Any) -> float:
    if v is None:  # rosbridge encodes NaN / inf as null
        return float("nan")
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _convert_field(slot, current: Any, value: Any) -> Any:
    if isinstance(slot, NamespacedType):
        if isinstance(value, dict):
            fill_message(current, value)
        return current

    if isinstance(slot, AbstractNestedType):
        vt = slot.value_type
        if isinstance(vt, NamespacedType):
            cls = import_message_from_namespaced_type(vt)
            return [fill_message(cls(), v) for v in (value or []) if isinstance(v, dict)]
        if value is None:
            value = []
        if isinstance(vt, BasicType) and vt.typename in _BYTE_TYPES:
            if isinstance(value, str):  # rosbridge JSON sends uint8[] as base64
                value = base64.b64decode(value)
            if isinstance(slot, Array):
                return list(bytes(value) if isinstance(value, (bytes, bytearray)) else value)
            if vt.typename == "octet":
                return [bytes([b]) for b in (bytes(value) if isinstance(value, (bytes, bytearray)) else value)]
            return array.array("B", bytes(value) if isinstance(value, (bytes, bytearray)) else [int(b) for b in value])
        if isinstance(vt, BasicType) and vt.typename in _FLOAT_TYPES:
            vals = [_float(v) for v in value]
            return vals if isinstance(slot, Array) else array.array(_FLOAT_TYPES[vt.typename], vals)
        if isinstance(vt, BasicType) and vt.typename == "boolean":
            return [bool(v) for v in value]
        if isinstance(vt, BasicType):
            return [int(v) for v in value]
        return [str(v) for v in value]

    if isinstance(slot, AbstractGenericString):
        return "" if value is None else str(value)

    if isinstance(slot, BasicType):
        if slot.typename in _FLOAT_TYPES:
            return _float(value)
        if slot.typename == "boolean":
            return bool(value)
        if slot.typename == "octet":
            return bytes([int(value or 0)])
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return current
    return current


def fill_message(msg: Any, data: dict) -> Any:
    """Fill a ROS 2 message from a (normalized) dict, ignoring fields the message does not have.

    Field names are matched case-insensitively as a fallback, which covers ROS 1 -> ROS 2 renames
    like CameraInfo D/K/R/P -> d/k/r/p.
    """
    fields = list(msg.get_fields_and_field_types().keys())
    lower = {f.lower(): f for f in fields}
    for key, value in data.items():
        name = key if key in fields else lower.get(key.lower())
        if name is None:
            continue
        slot = msg.SLOT_TYPES[fields.index(name)]
        setattr(msg, name, _convert_field(slot, getattr(msg, name), value))
    return msg


def _generic(cls: type) -> Callable[[dict], Any]:
    return lambda d: fill_message(cls(), _normalize_ros1(d))


def _yaw_quaternion(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def _stamp_dict(stamp: Time) -> dict:
    return {"secs": stamp.sec, "nsecs": stamp.nanosec}


# ----------------------------------------------------------------------------------------------
# Navis-specific message converters
# ----------------------------------------------------------------------------------------------

def _module(d: Optional[dict]) -> ModuleStatus:
    d = d or {}
    m = ModuleStatus()
    m.state = bool(d.get("state", False))
    m.name = str(d.get("name") or "")
    m.task = str(d.get("task") or "")
    return m


def _slam_status(d: dict) -> SlamStatus:
    m = SlamStatus()
    m.navigation = _module(d.get("nav"))
    m.mapping_3d = _module(d.get("mapping_3d"))
    m.mapping_2d = _module(d.get("mapping_2d"))
    m.record = _module(d.get("record"))
    return m


def _robot_status(d: dict) -> RobotStatus:
    m = RobotStatus()
    m.linear_velocity = _float(d.get("linear_velocity"))
    m.angular_velocity = _float(d.get("angular_velocity"))
    m.fault_code = int(d.get("fault_code") or 0)
    m.base_state = int(d.get("base_state") or 0)
    m.control_mode = int(d.get("control_mode") or 0)
    return m


def _battery(d: dict) -> BatteryState:
    m = BatteryState()
    m.voltage = _float(d.get("batteryVoltage"))
    m.current = _float(d.get("batteryCurrent"))
    m.temperature = _float(d.get("batteryTemperature"))
    m.percentage = _float(d.get("SOC")) / 100.0
    m.charge = float("nan")
    m.capacity = float("nan")
    m.design_capacity = float("nan")
    m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    m.present = True
    return m


def _sensor_status(d: dict) -> SensorStatus:
    def on(key: str) -> bool:
        return str(d.get(key, "")).strip().upper() == "ON"

    m = SensorStatus()
    m.lidar_on = on("lidar_status")
    m.imu_on = on("imu_status")
    m.camera_on = on("camera_status")
    m.rtk_on = on("RTK_status")
    return m


def _gnss_status(d: dict) -> GnssStatus:
    m = GnssStatus()
    try:
        m.gps_qual = int(float(d.get("gps_qual") or 0))
    except (TypeError, ValueError):
        m.gps_qual = 0
    try:
        m.num_sats = int(float(d.get("num_sats") or 0))
    except (TypeError, ValueError):
        m.num_sats = 0
    m.fix.latitude = _float(d.get("latitude"))
    m.fix.longitude = _float(d.get("longitude"))
    m.fix.altitude = _float(d.get("altitude"))
    m.fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    m.fix.status.service = NavSatStatus.SERVICE_GPS
    if m.gps_qual in (GnssStatus.QUAL_INITIALIZING, GnssStatus.QUAL_INVALID_PPS):
        m.fix.status.status = NavSatStatus.STATUS_NO_FIX
    elif m.gps_qual in (GnssStatus.QUAL_CODE_DIFFERENTIAL, GnssStatus.QUAL_RTK_FIXED, GnssStatus.QUAL_RTK_FLOAT):
        m.fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    elif m.gps_qual == GnssStatus.QUAL_WAAS_DIFFERENTIAL:
        m.fix.status.status = NavSatStatus.STATUS_SBAS_FIX
    else:
        m.fix.status.status = NavSatStatus.STATUS_FIX
    return m


# Navis DEBUG=1, INFO=2, WARN=4, ERROR=8, FATAL=16 -> rcl_interfaces/Log 10..50.
# (Log.DEBUG etc. are `byte` constants, i.e. bytes in Python, so use plain ints.)
_LOG_LEVELS = {1: 10, 2: 20, 4: 30, 8: 40, 16: 50}


def _log(d: dict) -> Log:
    d = _normalize_ros1(d)
    m = Log()
    stamp = d.get("stamp") or {}
    m.stamp.sec = int(stamp.get("sec") or 0)
    m.stamp.nanosec = int(stamp.get("nanosec") or 0)
    m.level = _LOG_LEVELS.get(int(d.get("level") or 2), 20)
    m.name = "navis"
    m.msg = str(d.get("msg") or "")
    return m


def _disk_states(d: dict) -> DiskStates:
    m = DiskStates()
    for disk in d.get("disks") or []:
        s = DiskState()
        s.name = str(disk.get("disk_name") or "")
        s.total_size = _float(disk.get("disk_total_size"))
        s.free_size = _float(disk.get("disk_free_size"))
        m.disks.append(s)
    return m


def _navigation_status(d: dict) -> NavigationStatus:
    m = NavigationStatus()
    m.map_name = str(d.get("map_name") or "")
    m.status = int(d.get("status") or 0)
    m.text = str(d.get("text") or "")
    return m


def _task_status(d: dict) -> TaskStatus:
    m = TaskStatus()
    m.task_name = str(d.get("task_name") or "")
    m.task_type = str(d.get("task_type") or "")
    m.total_looptimes = int(d.get("total_looptimes") or 0)
    m.loop_index = int(d.get("loop_index") or 0)
    m.task_index = int(d.get("task_index") or 0)
    m.total_tasks = int(d.get("total_tasks") or 0)
    m.countdown = int(d.get("countdown") or 0)
    m.is_goal_reached = bool(d.get("isGoalReach", False))
    return m


def _point_cloud(d: dict) -> PointCloud2:
    m = fill_message(PointCloud2(), _normalize_ros1(d))
    if m.row_step == 0:  # Navis sends row_step 0
        m.row_step = m.point_step * m.width
    return m


def _charge_point_pose(d: dict, frame_id: str) -> Optional[PoseStamped]:
    if not d.get("get_charge_point", False):
        return None
    cp = d.get("charge_point") or {}
    m = PoseStamped()
    m.header.frame_id = frame_id
    m.pose.position.x = _float(cp.get("x"))
    m.pose.position.y = _float(cp.get("y"))
    _yaw_quaternion(m.pose.orientation, _float(cp.get("theta")))  # radians
    return m


@dataclass
class TopicSpec:
    navis_topic: str
    navis_type: str               # ROS 1 type sent with the subscribe request
    ros_type: type
    convert: Callable[[dict], Any]
    bulky: bool = False           # request CBOR compression when use_cbor is enabled


# ----------------------------------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------------------------------

class _PendingCall:
    def __init__(self, service: str):
        self.service = service
        self.event = threading.Event()
        self.response: Optional[dict] = None
        self.error: Optional[str] = None


class NavisWebsocketClient(Node):
    def __init__(self):
        super().__init__("navis_websocket")

        self.declare_parameter("ws_url", "ws://192.168.6.102:9090")
        self.declare_parameter("always_subscribe", ["/slam_status"])
        self.declare_parameter("use_cbor", False)            # CBOR for point clouds / scans / images
        self.declare_parameter("use_navis_stamps", False)    # False: restamp with local time on receipt
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gnss_frame", "gnss")
        self.declare_parameter("service_timeout", 30.0)
        self.declare_parameter("reconnect_delay", 2.0)
        self.declare_parameter("heartbeat_period", 1.0)      # Navis drops clients silent for > 3 s
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_enabled", False)
        self.declare_parameter("max_linear_speed", 1.5)      # m/s, Navis limit
        self.declare_parameter("max_angular_speed", 1.5)     # rad/s, Navis limit

        self._url = self.get_parameter("ws_url").value
        self._always = {t if t.startswith("/") else "/" + t
                        for t in self.get_parameter("always_subscribe").value if t}
        self._use_cbor = bool(self.get_parameter("use_cbor").value)
        if self._use_cbor and cbor2 is None:
            self.get_logger().warning("use_cbor is set but cbor2 is not installed; falling back to JSON")
            self._use_cbor = False
        self._use_navis_stamps = bool(self.get_parameter("use_navis_stamps").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._gnss_frame = self.get_parameter("gnss_frame").value
        self._service_timeout = float(self.get_parameter("service_timeout").value)
        self._reconnect_delay = float(self.get_parameter("reconnect_delay").value)
        self._heartbeat_period = float(self.get_parameter("heartbeat_period").value)
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._cmd_vel_enabled = bool(self.get_parameter("cmd_vel_enabled").value)
        self._max_lin = float(self.get_parameter("max_linear_speed").value)
        self._max_ang = float(self.get_parameter("max_angular_speed").value)

        self._cb_group = ReentrantCallbackGroup()

        # --- Navis topics republished to ROS -----------------------------------------------
        specs = [
            TopicSpec("/slam_status", "tools_msgs/slamStatus", SlamStatus, _slam_status),
            TopicSpec("/interface_management/robot_status", "tools_msgs/RobotStatus", RobotStatus, _robot_status),
            TopicSpec("/interface_management/BMS_status", "tools_msgs/RobotBmsStatus", BatteryState, _battery),
            TopicSpec("/sensor_status", "tools_msgs/SensorStatus", SensorStatus, _sensor_status),
            TopicSpec("/gnss_status", "tools_msgs/GnssStatus", GnssStatus, self._gnss_with_frame),
            TopicSpec("/imu/data", "sensor_msgs/Imu", Imu, _generic(Imu)),
            TopicSpec("/points_raw", "sensor_msgs/PointCloud2", PointCloud2, _point_cloud, bulky=True),
            TopicSpec("/scan", "sensor_msgs/LaserScan", LaserScan, _generic(LaserScan), bulky=True),
            TopicSpec("/localizer_pose", "geometry_msgs/PoseStamped", PoseStamped, _generic(PoseStamped)),
            TopicSpec("/camera/color/image_raw", "sensor_msgs/Image", Image, _generic(Image), bulky=True),
            TopicSpec("/camera/color/image_raw/compressed", "sensor_msgs/CompressedImage", CompressedImage,
                      _generic(CompressedImage), bulky=True),
            TopicSpec("/camera/color/camera_info", "sensor_msgs/CameraInfo", CameraInfo, _generic(CameraInfo)),
            TopicSpec("/camera/depth/image_rect_raw", "sensor_msgs/Image", Image, _generic(Image), bulky=True),
            TopicSpec("/camera/depth/image_rect_raw/compressed", "sensor_msgs/CompressedImage", CompressedImage,
                      _generic(CompressedImage), bulky=True),
            TopicSpec("/camera/depth/camera_info", "sensor_msgs/CameraInfo", CameraInfo, _generic(CameraInfo)),
            TopicSpec("/freego_log/log", "freego_log/Log", Log, _log),
            TopicSpec("/disk_states", "tools_msgs/DiskStates", DiskStates, _disk_states),
            TopicSpec("/run_management/global_status", "support_ros/GlobalStatus", NavigationStatus,
                      _navigation_status),
            TopicSpec("/run_management/task_status", "support_ros/TaskStatus", TaskStatus, _task_status),
            TopicSpec("/run_management/task_progress", "std_msgs/Float64", Float64, _generic(Float64)),
            TopicSpec("/localization_lost", "std_msgs/Bool", Bool, _generic(Bool)),
            TopicSpec("/odom", "nav_msgs/Odometry", Odometry, _generic(Odometry)),
            TopicSpec("/odom_raw", "nav_msgs/Odometry", Odometry, _generic(Odometry)),
            TopicSpec("/run_management/visual_path", "nav_msgs/Path", Path, _generic(Path)),
            TopicSpec("/map_progress", "std_msgs/Int32", Int32, _generic(Int32)),
            TopicSpec("/find_charger/get_charge_point", "tools_msgs/getChargePoint", PoseStamped,
                      lambda d: _charge_point_pose(d, self._map_frame)),
        ]
        self._specs = {s.navis_topic: s for s in specs}
        self._pubs = {
            s.navis_topic: self.create_publisher(s.ros_type, "navis" + s.navis_topic, 10) for s in specs
        }
        unknown = self._always - self._specs.keys()
        if unknown:
            self.get_logger().warning(f"always_subscribe contains unsupported topics: {sorted(unknown)}")

        # --- Commands from ROS -----------------------------------------------------------------
        self.create_subscription(Twist, "navis/cmd_vel", self._cmd_vel_cb, 10, callback_group=self._cb_group)
        self.create_subscription(LightCommand, "navis/light_command", self._light_cb, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(PoseWithCovarianceStamped, "navis/initialpose", self._initialpose_cb, 10,
                                 callback_group=self._cb_group)

        services = [
            (NavInput, "navis/nav_input", self._nav_input_cb),
            (Trigger, "navis/cancel_task", self._cancel_task_cb),
            (SetBool, "navis/pause_task", self._pause_task_cb),
            (SetBool, "navis/next_goal", self._next_goal_cb),
            (SetBool, "navis/set_obstacle_avoidance", self._obstacle_avoidance_cb),
            (Trigger, "navis/set_charge_point", self._set_charge_point_cb),
            (GetChargePoint, "navis/get_charge_point", self._get_charge_point_cb),
            (SetBool, "navis/set_charging", self._set_charging_cb),
            (SetChargeLevel, "navis/set_charge_level", self._set_charge_level_cb),
            (SetBool, "navis/enable_cmd_vel", self._enable_cmd_vel_cb),
        ]
        for srv_type, name, handler in services:
            self.create_service(srv_type, name, self._wrap(handler), callback_group=self._cb_group)

        # --- Service call bookkeeping ---------------------------------------------------------
        self._lock = threading.Lock()
        self._pending: dict[str, _PendingCall] = {}
        self._call_ids = itertools.count(1)

        # --- Navis subscription management ----------------------------------------------------
        self._active_subs: set[str] = set()  # Navis topics currently subscribed on this connection
        self._sub_lock = threading.Lock()
        self.create_timer(1.0, self._update_subscriptions, callback_group=self._cb_group)

        # --- Websocket thread ------------------------------------------------------------------
        self._ws = None
        self._stopping = False
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="navis_ws", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------------------------------
    # Websocket thread (asyncio)
    # ------------------------------------------------------------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connection_manager())

    async def _connection_manager(self) -> None:
        connected_once = False
        while not self._stopping:
            try:
                # max_size=None: point clouds/images easily exceed the 1 MiB default.
                # ping_interval=None: Navis uses its own "ping" op instead of protocol pings.
                ws = await asyncio.wait_for(
                    websockets.connect(self._url, max_size=None, ping_interval=None), timeout=5.0
                )
            except Exception as e:  # noqa: BLE001
                if not connected_once:
                    self.get_logger().warning(f"Cannot connect to {self._url}: {e}", throttle_duration_sec=30.0)
                await asyncio.sleep(self._reconnect_delay)
                continue

            connected_once = True
            self._ws = ws
            with self._sub_lock:
                self._active_subs.clear()  # new connection: resubscribe on the next timer tick
            self.get_logger().info(f"Connected to Navis websocket at {self._url}")
            heartbeat = asyncio.ensure_future(self._heartbeat(ws))
            try:
                async for raw in ws:
                    try:
                        self._handle_raw(raw)
                    except Exception as e:  # noqa: BLE001
                        self.get_logger().error(f"Error handling Navis message: {type(e).__name__}: {e}")
            except Exception as e:  # noqa: BLE001
                if not self._stopping:
                    self.get_logger().warning(f"Navis websocket error: {e}")
            finally:
                heartbeat.cancel()
                self._ws = None
                self._fail_pending("websocket connection lost")
                try:
                    await ws.close()
                except Exception:  # noqa: BLE001
                    pass
            if not self._stopping:
                self.get_logger().warning(f"Disconnected from Navis, reconnecting in {self._reconnect_delay:.1f} s")
                await asyncio.sleep(self._reconnect_delay)

    async def _heartbeat(self, ws) -> None:
        while True:
            try:
                await ws.send(self._encode({"op": "ping", "timeStamp": int(time.time() * 1000)}))
            except Exception:  # noqa: BLE001 - the receive loop will notice the disconnect
                return
            await asyncio.sleep(self._heartbeat_period)

    @staticmethod
    def _encode(msg: dict) -> bytes:
        # Sent as UTF-8 bytes, like the Navis manual's examples.
        return json.dumps(msg, ensure_ascii=False).encode("utf-8")

    async def _async_send(self, payload: bytes) -> None:
        ws = self._ws
        if ws is None:
            raise NavisError("not connected to Navis")
        await ws.send(payload)

    def _send(self, msg: dict) -> None:
        """Send a message from a ROS thread. Raises NavisError if it could not be sent."""
        if self._ws is None:
            raise NavisError(f"not connected to Navis at {self._url}")
        future = asyncio.run_coroutine_threadsafe(self._async_send(self._encode(msg)), self._loop)
        try:
            future.result(timeout=5.0)
        except NavisError:
            raise
        except Exception as e:  # noqa: BLE001
            raise NavisError(f"send failed: {e}") from e

    # ------------------------------------------------------------------------------------------
    # Incoming messages
    # ------------------------------------------------------------------------------------------

    def _decode(self, raw) -> Optional[dict]:
        if isinstance(raw, str):
            return json.loads(raw)
        stripped = raw.lstrip()
        if stripped[:1] in (b"{", b"["):
            return json.loads(raw.decode("utf-8"))
        if cbor2 is None:
            self.get_logger().warning("Received binary (CBOR) frame but cbor2 is not installed",
                                      throttle_duration_sec=10.0)
            return None
        return cbor2.loads(raw, tag_hook=self._cbor_tag_hook)

    @staticmethod
    def _cbor_tag_hook(decoder, tag):
        if tag.tag in _CBOR_TYPED_ARRAYS and isinstance(tag.value, (bytes, bytearray)):
            dtype = _CBOR_TYPED_ARRAYS[tag.tag]
            if dtype is None:
                return bytes(tag.value)
            import numpy as np
            return np.frombuffer(tag.value, dtype=dtype).tolist()
        return tag

    def _handle_raw(self, raw) -> None:
        data = self._decode(raw)
        if not isinstance(data, dict):
            return
        op = data.get("op")
        if op == "publish":
            self._handle_publish(data)
        elif op == "service_response":
            self._handle_service_response(data)
        elif op == "status":
            self.get_logger().warning(f"Navis status: {data.get('level')} {data.get('msg')}")
        # "pong" (heartbeat reply) and anything else is ignored

    def _handle_publish(self, data: dict) -> None:
        topic = data.get("topic")
        spec = self._specs.get(topic)
        if spec is None:
            return
        msg = spec.convert(data.get("msg") or {})
        if msg is None:
            return
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            if not self._use_navis_stamps or (stamp.sec == 0 and stamp.nanosec == 0):
                msg.header.stamp = self.get_clock().now().to_msg()
        elif isinstance(msg, Log) and msg.stamp.sec == 0 and msg.stamp.nanosec == 0:
            msg.stamp = self.get_clock().now().to_msg()
        self._pubs[topic].publish(msg)

    def _gnss_with_frame(self, d: dict) -> GnssStatus:
        m = _gnss_status(d)
        m.fix.header.frame_id = self._gnss_frame
        m.fix.header.stamp = self.get_clock().now().to_msg()
        return m

    # ------------------------------------------------------------------------------------------
    # Navis subscriptions (only stream what someone listens to)
    # ------------------------------------------------------------------------------------------

    def _update_subscriptions(self) -> None:
        if self._ws is None:
            return
        wanted = {t for t, pub in self._pubs.items() if t in self._always or pub.get_subscription_count() > 0}
        with self._sub_lock:
            to_add = wanted - self._active_subs
            to_remove = self._active_subs - wanted
        for topic in sorted(to_add):
            spec = self._specs[topic]
            request = {"op": "subscribe", "topic": topic, "type": spec.navis_type}
            if self._use_cbor and spec.bulky:
                request["compression"] = "cbor"
            try:
                self._send(request)
                with self._sub_lock:
                    self._active_subs.add(topic)
                self.get_logger().info(f"Subscribed to Navis {topic}")
            except NavisError as e:
                self.get_logger().warning(f"Subscribe to {topic} failed: {e}")
                return
        for topic in sorted(to_remove):
            try:
                self._send({"op": "unsubscribe", "topic": topic})
                with self._sub_lock:
                    self._active_subs.discard(topic)
                self.get_logger().info(f"Unsubscribed from Navis {topic}")
            except NavisError as e:
                self.get_logger().warning(f"Unsubscribe from {topic} failed: {e}")
                return

    # ------------------------------------------------------------------------------------------
    # Service calls to Navis
    # ------------------------------------------------------------------------------------------

    def _call_service(self, service: str, srv_type: str, args: dict) -> dict:
        """Call a Navis service and block until its response (from a ROS executor thread)."""
        call_id = f"call_service:{service}:{next(self._call_ids)}"
        pending = _PendingCall(service)
        with self._lock:
            self._pending[call_id] = pending
        try:
            self._send({"op": "call_service", "id": call_id, "service": service, "type": srv_type, "args": args})
            if not pending.event.wait(self._service_timeout):
                raise NavisError(f"{service}: no response within {self._service_timeout:.0f} s")
            if pending.error:
                raise NavisError(f"{service}: {pending.error}")
            return pending.response or {}
        finally:
            with self._lock:
                self._pending.pop(call_id, None)

    def _handle_service_response(self, data: dict) -> None:
        with self._lock:
            pending = self._pending.get(data.get("id"))
            if pending is None or pending.event.is_set():
                # Server did not echo our id: match the oldest outstanding call to that service.
                pending = next(
                    (p for p in self._pending.values()
                     if p.service == data.get("service") and not p.event.is_set()),
                    None,
                )
            if pending is None:
                return
            pending.response = data
            pending.event.set()

    def _fail_pending(self, reason: str) -> None:
        with self._lock:
            for pending in self._pending.values():
                if not pending.event.is_set():
                    pending.error = reason
                    pending.event.set()

    @staticmethod
    def _values(data: dict) -> dict:
        values = data.get("values")
        return values if isinstance(values, dict) else {}

    def _call_checked(self, service: str, srv_type: str, args: dict) -> tuple[bool, str, dict]:
        """Call a service whose response has values.success / values.message."""
        data = self._call_service(service, srv_type, args)
        values = self._values(data)
        if not data.get("result", False):
            return False, str(values.get("message") or data.get("values") or f"{service} call failed"), values
        return bool(values.get("success", False)), str(values.get("message") or ""), values

    def _wrap(self, handler: Callable) -> Callable:
        """Turn a handler that raises NavisError into a service callback that never throws."""
        def callback(request, response):
            try:
                handler(request, response)
            except NavisError as e:
                response.success = False
                response.message = str(e)
                self.get_logger().warning(str(e))
            except Exception as e:  # noqa: BLE001
                response.success = False
                response.message = f"{type(e).__name__}: {e}"
                self.get_logger().error(f"Unexpected error in {handler.__name__}: {response.message}")
            return response
        return callback

    # ------------------------------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------------------------------

    def _nav_input_cb(self, request: NavInput.Request, response: NavInput.Response):
        args = {"file_name": request.file_name, "op_type": request.op_type, "id_type": request.id_type}
        data = self._call_service("/input/op", "tools_msgs/input", args)
        values = self._values(data)
        response.result = bool(data.get("result", False))
        response.success = response.result and bool(values.get("success", False))
        response.message = str(values.get("message") or "")

    def _cancel_task_cb(self, request: Trigger.Request, response: Trigger.Response):
        self._send({
            "op": "publish",
            "topic": "/run_management/navi_task/cancel",
            "type": "actionlib_msgs/GoalID",
            "msg": {"stamp": {"secs": 0, "nsecs": 0}, "id": ""},
        })
        response.success = True
        response.message = "Cancel sent; check navis/run_management/global_status for the result"

    def _pause_task_cb(self, request: SetBool.Request, response: SetBool.Response):
        # The manual lists actionlib_msgs/GoalID as the type; it is only used by roslib.js.
        response.success, response.message, _ = self._call_checked(
            "/run_management/pause", "actionlib_msgs/GoalID", {"pause": request.data, "reason": 0}
        )

    def _next_goal_cb(self, request: SetBool.Request, response: SetBool.Response):
        response.success, response.message, _ = self._call_checked(
            "/run_management/next_goal", "std_srvs/SetBool", {"data": request.data}
        )

    def _obstacle_avoidance_cb(self, request: SetBool.Request, response: SetBool.Response):
        response.success, response.message, _ = self._call_checked(
            "/run_management/switch_obstacle_avoid_model", "std_srvs/SetBool", {"data": request.data}
        )

    def _set_charge_point_cb(self, request: Trigger.Request, response: Trigger.Response):
        data = self._call_service("/find_charger/set_charge_point", "std_srvs/Empty", {})
        response.success = bool(data.get("result", False))
        response.message = "Charge point set to the current robot position" if response.success \
            else "set_charge_point failed"

    def _get_charge_point_cb(self, request: GetChargePoint.Request, response: GetChargePoint.Response):
        data = self._call_service("/find_charger/get_charge_point", "tools_msgs/getChargePoint", {})
        values = self._values(data)
        pose = _charge_point_pose(values, self._map_frame) if data.get("result", False) else None
        if pose is None:
            response.success = False
            response.message = "No charge point set" if data.get("result", False) else "get_charge_point failed"
            return
        pose.header.stamp = self.get_clock().now().to_msg()
        response.charge_point = pose
        response.success = True
        response.message = "success"

    def _set_charging_cb(self, request: SetBool.Request, response: SetBool.Response):
        response.success, response.message, _ = self._call_checked(
            "/find_charger/set_charge", "std_srvs/SetBool", {"data": request.data}
        )

    def _set_charge_level_cb(self, request: SetChargeLevel.Request, response: SetChargeLevel.Response):
        response.success, response.message, values = self._call_checked(
            "/find_charger/set_charge_level", "tools_msgs/setChargeLevel",
            {"min_charge_level": request.min_charge_level, "max_charge_level": request.max_charge_level},
        )
        response.min_charge_level = max(0, min(255, int(values.get("min_charge_level") or 0)))
        response.max_charge_level = max(0, min(255, int(values.get("max_charge_level") or 0)))

    def _enable_cmd_vel_cb(self, request: SetBool.Request, response: SetBool.Response):
        self._cmd_vel_enabled = request.data
        if not request.data:
            try:
                self._send_twist(0.0, 0.0)  # stop the robot when manual control is released
            except NavisError as e:
                self.get_logger().warning(f"Could not send stop command: {e}")
        response.success = True
        response.message = f"cmd_vel forwarding {'enabled' if request.data else 'disabled'}"

    # ------------------------------------------------------------------------------------------
    # Topic commands
    # ------------------------------------------------------------------------------------------

    def _send_twist(self, linear_x: float, angular_z: float) -> None:
        self._send({
            "op": "publish",
            "topic": self._cmd_vel_topic,
            "type": "geometry_msgs/Twist",
            "msg": {
                "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
            },
        })

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if not self._cmd_vel_enabled:
            self.get_logger().warning("navis/cmd_vel ignored: call navis/enable_cmd_vel first",
                                      throttle_duration_sec=5.0)
            return
        lin = max(-self._max_lin, min(self._max_lin, msg.linear.x))
        ang = max(-self._max_ang, min(self._max_ang, msg.angular.z))
        try:
            self._send_twist(lin, ang)
        except NavisError as e:
            self.get_logger().warning(f"cmd_vel not sent: {e}", throttle_duration_sec=5.0)

    def _light_cb(self, msg: LightCommand) -> None:
        try:
            self._send({
                "op": "publish",
                "topic": "/scout_light_control",
                "type": "scout_msgs/ScoutLightCmd",
                "msg": {
                    "enable_cmd_light_control": msg.enable_cmd_light_control,
                    "front_mode": msg.front_mode,
                    "front_custom_value": min(100, msg.front_custom_value),
                    "rear_mode": msg.rear_mode,
                    "rear_custom_value": min(100, msg.rear_custom_value),
                },
            })
        except NavisError as e:
            self.get_logger().warning(f"Light command not sent: {e}")

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        try:
            self._send({
                "op": "publish",
                "topic": "/initialpose",
                "type": "geometry_msgs/PoseWithCovarianceStamped",
                "msg": {
                    "header": {
                        "seq": 0,
                        "stamp": _stamp_dict(msg.header.stamp),
                        "frame_id": msg.header.frame_id or self._map_frame,
                    },
                    "pose": {
                        "pose": {
                            "position": {"x": p.position.x, "y": p.position.y, "z": p.position.z},
                            "orientation": {"x": p.orientation.x, "y": p.orientation.y,
                                            "z": p.orientation.z, "w": p.orientation.w},
                        },
                        "covariance": [float(c) for c in msg.pose.covariance],
                    },
                },
            })
            self.get_logger().info(f"Initial pose sent: ({p.position.x:.3f}, {p.position.y:.3f})")
        except NavisError as e:
            self.get_logger().warning(f"Initial pose not sent: {e}")

    # ------------------------------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------------------------------

    def destroy_node(self):
        self._stopping = True
        ws = self._ws
        if ws is not None and self._loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(ws.close(), self._loop).result(timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        self._fail_pending("node shutting down")
        self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavisWebsocketClient()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
