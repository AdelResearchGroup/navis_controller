import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from navis_msgs.srv import GetPose, GetPoseStr
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.parameter import Parameter

def _normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q if n == 0 else q / n

def _quat_dot(q1: np.ndarray, q2: np.ndarray) -> float:
    return float(np.dot(q1, q2))

def _slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    q1 = _normalize_quat(q1)
    q2 = _normalize_quat(q2)
    dot = _quat_dot(q1, q2)

    if dot < 0.0:
        q2 = -q2
        dot = -dot

    if dot > 0.9995:
        return _normalize_quat((1 - t) * q1 + t * q2)

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s1 = math.sin(theta_0 - theta) / sin_theta_0
    s2 = sin_theta / sin_theta_0

    return _normalize_quat(s1 * q1 + s2 * q2)

def _ema_alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    a = dt / (tau + dt) if dt > 0.0 else 0.0
    return max(0.0, min(1.0, a))

class OdomSmoother(Node):
    def __init__(self):
        super().__init__("odom_smoother")

        self.declare_parameter("tau_pos", 1.0)
        self.declare_parameter("tau_rot", 1.0)

        self.tau_pos: float = self.get_parameter("tau_pos").get_parameter_value().double_value
        self.tau_rot: float = self.get_parameter("tau_rot").get_parameter_value().double_value
        self.input_topic = "/navis/odom_raw"
        self.output_topic = "/navis/odom_filtered"
        self.srv_topic = "/navis/get_pose"

        self._peh = ParameterEventHandler(self)
        self._peh.add_parameter_callback(
            node_name=self.get_fully_qualified_name(),
            parameter_name="tau_pos",
            callback=self._on_tau_pos_changed,
        )
        self._peh.add_parameter_callback(
            node_name=self.get_fully_qualified_name(),
            parameter_name="tau_rot",
            callback=self._on_tau_rot_changed,
        )

        self.sub = self.create_subscription(Odometry, self.input_topic, self._on_odom, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.get_pose_srv = self.create_service(GetPose, self.srv_topic, callback=self.get_pose_cb)
        self.get_pose_str_srv = self.create_service(GetPoseStr, f"{self.srv_topic}_str", callback=self.get_pose_str_cb)

        self._have_state = False
        self._pos_filt = np.zeros(3)
        self._quat_filt = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self._last_stamp_ns = None

        self.get_logger().info(
            f"Odom smoother listening on {self.input_topic}, publishing {self.output_topic} "
            f"(tau_pos={self.tau_pos:.2f}s, tau_ori={self.tau_rot:.2f}s)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        t_ns = (msg.header.stamp.sec * 1e9 ) + msg.header.stamp.nanosec
        if self._last_stamp_ns is None:
            dt = 0.0
        else:
            dt = max(0.0, (t_ns - self._last_stamp_ns) * 1e-9)
        self._last_stamp_ns = t_ns

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pos_meas = np.array([p.x, p.y, p.z], dtype=float)
        quat_meas = np.array([q.w, q.x, q.y, q.z], dtype=float)

        if not self._have_state:
            self._pos_filt = pos_meas.copy()
            self._quat_filt = _normalize_quat(quat_meas)
            self._have_state = True
        else:
            a_pos = _ema_alpha(dt, self.tau_pos)
            a_rot = _ema_alpha(dt, self.tau_rot)
            self._pos_filt = (1 - a_pos) * self._pos_filt + a_pos * pos_meas
            self._quat_filt = _slerp(self._quat_filt, quat_meas, a_rot)
        
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        out.pose.pose.position.x = float(self._pos_filt[0])
        out.pose.pose.position.y = float(self._pos_filt[1])
        out.pose.pose.position.z = float(self._pos_filt[2])
        out.pose.pose.orientation = Quaternion(
            w=float(self._quat_filt[0]),
            x=float(self._quat_filt[1]),
            y=float(self._quat_filt[2]),
            z=float(self._quat_filt[3])
        )

        out.pose.covariance = list(msg.pose.covariance)
        out.twist = msg.twist
        self.pub.publish(out)

    def get_pose_cb(self, request: GetPose.Request, response: GetPose.Response) -> GetPose.Response:
        if not self._have_state:
            response.success = False
            response.message = "No odometry data available"
            return response

        response.success = True
        # TODO: add header
        response.pose.pose.position.x = float(self._pos_filt[0])
        response.pose.pose.position.y = float(self._pos_filt[1])
        response.pose.pose.position.z = float(self._pos_filt[2])
        response.pose.pose.orientation.w = float(self._quat_filt[0])
        response.pose.pose.orientation.x = float(self._quat_filt[1])
        response.pose.pose.orientation.y = float(self._quat_filt[2])
        response.pose.pose.orientation.z = float(self._quat_filt[3])
        return response
    
    def get_pose_str_cb(self, request: GetPoseStr.Request, response: GetPoseStr.Response) -> GetPoseStr.Response:
        if not self._have_state:
            response.success = False
            response.message = "No odometry data available"
            return response

        delimiter = request.delimiter if request.delimiter else " "
        pos_precision = request.pos_precision if request.pos_precision > 0 else 3
        quat_precision = request.quat_precision if request.quat_precision > 0 else 7

        pos_str = delimiter.join(
            f"{coord:.{pos_precision}f}" for coord in self._pos_filt
        )
        quat_str = delimiter.join(
            f"{coord:.{quat_precision}f}" for coord in self._quat_filt
        )
        response.pose_str = f"{pos_str}{delimiter}{quat_str}"
        response.success = True
        response.message = "Pose string generated successfully"
        return response

    def _on_tau_pos_changed(self, p: Parameter) -> None:
        self.tau_pos = float(p.value.double_value)
        self.get_logger().info(f"tau_pos changed to {self.tau_pos:.2f}s")
    
    def _on_tau_rot_changed(self, p: Parameter) -> None:
        self.tau_rot = float(p.value.double_value)
        self.get_logger().info(f"tau_rot changed to {self.tau_rot:.2f}s")

def main() -> None:
    rclpy.init()
    node = OdomSmoother()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
