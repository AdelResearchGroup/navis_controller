import asyncio
import json
import threading
import time
from typing import Any, Type
import numpy as np

import rclpy
import rclpy.publisher
import websockets
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_parser.definition import NamespacedType
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from navis_msgs.srv import NavInput
from navis_msgs.msg import Position
from nav_msgs.msg import Odometry

class NavisWebsocketClient(Node):
    def __init__(self, uri: str = "ws://192.168.6.102:9090"):
        super().__init__("navis_websocket") # Inits as ROS2 Node
        self.uri = uri
        self._publishers_dict: dict[str, rclpy.publisher.Publisher] = {} # Dictionary of publishers to send navis info as ROS2 Objects
        self._service_lock = threading.Lock() # A thread lock, since _service_responses and _pending_response_flags are accessed by both the main ROS2 threads and the websockets thread
        self._service_responses: dict[str, dict] = {} # Dictionary of most recent response from service calls
        self._pending_response_flags: dict[str, threading.Event] = {} # Dictionary of resposne flags (not set, set) from service calls
        self._msg_types: dict[str, Type] = {} # Dictionary of type of msg data for each subscribed publisher
        self._cmd_vel_on = False
        self._tracking_end_point = False
        self._task_tolerance = 0.1

        # Callback groups to make sure cbs can run concurrently, reentrant means cbs of the same group can also run concurrently
        self._topic_group = ReentrantCallbackGroup()
        self._service_group = ReentrantCallbackGroup()
        self._timer_group = ReentrantCallbackGroup()
        self._movement_group = MutuallyExclusiveCallbackGroup()

        self._nav_input_srv = "navis/nav_input"
        self._start_odom_srv = "navis/start_odom"
        self._odom_topic = "/odom"
        self._start_device_status_srv = "navis/start_device_status"
        self._device_status_topic = "/slam_status"
        self._get_nav_status_srv = "navis/get_nav_status"
        self._nav_status_topic = "/run_management/global_status"
        self._start_cmd_vel_srv = "/navis/start_cmd_vel"
        self._stop_cmd_vel_srv = "/navis/stop_cmd_vel"
        # self._start_nav_srv = "navis/start_nav"
        # self._stop_nav_srv = "navis/stop_nav"
        self._cancel_nav_task_srv = "navis/cancel_nav_task"
        self._pause_nav_task_srv = "navis/pause_nav_task"
        self._resume_nav_task_srv = "navis/resume_nav_task"
        self._cmd_vel_topic = "navis/cmd_vel_always"
        self._end_point_topic = "navis/end_point"

        self.create_service(
            NavInput, self._nav_input_srv, self._nav_input_cb, callback_group=self._service_group
        )
        self.create_service(
            Trigger, self._start_odom_srv, self._start_odom_cb, callback_group=self._service_group
        )
        self.create_service(Trigger, self._start_device_status_srv, self._start_device_status_cb, callback_group=self._service_group)
        self.create_service(Trigger, self._get_nav_status_srv, self._get_nav_status_cb, callback_group=self._service_group)

        self.create_service(Trigger, self._start_cmd_vel_srv, self._start_cmd_vel_cb, callback_group=self._movement_group)
        self.create_service(Trigger, self._stop_cmd_vel_srv, self._stop_cmd_vel_cb, callback_group=self._movement_group)
        # self.create_service(Trigger, self._start_nav_srv, self._start_nav_cb, callback_group=self._movement_group)
        # self.create_service(Trigger, self._stop_nav_srv, self._stop_nav_cb, callback_group=self._movement_group)
        self.create_service(Trigger, self._cancel_nav_task_srv, self._cancel_nav_task_cb, callback_group=self._movement_group)
        self.create_service(Trigger, self._pause_nav_task_srv, self._pause_nav_task_cb, callback_group=self._movement_group)
        self.create_service(Trigger, self._resume_nav_task_srv, self._resume_nav_task_cb, callback_group=self._movement_group)

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_cb, 10, callback_group=self._movement_group)
        self.create_subscription(Position, self._end_point_topic, self._end_point_cb, 10, callback_group=self._topic_group)
        self.create_subscription(Odometry, f"navis{self._odom_topic}", self._odom_cb, 10, callback_group=self._movement_group)

        self._ws = None # websocket object
        self._ws_loop = asyncio.new_event_loop() # Creates a new asyncio event loop to manage calls
        self._ws_thread = threading.Thread(target=self._start_websocket_loop, daemon=True) # Creates a new thread for websockets, independent of main ROS2, which runs _start_websocket_loop when started
        self._ws_thread.start() # Starts the thread

        self.heartbeat_timer = self.create_timer(1.0, self._send_heartbeat, callback_group=self._timer_group)

        # region Temporary hard coded navigation start
        time.sleep(1)
        # args = {
        #     "file_name": "251009_UserStudy",
        #     "op_type": "start",
        #     "id_type": "navigation",
        # }
        # self._call_service("/input/op", "tools_msgs/input", args)
        self._subscribe(self._odom_topic, "nav_msgs/msg/Odometry")
        # endregion Temporary hard coded navigation start

    @property
    def logger(self):
        return self.get_logger()

    def _start_websocket_loop(self) -> None:
        asyncio.set_event_loop(self._ws_loop) # Sets the event loop of the websocket thread as the asyncio made one
        self._ws_loop.run_until_complete(self._websocket_handler()) # Keeps running the event loop

    # Continuously runs, processing incoming messages from websockets
    async def _websocket_handler(self) -> None:
        async with websockets.connect(self.uri) as ws: # Opens websocket link
            self._ws = ws
            self.logger.info(f"Connected to WebSocket server at {self.uri}")
            while rclpy.ok():
                raw = await ws.recv() # Pauses until ws has received a message, other tasks can run
                try:
                    self._process_websocket_message(raw)
                except Exception as e:
                    self.logger.error(f"Error processing websocket message: {e}")
                    continue

    def _process_websocket_message(self, raw: str) -> None:
        data = json.loads(raw) # Loads string data as a dictionary/Python type
        self.logger.debug(f"Received message: {data}")
        assert isinstance(data, dict), "Invalid websocket response format"
        op = data.get("op")
        if op == "service_response": # Means a service_call got responded to
            service = data.get("service")
            with self._service_lock: # Locks so that ROS2 thread can't do anything inside a lock, waits if locked until unlocked
                if service in self._pending_response_flags: # If the service was actually called
                    self._service_responses[service] = data 
                    self._pending_response_flags[service].set() # Sets the response flag, allowing ROS2 thread waiting on it to continue
            return
        elif op != "publish":
            return
        topic = data.get("topic")

        if topic not in self._publishers_dict:
            self.logger.warning(f"Received message for unsubscribed topic: {data}")
            return

        msg_cls = self._msg_types[topic] # ie. Odometry, set when subscribed
        content = data.get("msg")

        def parse_dict(d: dict, msg: Any) -> Any: # Returns a msg type object, ie. Odometry, with proper attributes set
            for key, value in d.items():
                if key == "secs":
                    key = "sec"
                elif key == "nsecs":
                    key = "nanosec"
                elif key == "seq":
                    continue
                if isinstance(value, dict):
                    sub_msg = getattr(msg, key)
                    parse_dict(value, sub_msg)
                else:
                    setattr(msg, key, value)
            return msg

        msg = parse_dict(content, msg_cls())
        if(hasattr(msg, "header")):
            msg.header.stamp = self.get_clock().now().to_msg()  # !!! Temp timing fix
        self._publishers_dict[topic].publish(msg) # Publishes to ROS2 

    def _send(self, msg: dict | str) -> None:
        if self._ws is None:
            self.logger.error("WebSocket connection is not established, cannot send message.")
            return
        if isinstance(msg, dict):
            msg = json.dumps(msg)
        try:
            asyncio.run_coroutine_threadsafe(self._ws.send(msg.encode("utf-8")), self._ws_loop).result() # .result() forces ROS2 thread to wait until coroutine (_ws.send) has finished
        except Exception as e:
            self.logger.error(f"Failed to send message: {e}")

    def _call_service(self, service: str, srv_type: str, args: dict) -> dict:
        if not service.startswith("/"):
            service = f"/{service}"
        request = {"op": "call_service", "service": service, "type": srv_type, "args": args}
        response_flag = threading.Event() # Creates an event, default not set
        with self._service_lock:
            self._pending_response_flags[service] = response_flag # adds flag to dict, so other thread can change it
        self._send(request) # Sends the request
        response_flag.wait() # Waits until a response is received and the websocket thread modifies the flag
        with self._service_lock:
            response = self._service_responses.pop(service) # Gets resonse and removes from dict
            self._pending_response_flags.pop(service) # Removes flag from dict
        return response

    def _send_heartbeat(self):
        if self._ws is None:
            return
        heartbeat_msg = {"op": "ping", "timeStamp": str(time.time() * 1000).split(".")[0]}
        self._send(heartbeat_msg)

    def _subscribe(self, topic: str, msg_type: str) -> None:
        if not topic.startswith("/"):
            topic = f"/{topic}"
        try:
            split_type = msg_type.split("/")
            namespace_type = NamespacedType(namespaces=split_type[:-1], name=split_type[-1])
            msg_cls = import_message_from_namespaced_type(namespace_type) # Gets the type of message, ie. Odometry
        except (ModuleNotFoundError, ValueError) as e:
            raise RuntimeError(f"Failed to import message type {msg_type}: {e}")

        if topic not in self._publishers_dict:
            self._publishers_dict[topic] = self.create_publisher(msg_cls, f"navis{topic}", 10, callback_group=self._topic_group)
            self._msg_types[topic] = msg_cls
            self.logger.info(f"Create publisher for topic {topic} with type {msg_cls}")

        subscribe_msg = {"op": "subscribe", "topic": topic}
        self._send(subscribe_msg) # Sends message to subscribe to topic by navis protocol

    def _nav_input_cb(self, request: NavInput.Request, response: NavInput.Response) -> NavInput.Response:
        args = {
            "file_name": request.file_name,
            "op_type": request.op_type,
            "id_type": request.id_type,
        }
        try:
            response_data = self._call_service("/input/op", "tools_msgs/input", args)
            response.result = response_data.get("result")
            response.success = response_data.get("values", {}).get("success")
            response.message = str(response_data.get("values", {}).get("message"))
        except Exception as e:
            response.result = False
            response.success = False
            response.message = f"Failed to send navigation request: {e}"
        return response


    def _start_odom_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._subscribe(self._odom_topic, "nav_msgs/msg/Odometry")
            response.success = True
            response.message = f"Subscribed to {self._odom_topic}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to subscribe to {self._odom_topic}: {e}"
        return response
    
    def _start_device_status_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._subscribe(self._device_status_topic, "navis_msgs/msg/DeviceStatus")
            response.success = True
            response.message = f"Subscribed to {self._device_status_topic}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to subscribe to {self._device_status_topic}: {e}"
        return response
    
    def _get_nav_status_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response: 
        try:
            self._subscribe(self._nav_status_topic, "navis_msgs/msg/NavStatus")
            response.success = True
            response.message = f"Subscribed to {self._nav_status_topic}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to subscribe to {self._nav_status_topic}: {e}"
        return response
    
    def _start_cmd_vel_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._cmd_vel_on = True
        response.success = True
        response.message = "cmd_vel on, move robot with: " + self._cmd_vel_topic
        return response
    
    def _stop_cmd_vel_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._cmd_vel_on = False
        response.success = True
        response.message = "cmd_vel off"
        return response

    def _cmd_vel_cb(self, msg: Twist):
        if(not self._cmd_vel_on):
            return
        
        request_data = {
            "op": "publish", 
            "topic": "/run_management/virtual_joy", 
            "type": "geometry_msgs/Twist", 
            "msg": {
                "linear": {
                    "x": msg.linear.x,
                    "y": 0,
                    "z": 0
                },
                "angular": {
                    "x": 0,
                    "y": 0,
                    "z": msg.angular.z
                }
            }
        }
        # try:
        #     self._send(request_data)
        # except Exception as e:
        #     self.logger.error(f"Failed to send vel data: {e}")
        #     pass

    def _end_point_cb(self, msg: Position):
        self._end_point = msg
        self._tracking_end_point = True

    def _odom_cb(self, msg: Odometry):
        if(not self._tracking_end_point):
            return
        
        base_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        final_pose = np.array([self._end_point.x, self._end_point.y])
        if(np.linalg.norm(base_pose - final_pose) < self._task_tolerance):
            self._cancel_nav_task()
            self.logger.info("Got to end point")

    # def _start_nav_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
    #     args = {
    #         "file_name": "",
    #         "op_type": "start",
    #         "id_type": "follow_line"
    #     }
    #     try:
    #         response_data = self._call_service("/input/op", "tools_msgs/input", args)
    #         response.result = response_data.get("result")
    #         response.success = response_data.get("success")
    #         response.message = response_data.get("message")
    #     except Exception as e:
    #         response.result = False
    #         response.success = False
    #         response.message = f"Failed to send navigation request: {e}"
    #     return response

    def _cancel_nav_task(self) -> Trigger.Response:
        response = Trigger.Response()

        stamp = self.get_clock().now().to_msg()
        request_data = {
            "op": "publish",
            "topic": "/run_management/navi_task/cancel",
            "type": "actionlib_msgs/GoalID",
            "msg":{
                "stamp": {
                    "secs": stamp.sec,
                    "nsecs": stamp.nanosec
                },
                "id": ""
            }
        }

        try:
            self._send(request_data)
            response.message = "Sent cancel request succesfully, check task for status"
            response.success = True
            self._tracking_end_point = False
        except Exception as e:
            response.message = f"Failed to send cancel request: {e}"
            response.success = False

        return response


    def _cancel_nav_task_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response = self._cancel_nav_task()
        return response
    
    def _pause_nav_task_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        args = {     
            "pause": True,
            "reason": 0
        }
        try:
            response_data = self._call_service("/run_management/pause", "actionlib_msgs/GoalID", args)
            response.success = response_data.get("values", {}).get("success")
            response.message = response_data.get("values", {}).get("message")
        except Exception as e:
            response.success = False
            response.message = f"Failed to send pause request: {e}"
        return response
    
    def _resume_nav_task_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        args = {     
            "pause": False,
            "reason": 0
        }
        try:
            response_data = self._call_service("/run_management/pause", "actionlib_msgs/GoalID", args)
            response.success = response_data.get("values", {}).get("success")
            response.message = response_data.get("values", {}).get("message")
        except Exception as e:
            response.success = False
            response.message = f"Failed to send resume request: {e}"
        return response



def main(args=None):
    rclpy.init(args=args)

    node = NavisWebsocketClient()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    args = []
    # args = ["--ros-args", "--log-level", "DEBUG"]
    main(args=args)
