import json
from PIL import Image
from io import BytesIO
import numpy as np
import time

import rclpy
# import rclpy.publisher
import requests
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict, set_message_fields

from std_srvs.srv import Trigger
from navis_msgs.srv import GetMap, GetTask, GetAction, GetMapInfo, GetMapList, ExecuteRTTask, ExecuteListTask, SetTaskInfo
from navis_msgs.msg import Task, TaskPoint, Position
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import uuid


class NavisHTTPClient(Node):
    def __init__(self, url: str = "http://192.168.6.102:8880"):
        super().__init__("navis_http")
        self._url = url
        self._logged_in = False
        self._authorization_token = None

        self._login_srv_name = "navis/http_login"
        self._get_map_srv_name = "navis/get_map_img"
        self._get_task_srv_name = "navis/get_task"
        self._get_map_list_srv_name = "navis/get_map_list"
        self._get_map_info_srv_name = "navis/get_map_info"
        self._get_action_srv_name = "navis/get_action"
        self._set_task_info_name = "navis/set_task_info"
        self._execute_rt_task_srv_name = "navis/execute_rt_task"
        self._execute_list_task_srv_name = "navis/execute_list_task"

        self._map_topic = "navis/map_img"
        self._end_point_topic = "navis/end_point"
        self._click_topic = "navis/click_pt"

        self._bridge = CvBridge()
        self._end_point = Position()

        self._http_group = ReentrantCallbackGroup() # Probably unecessary right now
        self._topic_group = ReentrantCallbackGroup()

        self.create_service(Trigger, self._login_srv_name, self._login_cb, callback_group=self._http_group)
        self.create_service(GetMap, self._get_map_srv_name, self._get_map_cb, callback_group=self._http_group)
        self.create_service(GetTask, self._get_task_srv_name, self._get_task_cb, callback_group=self._http_group)
        self.create_service(GetMapList, self._get_map_list_srv_name, self._get_map_list_cb, callback_group=self._http_group)
        self.create_service(GetMapInfo, self._get_map_info_srv_name, self._get_map_info_cb, callback_group=self._http_group)
        self.create_service(GetAction, self._get_action_srv_name, self._get_action_cb, callback_group=self._http_group)

        self._map_publisher = self.create_publisher(ImageMsg, self._map_topic, 10, callback_group=self._topic_group)
        self._end_point_publisher = self.create_publisher(Position, self._end_point_topic, 10, callback_group=self._topic_group)
        self.create_subscription(Point, self._click_topic, self._click_cb, 10, callback_group=self._http_group)
        self.create_service(SetTaskInfo, self._set_task_info_name, self._set_task_info_cb, callback_group=self._http_group)

        self.create_service(ExecuteRTTask, self._execute_rt_task_srv_name, self._execute_rt_task_cb, callback_group=self._http_group)
        self.create_service(ExecuteListTask, self._execute_list_task_srv_name, self._execute_list_task_cb, callback_group=self._http_group)

        login_response = self._login()
        while(not login_response.success):
            self.logger.error(login_response.message)
            time.sleep(1)


    @property
    def logger(self):
        return self.get_logger()

    def _login_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Callback to login service, sends a login request."""
        response = self._login()
        return response
    
    def _login(self) -> Trigger.Response:
        """Logs in to the HTTP server."""
        response = Trigger.Response()
        self._logged_in = False
        payload = {
            "username": "agilex",
            "password": "NTdhMTE5NGNiMDczY2U4YjNiYjM2NWU0YjgwNWE5YWU="
        } 
        try:
            r = requests.post(self._url + "/user/passport/login", json=payload, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
        except Exception as e:
            response.success = False
            response.message = f"ERROR, proper http connection failed: {e}"
            return response

        if(not web_response["success"]):
            response.success = False
            msg = web_response["msg"]
            code = web_response["code"]
            response.message = f"Login unsuccesful, {msg}: {code}"
            return response
        
        self._authorization_token = web_response["data"]
        self._logged_in = True
        response.success = True
        response.message = f"Succesfully logged in to {self._url}"
        return response
    
    def _get_map_cb(self, request: GetMap.Request, response: GetMap.Response) -> GetMap.Response:
        """
        Sends a request to get the png image of a given map stored on the Navis.
        
        Args:
            request (GetMap.Request): The service request, includes the map's name.
            response (GetMap.Response): The service response.

        Returns:
            GetMap.Response: The service response.
        """
        if(not self._logged_in):
            response.message = "Not logged in"
            response.success = False
            # response.map = [] # sensor_msgs/Image map removed from GetMap
            return response
        
        headers = {
            "Authorization": self._authorization_token
        }
        payload = {
            "mapName": request.map_name
        }
        try:
            r = requests.get(self._url + "/downloadpng", headers=headers, params=payload, timeout=5.0)
            r.raise_for_status()
            pil_img = Image.open(BytesIO(r.content))
            cv_img = np.array(pil_img)
            ros_img = self._bridge.cv2_to_imgmsg(cv_img, encoding="passthrough")
            # response.map = ros_img
            response.message = f"Succesfully got map {request.map_name}"
            response.success = True
            self._map_publisher.publish(ros_img)
        except Exception as e:
            response.message = f"Failed to get map: {e}"
            response.success = False
            # response.map = []

        self._curr_click_map_name = request.map_name # Temporary variable for testing
        return response
    
    def _click_cb(self, msg):
        """
        Callback when a point is published by map viewer, moves the robot to that point for testing.

        This function is just used for testing so that the user can easily command the robot to move
        to different places without manually inputting coordinates. The published coordinates are
        in pixel coordinates, so they get transformed into map coordinates. Then a real time task
        is started and commands the robot to the calcuated coordinates.
        
        Args:
            msg (Point): The pixel coordinates clicked.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        payload = {
            "mapName": self._curr_click_map_name,
            "pngX": int(msg.x),
            "pngY": int(msg.y)
        }
        try:
            r = requests.get(self._url + "/get_map/positon", headers=headers, params=payload, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
        except Exception as e:
            self.logger.error(f"Failed to get proper coordinates: {e}")
            return

        if(web_response["success"]):
            self._curr_x = web_response["data"]["map_pose_x"]
            self._curr_y = web_response["data"]["map_pose_y"]
            self.logger.info(f"x: {self._curr_x}, y: {self._curr_y}")
        else:
            message = web_response["msg"]
            code = web_response["code"]
            self.logger.error(f"Unsuccesful coordinate transformation, {message}: {code}")
            return
        
        position = Position()
        position.x = float(self._curr_x)
        position.y = float(self._curr_y)
        position.theta = 0.0

        # actions = Actions()
        # actions.device_id = "robot_666"
        # actions.action_type = "move"
        # actions.action_content = "1"
        actions = []
        
        point = TaskPoint()
        # point.id = "task-" + str(uuid.uuid4())
        point.id = str(uuid.uuid4())
        point.position = position
        point.point_type = "navigation"
        point.point_name = "click_point"
        point.actions = actions
        points = [point]

        task = Task()
        task.map_name = self._curr_click_map_name
        task.task_name = "click_task"
        task.edited_name = "click_task"
        task.mode = "point"
        task.evadible = 1
        task.speed = 1.0
        task.rotate_speed = 1.0
        task.remark = ""
        task.person_name = "Alex C"
        task.points = points

        # task_dict = dict(message_to_ordereddict(task))
        # self.logger.info(json.dumps(task_dict, indent=4))
        # request = SetTaskInfo.Request()
        # response = SetTaskInfo.Response()
        # request.task = task
        # set_response = self._set_task_info(request, response)
        # success = set_response.success
        # message = set_response.msg
        # code = set_response.code
        # self.logger.info(f"Success: {success}, {message}: {code}")


        request = ExecuteRTTask.Request()
        response = ExecuteRTTask.Response()

        request.loop_time = 1
        request.mode = "point"
        request.points = points

        response = self._start_rt_task(request, response)
        success = response.success
        message = response.msg
        code = response.code
        self.logger.info(f"Success: {success}, {message}: {code}")

    def _get_action_cb(self, request: GetAction.Request, response: GetAction.Response) -> GetAction.Response:
        """
        Get the list of actions for all devices, including fixed actions for the robot itself and additional device actions.
        
        Args:
            request (GetAction.Request): The service request.
            response (GetAction.Response): The service response.

        Returns:
            GetAction.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        try:
            r = requests.get(self._url + "/get_action_list", headers=headers, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            # self.logger.info(str(web_response))
            for i in range(len(web_response["data"])):
                web_response["data"][i]["device_type"] = web_response["data"][i].pop("deviceType")
                for j in range(len(web_response["data"][i]["children"])):
                    for k in range(len(web_response["data"][i]["children"][j]["action"])):
                        web_response["data"][i]["children"][j]["action"][k]["content_options"] = web_response["data"][i]["children"][j]["action"][k].pop("contentOptions")
            set_message_fields(response, web_response)
        except Exception as e:
            response.success = False
            response.msg = f"Failed to get action info: {e}"
        return response

    def _get_task_cb(self, request: GetTask.Request, response: GetTask.Response) -> GetTask.Response:
        """
        Gets information about a given task in a given map, such as its points.
        
        Args:
            request (GetTask.Request): The service request, includes map name and task name.
            response (GetTask.Response): The service response.

        Returns:
            GetTask.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        payload = {
            "mapName": request.map_name,
            "taskName": request.task_name
        }
        try:
            r = requests.get(self._url + "/get_tasks", headers=headers, params=payload, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
        except Exception as e:
            response.success = False
            response.message = f"Failed to get task info: {e}"
            return response
        
        message = web_response["msg"]
        code = web_response["code"]

        if(not web_response["success"]):
            response.success = False
            response.message = f"Task info request returned failed, {message}: {code}"
            return response
        
        web_response = web_response["data"]
        web_response["map_name"] = web_response.pop("mapName")
        web_response["task_name"] = web_response.pop("taskName")
        web_response["edited_name"] = web_response.pop("editedName")
        web_response["rotate_speed"] = web_response.pop("rotateSpeed")
        web_response["person_name"] = web_response.pop("personName")
        for i in range(len(web_response["points"])):
            web_response["points"][i]["is_new"] = web_response["points"][i].pop("isNew")
            web_response["points"][i]["point_type"] = web_response["points"][i].pop("pointType")
            web_response["points"][i]["point_name"] = web_response["points"][i].pop("pointName")
            for j in range(len(web_response["points"][i]["actions"])):
                web_response["points"][i]["actions"][j]["device_id"] = web_response["points"][i]["actions"][j].pop("deviceId")
                web_response["points"][i]["actions"][j]["action_type"] = web_response["points"][i]["actions"][j].pop("actionType")
                web_response["points"][i]["actions"][j]["action_content"] = web_response["points"][i]["actions"][j].pop("actionContent")

        set_message_fields(response.task, web_response) 
        response.success = True
        response.code = code
        response.message = message
        return response
    
    def _get_map_list_cb(self, request: GetMapList.Request, response: GetMapList.Response) -> GetMapList.Response:
        """
        Gets a list of available maps stored on the Navis. 
        
        Args:
            request (GetMapList.Request): The service request, includes page number. 
            response (GetMapList.Response): The service response.

        Returns:
            GetMapList.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        payload = {
            "page": request.page,
            "limit": request.limit,
            "sortType": request.sort_type,
            "reverse": request.reverse
        }
        try:
            r = requests.get(self._url + "/map_list", headers=headers, params=payload, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            web_response["data"]["map_list"] = web_response["data"].pop("mapList")
            for i in range(len(web_response["data"]["map_list"])):
                web_response["data"]["map_list"][i]["map_time"] = web_response["data"]["map_list"][i].pop("mapTime")
                web_response["data"]["map_list"][i]["map_name"] = web_response["data"]["map_list"][i].pop("mapName")
            set_message_fields(response, web_response)
            # self.logger.info(f"Succesfully got map list")
        except Exception as e:
            response.success = False
            self.logger.error(f"Failed to get map list: {e}")
            return
        
        return response
    
    def _get_map_info_cb(self, request: GetMapInfo.Request, response: GetMapInfo.Response) -> GetMapInfo.Response:
        """
        Gets information about a specific map, such as size, origin, and tasks
        
        Args:
            request (GetMapInfo.Request): The service request, includes map name.
            response (GetMapInfo.Response): The service response.

        Returns:
            GetMapInfo.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        payload = {
            "mapName": request.map_name
        }
        try:
            r = requests.get(self._url + "/map_info", headers=headers, params=payload, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            web_response["data"]["map_info"] = web_response["data"].pop("mapInfo")
            web_response["data"]["map_info"]["grid_height"] = web_response["data"]["map_info"].pop("gridHeight")
            web_response["data"]["map_info"]["grid_width"] = web_response["data"]["map_info"].pop("gridWidth")
            web_response["data"]["map_info"]["origin_x"] = web_response["data"]["map_info"].pop("originX")
            web_response["data"]["map_info"]["origin_y"] = web_response["data"]["map_info"].pop("originY")
            set_message_fields(response, web_response)
            # self.logger.info(f"Succesfully got map info for {request.map_name}")
        except Exception as e:
            response.success = False
            self.logger.error(f"Failed to get map info: {e}")
            return
        
        return response

    def _set_task_info_cb(self, request: SetTaskInfo.Request, response: SetTaskInfo.Response) -> SetTaskInfo.Response:
        return self._set_task_info(request, response)

    def _set_task_info(self, request: SetTaskInfo.Request, response: SetTaskInfo.Response) -> SetTaskInfo.Response:
        """
        Sets info for a specific task, such as points. 
        
        Args:
            request (SetTaskInfo.Request): The service request. 
            response (SetTaskInfo.Response): The service response.

        Returns:
            SetTaskInfo.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        request_dict = dict(message_to_ordereddict(request))

        request_dict["mapName"] = request_dict.pop("map_name")
        request_dict["taskName"] = request_dict.pop("task_name")
        request_dict["editedName"] = request_dict.pop("edited_name")
        request_dict["rotateSpeed"] = request_dict.pop("rotate_speed")
        request_dict["personName"] = request_dict.pop("person_name")
        for i in range(len(request_dict["points"])):
            request_dict["points"][i]["isNew"] = request_dict["points"][i].pop("is_new")
            request_dict["points"][i]["pointType"] = request_dict["points"][i].pop("point_type")
            request_dict["points"][i]["pointName"] = request_dict["points"][i].pop("point_name")
            for j in range(len(request_dict["points"][i]["actions"])):
                request_dict["points"][i]["actions"][j]["deviceId"] = request_dict["points"][i]["actions"][j].pop("device_id")
                request_dict["points"][i]["actions"][j]["actionType"] = request_dict["points"][i]["actions"][j].pop("action_type")
                request_dict["points"][i]["actions"][j]["actionContent"] = request_dict["points"][i]["actions"][j].pop("action_content")
        # self.logger.info(json.dumps(request_dict, indent=4))
        try:
            r = requests.post(self._url + "/set_task", headers=headers, json=request_dict, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            set_message_fields(response, web_response)
            # self.logger.info(f"Sent set task request")
        except Exception as e:
            response.success = False
            # self.logger.error(f"Failed to set task info: {e}")
            return
        
        return response
    
    def _start_rt_task(self, request: ExecuteRTTask.Request, response: ExecuteRTTask.Response) -> ExecuteRTTask.Response:
        """
        Starts a realtime navigation task, for example when clicking the displayed map. 
        
        Args:
            request (ExecuteRTTask.Request): The service request. 
            response (ExecuteRTTask.Response): The service response.

        Returns:
            ExecuteRTTask.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        request_dict = dict(message_to_ordereddict(request))
        request_dict["loopTime"] = request_dict.pop("loop_time")
        for i in range(len(request_dict["points"])):
            request_dict["points"][i].pop("id")
            request_dict["points"][i].pop("point_name")
            request_dict["points"][i].pop("point_type")
            request_dict["points"][i].pop("actions")
            request_dict["points"][i]["isNew"] = request_dict["points"][i].pop("is_new")
        
        try:
            r = requests.post(self._url + "/realtime_task", headers=headers, json=request_dict, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            set_message_fields(response, web_response)
            if(response.success):
                end_point_dict = request_dict["points"][-1]["position"]
                set_message_fields(self._end_point, end_point_dict)
                self._end_point_publisher.publish(self._end_point)
        except Exception as e:
            response.success = False
            response.msg = f"Failed to send execute rt task request: {e}"
        return response
    
    def _execute_rt_task_cb(self, request: ExecuteRTTask.Request, response: ExecuteRTTask.Response) -> ExecuteRTTask.Response:
        return (self._start_rt_task(request, response))
    
    def _execute_list_task_cb(self, request: ExecuteListTask.Request, response: ExecuteListTask.Response) -> ExecuteListTask.Response:
        """
        Starts a navigation task already saved on the Navis. 
        
        Args:
            request (ExecuteListTask.Request): The service request. 
            response (ExecuteListTask.Response): The service response.

        Returns:
            ExecuteListTask.Response: The service response.
        """
        if(not self._logged_in):
            self.logger.error("Not logged in")
            return
        
        headers = {
            "Authorization": self._authorization_token
        }
        request_dict = dict(message_to_ordereddict(request))
        request_dict["mapName"] = request_dict.pop("map_name")
        request_dict["taskName"] = request_dict.pop("task_name")
        request_dict["loopTime"] = request_dict.pop("loop_time")
        try:
            r = requests.post(self._url + "/run_list_task", headers=headers, json=request_dict, timeout=5.0)
            r.raise_for_status()
            web_response = r.json()
            set_message_fields(response, web_response)
        except Exception as e:
            response.success = False
            response.msg = f"Failed to send execute list task request: {e}"
        
        return response

        

def main(args=None):
    rclpy.init(args=args)

    node = NavisHTTPClient()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
