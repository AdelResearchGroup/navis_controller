"""ROS 2 bridge for the AgileX Navis HTTP API (user API v0.0.12).

Every Navis HTTP endpoint is exposed as a ROS 2 service using the interfaces from ``navis_msgs``
(see the navis_msgs README for the endpoint <-> interface table). The login token is handled
internally: the node logs in on startup, refreshes the token before it expires (24 h), and logs in
again if a request is rejected as unauthorized.

Services (all relative to the node namespace):
    navis/login                  navis_msgs/srv/Login
    navis/slam_operate           navis_msgs/srv/NavInput            POST /slam_operate
    navis/get_map_list           navis_msgs/srv/GetMapList          GET  /map_list
    navis/get_map_info           navis_msgs/srv/GetMapInfo          GET  /map_info
    navis/get_map_image          navis_msgs/srv/GetMapImage         GET  /downloadpng
    navis/rename_map             navis_msgs/srv/RenameMap           POST /rename_map
    navis/delete_map             navis_msgs/srv/DeleteMap           DELETE /delete_map
    navis/export_map             navis_msgs/srv/ExportMap           GET  /download_map_zip
    navis/export_maps            navis_msgs/srv/ExportMaps          POST /download_map_list
    navis/import_map             navis_msgs/srv/ImportMap           POST /upload_map_zip
    navis/pixel_to_map           navis_msgs/srv/PixelToMap          GET  /get_map/positon
    navis/map_to_pixel           navis_msgs/srv/MapToPixel          GET  /get_world/positon
    navis/get_task               navis_msgs/srv/GetTask             GET  /get_tasks
    navis/set_task               navis_msgs/srv/SetTask             POST /set_task
    navis/delete_task            navis_msgs/srv/DeleteTask          DELETE /delete_task
    navis/run_realtime_task      navis_msgs/srv/RunRealtimeTask     POST /realtime_task
    navis/run_task               navis_msgs/srv/RunTask             POST /run_list_task
    navis/get_software_versions  navis_msgs/srv/GetVersions         GET  /version_info
    navis/get_device_versions    navis_msgs/srv/GetVersions         GET  /dev_version
    navis/get_camera_list        navis_msgs/srv/GetCameraList       GET  /get_camera_list
    navis/add_camera             navis_msgs/srv/AddCamera           POST /add_camera
    navis/set_camera             navis_msgs/srv/SetCamera           POST /set_camera
    navis/delete_camera          navis_msgs/srv/DeleteCamera        DELETE /delete_camera
"""

import array
import math
import os
import re
import threading
import time
from typing import Any, Callable, Optional

import requests
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from navis_msgs.msg import CameraStream, MapSummary, Task, TaskAction, TaskPoint, VersionInfo
from navis_msgs.srv import (
    AddCamera,
    DeleteCamera,
    DeleteMap,
    DeleteTask,
    ExportMap,
    ExportMaps,
    GetCameraList,
    GetMapImage,
    GetMapInfo,
    GetMapList,
    GetTask,
    GetVersions,
    ImportMap,
    Login,
    MapToPixel,
    NavInput,
    PixelToMap,
    RenameMap,
    RunRealtimeTask,
    RunTask,
    SetCamera,
    SetTask,
)

DEFAULT_PASSWORD = "NTdhMTE5NGNiMDczY2U4YjNiYjM2NWU0YjgwNWE5YWU="  # from the Navis user manual
TOKEN_LIFETIME_S = 23 * 3600.0  # Navis tokens expire after 24 h; refresh a bit earlier


class NavisError(Exception):
    """Navis could not be reached or returned an error."""


# ----------------------------------------------------------------------------------------------
# Small conversion helpers
# ----------------------------------------------------------------------------------------------

def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_int(value: Any, default: int = 0) -> int:
    """Lenient int parse. Navis sometimes sends ints as strings like "point-1"."""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (int, float)):
        return int(value)
    if isinstance(value, str):
        # A minus sign only counts at the start ("-3"); in "point-1" the hyphen is a separator.
        match = re.fullmatch(r"\s*(-?\d+)\s*", value) or re.search(r"\d+", value)
        if match:
            return int(match.group())
    return default


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _set_yaw(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def _navis_ok(body: dict) -> bool:
    """Navis standard response: code 0 means success (prioritised over the success flag)."""
    if "code" in body and body["code"] not in (None, ""):
        return str(body["code"]) == "0"
    return bool(body.get("success"))


def _navis_error_text(path: str, body: dict) -> str:
    return f"{path}: {body.get('msg') or body.get('data') or 'request failed'} (code {body.get('code')})"


# ----------------------------------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------------------------------

class NavisHTTPClient(Node):
    def __init__(self, **kwargs):
        super().__init__("navis_http", **kwargs)

        self.declare_parameter("base_url", "http://192.168.6.102:8880")
        self.declare_parameter("username", "agilex")
        self.declare_parameter("password", DEFAULT_PASSWORD)
        self.declare_parameter("timeout", 10.0)           # s, regular requests
        self.declare_parameter("download_timeout", 120.0) # s, map image / zip transfers
        self.declare_parameter("download_dir", "~/navis_maps")  # default target for exports
        self.declare_parameter("frame_id", "map")
        # Units of the task point "theta" field. The manual's set_task / realtime_task examples
        # use degrees (78.24, 169.32, 80), so degrees is the default. Flip if your robot disagrees.
        self.declare_parameter("task_theta_in_degrees", True)

        self._base_url = self.get_parameter("base_url").value.rstrip("/")
        self._username = self.get_parameter("username").value
        self._password = self.get_parameter("password").value
        self._timeout = float(self.get_parameter("timeout").value)
        self._download_timeout = float(self.get_parameter("download_timeout").value)
        self._download_dir = os.path.expanduser(self.get_parameter("download_dir").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._theta_deg = bool(self.get_parameter("task_theta_in_degrees").value)

        self._session = requests.Session()
        self._token: Optional[str] = None
        self._token_time = 0.0
        self._token_lock = threading.Lock()

        self._cb_group = ReentrantCallbackGroup()

        services = [
            (Login, "navis/login", self._login_cb),
            (NavInput, "navis/slam_operate", self._slam_operate_cb),
            (GetMapList, "navis/get_map_list", self._get_map_list_cb),
            (GetMapInfo, "navis/get_map_info", self._get_map_info_cb),
            (GetMapImage, "navis/get_map_image", self._get_map_image_cb),
            (RenameMap, "navis/rename_map", self._rename_map_cb),
            (DeleteMap, "navis/delete_map", self._delete_map_cb),
            (ExportMap, "navis/export_map", self._export_map_cb),
            (ExportMaps, "navis/export_maps", self._export_maps_cb),
            (ImportMap, "navis/import_map", self._import_map_cb),
            (PixelToMap, "navis/pixel_to_map", self._pixel_to_map_cb),
            (MapToPixel, "navis/map_to_pixel", self._map_to_pixel_cb),
            (GetTask, "navis/get_task", self._get_task_cb),
            (SetTask, "navis/set_task", self._set_task_cb),
            (DeleteTask, "navis/delete_task", self._delete_task_cb),
            (RunRealtimeTask, "navis/run_realtime_task", self._run_realtime_task_cb),
            (RunTask, "navis/run_task", self._run_task_cb),
            (GetVersions, "navis/get_software_versions", self._get_software_versions_cb),
            (GetVersions, "navis/get_device_versions", self._get_device_versions_cb),
            (GetCameraList, "navis/get_camera_list", self._get_camera_list_cb),
            (AddCamera, "navis/add_camera", self._add_camera_cb),
            (SetCamera, "navis/set_camera", self._set_camera_cb),
            (DeleteCamera, "navis/delete_camera", self._delete_camera_cb),
        ]
        for srv_type, name, handler in services:
            self.create_service(srv_type, name, self._wrap(handler), callback_group=self._cb_group)

        # Try to log in right away so problems show up at startup. Failure is not fatal:
        # every request logs in lazily if there is no valid token.
        try:
            self._login(self._username, self._password)
            self.get_logger().info(f"Logged in to Navis at {self._base_url}")
        except NavisError as e:
            self.get_logger().warning(f"Initial Navis login failed, will retry on first request: {e}")

    # ------------------------------------------------------------------------------------------
    # HTTP plumbing
    # ------------------------------------------------------------------------------------------

    def _login(self, username: str, password: str) -> None:
        path = "/user/passport/login"
        try:
            r = self._session.post(
                self._base_url + path,
                json={"username": username, "password": password},
                timeout=self._timeout,
            )
            r.raise_for_status()
            body = r.json()
        except (requests.RequestException, ValueError) as e:
            raise NavisError(f"login to {self._base_url} failed: {e}") from e
        if not _navis_ok(body) or not body.get("data"):
            raise NavisError(_navis_error_text(path, body))
        with self._token_lock:
            self._token = str(body["data"])
            self._token_time = time.monotonic()

    def _get_token(self) -> str:
        with self._token_lock:
            token = self._token
            fresh = token is not None and time.monotonic() - self._token_time < TOKEN_LIFETIME_S
        if not fresh:
            self._login(self._username, self._password)
            with self._token_lock:
                token = self._token
        return token

    def _invalidate_token(self) -> None:
        with self._token_lock:
            self._token = None

    def _request(
        self,
        method: str,
        path: str,
        *,
        params: Optional[dict] = None,
        json_body: Any = None,
        files: Optional[dict] = None,
        raw: bool = False,
        check: bool = True,
        timeout: Optional[float] = None,
    ):
        """Send an authorised request.

        Returns the parsed JSON body, or the ``requests.Response`` when ``raw`` is set (file
        downloads). Raises NavisError on transport errors and, if ``check`` is set, when Navis
        reports failure. Retries once with a fresh token on HTTP 401/403.
        """
        url = self._base_url + path
        timeout = self._timeout if timeout is None else timeout
        for attempt in range(2):
            headers = {"Authorization": self._get_token()}
            try:
                r = self._session.request(
                    method, url, params=params, json=json_body, files=files,
                    headers=headers, timeout=timeout,
                )
            except requests.RequestException as e:
                raise NavisError(f"{method} {path} failed: {e}") from e

            if r.status_code in (401, 403) and attempt == 0:
                self._invalidate_token()
                continue
            if r.status_code >= 400:
                raise NavisError(f"{method} {path}: HTTP {r.status_code} {r.text[:200]}")

            is_json = "json" in r.headers.get("Content-Type", "")
            if raw:
                if is_json:  # file endpoints answer with a JSON error body on failure
                    try:
                        body = r.json()
                    except ValueError:
                        body = {}
                    raise NavisError(_navis_error_text(path, body))
                return r

            try:
                body = r.json()
            except ValueError as e:
                raise NavisError(f"{method} {path}: response is not JSON: {r.text[:200]}") from e
            if check and not _navis_ok(body):
                raise NavisError(_navis_error_text(path, body))
            return body
        raise NavisError(f"{method} {path}: unauthorized")

    def _wrap(self, handler: Callable) -> Callable:
        """Turn a handler that raises NavisError into a service callback that never throws."""
        def callback(request, response):
            try:
                handler(request, response)
                if not response.message:
                    response.message = "success"
            except NavisError as e:
                response.success = False
                response.message = str(e)
                self.get_logger().warning(str(e))
            except Exception as e:  # noqa: BLE001 - never let a service callback crash the node
                response.success = False
                response.message = f"{type(e).__name__}: {e}"
                self.get_logger().error(f"Unexpected error in {handler.__name__}: {response.message}")
            return response
        return callback

    @staticmethod
    def _message(body: dict) -> str:
        data = body.get("data")
        if isinstance(data, str) and data:
            return data
        return str(body.get("msg") or "success")

    def _write_file(self, content: bytes, path: str) -> str:
        path = os.path.abspath(os.path.expanduser(path))
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "wb") as f:
            f.write(content)
        return path

    # ------------------------------------------------------------------------------------------
    # Theta / pose conversion
    # ------------------------------------------------------------------------------------------

    def _theta_to_yaw(self, theta: Any) -> float:
        theta = _to_float(theta)
        return math.radians(theta) if self._theta_deg else theta

    def _yaw_to_theta(self, yaw: float) -> float:
        return math.degrees(yaw) if self._theta_deg else yaw

    # ------------------------------------------------------------------------------------------
    # Task (de)serialisation
    # ------------------------------------------------------------------------------------------

    def _point_from_navis(self, d: dict) -> TaskPoint:
        p = TaskPoint()
        pos = d.get("position") or {}
        p.pose.position.x = _to_float(pos.get("x"))
        p.pose.position.y = _to_float(pos.get("y"))
        _set_yaw(p.pose.orientation, self._theta_to_yaw(pos.get("theta")))
        p.point_name = str(d.get("pointName") or "")
        p.point_type = str(d.get("pointType") or "")
        p.index = _to_int(d.get("index"))
        p.is_new = bool(d.get("isNew", False))
        p.control_point.x = _to_float(d.get("cpx"))
        p.control_point.y = _to_float(d.get("cpy"))
        for a in d.get("actions") or []:
            action = TaskAction()
            action.action_type = str(a.get("actionType") or "")
            action.action_name = str(a.get("actionName") or "")
            action.action_order = _to_int(a.get("actionOrder"))
            action.duration = _to_float(a.get("actionContent"))
            p.actions.append(action)
        return p

    def _point_to_navis(self, p: TaskPoint, full: bool = True) -> dict:
        d = {
            "position": {
                "x": p.pose.position.x,
                "y": p.pose.position.y,
                "theta": self._yaw_to_theta(_yaw_from_quaternion(p.pose.orientation)),
            },
            "isNew": p.is_new,
            "cpx": p.control_point.x,
            "cpy": p.control_point.y,
        }
        if full:
            d.update({
                "pointType": p.point_type or TaskPoint.POINT_TYPE_NAVIGATION,
                "pointName": p.point_name,
                "index": p.index,
                "actions": [
                    {
                        "actionContent": f"{a.duration:g}",
                        "actionType": a.action_type,
                        "actionOrder": a.action_order,
                        "actionName": a.action_name,
                    }
                    for a in p.actions
                ],
            })
        return d

    def _task_from_navis(self, d: dict) -> Task:
        t = Task()
        t.task_name = str(d.get("taskName") or "")
        t.map_name = str(d.get("mapName") or "")
        t.edited_name = str(d.get("editedName") or "")
        t.mode = str(d.get("mode") or "")
        t.evadible = _to_int(d.get("evadible"), Task.OBSTACLE_AVOID)
        t.speed = _to_float(d.get("speed"))
        t.grid_item_idx = _to_int(d.get("gridItemIdx"))
        t.remark = str(d.get("remark") or "")
        t.person_name = str(d.get("personName") or "")
        t.points = [self._point_from_navis(p) for p in d.get("points") or []]
        return t

    def _task_to_navis(self, t: Task) -> dict:
        return {
            "taskName": t.task_name,
            "gridItemIdx": t.grid_item_idx,
            "points": [self._point_to_navis(p) for p in t.points],
            "mode": t.mode or Task.MODE_POINT,
            "evadible": t.evadible or Task.OBSTACLE_AVOID,
            "mapName": t.map_name,
            "speed": t.speed,
            "editedName": t.edited_name or t.task_name,
            "remark": t.remark,
            "personName": t.person_name,
        }

    # ------------------------------------------------------------------------------------------
    # Auth / SLAM operation
    # ------------------------------------------------------------------------------------------

    def _login_cb(self, request: Login.Request, response: Login.Response):
        username = request.username or self._username
        password = request.password or self._password
        self._login(username, password)
        response.success = True
        response.message = f"Logged in to {self._base_url} as {username}"

    def _slam_operate_cb(self, request: NavInput.Request, response: NavInput.Response):
        payload = {
            "opType": request.op_type,
            "idType": request.id_type,
            "fileName": request.file_name,
            "mapOperate": request.map_operate,
        }
        body = self._request("POST", "/slam_operate", json_body=payload, check=False)
        response.result = True
        response.success = _navis_ok(body)
        response.message = self._message(body) if response.success else _navis_error_text("/slam_operate", body)

    # ------------------------------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------------------------------

    def _get_map_list_cb(self, request: GetMapList.Request, response: GetMapList.Response):
        sort_type = {GetMapList.Request.SORT_BY_NAME: "name", GetMapList.Request.SORT_BY_TIME: "time"}.get(
            request.sort_type, ""
        )
        params = {
            "page": request.page if request.page > 0 else 1,
            "limit": request.limit if request.limit != 0 else -1,
            "sortType": sort_type,
            "reverse": ("true" if request.reverse else "false") if sort_type else "",
        }
        body = self._request("GET", "/map_list", params=params)
        data = body.get("data") or {}
        for m in data.get("mapList") or []:
            summary = MapSummary()
            summary.id = _to_int(m.get("id"))
            summary.map_name = str(m.get("mapName") or "")
            summary.map_time = str(m.get("mapTime") or "")
            summary.tasks = [str(t) for t in m.get("tasks") or []]
            response.maps.append(summary)
        response.total = _to_int(data.get("total"), len(response.maps))
        response.success = True
        response.message = f"{len(response.maps)} of {response.total} maps"

    def _get_map_info_cb(self, request: GetMapInfo.Request, response: GetMapInfo.Response):
        body = self._request("GET", "/map_info", params={"mapName": request.map_name})
        data = body.get("data") or {}
        info = data.get("mapInfo") or {}
        response.info.resolution = _to_float(info.get("resolution"))
        response.info.width = _to_int(info.get("gridWidth"))
        response.info.height = _to_int(info.get("gridHeight"))
        response.info.origin.position.x = _to_float(info.get("originX"))
        response.info.origin.position.y = _to_float(info.get("originY"))
        response.info.origin.orientation.w = 1.0
        response.tasks = [str(t) for t in data.get("tasks") or []]
        response.success = True

    def _get_map_image_cb(self, request: GetMapImage.Request, response: GetMapImage.Response):
        r = self._request(
            "GET", "/downloadpng", params={"mapName": request.map_name},
            raw=True, timeout=self._download_timeout,
        )
        content = r.content
        response.image.header.stamp = self.get_clock().now().to_msg()
        response.image.header.frame_id = self._frame_id
        response.image.format = "png"
        response.image.data = array.array("B", content)
        if request.save_path:
            response.saved_path = self._write_file(content, request.save_path)
        response.success = True
        response.message = f"Got map image {request.map_name} ({len(content)} bytes)"

    def _rename_map_cb(self, request: RenameMap.Request, response: RenameMap.Response):
        body = self._request(
            "POST", "/rename_map",
            json_body={"mapName": request.map_name, "newMapName": request.new_map_name},
        )
        response.success = True
        response.message = self._message(body)

    def _delete_map_cb(self, request: DeleteMap.Request, response: DeleteMap.Response):
        body = self._request("DELETE", "/delete_map", params={"mapName": request.map_name})
        response.success = True
        response.message = self._message(body)

    def _export_map_cb(self, request: ExportMap.Request, response: ExportMap.Response):
        r = self._request(
            "GET", "/download_map_zip", params={"id": request.map_name},
            raw=True, timeout=self._download_timeout,
        )
        target = request.save_path or os.path.join(self._download_dir, f"{request.map_name}.zip")
        response.saved_path = self._write_file(r.content, target)
        response.success = True
        response.message = f"Exported {request.map_name} to {response.saved_path}"

    def _export_maps_cb(self, request: ExportMaps.Request, response: ExportMaps.Response):
        ids = [int(i) for i in request.map_ids]
        r = self._request(
            "POST", "/download_map_list", json_body=ids,
            raw=True, timeout=self._download_timeout,
        )
        target = request.save_path or os.path.join(self._download_dir, f"navis_maps_{int(time.time())}.zip")
        response.saved_path = self._write_file(r.content, target)
        response.success = True
        response.message = f"Exported {len(ids)} maps to {response.saved_path}"

    def _import_map_cb(self, request: ImportMap.Request, response: ImportMap.Response):
        path = os.path.expanduser(request.file_path)
        try:
            with open(path, "rb") as f:
                content = f.read()
        except OSError as e:
            raise NavisError(f"cannot read {path}: {e}") from e
        files = {"file": (os.path.basename(path), content, "application/zip")}
        body = self._request("POST", "/upload_map_zip", files=files, timeout=self._download_timeout)
        response.success = True
        response.message = self._message(body)

    def _pixel_to_map_cb(self, request: PixelToMap.Request, response: PixelToMap.Response):
        body = self._request(
            "GET", "/get_map/positon",  # sic, Navis spelling
            params={"mapName": request.map_name, "pngX": request.png_x, "pngY": request.png_y},
        )
        data = body.get("data") or {}
        response.position.x = _to_float(data.get("map_pose_x"))
        response.position.y = _to_float(data.get("map_pose_y"))
        response.success = True

    def _map_to_pixel_cb(self, request: MapToPixel.Request, response: MapToPixel.Response):
        body = self._request(
            "GET", "/get_world/positon",  # sic, Navis spelling
            params={"mapName": request.map_name, "poseX": request.position.x, "poseY": request.position.y},
        )
        data = body.get("data") or {}
        response.png_x = _to_float(data.get("png_pose_x"))
        response.png_y = _to_float(data.get("png_pose_y"))
        response.success = True

    # ------------------------------------------------------------------------------------------
    # Tasks
    # ------------------------------------------------------------------------------------------

    def _get_task_cb(self, request: GetTask.Request, response: GetTask.Response):
        body = self._request("GET", "/get_tasks", params={"mapName": request.map_name, "taskName": request.task_name})
        response.task = self._task_from_navis(body.get("data") or {})
        response.success = True

    def _set_task_cb(self, request: SetTask.Request, response: SetTask.Response):
        body = self._request("POST", "/set_task", json_body=self._task_to_navis(request.task))
        response.success = True
        response.message = self._message(body)

    def _delete_task_cb(self, request: DeleteTask.Request, response: DeleteTask.Response):
        body = self._request(
            "DELETE", "/delete_task", params={"mapName": request.map_name, "taskName": request.task_name}
        )
        response.success = True
        response.message = self._message(body)

    def _run_realtime_task_cb(self, request: RunRealtimeTask.Request, response: RunRealtimeTask.Response):
        if not request.points:
            raise NavisError("real-time task needs at least one point")
        payload = {
            "loopTime": request.loop_time,
            "points": [self._point_to_navis(p, full=False) for p in request.points],
            "mode": request.mode or RunRealtimeTask.Request.MODE_POINT,
        }
        body = self._request("POST", "/realtime_task", json_body=payload)
        response.success = True
        response.message = self._message(body)

    def _run_task_cb(self, request: RunTask.Request, response: RunTask.Response):
        payload = {"mapName": request.map_name, "taskName": request.task_name, "loopTime": request.loop_time}
        body = self._request("POST", "/run_list_task", json_body=payload)
        response.success = True
        response.message = self._message(body)

    # ------------------------------------------------------------------------------------------
    # Versions
    # ------------------------------------------------------------------------------------------

    def _get_software_versions_cb(self, request: GetVersions.Request, response: GetVersions.Response):
        body = self._request("GET", "/version_info")
        for v in body.get("data") or []:
            info = VersionInfo()
            info.id = str(v.get("id") or "")
            info.version = str(v.get("version") or "")
            response.versions.append(info)
        response.success = True

    def _get_device_versions_cb(self, request: GetVersions.Request, response: GetVersions.Response):
        body = self._request("GET", "/dev_version")
        for v in body.get("data") or []:
            info = VersionInfo()
            info.id = str(v.get("id") or "")
            info.type = str(v.get("type") or "")
            info.software_version = str(v.get("s_version") or "")
            info.hardware_version = str(v.get("h_version") or "")
            response.versions.append(info)
        response.success = True

    # ------------------------------------------------------------------------------------------
    # Cameras
    # ------------------------------------------------------------------------------------------

    @staticmethod
    def _camera_to_navis(c: CameraStream) -> dict:
        return {
            "cameraId": c.camera_id,
            "cameraName": c.camera_name,
            "onDemand": c.on_demand,
            "disableAudio": c.disable_audio,
            "rtspUrl": c.rtsp_url,
        }

    def _get_camera_list_cb(self, request: GetCameraList.Request, response: GetCameraList.Response):
        body = self._request("GET", "/get_camera_list")
        for s in (body.get("data") or {}).get("streams") or []:
            c = CameraStream()
            c.camera_id = str(s.get("cameraId") or "")
            c.camera_name = str(s.get("cameraName") or "")
            c.on_demand = bool(s.get("onDemand", False))
            c.disable_audio = bool(s.get("disableAudio", True))
            c.rtsp_url = str(s.get("rtspUrl") or "")
            response.streams.append(c)
        response.success = True

    def _add_camera_cb(self, request: AddCamera.Request, response: AddCamera.Response):
        body = self._request("POST", "/add_camera", json_body=self._camera_to_navis(request.camera))
        response.success = True
        response.message = self._message(body)

    def _set_camera_cb(self, request: SetCamera.Request, response: SetCamera.Response):
        body = self._request("POST", "/set_camera", json_body=self._camera_to_navis(request.camera))
        response.success = True
        response.message = self._message(body)

    def _delete_camera_cb(self, request: DeleteCamera.Request, response: DeleteCamera.Response):
        body = self._request("DELETE", "/delete_camera", params={"cameraId": request.camera_id})
        response.success = True
        response.message = self._message(body)


def main(args=None):
    rclpy.init(args=args)
    node = NavisHTTPClient()
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
