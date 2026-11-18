"""A small stand-in for the Navis HTTP and websocket APIs, for testing without a robot.

It answers with the example data from the Navis user manual and records everything it receives,
so tests can check what the bridge nodes sent. It does not simulate the robot: nothing moves, and
navigation "starts" instantly.

Use from tests:

    fake = FakeNavis()          # port 0 = pick free ports
    fake.start()
    ... fake.http_port, fake.ws_port, fake.http_requests, fake.ws_received ...
    fake.stop()

Or run standalone and point the launch file at it:

    python3 test/fake_navis_server.py --http-port 8880 --ws-port 9090 -v
    ros2 launch navis_controller navis.launch.py robot_ip:=127.0.0.1
"""

import argparse
import asyncio
import json
import threading
import time
from collections.abc import Callable
from typing import Optional
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import websockets

USERNAME = "agilex"
PASSWORD = "NTdhMTE5NGNiMDczY2U4YjNiYjM2NWU0YjgwNWE5YWU="
FAKE_PNG = b"\x89PNG\r\n\x1a\n" + b"fake map image"

MAP_INFO = {"gridHeight": 1000, "gridWidth": 1200, "originX": -10.0, "originY": -20.0, "resolution": 0.05}

# Task "p1" on map "office", from the manual's set_list_task example (theta in degrees,
# index sent as "point-1", speed as an int).
TASK_P1 = {
    "taskName": "p1",
    "gridItemIdx": 0,
    "mode": "point",
    "speed": 1,
    "evadible": 1,
    "mapName": "office",
    "editedName": "p1",
    "remark": "",
    "personName": "",
    "points": [
        {
            "isNew": True, "index": "point-1", "pointType": "navigation", "pointName": "",
            "actions": [{"actionContent": "5", "actionType": "stop", "actionOrder": 0, "actionName": "wait"}],
            "position": {"x": -3.35, "y": 37.48, "theta": 78.24}, "cpx": 0, "cpy": 0,
        },
        {
            "isNew": True, "index": "point-2", "pointType": "navigation", "pointName": "",
            "actions": [],
            "position": {"x": -2.45, "y": 34.53, "theta": 169.32}, "cpx": 0, "cpy": 0,
        },
    ],
}

# Messages streamed to subscribers, in the ROS 1 JSON form Navis uses.
TOPIC_MESSAGES = {
    "/slam_status": {
        "nav": {"state": True, "name": "office", "task": "p1"},
        "mapping_3d": {"state": False, "name": ""},
        "mapping_2d": {"state": False, "name": ""},
        "record": {"state": False, "name": ""},
    },
    "/odom": {
        "header": {"stamp": {"secs": 1681805920, "nsecs": 64378373}, "frame_id": "/map", "seq": 1098806},
        "child_frame_id": "base_link",
        "pose": {
            "pose": {"position": {"x": 1.5, "y": -2.0, "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
            "covariance": [0.0] * 36,
        },
        "twist": {
            "twist": {"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
            "covariance": [0.0] * 36,
        },
    },
    "/interface_management/BMS_status": {
        "batteryCurrent": -1.5, "SOH": 98, "SOC": 76, "batteryTemperature": 25.0, "batteryVoltage": 26.4,
    },
    "/run_management/global_status": {"map_name": "office", "status": 1, "text": ""},
}


def _standard(data="", ok=True, msg=None) -> dict:
    return {"success": ok, "code": 0 if ok else -1, "msg": msg or ("success" if ok else "failure"), "data": data}


class FakeNavis:
    def __init__(self, http_port: int = 0, ws_port: int = 0, publish_period: float = 0.1, verbose: bool = False):
        self._http_port_req = http_port
        self._ws_port_req = ws_port
        self.publish_period = publish_period
        self.verbose = verbose

        # What the fake received (inspect these in tests)
        self.http_requests: list[tuple[str, str, dict, object]] = []  # (method, path, query, json body)
        self.ws_received: list[dict] = []
        self.logins = 0
        self.pings = 0
        self.connections = 0  # total websocket connections accepted so far

        # Knobs for tests
        self.echo_ids = True  # echo the call_service "id" in service_response
        self.service_delay: Callable[[str, dict], float] = lambda service, args: 0.0

        self.valid_tokens: set[str] = set()
        self.tasks = {("office", "p1"): json.loads(json.dumps(TASK_P1))}
        self.topic_messages = dict(TOPIC_MESSAGES)

        self._conns: list[dict] = []
        self._http: Optional[ThreadingHTTPServer] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_server = None
        self._ws_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        self._http = ThreadingHTTPServer(("127.0.0.1", self._http_port_req), _make_http_handler(self))
        threading.Thread(target=self._http.serve_forever, daemon=True).start()

        self._loop = asyncio.new_event_loop()
        ready = threading.Event()
        startup_error: list[BaseException] = []

        async def serve():
            # websockets >= 14 needs a running loop when serve() is constructed
            return await websockets.serve(self._ws_handler, "127.0.0.1", self._ws_port_req, max_size=None)

        def run():
            asyncio.set_event_loop(self._loop)
            try:
                self._ws_server = self._loop.run_until_complete(serve())
            except BaseException as exc:  # noqa: BLE001
                startup_error.append(exc)
                ready.set()
                return
            ready.set()
            self._loop.run_forever()

        self._ws_thread = threading.Thread(target=run, daemon=True)
        self._ws_thread.start()
        if not ready.wait(5.0):
            raise RuntimeError("fake websocket server did not start")
        if startup_error:
            raise RuntimeError("fake websocket server did not start") from startup_error[0]

    def stop(self) -> None:
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        if self._loop is not None and self._ws_server is not None:
            async def shutdown():
                self._ws_server.close()
                await self._ws_server.wait_closed()
            try:
                asyncio.run_coroutine_threadsafe(shutdown(), self._loop).result(timeout=5.0)
            except Exception:  # noqa: BLE001
                pass
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._ws_thread.join(timeout=5.0)

    @property
    def http_port(self) -> int:
        return self._http.server_address[1]

    @property
    def ws_port(self) -> int:
        return self._ws_server.sockets[0].getsockname()[1]

    # ------------------------------------------------------------------ test helpers

    @property
    def subscriptions(self) -> set:
        """Topics subscribed on any open connection."""
        subs = set()
        for conn in list(self._conns):
            subs |= conn["subs"]
        return subs

    def expire_tokens(self) -> None:
        """Make every issued token invalid, as if it had expired."""
        self.valid_tokens.clear()

    def drop_connections(self) -> None:
        """Close every websocket connection from the server side."""
        for conn in list(self._conns):
            asyncio.run_coroutine_threadsafe(conn["ws"].close(), self._loop)

    def published(self, topic: str) -> list[dict]:
        """Messages the client published to a Navis topic."""
        return [m.get("msg") for m in list(self.ws_received) if m.get("op") == "publish" and m.get("topic") == topic]

    def service_calls(self, service: str) -> list[dict]:
        return [m for m in list(self.ws_received) if m.get("op") == "call_service" and m.get("service") == service]

    def http_bodies(self, method: str, path: str) -> list:
        return [body for (m, p, _, body) in list(self.http_requests) if m == method and p == path]

    def _log(self, text: str) -> None:
        if self.verbose:
            print(text, flush=True)

    # ------------------------------------------------------------------ HTTP routes

    def handle_http(self, method: str, path: str, query: dict, body, headers) -> tuple[int, str, bytes]:
        if path.startswith("/apiUrl"):
            path = path[len("/apiUrl"):]
        self.http_requests.append((method, path, query, body))
        self._log(f"HTTP {method} {path} {query or ''} {json.dumps(body) if body is not None else ''}")

        if (method, path) == ("POST", "/user/passport/login"):
            body = body or {}
            if body.get("username") != USERNAME or body.get("password") != PASSWORD:
                return self._json(_standard(ok=False, msg="wrong username or password"))
            self.logins += 1
            token = f"token-{self.logins}"
            self.valid_tokens.add(token)
            return self._json(_standard(token))

        if headers.get("Authorization") not in self.valid_tokens:
            return self._json(_standard(ok=False, msg="unauthorized"), status=401)

        route = {
            ("POST", "/slam_operate"): self._slam_operate,
            ("GET", "/map_list"): self._map_list,
            ("GET", "/map_info"): self._map_info,
            ("GET", "/downloadpng"): self._download_png,
            ("GET", "/download_map_zip"): self._download_zip,
            ("POST", "/download_map_list"): self._download_zip,
            ("GET", "/get_map/positon"): self._pixel_to_map,
            ("GET", "/get_world/positon"): self._map_to_pixel,
            ("GET", "/get_tasks"): self._get_task,
            ("POST", "/set_task"): self._set_task,
            ("GET", "/version_info"): lambda q, b: self._json(_standard([
                {"id": "Navis Board", "version": "0.8.1.20230201_beta1"},
                {"id": "Navis Brain", "version": "0.0.16.0109_beta"},
                {"id": "Navis Bridge", "version": "0.0.2.0201_beta"},
            ])),
            ("GET", "/dev_version"): lambda q, b: self._json(_standard([
                {"id": "computer", "type": "X-7010", "s_version": "---", "h_version": "---"},
            ])),
            ("GET", "/get_camera_list"): lambda q, b: self._json(_standard({"total": 1, "streams": [
                {"cameraId": "cam1", "cameraName": "front", "onDemand": False, "disableAudio": True,
                 "rtspUrl": "rtsp://127.0.0.1/front"},
            ]})),
        }.get((method, path))
        if route is not None:
            return route(query, body)
        if path in {"/rename_map", "/delete_map", "/upload_map_zip", "/delete_task", "/realtime_task",
                    "/run_list_task", "/add_camera", "/set_camera", "/delete_camera"}:
            return self._json(_standard())
        return self._json(_standard(ok=False, msg=f"unknown endpoint {method} {path}"), status=404)

    @staticmethod
    def _json(obj, status: int = 200) -> tuple[int, str, bytes]:
        return status, "application/json;charset=UTF-8", json.dumps(obj).encode("utf-8")

    def _slam_operate(self, query, body):
        body = body or {}
        return self._json(_standard(f"{body.get('opType')} {body.get('idType')} success."))

    def _map_list(self, query, body):
        maps = [{"mapTime": "2022-12-15 15:00:53", "tasks": ["p1"], "mapName": "office", "id": 1671087653}]
        return self._json(_standard({"total": len(maps), "mapList": maps}))

    def _map_info(self, query, body):
        if query.get("mapName") != "office":
            return self._json(_standard(ok=False, msg="map not found"))
        return self._json(_standard({"mapInfo": MAP_INFO, "tasks": ["p1"]}))

    def _download_png(self, query, body):
        if query.get("mapName") != "office":
            return self._json(_standard(ok=False, msg="map not found"))
        return 200, "image/png", FAKE_PNG

    def _download_zip(self, query, body):
        return 200, "application/octet-stream", b"PK\x03\x04fake zip"

    def _pixel_to_map(self, query, body):
        png_x, png_y = float(query["pngX"]), float(query["pngY"])
        x = png_x * MAP_INFO["resolution"] + MAP_INFO["originX"]
        y = (MAP_INFO["gridHeight"] - png_y) * MAP_INFO["resolution"] + MAP_INFO["originY"]
        return self._json(_standard({"map_pose_x": x, "map_pose_y": y}))

    def _map_to_pixel(self, query, body):
        x, y = float(query["poseX"]), float(query["poseY"])
        png_x = (x - MAP_INFO["originX"]) / MAP_INFO["resolution"]
        png_y = MAP_INFO["gridHeight"] - (y - MAP_INFO["originY"]) / MAP_INFO["resolution"]
        return self._json(_standard({"png_pose_x": png_x, "png_pose_y": png_y}))

    def _get_task(self, query, body):
        task = self.tasks.get((query.get("mapName"), query.get("taskName")))
        if task is None:
            return self._json(_standard(ok=False, msg="task not found"))
        return self._json(_standard(task))

    def _set_task(self, query, body):
        body = body or {}
        self.tasks[(body.get("mapName"), body.get("editedName") or body.get("taskName"))] = body
        return self._json(_standard())

    # ------------------------------------------------------------------ websocket

    async def _ws_handler(self, ws, path=None):  # path: websockets < 10.1 passes it
        self.connections += 1
        conn = {"ws": ws, "subs": set()}
        self._conns.append(conn)
        self._log(f"WS connection #{self.connections}")
        publisher = asyncio.ensure_future(self._publisher(conn))
        try:
            async for raw in ws:
                msg = json.loads(raw if isinstance(raw, str) else raw.decode("utf-8"))
                self.ws_received.append(msg)
                op = msg.get("op")
                if op != "ping":
                    self._log(f"WS <- {json.dumps(msg)[:300]}")
                if op == "ping":
                    self.pings += 1
                    await ws.send(json.dumps({"op": "pong", "timeStamp": msg.get("timeStamp"),
                                              "serverTime": int(time.time() * 1000)}))
                elif op == "subscribe":
                    conn["subs"].add(msg.get("topic"))
                elif op == "unsubscribe":
                    conn["subs"].discard(msg.get("topic"))
                elif op == "call_service":
                    asyncio.ensure_future(self._answer(ws, msg))
        except websockets.ConnectionClosed:
            pass
        finally:
            publisher.cancel()
            self._conns.remove(conn)
            self._log("WS connection closed")

    async def _publisher(self, conn: dict) -> None:
        while True:
            for topic in list(conn["subs"]):
                if topic in self.topic_messages:
                    try:
                        await conn["ws"].send(json.dumps(
                            {"op": "publish", "topic": topic, "msg": self.topic_messages[topic]}
                        ))
                    except Exception:  # noqa: BLE001
                        return
            await asyncio.sleep(self.publish_period)

    async def _answer(self, ws, msg: dict) -> None:
        service, args = msg.get("service"), msg.get("args") or {}
        await asyncio.sleep(self.service_delay(service, args))
        result, values = True, {}
        if service == "/input/op":
            values = {"success": True, "message": f"{args.get('op_type')} {args.get('id_type')} success."}
        elif service == "/run_management/pause":
            values = {"success": True, "message": f"pause={args.get('pause')}"}
        elif service in ("/run_management/next_goal", "/run_management/switch_obstacle_avoid_model",
                         "/find_charger/set_charge"):
            values = {"success": True, "message": f"data={args.get('data')}"}
        elif service == "/find_charger/set_charge_point":
            values = {}
        elif service == "/find_charger/get_charge_point":
            values = {"get_charge_point": True,
                      "charge_point": {"x": -6.903390407562256, "y": 1.3843873739242554,
                                       "theta": 1.7618789011855123}}
        elif service == "/find_charger/set_charge_level":
            values = {"message": "success.", "min_charge_level": args.get("min_charge_level"),
                      "max_charge_level": args.get("max_charge_level"), "success": True}
        else:
            result, values = False, f"unknown service {service}"
        response = {"op": "service_response", "service": service, "result": result, "values": values}
        if self.echo_ids and "id" in msg:
            response["id"] = msg["id"]
        try:
            await ws.send(json.dumps(response))
        except Exception:  # noqa: BLE001
            pass


def _make_http_handler(fake: FakeNavis):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # silence default stderr logging
            pass

        def _handle(self, method: str) -> None:
            url = urlparse(self.path)
            query = {k: v[0] for k, v in parse_qs(url.query, keep_blank_values=True).items()}
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b""
            body = None
            if raw and "json" in (self.headers.get("Content-Type") or ""):
                body = json.loads(raw)
            status, content_type, data = fake.handle_http(method, url.path, query, body, self.headers)
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._handle("GET")

        def do_POST(self):
            self._handle("POST")

        def do_DELETE(self):
            self._handle("DELETE")

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Fake Navis HTTP + websocket server")
    parser.add_argument("--http-port", type=int, default=8880)
    parser.add_argument("--ws-port", type=int, default=9090)
    parser.add_argument("-v", "--verbose", action="store_true", help="print every request")
    args = parser.parse_args()

    fake = FakeNavis(args.http_port, args.ws_port, verbose=args.verbose)
    fake.start()
    print(f"Fake Navis: http://127.0.0.1:{fake.http_port}  ws://127.0.0.1:{fake.ws_port}  (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()


if __name__ == "__main__":
    main()
