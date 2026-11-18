# navis_controller

ROS 2 nodes that bridge the AgileX Navis navigation module (user API v0.0.12) to ROS 2.
Navis exposes two APIs, and there is one node for each:

| Node | Navis API | What it covers |
|---|---|---|
| `navis_http` | HTTP (`:8880` or `/apiUrl`) | maps, tasks, mapping/navigation start-stop, versions, cameras |
| `navis_websocket` | rosbridge-style websocket (`:9090`) | live robot/navigation data, motion, initial pose, task control, charging |

All interfaces come from [`navis_msgs`](../navis_msgs); its README has the full Navis ↔ ROS mapping.
Both nodes handle the Navis housekeeping internally: the HTTP login token (renewed before its 24 h
expiry) and the 1 Hz websocket heartbeat (Navis drops clients that stay silent for more than 3 s).

## Setup (pixi)

[`pixi.toml`](pixi.toml) sets up ROS 2 Jazzy (RoboStack) plus the Python dependencies on Linux and
Windows. `navis_msgs` is expected next to this repository (`../navis_msgs`) and is built
together with it:

```bash
pixi install
pixi run build        # colcon build of navis_msgs + navis_controller
pixi shell            # shell with ROS and install/ sourced
```

| Task | Does |
|---|---|
| `pixi run build` / `clean` | build the workspace / delete `build/`, `install/`, `log/` |
| `pixi run test` | build, then `colcon test` with results |
| `pixi run pytest` | run the tests directly (after a build) |
| `pixi run launch` | `ros2 launch navis_controller navis.launch.py` (append launch args) |
| `pixi run fake-navis` | fake Navis server on ports 8880/9090; launch with `robot_ip:=127.0.0.1` |
| `pixi run format` / `lint` | ruff |

On Windows, building `navis_msgs` needs Visual Studio 2022 (or Build Tools) with the C++ workload.
The `install/setup.*` activation script only exists after the first build, so pixi may warn
about it until then.

## Running

Start both nodes with the launch file:

```bash
ros2 launch navis_controller navis.launch.py
ros2 launch navis_controller navis.launch.py robot_ip:=192.168.6.102 namespace:=bunker
```

| Launch argument | Default | Meaning |
|---|---|---|
| `robot_ip` | `192.168.6.102` | IP address of the Navis module |
| `http_port` | `8880` | Navis HTTP port |
| `ws_port` | `9090` | Navis websocket port |
| `namespace` | (none) | namespace for both nodes |
| `params_file` | `config/navis.yaml` | parameters for both nodes |
| `log_level` | `info` | log level for both nodes |

All other node parameters (listed below) live in [config/navis.yaml](config/navis.yaml). To change
them, copy that file and pass it with `params_file:=/path/to/copy.yaml`. The robot address always
comes from the launch arguments, which override `base_url` / `ws_url` from the file.

To run a single node:

```bash
ros2 run navis_controller navis_http --ros-args -p base_url:=http://192.168.6.102:8880
ros2 run navis_controller navis_websocket --ros-args -p ws_url:=ws://192.168.6.102:9090
```

Dependencies: `navis_msgs`, `python3-requests`, `python3-websockets`. `python3-cbor2` and
`python3-numpy` are only needed when `use_cbor` is enabled.

## Typical workflow

1. Start navigation on a map:
   `ros2 service call /navis/nav_input navis_msgs/srv/NavInput "{file_name: my_map, op_type: start, id_type: navigation}"`
2. Check that it is running: `ros2 topic echo /navis/slam_status` (`navigation.state` is true).
3. If localization is off, publish an initial pose to `/navis/initialpose`.
4. Run a stored task (`/navis/run_task`) or an ad-hoc one (`/navis/run_realtime_task`).
5. Monitor progress on `/navis/run_management/global_status` and `/navis/run_management/task_status`.
   Pause, skip or cancel with `/navis/pause_task`, `/navis/next_goal` and `/navis/cancel_task`.

Most navigation interfaces only work while navigation is running (step 1).

Task poses are in map coordinates (frame `map`). Use `/navis/pixel_to_map` and `/navis/map_to_pixel`
to convert from or to map-image pixels.

## navis_http

### Services

| Service | Type | Navis endpoint |
|---|---|---|
| `navis/login` | `navis_msgs/srv/Login` | `POST /user/passport/login` (empty fields = configured credentials) |
| `navis/slam_operate` | `navis_msgs/srv/NavInput` | `POST /slam_operate` |
| `navis/get_map_list` | `navis_msgs/srv/GetMapList` | `GET /map_list` |
| `navis/get_map_info` | `navis_msgs/srv/GetMapInfo` | `GET /map_info` |
| `navis/get_map_image` | `navis_msgs/srv/GetMapImage` | `GET /downloadpng` |
| `navis/rename_map` | `navis_msgs/srv/RenameMap` | `POST /rename_map` |
| `navis/delete_map` | `navis_msgs/srv/DeleteMap` | `DELETE /delete_map` |
| `navis/export_map` | `navis_msgs/srv/ExportMap` | `GET /download_map_zip` (sends the numeric map id, falls back to the name) |
| `navis/export_maps` | `navis_msgs/srv/ExportMaps` | `POST /download_map_list` |
| `navis/import_map` | `navis_msgs/srv/ImportMap` | `POST /upload_map_zip` |
| `navis/pixel_to_map` | `navis_msgs/srv/PixelToMap` | `GET /get_map/positon` (pixels are rounded to integers; Navis rejects fractions) |
| `navis/map_to_pixel` | `navis_msgs/srv/MapToPixel` | `GET /get_world/positon` |
| `navis/get_task` | `navis_msgs/srv/GetTask` | `GET /get_tasks` |
| `navis/set_task` | `navis_msgs/srv/SetTask` | `POST /set_task` |
| `navis/delete_task` | `navis_msgs/srv/DeleteTask` | `DELETE /delete_task` |
| `navis/run_realtime_task` | `navis_msgs/srv/RunRealtimeTask` | `POST /realtime_task` |
| `navis/run_task` | `navis_msgs/srv/RunTask` | `POST /run_list_task` |
| `navis/get_software_versions` | `navis_msgs/srv/GetVersions` | `GET /version_info` |
| `navis/get_device_versions` | `navis_msgs/srv/GetVersions` | `GET /dev_version` |
| `navis/get_camera_list` | `navis_msgs/srv/GetCameraList` | `GET /get_camera_list` |
| `navis/add_camera` | `navis_msgs/srv/AddCamera` | `POST /add_camera` |
| `navis/set_camera` | `navis_msgs/srv/SetCamera` | `POST /set_camera` |
| `navis/delete_camera` | `navis_msgs/srv/DeleteCamera` | `DELETE /delete_camera` |

Every response has `success` and `message`. On failure, `message` carries the Navis error or the
connection error. The node does not crash on failures.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `base_url` | `http://192.168.6.102:8880` | Navis HTTP server |
| `username`, `password` | manual defaults | login credentials |
| `timeout` | `10.0` | request timeout (s) |
| `download_timeout` | `120.0` | timeout for map image / ZIP transfers (s) |
| `download_dir` | `~/navis_maps` | where exports go when `save_path` is empty |
| `frame_id` | `map` | frame of returned images |
| `task_theta_in_degrees` | `true` | units of the Navis task point `theta` field (see below) |

**Task heading units:** Navis task points use `x, y, theta`. In ROS, the heading is a yaw quaternion
on `TaskPoint.pose`. The manual's task examples use degrees, so the node converts to and from degrees
by default. If headings come out wrong on the robot, set `task_theta_in_degrees:=false`.

## navis_websocket

### Published topics

Navis topics are republished as `navis/<navis topic>`:

| ROS topic | Type |
|---|---|
| `navis/slam_status` | `navis_msgs/msg/SlamStatus` |
| `navis/interface_management/robot_status` | `navis_msgs/msg/RobotStatus` |
| `navis/interface_management/BMS_status` | `sensor_msgs/msg/BatteryState` |
| `navis/sensor_status` | `navis_msgs/msg/SensorStatus` |
| `navis/gnss_status` | `navis_msgs/msg/GnssStatus` |
| `navis/imu/data` | `sensor_msgs/msg/Imu` |
| `navis/points_raw` | `sensor_msgs/msg/PointCloud2` |
| `navis/scan` | `sensor_msgs/msg/LaserScan` (navigation only) |
| `navis/localizer_pose` | `geometry_msgs/msg/PoseStamped` (navigation only) |
| `navis/camera/color/image_raw`, `navis/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `navis/camera/color/image_raw/compressed`, `navis/camera/depth/image_rect_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `navis/camera/color/camera_info`, `navis/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `navis/freego_log/log` | `rcl_interfaces/msg/Log` |
| `navis/disk_states` | `navis_msgs/msg/DiskStates` |
| `navis/run_management/global_status` | `navis_msgs/msg/NavigationStatus` |
| `navis/run_management/task_status` | `navis_msgs/msg/TaskStatus` |
| `navis/run_management/task_progress` | `std_msgs/msg/Float64` (0-1) |
| `navis/localization_lost` | `std_msgs/msg/Bool` |
| `navis/odom` | `nav_msgs/msg/Odometry` (robot pose in the map, navigation only) |
| `navis/odom_raw` | `nav_msgs/msg/Odometry` (relative to the startup point) |
| `navis/run_management/visual_path` | `nav_msgs/msg/Path` |
| `navis/map_progress` | `std_msgs/msg/Int32` (mapping progress, %) |
| `navis/find_charger/get_charge_point` | `geometry_msgs/msg/PoseStamped` |

**Navis topics are only streamed while someone listens.** Once a second the node checks each ROS
publisher and subscribes or unsubscribes on Navis accordingly. Topics listed in `always_subscribe`
are always streamed. There is a delay of up to 1 s before the first message after you start listening.

### Subscribed topics (commands)

| ROS topic | Type | Effect |
|---|---|---|
| `navis/cmd_vel` | `geometry_msgs/msg/Twist` | drive the base (`linear.x`, `angular.z`), only after `navis/enable_cmd_vel` |
| `navis/light_command` | `navis_msgs/msg/LightCommand` | chassis lights |
| `navis/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | re-localize during navigation |

### Services

| Service | Type | Navis operation |
|---|---|---|
| `navis/nav_input` | `navis_msgs/srv/NavInput` | `/input/op`: record, map, start/stop navigation |
| `navis/cancel_task` | `std_srvs/srv/Trigger` | cancel the running task |
| `navis/pause_task` | `std_srvs/srv/SetBool` | `true` = pause, `false` = resume |
| `navis/next_goal` | `std_srvs/srv/SetBool` | skip to the next target point |
| `navis/set_obstacle_avoidance` | `std_srvs/srv/SetBool` | `true` = drive around obstacles, `false` = stop at them |
| `navis/set_charge_point` | `std_srvs/srv/Trigger` | store the current position as the charging point |
| `navis/get_charge_point` | `navis_msgs/srv/GetChargePoint` | read the charging point |
| `navis/set_charging` | `std_srvs/srv/SetBool` | go charge / stop charging |
| `navis/set_charge_level` | `navis_msgs/srv/SetChargeLevel` | auto-charge thresholds |
| `navis/enable_cmd_vel` | `std_srvs/srv/SetBool` | enable/disable `navis/cmd_vel` forwarding (disabling sends a stop) |

`cancel_task` is fire-and-forget because Navis does not reply to it. Check
`navis/run_management/global_status` for the result.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `ws_url` | `ws://192.168.6.102:9090` | Navis websocket server |
| `always_subscribe` | `["/slam_status"]` | Navis topics to stream even without ROS subscribers |
| `use_cbor` | `false` | request CBOR for point clouds, scans and images (needs `cbor2`) |
| `use_navis_stamps` | `false` | keep Navis timestamps instead of restamping with local time on receipt |
| `map_frame` | `map` | frame for the charge point and the default initial pose frame |
| `gnss_frame` | `gnss` | frame of `GnssStatus.fix` |
| `service_timeout` | `30.0` | max wait for a Navis service response (s) |
| `reconnect_delay` | `2.0` | delay between reconnect attempts (s) |
| `heartbeat_period` | `1.0` | ping period (s) |
| `cmd_vel_topic` | `/cmd_vel` | Navis topic that velocity commands are published to |
| `cmd_vel_enabled` | `false` | start with `navis/cmd_vel` forwarding enabled |
| `max_linear_speed`, `max_angular_speed` | `1.5`, `1.5` | clamp for forwarded velocities (m/s, rad/s) |

Timestamps are replaced with local time by default because the robot clock is not guaranteed to be
in sync with the ROS machine.

## Testing

The tests need no robot. Build first, then run them with colcon:

```bash
colcon build --packages-select navis_msgs navis_controller && source install/setup.bash
colcon test --packages-select navis_controller --event-handlers console_direct+
colcon test-result --verbose
```

or directly with pytest (after sourcing the workspace): `python3 -m pytest src/navis_controller/test -v`.

With pixi, `pixi run style-check` checks formatting and import order, and `pixi run test` builds and runs
all tests. CI ([`.github/workflows/main.yml`](.github/workflows/main.yml)) runs both on Linux and Windows
for every push and pull request; run `pixi run style` to fix formatting before pushing.

| File | What it checks |
|---|---|
| `test/test_conversions.py` | Navis JSON ↔ ROS 2 conversion: headers and frames, base64 data, null → NaN, CameraInfo fields, Navis-specific messages, task theta units |
| `test/test_integration.py` | both nodes against the fake server: login and re-login, service calls, lazy topic subscription, heartbeat, concurrent service calls, `cmd_vel` gating, reconnection |
| `test/test_robot.py` | both nodes against a real Navis, without moving the base (skipped unless `NAVIS_ROBOT_IP` is set, see below) |
| `test/fake_navis_server.py` | fake Navis HTTP + websocket server used by the integration tests |
| `test/navis_test_utils.py` | helpers shared by the integration and robot tests |

The fake server can also be run on its own, to try the nodes or the launch file without the robot:

```bash
python3 src/navis_controller/test/fake_navis_server.py --http-port 8880 --ws-port 9090 -v
ros2 launch navis_controller navis.launch.py robot_ip:=127.0.0.1
```

It answers with the example data from the Navis manual (map `office`, task `p1`) and prints every
request with `-v`. It does not simulate the robot, so these tests do not replace checks on the real
hardware, such as task heading units and whether `/cmd_vel` moves the base.

### Tests on the robot

`test/test_robot.py` checks the nodes against a real Navis. It is skipped unless `NAVIS_ROBOT_IP` is
set, so CI and the commands above never run it. **It never moves the base**: it does not start
mapping or recording, run or pause tasks, send `cmd_vel` or an initial pose, or change charging,
obstacle-avoidance or map settings. By default it only reads; existing maps and tasks are never changed.
Navigation is only started with `NAVIS_START_NAVIGATION=1` (this loads the map and localization; the
base does not move).

```bash
NAVIS_ROBOT_IP=192.168.6.102 pixi run test-robot              # Linux
$env:NAVIS_ROBOT_IP="192.168.6.102"; pixi run test-robot      # Windows PowerShell
```

| Variable | Meaning |
|---|---|
| `NAVIS_ROBOT_IP` | Navis address (required) |
| `NAVIS_HTTP_PORT` / `NAVIS_WS_PORT` | default `8880` / `9090` |
| `NAVIS_PASSWORD` | if the robot does not use the manual's default password |
| `NAVIS_TEST_MAP` | map to read; default is the map navigation is running on, else the first map |
| `NAVIS_ALLOW_WRITES=1` | also create a temporary task, read it back and delete it |
| `NAVIS_START_NAVIGATION=1` | if navigation is not running, start it on the test map for the navigation tests (`/navis/slam_operate`) and stop it again at the end (`/navis/nav_input`); navigation that is already running is left alone |

The navigation tests (`/odom`, `localizer_pose`, `global_status`, charge point) run last and are
skipped when navigation is off and `NAVIS_START_NAVIGATION` is not set. `localizer_pose` needs the
robot to be localized; if it is not, set an initial pose (e.g. in the web UI) first.

What it checks: login and automatic re-login after an invalid token, map list/info/image (the PNG
size must match the map info), pixel ↔ map conversion against the manual's formula, the tasks of
the map (indices, orientations, points inside the map), map export ZIP, versions, camera list, that
the websocket connection survives the heartbeat window, the converted topics (frames without leading
slash, valid quaternions, consistent LaserScan/PointCloud2 sizes, battery percentage), unsubscribing
when unused, and the charge point. Topics that depend on hardware or the current mode (camera, GNSS,
point cloud, localization) are skipped when nothing arrives within 5 s. It cannot tell whether task
headings are in degrees or radians; compare a task's heading with the Navis web UI for that.
