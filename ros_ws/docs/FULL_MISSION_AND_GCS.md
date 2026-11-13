# Full mission and Jetson GCS interface

## Start

```bash
source /opt/ros/jazzy/setup.bash
source /dagozilla_ws/install/setup.bash
ros2 launch bringup asv_full_mission.launch.py use_mavros:=true main_camera_device:=/dev/video0 underwater_camera_device:=/dev/video1 control_backend:=rc_override fcu_url:=/dev/ttyACM0:115200 stream_host:=0.0.0.0 stream_port:=8080
ros2 service call /asv/start_mission std_srvs/srv/Trigger "{}"
```

This launch never arms the Pixhawk. The full sequence is `NAVIGATING_BUOYS -> UNDERWATER_IMAGING -> SURFACE_IMAGING -> DOCKING -> COMPLETED`. Gate count 10 triggers the first automatic transition. The mission, cameras, and motor path do not depend on either HTTP server or the laptop.

## Network API

Default base URL: `http://JETSON_IP:8090`.

- `GET /health` returns bridge process health.
- `GET /api/telemetry` returns `latitude`, `longitude`, horizontal `speed_mps`, compass `heading_deg`, `armed`, `mode`, MAVROS `connected`, `mission_state`, numeric `mission_state_id`, `gate_count`, Unix `timestamp`, per-source `stale`, and aggregate `telemetry_stale`. Missing data is JSON `null`; it is never fabricated.
- `GET /api/mission` returns mission state, gate count, timestamp, and staleness.
- `GET /api/images/underwater/latest` and `/api/images/surface/latest` return JPEG data.
- Append `/metadata` to either image URL for capture type, timestamp, success, fallback, filename, and download URL.

Responses allow cross-origin browser requests. The API is read-only and intentionally has no motor, arm, or mission-control endpoint. `gcs_telemetry_rate_hz` is documented as the recommended laptop polling rate (default 5 Hz); REST clients control their actual request rate.

Main-camera MJPEG is `http://JETSON_IP:8080/stream.mjpg`, health/snapshot are `/health` and `/snapshot.jpg`. Frames are available only while a mission is active and are cleared at `COMPLETED`, `ABORTED`, `TIMEOUT`, or `FINISH`; the captured-image API remains available afterward.

## Images

Jetson defaults:

- `~/dagozilla_logs/images/underwater/underwater_<UTC timestamp>.jpg`
- `~/dagozilla_logs/images/surface/surface_<UTC timestamp>.jpg`

Each JPEG has a `.jpg.json` sidecar. Underwater timeout saves the latest valid frame with `fallback: true` and continues. Surface timeout uses the same conservative fallback policy so the mission cannot drive indefinitely when the marker is absent.

## Control ownership and safety

The full launch remaps the unchanged buoy output to `/asv/cmd_vel/buoy`. Imaging sources are `/asv/cmd_vel/underwater`, `/asv/cmd_vel/surface`, and `/asv/cmd_vel/docking`. Only `mission_cmd_mux_node` publishes `/asv/cmd_vel`; it selects by mission state, publishes neutral on every handoff, on stale commands, on stale mission-manager heartbeat, and in terminal states. The existing RC override deadman remains downstream.

## Field tuning

Tune HSV bounds and `min_target_area` under actual lighting first. Then validate `underwater_search_*`, both imaging timeouts/tolerances/stabilization delays, `surface_target_width_ratio_capture`, `max_mission_forward_speed`, and all docking size/speed ratios in a controlled pool. Image-width ratios depend on camera field of view and mounting.

Runtime dependencies are ROS 2 Jazzy packages `cv_bridge`, `mavros`, `mavros_msgs`, OpenCV Python (`python3-opencv`), and working V4L2 devices. The HTTP servers use Python's standard library; no internet, Flask, or WebSocket dependency is required.
