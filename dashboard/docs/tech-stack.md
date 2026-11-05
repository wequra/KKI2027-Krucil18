# Tech stack decision (draft for M1)

| Layer | Choice | Why | Alternative |
|---|---|---|---|
| Autopilot link | ArduPilot Rover + MAVROS (ROS) | Boat frame supported, SITL available | PX4 |
| Simulation | ArduPilot SITL (Rover, boat) + recorded video | Runs without hardware | Gazebo later if time |
| Vision | YOLOv8 (ultralytics) on Jetson | Fast to fine-tune for red/green balls | Classic HSV thresholding as fallback |
| Backend | FastAPI (Python) | Same language as pymavlink/ROS nodes, async WebSocket | Node/Express |
| DB | PostgreSQL (events, missions) + InfluxDB or Timescale (telemetry) | Time-series fits telemetry | SQLite for demo only |
| Realtime | WebSocket for telemetry/events; WebRTC via MediaMTX for video | Low latency video | MJPEG (simpler, heavier) |
| Dashboard | React + Vite, Canvas for 5x5 grid | Reuses last year's approach | - |
| Contract | JSON Schema in `docs/contract/` | Language-neutral, testable | OpenAPI only |
| Ops | Docker Compose for backend+DB+MediaMTX | One command demo | - |

## Decisions to confirm with the team
1. Telemetry store: Influx vs Timescale (default: Timescale to run only one DB engine).
2. Video: WebRTC (default) vs MJPEG fallback.
3. Hardware list (Jetson model, camera) must match the electrical division's list. See `power-needs.md`.
