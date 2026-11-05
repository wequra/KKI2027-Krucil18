# System architecture (draft v0.1)

Three zones: **Boat**, **GCS laptop** (shore, runs mission and local dashboard), **Cloud mirror** (optional, for remote viewing).

```mermaid
flowchart LR
  subgraph Boat
    FC[Flight controller<br/>ArduPilot Rover]
    JET[Companion computer<br/>ROS + MAVROS + YOLOv8]
    CAM[Camera]
    GPS[GPS/IMU]
    RAD[Telemetry radio]
    CAM --> JET
    GPS --> FC
    FC <-->|MAVLink serial| JET
    FC <--> RAD
    JET <-->|Wi-Fi / video| WIFI[Onboard Wi-Fi AP]
  end
  subgraph GCS[GCS laptop]
    MP[MAVProxy / pymavlink ingest]
    API[FastAPI backend<br/>REST + WebSocket]
    DB[(PostgreSQL / Timescale)]
    MTX[MediaMTX<br/>WebRTC]
    DASH[React dashboard]
    MP --> API --> DB
    API -->|WebSocket| DASH
    MTX -->|WebRTC| DASH
  end
  RAD <-->|MAVLink radio| MP
  WIFI -->|events, images, video| API
  WIFI --> MTX
  API -.->|optional sync| CLOUD[(Cloud mirror)]
```

## Data flows
1. **Telemetry** (position, heading, SOG/COG, battery): FC -> radio -> MAVProxy/pymavlink -> `POST /api/v1/telemetry` -> DB + WebSocket.
2. **Mission events** (buoy pass, penalty, capture, docking): ROS mission node -> Wi-Fi -> `POST /api/v1/events`.
3. **Images** (surface/underwater capture): ROS -> Wi-Fi -> `POST /api/v1/images` (multipart).
4. **Video**: camera -> Jetson -> MediaMTX -> WebRTC -> dashboard.
5. **Store-and-forward**: ROS keeps an outbox queue; resends events and images after link loss.
6. **Failsafe**: link loss -> boat stops and holds position (mode HOLD), per the guidebook. Backend flags `link_lost` on the dashboard.

## Ownership
- Boat/ROS side: Arya. Backend + contract: Jojo. Dashboard: Bertrant.
