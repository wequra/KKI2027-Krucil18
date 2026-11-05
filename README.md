# KKI 2027 ASV - Programming Division (Dagozilla Kru 18)

Software for the Autonomous Surface Vehicle (fishing-boat theme): ROS/MAVROS autonomy,
computer vision, telemetry backend and the web monitoring dashboard.

## Layout
| Path | Owner (lead) | Content |
|---|---|---|
| `ros_ws/` | Arya | ROS workspace: MAVROS bridge, detection, navigation, mission FSM, failsafe |
| `backend/` | Jojo | FastAPI ingest + WebSocket + DB |
| `dashboard/` | Bertrant | React monitoring dashboard |
| `docs/` | Jojo | Architecture, tech stack, data contract, proposal drafts |

## Quick start
```bash
# backend (Week 3 will add real code)
cd backend && python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest            # validates the data contract examples
```

## Docs
- [Tech stack](docs/tech-stack.md)
- [Architecture](docs/architecture.md)
- [Data contract](docs/data-contract.md) (frozen at M2, Fri 13 Nov)
- [Power needs for electrical](docs/power-needs.md)
- [Bab I draft](docs/bab1-draft.md)
- [Contributing / branch rules](CONTRIBUTING.md)
