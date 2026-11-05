# Data contract v0.1 (DRAFT, freeze Fri 13 Nov)

Schemas live in `contract/*.schema.json`; examples in `contract/examples/`. `backend/tests/test_contract.py` validates every example.

## REST (boat/GCS -> backend), header `X-API-Key: <key>`
| Method | Path | Body | Notes |
|---|---|---|---|
| POST | `/api/v1/telemetry` | telemetry | ~2-5 Hz |
| POST | `/api/v1/events` | event | idempotent on `event_id` (store-and-forward resends) |
| POST | `/api/v1/images` | multipart: `meta` (image-meta JSON) + `file` | |
| GET | `/api/v1/missions/{id}` | - | state, score, event list |

## WebSocket (backend -> dashboard): `/ws/live`
Messages are `{ "type": "telemetry" | "event" | "link_status", "data": {...} }`.
`link_status`: `{ "connected": bool, "last_seen_s": number }`, so the UI can show stale data.

## Telemetry fields
Team/PT/lane identify the run. Time is UTC ISO-8601; the dashboard shows day/date/time in WIB (guidebook geo-tag).
SOG is sent in m/s and converted for display (km/h and knots) by the dashboard, to avoid unit drift:
km/h = m/s x 3.6, knots = m/s x 1.943844.

## Mission states
`IDLE, ARMED, TRANSIT_BUOYS, SURFACE_IMAGING, UNDERWATER_IMAGING, DOCKING, FINISHED, HOLD_FAILSAFE, RESTART`

## Event types
`buoy_pass, buoy_miss, penalty, image_capture, dock_start, dock_complete, restart, failsafe_hold, link_lost, link_restored`
Penalty reasons: `buoy_touch, boundary_violation, other` (-5 each, restart after more than 5 violations per the rules).
