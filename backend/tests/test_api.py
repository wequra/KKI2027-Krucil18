import json
from pathlib import Path
from fastapi.testclient import TestClient
from app.main import app

EXAMPLES_DIR = Path(__file__).resolve().parents[2] / "docs" / "contract" / "examples"
HEADERS = {"X-API-Key": "changeme-dev-key"}

def load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)

def test_unauthorized_ingest_rejected():
    with TestClient(app) as client:
        payload = load_json(EXAMPLES_DIR / "telemetry.json")
        resp = client.post("/api/v1/ingest/telemetry", json=payload)
        assert resp.status_code == 401

def test_invalid_contract_payload_rejected():
    with TestClient(app) as client:
        resp = client.post("/api/v1/ingest/telemetry", json={"bad": "payload"}, headers=HEADERS)
        assert resp.status_code == 422

def test_ingest_and_websocket_broadcast():
    with TestClient(app) as client:
        with client.websocket_connect("/ws/stream") as ws:
            telemetry = load_json(EXAMPLES_DIR / "telemetry.json")
            r1 = client.post("/api/v1/ingest/telemetry", json=telemetry, headers=HEADERS)
            assert r1.status_code == 200, r1.text
            msg1 = ws.receive_json()
            assert msg1["channel"] == "telemetry"
            assert msg1["data"] == telemetry

            event = load_json(EXAMPLES_DIR / "event.json")
            r2 = client.post("/api/v1/ingest/event", json=event, headers=HEADERS)
            assert r2.status_code == 200, r2.text
            msg2 = ws.receive_json()
            assert msg2["channel"] == "event"
            assert msg2["data"] == event

            image_meta = load_json(EXAMPLES_DIR / "image-meta.json")
            r3 = client.post("/api/v1/ingest/image-meta", json=image_meta, headers=HEADERS)
            assert r3.status_code == 200, r3.text
            msg3 = ws.receive_json()
            assert msg3["channel"] == "image_meta"
            assert msg3["data"] == image_meta
