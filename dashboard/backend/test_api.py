import urllib.request, json

BASE = "http://127.0.0.1:8000"
API_KEY = "dagozilla_super_secret_key_2026"

def api(method, path, data=None, need_key=False):
    headers = {"Content-Type": "application/json"}
    if need_key:
        headers["X-API-Key"] = API_KEY
    body = json.dumps(data).encode() if data else None
    req = urllib.request.Request(BASE + path, data=body, headers=headers, method=method)
    try:
        resp = urllib.request.urlopen(req)
        result = json.loads(resp.read())
        print(f"  [{resp.status}] {method} {path}")
        return result
    except urllib.error.HTTPError as e:
        print(f"  [{e.code}] {method} {path} - {e.read().decode()}")
        return None

print("=== 1. CREATE RUN ===")
run = api("POST", "/api/v1/runs/", {"run_name": "Run Test Pertama", "status": "RUNNING"}, need_key=True)
print(f"     Run ID: {run['id']}, Name: {run['run_name']}, Status: {run['status']}")

print()
print("=== 2. POST EVENTS (with run_id) ===")
evt = api("POST", "/api/v1/events/", {
    "run_id": run["id"],
    "event_type": "START",
    "timestamp": "2026-07-09T20:00:00Z",
    "grid_position": "A1",
    "metadata": {"mode": "autonomous"}
}, need_key=True)
print(f"     Event ID: {evt['id']}, Type: {evt['event_type']}, Run: {evt['run_id']}")

evt2 = api("POST", "/api/v1/events/", {
    "run_id": run["id"],
    "event_type": "BUOY_PAIR",
    "timestamp": "2026-07-09T20:01:00Z",
    "grid_position": "B3",
    "metadata": {"buoy_color": "red", "confidence": 0.95}
}, need_key=True)
print(f"     Event ID: {evt2['id']}, Type: {evt2['event_type']}")

print()
print("=== 3. GET EVENTS (list) ===")
events = api("GET", "/api/v1/events/")
print(f"     Total events returned: {len(events)}")

print()
print("=== 4. GET EVENTS (filter by type) ===")
events = api("GET", "/api/v1/events/?event_type=BUOY_PAIR")
print(f"     BUOY_PAIR events: {len(events)}")

print()
print("=== 5. GET SINGLE EVENT ===")
evt_detail = api("GET", f"/api/v1/events/{evt2['id']}")
print(f"     Event: {evt_detail['event_type']} at {evt_detail['grid_position']}, metadata: {evt_detail['metadata']}")

print()
print("=== 6. GET RUNS (list) ===")
runs = api("GET", "/api/v1/runs/")
print(f"     Total runs: {len(runs)}")
r = runs[0]
print(f"     Latest: {r['run_name']} ({r['status']}), events: {r['event_count']}, images: {r['image_count']}")

print()
print("=== 7. PATCH RUN (update notes) ===")
updated = api("PATCH", f"/api/v1/runs/{run['id']}", {"notes": "Testing dari terminal"}, need_key=True)
print(f"     Notes updated: {updated['notes']}")

print()
print("=== 8. DASHBOARD SUMMARY ===")
summary = api("GET", "/api/v1/dashboard/summary")
print(f"     Active run: {summary['active_run_name']} ({summary['active_run_status']})")
print(f"     Total: {summary['total_runs']} runs, {summary['total_events']} events, {summary['total_images']} images")
print(f"     Last event: {summary['last_event_type']} at {summary['last_event_grid']}")

print()
print("=== 9. HEALTH CHECK ===")
health = api("GET", "/health")
print(f"     {health}")

print()
print("============ ALL TESTS PASSED! ============")
