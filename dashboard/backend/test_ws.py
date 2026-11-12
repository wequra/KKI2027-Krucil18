import asyncio
import websockets
import json
import urllib.request

async def test_websocket():
    uri = "ws://127.0.0.1:8000/ws/dashboard"
    print(f"Mencoba konek ke {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Berhasil konek ke WebSocket!")
            
            # Trigger simulator via REST API di background
            print("Menyalakan Simulator Telemetri...")
            req = urllib.request.Request("http://127.0.0.1:8000/api/v1/telemetry/simulator/start", method="POST")
            urllib.request.urlopen(req)
            
            # Dengarkan 5 pesan pertama yang masuk
            for i in range(5):
                response = await websocket.recv()
                data = json.loads(response)
                print(f"[{i+1}/5] Pesan diterima! Tipe: {data['type']}")
                print(f"      Data: {data['data']}")
            
            print("Mematikan Simulator Telemetri...")
            req = urllib.request.Request("http://127.0.0.1:8000/api/v1/telemetry/simulator/stop", method="POST")
            urllib.request.urlopen(req)
            print("Test WebSocket Berhasil! ✅")

    except Exception as e:
        print(f"Gagal koneksi atau error: {e}")

if __name__ == "__main__":
    asyncio.run(test_websocket())
