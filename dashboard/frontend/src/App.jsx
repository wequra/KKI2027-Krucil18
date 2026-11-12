import { useState, useEffect } from 'react'
import Header from './components/Header'
import LeftPanel from './components/LeftPanel'
import CenterPanel from './components/CenterPanel'
import RightPanel from './components/RightPanel'
import { Activity } from 'lucide-react'

const INITIAL_TRACKER_STATE = {
  resetVersion: 0,
};

function App() {
  // Global State untuk Telemetry - null sampai ada packet telemetry asli
  const [telemetry, setTelemetry] = useState({
    latitude: null,
    longitude: null,
    speed: 0.0,
    heading: 0.0,
    battery_voltage: null,
    battery_remaining: null
  });

  const [wsStatus, setWsStatus] = useState('CONNECTING...');
  const [trackerState, setTrackerState] = useState(INITIAL_TRACKER_STATE);

  // Reset hanya state tracker. WebSocket dan telemetry tetap berjalan agar
  // pembaruan telemetry berikutnya dapat membangun lintasan baru.
  const resetTracker = () => {
    setTrackerState(previous => ({
      ...previous,
      resetVersion: previous.resetVersion + 1,
    }));
  };

  // Koneksi WebSocket ke Backend GCS
  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8000/ws/dashboard');

    ws.onopen = () => {
      console.log('Connected to GCS Backend');
      setWsStatus('CONNECTED');
    };

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry_update') {
          setTelemetry(prev => ({ ...prev, ...msg.data }));
        }
      } catch (err) {
        console.error("Failed to parse WS message", err);
      }
    };

    ws.onclose = () => {
      setWsStatus('DISCONNECTED');
    };

    return () => {
      ws.close();
    };
  }, []);

  return (
    <div className="dashboard-container">
      <Header wsStatus={wsStatus} />
      
      <main className="main-grid">
        <LeftPanel telemetry={telemetry} />
        <CenterPanel telemetry={telemetry} resetVersion={trackerState.resetVersion} />
        <RightPanel onReset={resetTracker} />
      </main>
    </div>
  )
}

export default App
