import { useEffect, useState } from 'react';
import { ChevronDown, ChevronRight } from 'lucide-react';

function CollapsiblePanel({ title, children, defaultOpen = true }) {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  return (
    <div className="panel">
      <div 
        className="panel-header" 
        style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
        onClick={() => setIsOpen(!isOpen)}
      >
        <span>{title}</span>
        {isOpen ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
      </div>
      {isOpen && (
        <div className="panel-content">
          {children}
        </div>
      )}
    </div>
  );
}

function SurfaceCameraLiveStream() {
  const streamUrl =
    import.meta.env.VITE_SURFACE_CAMERA_WEBRTC_URL ||
    'http://192.168.88.253:8080/stream.mjpg';

  return (
    <div style={{ width: '100%', height: '100%', background: '#000' }}>
      <img
        src={streamUrl}
        alt="Surface Camera Live Stream"
        style={{
          width: '100%',
          height: '100%',
          objectFit: 'contain',
          display: 'block',
        }}
      />
    </div>
  );
}

function MissionImage({ imageType, accentColor, zoneLabel }) {
  const apiUrl = import.meta.env.VITE_GCS_API_URL || 'http://localhost:8000';
  const [image, setImage] = useState(null);

  useEffect(() => {
    let cancelled = false;
    const loadLatest = async () => {
      try {
        const response = await fetch(`${apiUrl}/api/v1/images/?image_type=${imageType}&limit=1`);
        if (!response.ok) return;
        const images = await response.json();
        if (!cancelled) setImage(images[0] || null);
      } catch (error) {
        console.warn(`Unable to load ${imageType} image`, error);
      }
    };
    loadLatest();
    const interval = setInterval(loadLatest, 5000);
    return () => { cancelled = true; clearInterval(interval); };
  }, [apiUrl, imageType]);

  return (
    <div className="video-container">
      {image ? (
        <img
          src={`${apiUrl}/api/v1/images/${image.id}/file`}
          alt={`${imageType} mission capture`}
          style={{ width: '100%', height: '100%', objectFit: 'contain' }}
        />
      ) : (
        <>
          <div className="video-status standby">
            <div className="dot"></div> STANDBY
          </div>
          <div style={{ color: accentColor, textAlign: 'center' }}>
            AWAITING CAPTURE<br/>
            <span style={{ fontSize: '0.6rem' }}>{zoneLabel}</span>
          </div>
        </>
      )}
    </div>
  );
}

export default function RightPanel({ onReset }) {
  const [missionStatus, setMissionStatus] = useState('RUNNING');

  return (
    <div className="right-col">
      {/* Mission Control Panel */}
      <CollapsiblePanel title="Mission Control">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
          <div className="flex-between" style={{ fontSize: '0.8rem' }}>
            <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
              <span className="text-muted">IP:</span>
              <input type="text" defaultValue="192.168.1.10" style={{ background: '#000', border: '1px solid #334155', color: '#fff', padding: '4px', width: '100px' }} />
              <span className="text-muted">Port:</span>
              <input type="text" defaultValue="8080" style={{ background: '#000', border: '1px solid #334155', color: '#fff', padding: '4px', width: '50px' }} />
            </div>
            <button className="btn btn-outline" style={{ color: 'var(--success)', borderColor: 'var(--success)' }}>CONNECT</button>
          </div>

          <div style={{ fontSize: '0.85rem' }}>
            <span className="text-muted">LANE: </span> <strong>A</strong> &nbsp;&nbsp;&nbsp;
            <span className="text-muted">RUN: </span> <strong>2</strong> &nbsp;&nbsp;&nbsp;
            <span className="text-muted">STATUS: </span> <span className="text-green">{missionStatus}</span>
          </div>

          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr 1fr', gap: '4px' }}>
            <button className="btn btn-green">START</button>
            <button className="btn btn-yellow">PAUSE</button>
            <button className="btn btn-red">ABORT</button>
            <button className="btn btn-red">E-STOP</button>
            <button className="btn btn-outline" onClick={onReset} style={{ gridColumn: '1 / -1' }}>RESET</button>
          </div>
        </div>
      </CollapsiblePanel>

      {/* Mission State Panel */}
      <CollapsiblePanel title="Mission State">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
          <div className="mission-grid">
            <button className="btn btn-green" style={{ fontSize: '0.7rem' }}>Find Buoy T1</button>
            <button className="btn btn-yellow" style={{ fontSize: '0.7rem' }}>Find Buoy T2</button>
            <button className="btn btn-outline" style={{ fontSize: '0.7rem' }}>Find Buoy T3</button>
            <button className="btn btn-outline" style={{ fontSize: '0.7rem' }}>Find Green Box</button>
            <button className="btn btn-green" style={{ fontSize: '0.7rem' }}>Clear T1</button>
            <button className="btn btn-outline" style={{ fontSize: '0.7rem' }}>Clear T2</button>
            <button className="btn btn-outline" style={{ fontSize: '0.7rem' }}>Clear T3</button>
            <button className="btn btn-outline" style={{ fontSize: '0.7rem' }}>Capture Green Box</button>
          </div>
          <div style={{ display: 'flex', justifyContent: 'center' }}>
             <button className="btn btn-outline" style={{ fontSize: '0.7rem', width: '50%' }}>Docking</button>
          </div>
          <div style={{ fontSize: '0.8rem', marginTop: '8px' }}>
            <span className="text-muted">SCORE: </span> <strong className="text-yellow">10</strong>
          </div>
        </div>
      </CollapsiblePanel>

      {/* Surface Imaging */}
      <CollapsiblePanel title="Surface Imaging — IMH">
        <MissionImage imageType="SURFACE" accentColor="var(--success)" zoneLabel="GREEN-BOX ZONE" />
      </CollapsiblePanel>

      {/* Underwater Imaging */}
      <CollapsiblePanel title="Underwater Imaging — IMB">
        <MissionImage imageType="UNDERWATER" accentColor="#3b82f6" zoneLabel="BLUE BOX ZONE" />
      </CollapsiblePanel>

      <CollapsiblePanel title="Surface Camera — Live Stream">
        <SurfaceCameraLiveStream />
      </CollapsiblePanel>

    </div>
  )
}
