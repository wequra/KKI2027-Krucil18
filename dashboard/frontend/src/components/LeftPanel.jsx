import { useState, useEffect } from 'react';

export default function LeftPanel({ telemetry }) {
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <div className="left-col">
      {/* Geo-Tag Panel */}
      <div className="panel">
        <div className="panel-header">Geo-Tag</div>
        <div className="panel-content">
          <div className="data-row">
            <span>Day</span>
            <span className="text-muted">{time.toLocaleDateString('en-US', { weekday: 'long' })}</span>
          </div>
          <div className="data-row">
            <span>Date</span>
            <span className="text-muted">{time.toLocaleDateString('en-GB')}</span>
          </div>
          <div className="data-row">
            <span>Time</span>
            <span className="text-muted">{time.toLocaleTimeString('en-GB')}</span>
          </div>
        </div>
      </div>

      {/* Coordinate Panel */}
      <div className="panel">
        <div className="panel-header">Coordinate</div>
        <div className="panel-content" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
          <div className="font-mono" style={{ fontSize: '1.1rem', fontWeight: 500, color: (telemetry.latitude != null && telemetry.longitude != null) ? '#38bdf8' : 'var(--text-muted)' }}>
            {telemetry.latitude != null && telemetry.longitude != null ? (
              `${telemetry.latitude > 0 ? 'N' : 'S'} ${Math.abs(telemetry.latitude).toFixed(6)} ${telemetry.longitude > 0 ? 'E' : 'W'} ${Math.abs(telemetry.longitude).toFixed(6)}`
            ) : (
              '-- NO GPS FIX --'
            )}
          </div>
        </div>
      </div>

      {/* Speed Panel */}
      <div className="panel">
        <div className="panel-header">Speed Over Ground (SOG)</div>
        <div className="panel-content flex-between">
          <div style={{ flex: 1, textAlign: 'center' }}>
            <div className="big-value">{(telemetry.speed * 3.6).toFixed(2)}</div>
            <div className="unit">km/h</div>
          </div>
          <div style={{ flex: 1, textAlign: 'center' }}>
            <div className="big-value">{(telemetry.speed * 1.94384).toFixed(2)}</div>
            <div className="unit">knot</div>
          </div>
        </div>
      </div>

      {/* Course Panel */}
      <div className="panel">
        <div className="panel-header">Course Over Ground (COG)</div>
        <div className="panel-content">
          <div className="big-value">{telemetry.heading.toFixed(0)}&deg;</div>
          <div className="unit" style={{ textAlign: 'center' }}>deg</div>
        </div>
      </div>

      {/* Event Log Panel */}
      <div className="panel" style={{ flex: 1 }}>
        <div className="panel-header">Event Log</div>
        <div className="panel-content event-log">
          <div className="log-entry">
            <span className="log-time">00:00</span>
            <span className="text-green">SYSTEM INIT -- Monitoring online</span>
          </div>
          <div className="log-entry">
            <span className="log-time">00:05</span>
            <span className="text-blue" style={{color: '#3b82f6'}}>START -- Departed dock, Lane A</span>
          </div>
          <div className="log-entry">
            <span className="log-time">00:18</span>
            <span className="text-green">BUOY PAIR #1  +10 pts</span>
          </div>
          <div className="log-entry">
            <span className="log-time">00:41</span>
            <span className="text-red">PENALTY -5 pts (hull contact red buoy #3)</span>
          </div>
        </div>
      </div>
    </div>
  )
}
