import { useEffect, useRef, useState } from 'react';
import { VENUES, DEFAULT_VENUE_KEY } from '../config/venues';

export default function CenterPanel({ telemetry, resetVersion }) {
  const canvasRef = useRef(null);
  const mapImageRef = useRef(null);

  const [selectedVenueKey, setSelectedVenueKey] = useState(DEFAULT_VENUE_KEY);
  const [showGrid, setShowGrid] = useState(true);
  const userManuallySelectedVenue = useRef(false);

  const activeVenue = VENUES[selectedVenueKey] || VENUES[DEFAULT_VENUE_KEY];

  // Tracker state stores GPS trajectory history
  const trackerRef = useRef({
    trajectory: [], // array of { latitude, longitude }
    current: null,   // { latitude, longitude, heading }
  });

  // Auto-detect venue on telemetry arrival unless user has manually chosen one
  useEffect(() => {
    const lat = Number(telemetry.latitude);
    const lng = Number(telemetry.longitude);
    if (!Number.isFinite(lat) || !Number.isFinite(lng)) return;

    if (!userManuallySelectedVenue.current) {
      for (const [key, venue] of Object.entries(VENUES)) {
        const [[southLat, westLng], [northLat, eastLng]] = venue.bounds;
        if (lat >= southLat && lat <= northLat && lng >= westLng && lng <= eastLng) {
          if (selectedVenueKey !== key) {
            setSelectedVenueKey(key);
          }
          break;
        }
      }
    }
  }, [telemetry.latitude, telemetry.longitude, selectedVenueKey]);

  // Load venue satellite background image
  useEffect(() => {
    const img = new Image();
    img.src = activeVenue.image;
    img.onload = () => {
      mapImageRef.current = img;
      renderCanvas();
    };
    img.onerror = () => {
      console.warn(`Could not load map image from ${activeVenue.image}`);
      mapImageRef.current = null;
      renderCanvas();
    };
  }, [selectedVenueKey]);

  // Reset trajectory on resetVersion change
  useEffect(() => {
    trackerRef.current.trajectory = [];
    renderCanvas();
  }, [resetVersion]);

  // Process incoming telemetry and update trajectory
  useEffect(() => {
    const lat = Number(telemetry.latitude);
    const lng = Number(telemetry.longitude);
    const hdg = Number(telemetry.heading);

    // Ignore missing coordinates and uninitialized (0, 0) Pixhawk GPS locks
    if (!Number.isFinite(lat) || !Number.isFinite(lng) || (lat === 0 && lng === 0)) return;

    const heading = Number.isFinite(hdg) ? hdg : 0;
    const tracker = trackerRef.current;

    // Check if position changed noticeably (> ~0.5 meter, ~0.000005 deg)
    const prev = tracker.current;
    const changed = !prev || Math.abs(prev.latitude - lat) > 0.000005 || Math.abs(prev.longitude - lng) > 0.000005;

    if (changed) {
      if (prev) {
        tracker.trajectory.push({ latitude: prev.latitude, longitude: prev.longitude });
        // Cap trajectory history to prevent memory issues
        if (tracker.trajectory.length > 500) {
          tracker.trajectory.shift();
        }
      }
      tracker.current = { latitude: lat, longitude: lng, heading };
    } else if (prev) {
      tracker.current.heading = heading;
    }

    renderCanvas();
  }, [telemetry]);

  // Redraw when grid toggle changes
  useEffect(() => {
    renderCanvas();
  }, [showGrid]);

  function handleVenueChange(e) {
    userManuallySelectedVenue.current = true;
    setSelectedVenueKey(e.target.value);
  }

  // Convert GPS (lat, lng) to canvas pixel (x, y) coordinates
  function gpsToCanvas(latitude, longitude, bounds, width, height) {
    const [[southLat, westLng], [northLat, eastLng]] = bounds;
    const xFrac = (longitude - westLng) / (eastLng - westLng);
    const yFrac = (northLat - latitude) / (northLat - southLat);

    return {
      x: xFrac * width,
      y: yFrac * height,
      inside: xFrac >= 0 && xFrac <= 1 && yFrac >= 0 && yFrac <= 1,
    };
  }

  function renderCanvas() {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // 1. Draw Background (Satellite map or fallback dark canvas)
    if (mapImageRef.current && mapImageRef.current.complete && mapImageRef.current.naturalWidth > 0) {
      ctx.drawImage(mapImageRef.current, 0, 0, width, height);
    } else {
      ctx.fillStyle = '#0a1120';
      ctx.fillRect(0, 0, width, height);

      // Loading watermark
      ctx.fillStyle = '#475569';
      ctx.font = '14px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`Loading ${activeVenue.name} Map...`, width / 2, height / 2);
    }

    // 2. Draw 5x5 Tactical Grid (if enabled)
    if (showGrid) {
      drawTacticalGrid(ctx, width, height);
    }

    const tracker = trackerRef.current;

    // 3. Draw Trajectory Line & Waypoints
    if (tracker.trajectory.length > 0) {
      const points = tracker.trajectory.map((pt) =>
        gpsToCanvas(pt.latitude, pt.longitude, activeVenue.bounds, width, height)
      );

      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; i++) {
        ctx.lineTo(points[i].x, points[i].y);
      }
      ctx.strokeStyle = 'rgba(251, 191, 36, 0.9)'; // Amber gold track
      ctx.lineWidth = 3;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();

      // Small dots for recent trajectory
      ctx.fillStyle = '#fbbf24';
      points.slice(-30).forEach((p) => {
        ctx.beginPath();
        ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // 4. Draw Current Ship Marker ONLY when real telemetry is connected
    const telemetryLat = Number(telemetry.latitude);
    const telemetryLng = Number(telemetry.longitude);
    const hasLiveGps =
      Number.isFinite(telemetryLat) &&
      Number.isFinite(telemetryLng) &&
      (telemetryLat !== 0 || telemetryLng !== 0);

    if (hasLiveGps) {
      const [[southLat, westLng], [northLat, eastLng]] = activeVenue.bounds;
      const isInside =
        telemetryLat >= southLat &&
        telemetryLat <= northLat &&
        telemetryLng >= westLng &&
        telemetryLng <= eastLng;

      const currentPos = {
        latitude: telemetryLat,
        longitude: telemetryLng,
        heading: Number(telemetry.heading) || 0,
      };

      const shipPt = gpsToCanvas(
        currentPos.latitude,
        currentPos.longitude,
        activeVenue.bounds,
        width,
        height
      );

      drawShipMarker(ctx, shipPt.x, shipPt.y, currentPos.heading, isInside);
      drawHud(ctx, activeVenue.name, currentPos, Number(telemetry.speed) || 0, true);
    } else {
      // No live boat connected - clear tracker and show standby status
      drawStandbyBanner(ctx, width, height, activeVenue.name);
      drawHud(ctx, activeVenue.name, null, 0, false);
    }
  }

  function drawStandbyBanner(ctx, width, height, venueName) {
    ctx.save();
    const cx = width / 2;
    const cy = height / 2;

    const w = 400;
    const h = 54;
    ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
    ctx.strokeStyle = 'rgba(234, 179, 8, 0.7)';
    ctx.lineWidth = 1.5;

    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(cx - w / 2, cy - h / 2, w, h, 8);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(cx - w / 2, cy - h / 2, w, h);
      ctx.strokeRect(cx - w / 2, cy - h / 2, w, h);
    }

    ctx.fillStyle = '#fbbf24';
    ctx.font = 'bold 13px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('📡 STANDBY — NO SHIP TELEMETRY RECEIVED', cx, cy - 8);

    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px Inter, sans-serif';
    ctx.fillText(`Venue: ${venueName} (Waiting for boat / simulator connection)`, cx, cy + 12);
    ctx.restore();
  }

  function drawShipMarker(ctx, x, y, heading, isInside) {
    ctx.save();
    ctx.translate(x, y);

    // 1. Radar Pulse Beacon Ring (Makes the vessel immediately visible on any map)
    ctx.save();
    ctx.beginPath();
    ctx.arc(0, 0, 32, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(56, 189, 248, 0.2)';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(0, 0, 32, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.85)';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(0, 0, 18, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)';
    ctx.lineWidth = 1;
    ctx.setLineDash([]);
    ctx.stroke();
    ctx.restore();

    // 2. Heading Vector Beam (projects 50px forward)
    ctx.save();
    ctx.rotate((heading * Math.PI) / 180);
    ctx.beginPath();
    ctx.moveTo(0, -22);
    ctx.lineTo(0, -55);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([4, 4]);
    ctx.stroke();

    // 3. Catamaran ASV Boat Hull (Dagozilla twin-hull USV)
    ctx.shadowColor = 'rgba(0, 0, 0, 0.85)';
    ctx.shadowBlur = 10;

    const hullColor = isInside ? '#ef4444' : '#eab308'; // Red inside, Gold if simulated/standby

    // Port Pontoon (Left hull)
    ctx.fillStyle = hullColor;
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-12, -20);
    ctx.quadraticCurveTo(-15, -10, -15, 14);
    ctx.lineTo(-8, 14);
    ctx.quadraticCurveTo(-8, -10, -10, -20);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Starboard Pontoon (Right hull)
    ctx.beginPath();
    ctx.moveTo(10, -20);
    ctx.quadraticCurveTo(8, -10, 8, 14);
    ctx.lineTo(15, 14);
    ctx.quadraticCurveTo(15, -10, 12, -20);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Cross-deck platform & Electronics Box
    ctx.beginPath();
    ctx.rect(-10, -5, 20, 14);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Center Cabin / GPS Dome
    ctx.beginPath();
    ctx.arc(0, 2, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#38bdf8';
    ctx.fill();

    // Bow Directional Arrow
    ctx.beginPath();
    ctx.moveTo(0, -16);
    ctx.lineTo(5, -6);
    ctx.lineTo(-5, -6);
    ctx.closePath();
    ctx.fillStyle = '#fbbf24';
    ctx.fill();

    // Stern Thruster Wash
    ctx.fillStyle = '#60a5fa';
    ctx.beginPath();
    ctx.arc(-11.5, 17, 2.5, 0, Math.PI * 2);
    ctx.arc(11.5, 17, 2.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore(); // restore heading rotation

    // 4. Ship Label Badge
    ctx.save();
    ctx.font = 'bold 11px Inter, sans-serif';
    const label = '⛵ DAGOZILLA ASV';
    const textWidth = ctx.measureText(label).width;
    const badgeX = -textWidth / 2 - 8;
    const badgeY = 36;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.8)';
    ctx.lineWidth = 1.5;
    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(badgeX, badgeY, textWidth + 16, 20, 5);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(badgeX, badgeY, textWidth + 16, 20);
      ctx.strokeRect(badgeX, badgeY, textWidth + 16, 20);
    }

    ctx.fillStyle = '#f8fafc';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, 0, badgeY + 10);
    ctx.restore();

    ctx.restore();
  }

  function drawTacticalGrid(ctx, width, height) {
    const cols = 5;
    const rows = 5;
    const cellW = width / cols;
    const cellH = height / rows;

    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
    ctx.lineWidth = 1;

    for (let i = 1; i < cols; i++) {
      ctx.beginPath();
      ctx.moveTo(i * cellW, 0);
      ctx.lineTo(i * cellW, height);
      ctx.stroke();
    }
    for (let i = 1; i < rows; i++) {
      ctx.beginPath();
      ctx.moveTo(0, i * cellH);
      ctx.lineTo(width, i * cellH);
      ctx.stroke();
    }

    // Column labels A-E at bottom
    ctx.font = 'bold 15px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ['A', 'B', 'C', 'D', 'E'].forEach((label, index) => {
      const cx = index * cellW + cellW / 2;
      const cy = height - 8;
      ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
      ctx.fillRect(cx - 14, cy - 20, 28, 22);
      ctx.fillStyle = '#38bdf8';
      ctx.fillText(label, cx, cy);
    });

    // Row labels 5-1 at left
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let i = 0; i < rows; i++) {
      const label = (5 - i).toString();
      const cx = 16;
      const cy = i * cellH + cellH / 2;
      ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
      ctx.fillRect(cx - 12, cy - 11, 24, 22);
      ctx.fillStyle = '#38bdf8';
      ctx.fillText(label, cx, cy);
    }

    ctx.restore();
  }

  function drawHud(ctx, venueName, coords, speed, isConnected) {
    ctx.save();
    ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
    ctx.strokeStyle = isConnected ? 'rgba(56, 189, 248, 0.8)' : 'rgba(234, 179, 8, 0.6)';
    ctx.lineWidth = 1;

    // Small rounded container
    const x = 12;
    const y = 12;
    const w = 230;
    const h = 56;
    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(x, y, w, h, 6);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(x, y, w, h);
      ctx.strokeRect(x, y, w, h);
    }

    ctx.fillStyle = '#38bdf8';
    ctx.font = 'bold 11px Inter, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(`📍 ${venueName.toUpperCase()}`, x + 10, y + 18);

    if (isConnected && coords) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px Roboto Mono, monospace';
      ctx.fillText(
        `${coords.latitude.toFixed(6)}°, ${coords.longitude.toFixed(6)}°`,
        x + 10,
        y + 34
      );
      ctx.fillText(
        `HDG: ${coords.heading.toFixed(0)}°  |  SPD: ${speed.toFixed(1)} m/s`,
        x + 10,
        y + 48
      );
    } else {
      ctx.fillStyle = '#fbbf24';
      ctx.font = '10px Roboto Mono, monospace';
      ctx.fillText('STATUS: NO GPS FIX', x + 10, y + 34);
      ctx.fillStyle = '#64748b';
      ctx.fillText('AWAITING TELEMETRY...', x + 10, y + 48);
    }

    ctx.restore();
  }

  return (
    <div className="center-col">
      {/* Top Menu Bar */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: '12px',
          background: 'var(--panel-bg)',
          padding: '8px 12px',
          borderRadius: '4px',
          fontSize: '0.85rem',
        }}
      >
        {/* Venue Selector */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
          <span style={{ color: 'var(--text-muted)' }}>Venue:</span>
          <select
            value={selectedVenueKey}
            onChange={handleVenueChange}
            style={{
              background: '#1e293b',
              color: '#f8fafc',
              border: '1px solid #334155',
              borderRadius: '4px',
              padding: '2px 8px',
              fontSize: '0.8rem',
              cursor: 'pointer',
            }}
          >
            {Object.values(VENUES).map((v) => (
              <option key={v.id} value={v.id}>
                {v.name}
              </option>
            ))}
          </select>
        </div>

        {/* Grid Toggle */}
        <button
          onClick={() => setShowGrid((prev) => !prev)}
          style={{
            background: showGrid ? '#0284c7' : '#334155',
            color: '#fff',
            border: 'none',
            borderRadius: '4px',
            padding: '2px 8px',
            fontSize: '0.75rem',
            cursor: 'pointer',
          }}
        >
          Grid: {showGrid ? 'ON' : 'OFF'}
        </button>

        {/* Existing Track Info */}
        <div style={{ marginLeft: 'auto', display: 'flex', alignItems: 'center', gap: '8px' }}>
          <span style={{ color: 'var(--text-muted)' }}>Current Track:</span>
          <span
            style={{
              background: 'var(--dago-blue)',
              padding: '1px 8px',
              borderRadius: '2px',
              color: '#fff',
              fontWeight: 600,
            }}
          >
            A
          </span>
          <span
            style={{
              background: '#334155',
              padding: '1px 8px',
              borderRadius: '2px',
              color: '#94a3b8',
            }}
          >
            B
          </span>

          <span style={{ marginLeft: '12px', color: 'var(--text-muted)' }}>Floating Ball Set:</span>
          <select
            style={{
              background: '#334155',
              color: '#fff',
              border: 'none',
              borderRadius: '2px',
              padding: '2px 6px',
            }}
          >
            <option>10</option>
          </select>
        </div>
      </div>

      {/* Canvas Georeferenced Map */}
      <div className="map-container">
        <canvas
          ref={canvasRef}
          width={1280}
          height={720}
          className="map-canvas"
          style={{ width: '100%', height: '100%', display: 'block', objectFit: 'contain' }}
        />
      </div>

      {/* Telemetry Table */}
      <div className="panel table-container">
        <div className="panel-content" style={{ padding: 0 }}>
          <table className="telemetry-table">
            <thead>
              <tr>
                <th>TIME</th>
                <th>LATITUDE</th>
                <th>LONGITUDE</th>
                <th>HDG</th>
                <th>SPD</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>LIVE</td>
                <td>{telemetry.latitude != null ? Number(telemetry.latitude).toFixed(6) : '--'}</td>
                <td>{telemetry.longitude != null ? Number(telemetry.longitude).toFixed(6) : '--'}</td>
                <td>{telemetry.heading != null && telemetry.latitude != null ? `${Number(telemetry.heading).toFixed(0)}°` : '--'}</td>
                <td>{telemetry.speed != null && telemetry.latitude != null ? `${Number(telemetry.speed).toFixed(1)} m/s` : '--'}</td>
              </tr>
              <tr style={{ color: 'var(--text-muted)' }}>
                <td>VENUE CTR</td>
                <td>{activeVenue.defaultCenter[0].toFixed(6)}</td>
                <td>{activeVenue.defaultCenter[1].toFixed(6)}</td>
                <td>-</td>
                <td>-</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}

