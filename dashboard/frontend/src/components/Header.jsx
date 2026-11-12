import { Wifi, Activity } from 'lucide-react';

export default function Header({ wsStatus }) {
  const isConnected = wsStatus === 'CONNECTED';
  
  return (
    <header className="header">
      <div className="header-logo">
        <img src="/logo.png" alt="Dagozilla Logo" style={{ height: '32px', width: '32px', objectFit: 'contain' }} />
        <span>DAGOZILLA</span>
        <span style={{color: '#94a3b8', fontSize: '0.8rem', marginLeft: '10px'}}>Institut Teknologi Bandung</span>
      </div>
      
      <div className="flex-between" style={{gap: '20px'}}>
        <div className="font-mono text-muted" style={{fontSize: '0.85rem'}}>
          GCS: 127.0.0.1 | MAVLink: Active
        </div>
        <div className="header-status" style={{
            borderColor: isConnected ? 'var(--success)' : 'var(--danger)',
            color: isConnected ? 'var(--success)' : 'var(--danger)',
            background: isConnected ? 'rgba(34, 197, 94, 0.1)' : 'rgba(239, 68, 68, 0.1)'
        }}>
          <div className="status-dot" style={{
              backgroundColor: isConnected ? 'var(--success)' : 'var(--danger)',
              boxShadow: `0 0 8px ${isConnected ? 'var(--success)' : 'var(--danger)'}`
          }}></div>
          {isConnected ? 'MONITORING ACTIVE' : 'DISCONNECTED'}
          <Wifi size={16} />
        </div>
      </div>
    </header>
  )
}
