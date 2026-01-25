import './StatusPanel.css'

type SignalingStatus = 'disconnected' | 'connecting' | 'connected'
type PeerStatus = 'disconnected' | 'connecting' | 'connected' | 'failed'

interface StatusPanelProps {
  signalingStatus: SignalingStatus
  peerStatus: PeerStatus
}

function StatusPanel({ signalingStatus, peerStatus }: StatusPanelProps) {
  const getStatusColor = (status: SignalingStatus | PeerStatus): string => {
    switch (status) {
      case 'connected':
        return 'status-success'
      case 'connecting':
        return 'status-pending'
      case 'failed':
        return 'status-error'
      default:
        return 'status-inactive'
    }
  }

  const getStatusLabel = (status: SignalingStatus | PeerStatus): string => {
    return status.charAt(0).toUpperCase() + status.slice(1)
  }

  return (
    <div className="status-panel">
      <h2>Connection Status</h2>

      <div className="status-item">
        <div className="status-label-wrapper">
          <span className="status-label">Signaling Server</span>
          <div className={`status-indicator ${getStatusColor(signalingStatus)}`} />
        </div>
        <span className={`status-value ${getStatusColor(signalingStatus)}`}>
          {getStatusLabel(signalingStatus)}
        </span>
      </div>

      <div className="status-item">
        <div className="status-label-wrapper">
          <span className="status-label">WebRTC Peer</span>
          <div className={`status-indicator ${getStatusColor(peerStatus)}`} />
        </div>
        <span className={`status-value ${getStatusColor(peerStatus)}`}>
          {getStatusLabel(peerStatus)}
        </span>
      </div>
    </div>
  )
}

export default StatusPanel
