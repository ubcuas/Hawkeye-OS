import './ConnectionControls.css'

interface ConnectionControlsProps {
  isConnected: boolean
  isConnecting: boolean
  onConnect: () => void
  onDisconnect: () => void
}

function ConnectionControls({
  isConnected,
  isConnecting,
  onConnect,
  onDisconnect,
}: ConnectionControlsProps) {
  return (
    <div className="connection-controls">
      <h2>Connection</h2>

      {!isConnected ? (
        <button
          className={`btn btn-connect ${isConnecting ? 'connecting' : ''}`}
          onClick={onConnect}
          disabled={isConnecting}
        >
          {isConnecting ? (
            <>
              <span className="btn-spinner" />
              Connecting...
            </>
          ) : (
            <>
              <span className="btn-icon">⚡</span>
              Connect to Server
            </>
          )}
        </button>
      ) : (
        <button
          className="btn btn-disconnect"
          onClick={onDisconnect}
          disabled={isConnecting}
        >
          <span className="btn-icon">✕</span>
          Disconnect
        </button>
      )}

      <div className="info-box">
        <p className="info-text">
          {isConnecting
            ? 'Establishing connection to signaling server...'
            : isConnected
              ? 'Connected to signaling server'
              : 'Click the button above to connect'}
        </p>
      </div>
    </div>
  )
}

export default ConnectionControls
