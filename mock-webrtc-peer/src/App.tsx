import './App.css'
import StatusPanel from './components/StatusPanel'
import VideoDisplay from './components/VideoDisplay'
import ConnectionControls from './components/ConnectionControls'
import { useWebRTCConnection } from './hooks/useWebRTCConnection'

function App() {
  const {
    signalingStatus,
    peerStatus,
    remoteStream,
    connect,
    disconnect,
    isConnecting,
  } = useWebRTCConnection()

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>WebRTC Peer</h1>
        <p className="subtitle">Real-time video streaming</p>
      </header>

      <main className="app-main">
        <div className="content-wrapper">
          <div className="video-section">
            <VideoDisplay stream={remoteStream} />
          </div>

          <div className="control-section">
            <StatusPanel
              signalingStatus={signalingStatus}
              peerStatus={peerStatus}
            />

            <ConnectionControls
              isConnected={signalingStatus === 'connected'}
              isConnecting={isConnecting}
              onConnect={connect}
              onDisconnect={disconnect}
            />
          </div>
        </div>
      </main>
    </div>
  )
}

export default App
