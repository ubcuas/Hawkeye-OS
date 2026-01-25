import { useRef, useEffect } from 'react'
import './VideoDisplay.css'

interface VideoDisplayProps {
  stream: MediaStream | null
}

function VideoDisplay({ stream }: VideoDisplayProps) {
  const videoRef = useRef<HTMLVideoElement>(null)

  useEffect(() => {
    if (videoRef.current && stream) {
      videoRef.current.srcObject = stream
    }
  }, [stream])

  return (
    <div className="video-display">
      <div className="video-container">
        <video
          ref={videoRef}
          className="video-element"
          autoPlay
          playsInline
          muted
        />
        {!stream && (
          <div className="video-overlay">
            <div className="video-placeholder">
              <svg
                className="camera-icon"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
              >
                <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z" />
                <circle cx="12" cy="13" r="4" />
              </svg>
              <p>Waiting for video stream...</p>
            </div>
          </div>
        )}
      </div>
      <div className="video-info">
        <span className="resolution-text">
          {stream ? 'Streaming video' : 'Ready for video stream'}
        </span>
      </div>
    </div>
  )
}

export default VideoDisplay
