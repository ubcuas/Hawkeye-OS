import { useRef, useEffect } from 'react'
import './VideoDisplay.css'

interface VideoDisplayProps {
  stream: MediaStream | null
}

function VideoDisplay({ stream }: VideoDisplayProps) {
  const videoRef = useRef<HTMLVideoElement>(null)

  useEffect(() => {
    if (videoRef.current && stream) {
      console.log('VideoDisplay: Setting stream to video element');
      console.log('Stream active:', stream.active);
      console.log('Stream tracks:', stream.getTracks().map(t => ({
        kind: t.kind,
        enabled: t.enabled,
        muted: t.muted,
        readyState: t.readyState,
      })));

      videoRef.current.srcObject = stream;

      const videoElement = videoRef.current;

      const handleLoadedMetadata = () => {
        console.log('Video metadata loaded');
        console.log('Video dimensions:', {
          videoWidth: videoElement.videoWidth,
          videoHeight: videoElement.videoHeight,
        });
      };

      const handleLoadedData = () => {
        console.log('Video data loaded');
      };

      const handleCanPlay = () => {
        console.log('Video can play');
      };

      const handlePlaying = () => {
        console.log('Video is playing');
      };

      const handleWaiting = () => {
        console.log('Video is waiting for data');
      };

      const handleStalled = () => {
        console.log('Video stalled');
      };

      const handleError = (e: Event) => {
        console.error('Video error:', e);
        if (videoElement.error) {
          console.error('Video error details:', {
            code: videoElement.error.code,
            message: videoElement.error.message,
          });
        }
      };

      videoElement.addEventListener('loadedmetadata', handleLoadedMetadata);
      videoElement.addEventListener('loadeddata', handleLoadedData);
      videoElement.addEventListener('canplay', handleCanPlay);
      videoElement.addEventListener('playing', handlePlaying);
      videoElement.addEventListener('waiting', handleWaiting);
      videoElement.addEventListener('stalled', handleStalled);
      videoElement.addEventListener('error', handleError);

      return () => {
        videoElement.removeEventListener('loadedmetadata', handleLoadedMetadata);
        videoElement.removeEventListener('loadeddata', handleLoadedData);
        videoElement.removeEventListener('canplay', handleCanPlay);
        videoElement.removeEventListener('playing', handlePlaying);
        videoElement.removeEventListener('waiting', handleWaiting);
        videoElement.removeEventListener('stalled', handleStalled);
        videoElement.removeEventListener('error', handleError);
      };
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
