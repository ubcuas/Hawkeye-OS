import { useEffect, useRef, useState } from 'react';
import { io, Socket } from 'socket.io-client';

type SignalingStatus = 'disconnected' | 'connecting' | 'connected';
type PeerStatus = 'disconnected' | 'connecting' | 'connected' | 'failed';

interface SignalData {
  type: string;
  data: {
    sdp?: string;
    type?: string;
    candidate?: string;
    sdpMid?: string | null;
    sdpMLineIndex?: number | null;
  };
  from: string;
}

interface UseWebRTCConnectionResult {
  signalingStatus: SignalingStatus;
  peerStatus: PeerStatus;
  remoteStream: MediaStream | null;
  connect: () => void;
  disconnect: () => void;
  isConnecting: boolean;
}

const SIGNALING_SERVER_URL = 'ws://localhost:8081';

export function useWebRTCConnection(): UseWebRTCConnectionResult {
  const [signalingStatus, setSignalingStatus] = useState<SignalingStatus>('disconnected');
  const [peerStatus, setPeerStatus] = useState<PeerStatus>('disconnected');
  const [remoteStream, setRemoteStream] = useState<MediaStream | null>(null);
  const [isConnecting, setIsConnecting] = useState(false);

  const socketRef = useRef<Socket | null>(null);
  const peerConnectionRef = useRef<RTCPeerConnection | null>(null);
  const iceCandidateQueueRef = useRef<RTCIceCandidateInit[]>([]);
  const remotePeerIdRef = useRef<string | null>(null);

  const createPeerConnection = () => {
    const pc = new RTCPeerConnection({
      iceServers: [
        { urls: ['stun:stun.l.google.com:19302'] },
        { urls: ['stun:stun1.l.google.com:19302'] },
      ],
    });

    pc.addEventListener('icecandidate', (event) => {
      if (event.candidate && socketRef.current && remotePeerIdRef.current) {
        console.log('Sending ICE candidate to peer');
        socketRef.current.emit('signal', {
          to: remotePeerIdRef.current,
          type: 'ice-candidate',
          data: {
            candidate: event.candidate.candidate,
            sdpMid: event.candidate.sdpMid,
            sdpMLineIndex: event.candidate.sdpMLineIndex,
          },
        });
      }
    });

    pc.addEventListener('track', (event) => {
      console.log('Remote track received:', event.track.kind);
      if (event.streams && event.streams[0]) {
        console.log('Setting remote stream');
        setRemoteStream(event.streams[0]);
      }
    });

    pc.addEventListener('connectionstatechange', () => {
      console.log('Connection state:', pc.connectionState);
      if (pc.connectionState === 'connected') {
        setPeerStatus('connected');
        setIsConnecting(false);
      } else if (pc.connectionState === 'failed') {
        setPeerStatus('failed');
        setIsConnecting(false);
      } else if (pc.connectionState === 'connecting') {
        setPeerStatus('connecting');
      } else if (pc.connectionState === 'disconnected' || pc.connectionState === 'closed') {
        setPeerStatus('disconnected');
      }
    });

    pc.addEventListener('iceconnectionstatechange', () => {
      console.log('ICE connection state:', pc.iceConnectionState);
    });

    pc.addEventListener('icegatheringstatechange', () => {
      console.log('ICE gathering state:', pc.iceGatheringState);
    });

    return pc;
  };

  const handleOffer = async (offerData: { sdp: string; type: string }, from: string) => {
    try {
      console.log('Received offer from peer:', from);

      if (peerConnectionRef.current) {
        const currentState = peerConnectionRef.current.signalingState;
        console.log('Current signaling state:', currentState);

        if (currentState !== 'stable') {
          console.warn('Rejecting offer - peer connection not in stable state:', currentState);
          return;
        }

        if (remotePeerIdRef.current && remotePeerIdRef.current !== from) {
          console.warn(`Rejecting offer from different peer. Current: ${remotePeerIdRef.current}, New: ${from}`);
          return;
        }
      }

      remotePeerIdRef.current = from;

      if (!peerConnectionRef.current) {
        console.log('Creating new peer connection');
        peerConnectionRef.current = createPeerConnection();
      }

      const pc = peerConnectionRef.current;

      console.log('Setting remote description');
      await pc.setRemoteDescription(new RTCSessionDescription({
        sdp: offerData.sdp,
        type: offerData.type as RTCSdpType,
      }));

      console.log('Creating answer');
      const answer = await pc.createAnswer();
      await pc.setLocalDescription(answer);

      console.log('Sending answer to peer');
      if (socketRef.current) {
        socketRef.current.emit('signal', {
          to: from,
          type: 'answer',
          data: {
            sdp: pc.localDescription?.sdp,
            type: pc.localDescription?.type,
          },
        });
      }

      console.log('Processing queued ICE candidates:', iceCandidateQueueRef.current.length);
      while (iceCandidateQueueRef.current.length > 0) {
        const candidateData = iceCandidateQueueRef.current.shift();
        if (candidateData) {
          await addIceCandidate(candidateData);
        }
      }
    } catch (error) {
      console.error('Error handling offer:', error);
      setPeerStatus('failed');
    }
  };

  const addIceCandidate = async (candidateData: RTCIceCandidateInit) => {
    try {
      if (!candidateData.candidate) {
        console.log('Received end-of-candidates signal');
        return;
      }

      const candidate = new RTCIceCandidate({
        candidate: candidateData.candidate,
        sdpMid: candidateData.sdpMid,
        sdpMLineIndex: candidateData.sdpMLineIndex,
      });

      if (peerConnectionRef.current) {
        await peerConnectionRef.current.addIceCandidate(candidate);
        console.log('Added ICE candidate');
      }
    } catch (error) {
      console.error('Error adding ICE candidate:', error);
    }
  };

  const handleIceCandidate = async (candidateData: RTCIceCandidateInit) => {
    try {
      if (!peerConnectionRef.current || !peerConnectionRef.current.remoteDescription) {
        console.log('Queueing ICE candidate (no remote description yet)');
        iceCandidateQueueRef.current.push(candidateData);
        return;
      }

      await addIceCandidate(candidateData);
    } catch (error) {
      console.error('Error handling ICE candidate:', error);
    }
  };

  const connect = () => {
    if (socketRef.current?.connected) {
      console.log('Already connected');
      return;
    }

    setIsConnecting(true);
    setSignalingStatus('connecting');

    const socket = io(SIGNALING_SERVER_URL, {
      transports: ['websocket'],
    });

    socket.on('connect', () => {
      console.log('Connected to signaling server');
      setSignalingStatus('connected');
    });

    socket.on('disconnect', () => {
      console.log('Disconnected from signaling server');
      setSignalingStatus('disconnected');
      setPeerStatus('disconnected');
      setIsConnecting(false);
    });

    socket.on('signal', async (data: SignalData) => {
      console.log('Received signal:', data.type);

      if (data.type === 'offer' && data.data.sdp && data.data.type) {
        await handleOffer({ sdp: data.data.sdp, type: data.data.type }, data.from);
      } else if (data.type === 'ice-candidate') {
        await handleIceCandidate(data.data);
      }
    });

    socket.on('error', (error: Error | { message: string }) => {
      console.error('Signaling server error:', error);
    });

    socketRef.current = socket;
  };

  const disconnect = () => {
    if (socketRef.current) {
      socketRef.current.disconnect();
      socketRef.current = null;
    }

    if (peerConnectionRef.current) {
      peerConnectionRef.current.close();
      peerConnectionRef.current = null;
    }

    iceCandidateQueueRef.current = [];
    remotePeerIdRef.current = null;
    setRemoteStream(null);
    setSignalingStatus('disconnected');
    setPeerStatus('disconnected');
    setIsConnecting(false);
  };

  useEffect(() => {
    return () => {
      disconnect();
    };
  }, []);

  return {
    signalingStatus,
    peerStatus,
    remoteStream,
    connect,
    disconnect,
    isConnecting,
  };
}
