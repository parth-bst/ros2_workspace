#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import pyaudio
import numpy as np
import threading
import time
import struct
from collections import deque

class AudioListener(Node):
    """
    Audio consumer that listens to the /audio_stream topic and plays it through speakers.
    """
    
    def __init__(self):
        super().__init__('audio_listener')
        
        # Audio playback configuration
        self.CHUNK = 1024
        self.CHANNELS = 1
        self.RATE = 16000
        self.WIDTH = 2  # 16-bit
        
        # Create subscriber for audio stream
        self.audio_subscription = self.create_subscription(
            Audio,
            '/audio_stream',
            self.audio_callback,
            10
        )
        
        # Initialize audio playback
        self.audio_interface = None
        self.audio_stream = None
        self.playback_active = False
        
        # Audio buffer for smooth playback
        self.audio_buffer = deque(maxlen=10)  # Keep last 10 audio chunks
        
        self.get_logger().info('🎧 Audio Listener initialized')
        self.get_logger().info(f'Playback config: {self.RATE}Hz, {self.CHANNELS}ch, {self.WIDTH*8}-bit')
        self.get_logger().info(f'Listening to: /audio_stream')
        
        # Start audio playback
        self._initialize_playback()
        self.playback_thread = threading.Thread(target=self._playback_loop)
        self.playback_thread.daemon = True
        self.playback_thread.start()
    
    def _initialize_playback(self):
        """Initialize PyAudio for playback."""
        try:
            self.audio_interface = pyaudio.PyAudio()
            
            self.audio_stream = self.audio_interface.open(
                format=self.audio_interface.get_format_from_width(self.WIDTH),
                channels=self.CHANNELS,
                rate=self.RATE,
                output=True,
                frames_per_buffer=self.CHUNK
            )
            
            self.playback_active = True
            self.get_logger().info('✅ Audio playback initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize audio playback: {e}')
            self.playback_active = False
    
    def audio_callback(self, msg: Audio):
        """Handle incoming audio data from the stream."""
        try:
            # Convert the byte array back to numpy array
            audio_bytes = bytes(msg.data)
            audio_array = struct.unpack(f'{len(audio_bytes)//2}h', audio_bytes)
            
            # Add to playback buffer
            self.audio_buffer.append(audio_array)
            
            # Log some stats
            if len(self.audio_buffer) % 50 == 0:  # Log every 50 chunks
                self.get_logger().info(f'🎵 Audio buffer: {len(self.audio_buffer)} chunks, '
                                     f'Rate: {msg.sample_rate}Hz, Channels: {msg.channels}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing audio data: {e}')
    
    def _playback_loop(self):
        """Main audio playback loop."""
        self.get_logger().info('🎵 Starting audio playback loop...')
        
        if not self.playback_active:
            self.get_logger().error('Playback not initialized, exiting playback loop')
            return
        
        while rclpy.ok() and self.playback_active:
            try:
                if self.audio_buffer:
                    # Get audio data from buffer
                    audio_data = self.audio_buffer.popleft()
                    
                    # Convert to bytes for playback
                    audio_bytes = struct.pack(f'{len(audio_data)}h', *audio_data)
                    
                    # Play audio
                    self.audio_stream.write(audio_bytes)
                else:
                    # No audio data, wait a bit
                    time.sleep(0.01)
                    
            except Exception as e:
                self.get_logger().error(f'Error in audio playback: {e}')
                time.sleep(0.1)
        
        # Cleanup
        self._cleanup_playback()
    
    def _cleanup_playback(self):
        """Clean up audio resources."""
        try:
            if self.audio_stream:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
            
            if self.audio_interface:
                self.audio_interface.terminate()
                
            self.playback_active = False
            self.get_logger().info('🧹 Audio playback resources cleaned up')
            
        except Exception as e:
            self.get_logger().error(f'Error cleaning up audio playback: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.playback_active = False
        self._cleanup_playback()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    audio_listener = AudioListener()
    
    try:
        print("\n" + "="*60)
        print("🎧 AUDIO STREAM LISTENER")
        print("="*60)
        print("Listening to live audio from /audio_stream topic...")
        print("You should hear audio from your re-speaker microphone.")
        print("Press Ctrl+C to stop.")
        print("="*60 + "\n")
        
        rclpy.spin(audio_listener)
    except KeyboardInterrupt:
        audio_listener.get_logger().info('🎧 Audio listener shutting down...')
    finally:
        audio_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
