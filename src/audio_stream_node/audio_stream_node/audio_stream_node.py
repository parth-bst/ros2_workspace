#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from audio_msgs.msg import Audio
import numpy as np
import pyaudio
import threading
import time
import struct
from typing import Optional

class AudioStreamNode(Node):
    """
    ROS2 node for streaming audio data from re-speaker to remote TensorFlow processing.
    Runs on Raspberry Pi - lightweight audio capture and streaming only.
    """
    
    def __init__(self):
        super().__init__('audio_stream_node')
        
        # Audio configuration for re-speaker
        self.RESPEAKER_RATE = 16000
        self.RESPEAKER_CHANNELS = 1  # Mono for wake word detection
        self.RESPEAKER_WIDTH = 2     # 16-bit
        self.RESPEAKER_INDEX = 1     # Adjust based on your device
        self.CHUNK = 1024
        self.STREAM_DURATION = 1.0   # Stream 1 second chunks
        
        # Create publisher for raw audio data
        self.audio_publisher = self.create_publisher(
            Audio,
            '/audio_stream',
            10
        )
        
        # Create publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            '/audio_stream_status',
            10
        )
        
        # Create subscriber for wake word results (from MacBook)
        self.wake_word_subscription = self.create_subscription(
            String,
            '/wake_word_detected',
            self.wake_word_callback,
            10
        )
        
        # Create subscriber for LLM responses (from MacBook)
        self.llm_response_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        self.get_logger().info('🎤 Audio Stream Node initialized')
        self.get_logger().info(f'Audio config: {self.RESPEAKER_RATE}Hz, {self.RESPEAKER_CHANNELS}ch, {self.RESPEAKER_WIDTH*8}-bit')
        self.get_logger().info(f'Streaming to: /audio_stream')
        self.get_logger().info(f'Listening for responses on: /wake_word_detected, /llm_response')
        
        # Initialize audio streaming
        self.audio_interface = None
        self.audio_stream = None
        self.streaming_active = False
        
        # Start audio streaming
        self.streaming_thread = threading.Thread(target=self._audio_streaming_loop)
        self.streaming_thread.daemon = True
        self.streaming_thread.start()
    
    def _initialize_audio(self):
        """Initialize PyAudio interface and stream."""
        try:
            self.audio_interface = pyaudio.PyAudio()
            
            self.audio_stream = self.audio_interface.open(
                format=self.audio_interface.get_format_from_width(self.RESPEAKER_WIDTH),
                channels=self.RESPEAKER_CHANNELS,
                rate=self.RESPEAKER_RATE,
                input=True,
                input_device_index=self.RESPEAKER_INDEX,
                frames_per_buffer=self.CHUNK
            )
            
            self.streaming_active = True
            self.get_logger().info('✅ Audio stream initialized successfully')
            
            # Publish status update
            status_msg = String()
            status_msg.data = "Audio streaming started"
            self.status_publisher.publish(status_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize audio: {e}')
            self.streaming_active = False
            return False
    
    def _audio_streaming_loop(self):
        """Main audio streaming loop."""
        self.get_logger().info('🎵 Starting audio streaming loop...')
        
        # Initialize audio
        if not self._initialize_audio():
            self.get_logger().error('Failed to initialize audio, exiting streaming loop')
            return
        
        while rclpy.ok() and self.streaming_active:
            try:
                # Read audio data
                audio_data = self.audio_stream.read(
                    self.CHUNK, 
                    exception_on_overflow=False
                )
                
                # Convert to numpy array for processing
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                
                # Create ROS2 Audio message
                audio_msg = Audio()
                audio_msg.header.stamp = self.get_clock().now().to_msg()
                audio_msg.header.frame_id = "respeaker_mic"
                
                # Set audio parameters
                audio_msg.sample_rate = self.RESPEAKER_RATE
                audio_msg.channels = self.RESPEAKER_CHANNELS
                audio_msg.sample_format = 2  # 2 = S16LE (Signed 16-bit Little Endian)
                audio_msg.bitrate = self.RESPEAKER_RATE * self.RESPEAKER_CHANNELS * 16
                audio_msg.coding_format = 1  # 1 = PCM
                
                # Pack audio data as bytes
                audio_bytes = struct.pack(f'{len(audio_array)}h', *audio_array)
                audio_msg.data = list(audio_bytes)
                
                # Publish audio data
                self.audio_publisher.publish(audio_msg)
                
                # Brief pause to prevent overwhelming the network
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f'Error in audio streaming: {e}')
                time.sleep(0.1)
        
        # Cleanup
        self._cleanup_audio()
    
    def _cleanup_audio(self):
        """Clean up audio resources."""
        try:
            if self.audio_stream:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
            
            if self.audio_interface:
                self.audio_interface.terminate()
                
            self.streaming_active = False
            self.get_logger().info('🧹 Audio resources cleaned up')
            
        except Exception as e:
            self.get_logger().error(f'Error cleaning up audio: {e}')
    
    def wake_word_callback(self, msg):
        """Handle wake word detection results from MacBook."""
        self.get_logger().info(f'🎯 Wake word detected: {msg.data}')
        
        # Here you could trigger local actions on the Pi:
        # - Flash LEDs
        # - Play confirmation sound
        # - Update status display
        # - etc.
    
    def llm_response_callback(self, msg):
        """Handle LLM responses from MacBook."""
        self.get_logger().info(f'🤖 LLM Response: {msg.data}')
        
        # Here you could trigger local actions on the Pi:
        # - Play audio response through speakers
        # - Display text on screen
        # - Control robot movements
        # - etc.
    
    def destroy_node(self):
        """Clean shutdown."""
        self.streaming_active = False
        self._cleanup_audio()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    audio_stream_node = AudioStreamNode()
    
    try:
        rclpy.spin(audio_stream_node)
    except KeyboardInterrupt:
        audio_stream_node.get_logger().info('🎤 Audio stream node shutting down...')
    finally:
        audio_stream_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
