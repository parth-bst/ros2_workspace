#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_msgs.msg import Audio
import numpy as np
from scipy import signal
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        import tensorflow as tf
        Interpreter = tf.lite.Interpreter
        print("Using TensorFlow Lite instead of tflite-runtime")
    except ImportError:
        print("Neither tflite-runtime nor TensorFlow found. Please install one of them.")
        exit(1)
import threading
import time
import os
import tempfile
from typing import Optional
import struct
import pyaudio
import queue

class AudioProcessingNode(Node):
    """
    ROS2 node for processing audio streams and running TensorFlow Lite wake word detection.
    Runs on MacBook - handles the heavy TensorFlow processing.
    """
    
    def __init__(self):
        super().__init__('audio_processing_node')
        
        # Get the path to the model file
        # When running as installed package, the model is in the share directory
        # When running from source, the model is in the source resources directory
        
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Check if we're running from installed package (has 'site-packages' in path)
        if 'site-packages' in current_file_dir:
            # Running from installed package: audio_processing_node is in lib/python3.x/site-packages/audio_processing_node/
            # Model is in share/audio_processing_node/resource/
            package_share_dir = current_file_dir.replace('lib/python3.12/site-packages/audio_processing_node', 'share/audio_processing_node')
            self.MODEL_PATH = os.path.join(package_share_dir, 'resource', 'model.tflite')
        else:
            # Running from source: model is in ../resource/ relative to current file
            self.MODEL_PATH = os.path.join(os.path.dirname(current_file_dir), 'resource', 'model.tflite')
        
        # Model configuration
        self.LABELS = ['down', 'go', 'left', 'no', 'right', 'stop', 'up', 'yes']
        self.CONFIDENCE_THRESHOLD = 0.3  # Minimum confidence for detection
        
        # Audio buffer for processing (optimized for low latency)
        self.audio_buffer = []
        self.buffer_lock = threading.Lock()
        self.buffer_size = 1600  # 100ms at 16kHz (reduced from 1 second)
        self.processing_interval = 0.1  # Process every 100ms (reduced from 500ms)
        
        # Audio playback configuration (optimized for minimal latency)
        self.playback_queue = queue.Queue(maxsize=10)  # Small buffer for minimal latency
        self.playback_active = False
        self.audio_interface = None
        self.playback_stream = None
        
        # Low-latency mode flags
        self.low_latency_mode = True
        self.playback_priority = True  # Prioritize playback over processing
        
        # Initialize audio playback
        self._initialize_audio_playback()
        
        # Check if model file exists
        if not os.path.exists(self.MODEL_PATH):
            self.get_logger().error(f'Model file not found at: {self.MODEL_PATH}')
            return
        
        # Initialize TensorFlow Lite interpreter
        try:
            self.interpreter = Interpreter(self.MODEL_PATH)
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            self.get_logger().info('✅ TensorFlow Lite model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load TensorFlow Lite model: {e}')
            return
        
        # Create subscriber for audio stream (from Pi)
        self.audio_subscription = self.create_subscription(
            Audio,
            '/audio_stream',
            self.audio_callback,
            10
        )
        
        # Create publishers for results
        self.wake_word_publisher = self.create_publisher(
            String,
            '/wake_word_detected',
            10
        )
        
        self.llm_input_publisher = self.create_publisher(
            String,
            '/llm_input',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/audio_processing_status',
            10
        )
        
        # Create subscriber for LLM responses (optional)
        self.llm_response_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        self.get_logger().info('🧠 Audio Processing Node initialized (LOW-LATENCY MODE)')
        self.get_logger().info(f'Listening for keywords: {", ".join(self.LABELS)}')
        self.get_logger().info(f'Model path: {self.MODEL_PATH}')
        self.get_logger().info(f'Subscribing to: /audio_stream (from Pi, Domain 0)')
        self.get_logger().info(f'Publishing to: /wake_word_detected, /llm_input')
        self.get_logger().info(f'🔊 Low-latency audio playback: MacBook speakers')
        self.get_logger().info(f'⚡ Buffer size: {self.buffer_size} samples ({self.buffer_size/16000*1000:.0f}ms)')
        self.get_logger().info(f'⚡ Playback chunk: {self.PLAYBACK_CHUNK} samples ({self.PLAYBACK_CHUNK/16000*1000:.1f}ms)')
        self.get_logger().info(f'⚡ Processing interval: {self.processing_interval*1000:.0f}ms')
        
        # Start audio processing loop
        self.processing_thread = threading.Thread(target=self._audio_processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Start audio playback loop
        self.playback_thread = threading.Thread(target=self._audio_playback_loop)
        self.playback_thread.daemon = True
        self.playback_thread.start()
        
        # Publish initial status
        self._publish_status("Audio processing node started")
    
    def audio_callback(self, msg):
        """Handle incoming audio stream from Pi - optimized for minimal latency."""
        try:
            # Convert ROS Audio message to numpy array
            audio_bytes = bytes(msg.data)
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            
            # PRIORITY 1: Immediate playback (non-blocking, highest priority)
            if self.playback_active:
                try:
                    self.playback_queue.put_nowait(audio_bytes)
                except queue.Full:
                    # If queue is full, drop oldest audio to maintain real-time playback
                    try:
                        self.playback_queue.get_nowait()  # Remove oldest
                        self.playback_queue.put_nowait(audio_bytes)  # Add new
                    except queue.Empty:
                        pass
            
            # PRIORITY 2: Processing buffer (only if not in low-latency mode or if buffer is small)
            if not self.low_latency_mode or len(self.audio_buffer) < self.buffer_size:
                with self.buffer_lock:
                    self.audio_buffer.extend(audio_array)
                    
                    # Keep buffer size small for low latency
                    if len(self.audio_buffer) > self.buffer_size * 1.5:
                        self.audio_buffer = self.audio_buffer[-self.buffer_size:]
            
            # Reduced debug frequency for performance
            if len(self.audio_buffer) % 8000 == 0:  # Every 0.5 seconds
                self.get_logger().debug(f'📊 Audio buffer: {len(self.audio_buffer)} samples, Queue: {self.playback_queue.qsize()}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing audio stream: {e}')
    
    def _audio_processing_loop(self):
        """Main audio processing loop - optimized for low-latency mode."""
        self.get_logger().info('🔄 Starting audio processing loop (low-latency mode)...')
        
        while rclpy.ok():
            try:
                # In low-latency mode, reduce processing frequency to prioritize playback
                if self.low_latency_mode:
                    time.sleep(self.processing_interval)  # Sleep first to prioritize playback
                
                # Check if we have enough audio data
                with self.buffer_lock:
                    if len(self.audio_buffer) >= self.buffer_size:
                        # Get audio chunk for processing
                        audio_chunk = np.array(self.audio_buffer[:self.buffer_size], dtype=np.float32)
                        # Normalize to [-1, 1]
                        audio_chunk = audio_chunk / 32768.0
                        
                        # Remove processed chunk from buffer
                        self.audio_buffer = self.audio_buffer[self.buffer_size:]
                    else:
                        audio_chunk = None
                
                if audio_chunk is not None:
                    # Process audio chunk
                    command, confidence = self._recognize_speech_from_buffer(audio_chunk)
                    
                    if command and confidence > self.CONFIDENCE_THRESHOLD:
                        self.get_logger().info(f'🎯 Wake word detected: {command.upper()} (confidence: {confidence:.2%})')
                        
                        # Publish wake word detection
                        wake_msg = String()
                        wake_msg.data = command.upper()
                        self.wake_word_publisher.publish(wake_msg)
                        
                        # Publish to LLM input (same as original wake word node)
                        llm_msg = String()
                        llm_msg.data = f"User said: {command}"
                        self.llm_input_publisher.publish(llm_msg)
                        
                        # Update status
                        self._publish_status(f"Wake word detected: {command.upper()}")
                
                # Adaptive sleep based on mode
                if not self.low_latency_mode:
                    time.sleep(self.processing_interval)
                
            except Exception as e:
                self.get_logger().error(f'Error in audio processing loop: {e}')
                time.sleep(0.1)  # Reduced error sleep time
    
    def _recognize_speech_from_buffer(self, audio_data):
        """Run inference on audio buffer."""
        try:
            # Convert audio to spectrogram
            spectrogram = self._get_spectrogram(audio_data)
            spectrogram = spectrogram[..., np.newaxis]
            spectrogram = spectrogram[np.newaxis, ...]
            
            # Run inference
            self.interpreter.set_tensor(
                self.input_details[0]['index'], spectrogram.astype(np.float32))
            self.interpreter.invoke()
            
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            prediction = np.argmax(output_data)
            
            # Calculate confidence using softmax
            exp_output = np.exp(output_data - np.max(output_data))  # Numerical stability
            confidence_scores = exp_output / np.sum(exp_output)
            max_confidence = confidence_scores[prediction]
            
            return self.LABELS[prediction], max_confidence
                
        except Exception as e:
            self.get_logger().error(f'Speech recognition failed: {e}')
            return None, 0.0
    
    def _get_spectrogram(self, waveform, expected_time_steps=124, expected_freq_bins=129):
        """Convert audio waveform to spectrogram."""
        _, _, Zxx = signal.stft(
            waveform,
            fs=16000,
            nperseg=255,
            noverlap=124,
            nfft=256
        )
        spectrogram = np.abs(Zxx)
        
        # Pad or truncate to expected dimensions
        if spectrogram.shape[0] != expected_freq_bins:
            spectrogram = np.pad(spectrogram, ((
                0, expected_freq_bins - spectrogram.shape[0]), (0, 0)), mode='constant')
        
        if spectrogram.shape[1] != expected_time_steps:
            spectrogram = np.pad(spectrogram, ((
                0, 0), (0, expected_time_steps - spectrogram.shape[1])), mode='constant')
        
        spectrogram = np.transpose(spectrogram)
        return spectrogram
    
    def _publish_status(self, status_message):
        """Publish status update."""
        try:
            status_msg = String()
            status_msg.data = status_message
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')
    
    def llm_response_callback(self, msg):
        """Handle responses from LLM node."""
        self.get_logger().info(f'🤖 LLM Response: {msg.data}')
        self._publish_status(f"LLM response received: {msg.data[:50]}...")
    
    def _initialize_audio_playback(self):
        """Initialize audio playback system for MacBook speakers - optimized for minimal latency."""
        try:
            self.audio_interface = pyaudio.PyAudio()
            
            # Audio playback configuration optimized for minimal latency
            self.PLAYBACK_RATE = 16000
            self.PLAYBACK_CHANNELS = 1  # Mono
            self.PLAYBACK_WIDTH = 2     # 16-bit
            self.PLAYBACK_CHUNK = 256   # Smaller chunk size for lower latency (reduced from 1024)
            
            # Configure for low-latency audio
            self.playback_stream = self.audio_interface.open(
                format=self.audio_interface.get_format_from_width(self.PLAYBACK_WIDTH),
                channels=self.PLAYBACK_CHANNELS,
                rate=self.PLAYBACK_RATE,
                output=True,
                frames_per_buffer=self.PLAYBACK_CHUNK,
                stream_callback=None,  # No callback for direct control
                start=False  # Start manually for better control
            )
            
            # Start the stream
            self.playback_stream.start_stream()
            self.playback_active = True
            
            self.get_logger().info(f'🔊 Low-latency audio playback initialized (chunk: {self.PLAYBACK_CHUNK})')
            self._publish_status("Low-latency audio playback initialized")
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize audio playback: {e}')
            self.playback_active = False
    
    def _audio_playback_loop(self):
        """Main audio playback loop - optimized for minimal latency."""
        self.get_logger().info('🔊 Starting low-latency audio playback loop...')
        
        # Set thread priority for real-time audio (if possible)
        try:
            import threading
            # Note: Python threading priority is limited, but we can set daemon=True
            current_thread = threading.current_thread()
            current_thread.daemon = True
        except:
            pass
        
        while rclpy.ok() and self.playback_active:
            try:
                # Get audio data from queue with minimal timeout for real-time playback
                try:
                    audio_data = self.playback_queue.get(timeout=0.01)  # 10ms timeout (reduced from 100ms)
                    
                    # Immediate playback - no buffering delays
                    if self.playback_stream and audio_data:
                        try:
                            self.playback_stream.write(audio_data, exception_on_underflow=False)
                        except Exception as stream_error:
                            # Handle underflow gracefully to maintain real-time playback
                            if "Input overflowed" not in str(stream_error):
                                self.get_logger().debug(f'Playback stream issue: {stream_error}')
                        
                except queue.Empty:
                    # No audio data - continue immediately (no sleep for minimal latency)
                    continue
                    
            except Exception as e:
                self.get_logger().error(f'Error in audio playback: {e}')
                time.sleep(0.001)  # Minimal sleep on error (1ms)
        
        # Cleanup
        self._cleanup_audio_playback()
    
    def _cleanup_audio_playback(self):
        """Clean up audio playback resources."""
        try:
            self.playback_active = False
            
            if self.playback_stream:
                self.playback_stream.stop_stream()
                self.playback_stream.close()
            
            if self.audio_interface:
                self.audio_interface.terminate()
                
            self.get_logger().info('🧹 Audio playback resources cleaned up')
            
        except Exception as e:
            self.get_logger().error(f'Error cleaning up audio playback: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('🧠 Audio processing node shutting down...')
        self._cleanup_audio_playback()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    audio_processing_node = AudioProcessingNode()
    
    try:
        rclpy.spin(audio_processing_node)
    except KeyboardInterrupt:
        audio_processing_node.get_logger().info('🧠 Audio processing node shutting down...')
    finally:
        audio_processing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
