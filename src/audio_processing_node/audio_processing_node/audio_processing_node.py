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
        
        # Audio buffer for processing
        self.audio_buffer = []
        self.buffer_lock = threading.Lock()
        self.buffer_size = 16000  # 1 second at 16kHz
        self.processing_interval = 0.5  # Process every 0.5 seconds
        
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
        
        self.get_logger().info('🧠 Audio Processing Node initialized')
        self.get_logger().info(f'Listening for keywords: {", ".join(self.LABELS)}')
        self.get_logger().info(f'Model path: {self.MODEL_PATH}')
        self.get_logger().info(f'Subscribing to: /audio_stream (from Pi)')
        self.get_logger().info(f'Publishing to: /wake_word_detected, /llm_input')
        
        # Start audio processing loop
        self.processing_thread = threading.Thread(target=self._audio_processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Publish initial status
        self._publish_status("Audio processing node started")
    
    def audio_callback(self, msg):
        """Handle incoming audio stream from Pi."""
        try:
            # Convert ROS Audio message to numpy array
            # msg.data is a list of bytes, we need to unpack it as int16
            audio_bytes = bytes(msg.data)
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            
            # Add to buffer with thread safety
            with self.buffer_lock:
                self.audio_buffer.extend(audio_array)
                
                # Keep buffer size manageable
                if len(self.audio_buffer) > self.buffer_size * 2:
                    self.audio_buffer = self.audio_buffer[-self.buffer_size:]
            
            # Debug info (occasionally)
            if len(self.audio_buffer) % 16000 == 0:  # Every second
                self.get_logger().debug(f'📊 Audio buffer size: {len(self.audio_buffer)} samples')
                
        except Exception as e:
            self.get_logger().error(f'Error processing audio stream: {e}')
    
    def _audio_processing_loop(self):
        """Main audio processing loop."""
        self.get_logger().info('🔄 Starting audio processing loop...')
        
        while rclpy.ok():
            try:
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
                
                # Wait before next processing cycle
                time.sleep(self.processing_interval)
                
            except Exception as e:
                self.get_logger().error(f'Error in audio processing loop: {e}')
                time.sleep(1)
    
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
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('🧠 Audio processing node shutting down...')
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
