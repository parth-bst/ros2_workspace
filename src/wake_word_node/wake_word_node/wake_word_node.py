#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
import soundfile as sf
import threading
import time
import os
import subprocess
import tempfile
import pyaudio
import wave
from typing import Optional

class WakeWordNode(Node):
    """
    ROS2 node for wake word detection using TensorFlow Lite.
    Detects keywords: yes, no, go, left, right, up, down, stop
    """
    
    def __init__(self):
        super().__init__('wake_word_node')
        
        # Get the path to the model file
        # When running as installed package, the model is in the share directory
        # When running from source, the model is in the source resources directory
        
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Check if we're running from installed package (has 'site-packages' in path)
        if 'site-packages' in current_file_dir:
            # Running from installed package: wake_word_node is in lib/python3.x/site-packages/wake_word_node/
            # Model is in share/wake_word_node/resources/
            package_share_dir = current_file_dir.replace('lib/python3.12/site-packages/wake_word_node', 'share/wake_word_node')
            self.MODEL_PATH = os.path.join(package_share_dir, 'resources', 'model.tflite')
        else:
            # Running from source: model is in ../resources/ relative to current file
            self.MODEL_PATH = os.path.join(os.path.dirname(current_file_dir), 'resources', 'model.tflite')
        
        # Model configuration
        self.LABELS = ['down', 'go', 'left', 'no', 'right', 'stop', 'up', 'yes']
        
        # Audio configuration from record.py
        self.RESPEAKER_RATE = 16000
        self.RESPEAKER_CHANNELS = 2 
        self.RESPEAKER_WIDTH = 2
        self.RESPEAKER_INDEX = 0  # refer to input device id
        self.CHUNK = 1024
        self.RECORD_SECONDS = 1  # Record 1 second for wake word detection
        
        self.CONFIDENCE_THRESHOLD = 0.3  # Minimum confidence for detection
        
        # Initialize PyAudio
        try:
            self.p = pyaudio.PyAudio()
            self.get_logger().info('PyAudio initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PyAudio: {e}')
            return
        
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
            self.get_logger().info('TensorFlow Lite model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load TensorFlow Lite model: {e}')
            return
        
        # Create publisher for detected wake words (same topic as cli_publisher)
        self.wake_word_publisher = self.create_publisher(
            String,
            '/llm_input',  # Same topic as cli_publisher
            10
        )
        
        # Create a separate publisher for wake word events (optional)
        self.wake_word_event_publisher = self.create_publisher(
            String,
            '/wake_word_detected',
            10
        )
        
        # Create subscriber for LLM responses (optional)
        self.llm_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        self.get_logger().info('Wake Word Node initialized')
        self.get_logger().info(f'Listening for keywords: {", ".join(self.LABELS)}')
        self.get_logger().info(f'Model path: {self.MODEL_PATH}')
        
        # Start continuous listening
        self.listening_thread = threading.Thread(target=self._continuous_listening)
        self.listening_thread.daemon = True
        self.listening_thread.start()
    
    def _continuous_listening(self):
        """Continuously listen for wake words."""
        self.get_logger().info('Starting continuous wake word detection...')
        
        while rclpy.ok():
            try:
                # Record audio
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                    audio_file = temp_file.name
                
                self._record_audio(audio_file)
                
                # Process and recognize
                command, confidence = self._recognize_speech(audio_file)
                
                if command and confidence > self.CONFIDENCE_THRESHOLD:
                    self.get_logger().info(f'🎤 Wake word detected: {command.upper()} (confidence: {confidence:.2%})')
                    
                    # Publish to same topic as cli_publisher
                    msg = String()
                    msg.data = f"User said: {command}"
                    self.wake_word_publisher.publish(msg)
                    
                    # Also publish wake word event
                    event_msg = String()
                    event_msg.data = command.upper()
                    self.wake_word_event_publisher.publish(event_msg)
                
                # Clean up temp file
                try:
                    os.unlink(audio_file)
                except:
                    pass
                
                time.sleep(0.5)  # Brief pause between recordings
                
            except Exception as e:
                self.get_logger().error(f'Error in continuous listening: {e}')
                time.sleep(1)
    
    def _record_audio(self, output_file):
        """Record audio using PyAudio configuration from record.py."""
        try:
            # Create audio stream
            stream = self.p.open(
                rate=self.RESPEAKER_RATE,
                format=self.p.get_format_from_width(self.RESPEAKER_WIDTH),
                channels=self.RESPEAKER_CHANNELS,
                input=True,
                input_device_index=self.RESPEAKER_INDEX,
            )
            
            frames = []
            
            # Record audio for the specified duration
            for i in range(0, int(self.RESPEAKER_RATE / self.CHUNK * self.RECORD_SECONDS)):
                data = stream.read(self.CHUNK)
                frames.append(data)
            
            stream.stop_stream()
            stream.close()
            
            # Write to WAV file
            wf = wave.open(output_file, 'wb')
            wf.setnchannels(self.RESPEAKER_CHANNELS)
            wf.setsampwidth(self.p.get_sample_size(self.p.get_format_from_width(self.RESPEAKER_WIDTH)))
            wf.setframerate(self.RESPEAKER_RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            
        except Exception as e:
            self.get_logger().warn(f'PyAudio recording failed: {e}')
            # Create a silent audio file as fallback
            self._create_silent_audio(output_file)
    
    def _create_silent_audio(self, output_file):
        """Create a silent audio file as fallback."""
        try:
            # Create 1 second of silence at 16kHz
            silent_data = np.zeros(16000, dtype=np.int16)
            sf.write(output_file, silent_data, 16000)
        except Exception as e:
            self.get_logger().error(f'Failed to create silent audio: {e}')
    
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
    
    def _preprocess_audio(self, file_path):
        """Preprocess audio file for inference."""
        try:
            waveform, sample_rate = sf.read(file_path)
            
            if sample_rate != 16000:
                # Resample if necessary
                from scipy.signal import resample
                num_samples = int(len(waveform) * 16000 / sample_rate)
                waveform = resample(waveform, num_samples)
            
            if len(waveform.shape) > 1:
                waveform = waveform[:, 0]  # Convert to mono
            
            spectrogram = self._get_spectrogram(waveform)
            spectrogram = spectrogram[..., np.newaxis]
            spectrogram = spectrogram[np.newaxis, ...]
            
            return spectrogram
            
        except Exception as e:
            self.get_logger().error(f'Audio preprocessing failed: {e}')
            return None
    
    def _recognize_speech(self, audio_file):
        """Run inference on audio file."""
        try:
            spectrogram = self._preprocess_audio(audio_file)
            
            if spectrogram is None:
                return None, 0.0
            
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
    
    def llm_response_callback(self, msg):
        """Handle responses from LLM node."""
        self.get_logger().info(f'🤖 LLM Response: {msg.data}')
    
    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        try:
            if hasattr(self, 'p'):
                self.p.terminate()
        except Exception as e:
            self.get_logger().warn(f'Error terminating PyAudio: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    wake_word_node = WakeWordNode()
    
    try:
        rclpy.spin(wake_word_node)
    except KeyboardInterrupt:
        wake_word_node.get_logger().info('Wake word node shutting down...')
    finally:
        wake_word_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
