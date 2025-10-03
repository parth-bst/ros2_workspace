#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

class WakeWordConsumerNode(Node):
    """
    ROS2 node for consuming wake word detection results and LLM responses.
    Runs on Raspberry Pi - handles local actions based on results from MacBook.
    """
    
    def __init__(self):
        super().__init__('wake_word_consumer_node')
        
        # Create subscribers for results from MacBook
        self.wake_word_subscription = self.create_subscription(
            String,
            '/wake_word_detected',
            self.wake_word_callback,
            10
        )
        
        self.llm_response_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        self.audio_processing_status_subscription = self.create_subscription(
            String,
            '/audio_processing_status',
            self.audio_processing_status_callback,
            10
        )
        
        # Create publisher for local status updates
        self.local_status_publisher = self.create_publisher(
            String,
            '/pi_local_status',
            10
        )
        
        self.get_logger().info('📱 Wake Word Consumer Node initialized')
        self.get_logger().info('Listening for results from MacBook:')
        self.get_logger().info('  - /wake_word_detected')
        self.get_logger().info('  - /llm_response')
        self.get_logger().info('  - /audio_processing_status')
        self.get_logger().info('Publishing to: /pi_local_status')
        
        # Initialize local hardware interfaces (placeholder)
        self.led_status = False
        self.speaker_available = False
        
        # Publish initial status
        self._publish_local_status("Wake word consumer node started")
    
    def wake_word_callback(self, msg):
        """Handle wake word detection results from MacBook."""
        self.get_logger().info(f'🎯 Wake word detected: {msg.data}')
        
        # Trigger local actions on Pi
        self._handle_wake_word_detected(msg.data)
    
    def llm_response_callback(self, msg):
        """Handle LLM responses from MacBook."""
        self.get_logger().info(f'🤖 LLM Response: {msg.data}')
        
        # Trigger local actions on Pi
        self._handle_llm_response(msg.data)
    
    def audio_processing_status_callback(self, msg):
        """Handle audio processing status updates from MacBook."""
        self.get_logger().info(f'📊 Audio Processing Status: {msg.data}')
        
        # Update local status based on MacBook status
        self._handle_audio_processing_status(msg.data)
    
    def _handle_wake_word_detected(self, wake_word):
        """Handle wake word detection locally on Pi."""
        try:
            # Flash LED to indicate wake word detected
            self._flash_led(3, 0.2)  # Flash 3 times, 200ms interval
            
            # Play confirmation sound (if speaker available)
            if self.speaker_available:
                self._play_confirmation_sound()
            
            # Update status
            self._publish_local_status(f"Wake word '{wake_word}' detected - LED flashed")
            
        except Exception as e:
            self.get_logger().error(f'Error handling wake word: {e}')
    
    def _handle_llm_response(self, response):
        """Handle LLM response locally on Pi."""
        try:
            # Flash LED to indicate response received
            self._flash_led(2, 0.5)  # Flash 2 times, 500ms interval
            
            # Play response audio (if speaker available)
            if self.speaker_available:
                self._play_response_audio(response)
            
            # Update status
            self._publish_local_status(f"LLM response received - LED flashed")
            
        except Exception as e:
            self.get_logger().error(f'Error handling LLM response: {e}')
    
    def _handle_audio_processing_status(self, status):
        """Handle audio processing status updates."""
        try:
            # Update LED based on status
            if "started" in status.lower():
                self._set_led(True)  # Solid LED when processing is active
            elif "error" in status.lower():
                self._flash_led(5, 0.1)  # Fast flash on error
            elif "stopped" in status.lower():
                self._set_led(False)  # Turn off LED when stopped
            
            # Update local status
            self._publish_local_status(f"Audio processing: {status}")
            
        except Exception as e:
            self.get_logger().error(f'Error handling audio processing status: {e}')
    
    def _flash_led(self, count, interval):
        """Flash LED on Pi (placeholder implementation)."""
        try:
            # Placeholder for actual GPIO LED control
            # In real implementation, you would use RPi.GPIO or similar
            for i in range(count):
                self.get_logger().info(f'💡 LED flash {i+1}/{count}')
                time.sleep(interval)
                if i < count - 1:  # Don't sleep after last flash
                    time.sleep(interval)
                    
        except Exception as e:
            self.get_logger().error(f'Error flashing LED: {e}')
    
    def _set_led(self, state):
        """Set LED state on Pi (placeholder implementation)."""
        try:
            # Placeholder for actual GPIO LED control
            self.led_status = state
            status = "ON" if state else "OFF"
            self.get_logger().info(f'💡 LED set to {status}')
            
        except Exception as e:
            self.get_logger().error(f'Error setting LED: {e}')
    
    def _play_confirmation_sound(self):
        """Play confirmation sound (placeholder implementation)."""
        try:
            # Placeholder for actual audio playback
            # In real implementation, you would use pygame, pyaudio, or similar
            self.get_logger().info('🔊 Playing confirmation sound')
            
        except Exception as e:
            self.get_logger().error(f'Error playing confirmation sound: {e}')
    
    def _play_response_audio(self, response):
        """Play LLM response audio (placeholder implementation)."""
        try:
            # Placeholder for actual text-to-speech
            # In real implementation, you would use espeak, festival, or similar
            self.get_logger().info(f'🔊 Playing response audio: {response[:50]}...')
            
        except Exception as e:
            self.get_logger().error(f'Error playing response audio: {e}')
    
    def _publish_local_status(self, status_message):
        """Publish local status update."""
        try:
            status_msg = String()
            status_msg.data = f"[Pi] {status_message}"
            self.local_status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing local status: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('📱 Wake word consumer node shutting down...')
        self._set_led(False)  # Turn off LED on shutdown
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    wake_word_consumer_node = WakeWordConsumerNode()
    
    try:
        rclpy.spin(wake_word_consumer_node)
    except KeyboardInterrupt:
        wake_word_consumer_node.get_logger().info('📱 Wake word consumer node shutting down...')
    finally:
        wake_word_consumer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
