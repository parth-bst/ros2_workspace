#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class WakeWordConsumer(Node):
    """
    CLI consumer for testing wake word detection.
    Monitors wake word events and LLM responses.
    """
    
    def __init__(self):
        super().__init__('wake_word_consumer')
        
        # Create subscriber for wake word events
        self.wake_word_subscription = self.create_subscription(
            String,
            '/wake_word_detected',
            self.wake_word_callback,
            10
        )
        
        # Create subscriber for LLM responses
        self.llm_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        self.get_logger().info('Wake Word Consumer started.')
        self.get_logger().info('Listening for wake word events and LLM responses...')
        self.get_logger().info('Press Ctrl+C to stop.')
        
        # Print initial instructions
        self._print_instructions()
    
    def _print_instructions(self):
        """Print usage instructions."""
        print("\n" + "="*60)
        print("🎤 WAKE WORD DETECTION MONITOR")
        print("="*60)
        print("This consumer monitors:")
        print("  📡 /wake_word_detected - Raw wake word detection events")
        print("  🤖 /llm_response - LLM responses to wake word inputs")
        print()
        print("Supported wake words: YES, NO, GO, LEFT, RIGHT, UP, DOWN, STOP")
        print("Speak clearly and wait for detection...")
        print("Press Ctrl+C to stop monitoring.")
        print("="*60 + "\n")
    
    def wake_word_callback(self, msg: String):
        """Handle wake word detection events"""
        print(f"🎤 WAKE WORD DETECTED: {msg.data}")
        print(f"   → Sending to LLM for processing...")
        print()
    
    def llm_response_callback(self, msg: String):
        """Handle LLM responses"""
        print(f"🤖 LLM RESPONSE: {msg.data}")
        print("-" * 60)
        print()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        wake_word_consumer = WakeWordConsumer()
        rclpy.spin(wake_word_consumer)
    except KeyboardInterrupt:
        print("\n👋 Wake word consumer stopped!")
    finally:
        if 'wake_word_consumer' in locals():
            wake_word_consumer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
