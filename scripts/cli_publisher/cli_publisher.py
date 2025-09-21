#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class CLIPublisher(Node):
    """
    Simple CLI publisher that sends user input to the llm_node
    """
    
    def __init__(self):
        super().__init__('cli_publisher')
        
        # Create publisher for llm_input topic
        self.publisher = self.create_publisher(
            String,
            '/llm_input',
            10
        )
        
        # Create subscriber for llm_response topic
        self.subscription = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )
        
        self.get_logger().info('CLI Publisher started. Type your messages and press Enter.')
        self.get_logger().info('Type "quit" or "exit" to stop.')
        
        # Start the interactive loop
        self.start_interactive_loop()
    
    def response_callback(self, msg: String):
        """Handle responses from llm_node"""
        print(f"\n🤖 LLM Response: {msg.data}")
        print("> ", end="", flush=True)
    
    def start_interactive_loop(self):
        """Start interactive CLI loop"""
        print("\n" + "="*50)
        print("🤖 LLM Node CLI Tester")
        print("="*50)
        print("Type your messages below and press Enter to send them to the LLM node.")
        print("You'll see the responses appear below your input.")
        print("Type 'quit' or 'exit' to stop.\n")
        
        try:
            while True:
                user_input = input("> ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("👋 Goodbye!")
                    break
                
                if not user_input:
                    continue
                
                # Create and publish message
                msg = String()
                msg.data = user_input
                self.publisher.publish(msg)
                
                self.get_logger().info(f'Sent: {user_input}')
                
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
        finally:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        cli_publisher = CLIPublisher()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
