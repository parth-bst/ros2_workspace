#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import openai
import os
import json
from typing import Optional


class LLMNode(Node):
    """
    ROS2 node that listens to ROS events and calls OpenAI API for LLM responses.
    """
    
    def __init__(self):
        super().__init__('llm_node')
        
        # Initialize OpenAI client
        self.openai_client = None
        self._setup_openai()
        
        # Create subscribers for different message types
        self.string_subscription = self.create_subscription(
            String,
            '/llm_input',
            self.string_callback,
            10
        )
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create a publisher for responses
        self.response_publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        self.get_logger().info('LLM Node initialized and listening for events...')
    
    def _setup_openai(self):
        """Setup OpenAI client with API key from environment variable."""
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set!')
            return
        
        try:
            self.openai_client = openai.OpenAI(api_key=api_key)
            self.get_logger().info('OpenAI client initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize OpenAI client: {e}')
    
    def string_callback(self, msg: String):
        """Handle string messages from ROS topic."""
        if not self.openai_client:
            self.get_logger().warn('OpenAI client not available, skipping request')
            return
        
        self.get_logger().info(f'Received string message: {msg.data}')
        
        try:
            response = self._call_openai_api(msg.data)
            if response:
                self.get_logger().info(f'LLM Response: {response}')
                
                # Publish response back to ROS
                response_msg = String()
                response_msg.data = response
                self.response_publisher.publish(response_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing string message: {e}')
    
    def image_callback(self, msg: Image):
        """Handle image messages from ROS topic."""
        if not self.openai_client:
            self.get_logger().warn('OpenAI client not available, skipping request')
            return
        
        self.get_logger().info(f'Received image message: {msg.width}x{msg.height}')
        
        try:
            # Convert image data to base64 for OpenAI API
            import base64
            import io
            from PIL import Image as PILImage
            
            # Convert ROS Image to PIL Image
            pil_image = PILImage.frombytes(
                'RGB',
                (msg.width, msg.height),
                msg.data
            )
            
            # Convert to base64
            buffer = io.BytesIO()
            pil_image.save(buffer, format='JPEG')
            image_base64 = base64.b64encode(buffer.getvalue()).decode()
            
            # Call OpenAI with image
            response = self._call_openai_api_with_image(
                "Describe what you see in this image.",
                image_base64
            )
            
            if response:
                self.get_logger().info(f'LLM Response: {response}')
                
                # Publish response back to ROS
                response_msg = String()
                response_msg.data = response
                self.response_publisher.publish(response_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image message: {e}')
    
    def _call_openai_api(self, prompt: str) -> Optional[str]:
        """Call OpenAI API with text prompt."""
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "user", "content": prompt}
                ],
                max_tokens=150,
                temperature=0.7
            )
            
            return response.choices[0].message.content.strip()
        
        except Exception as e:
            self.get_logger().error(f'OpenAI API call failed: {e}')
            return None
    
    def _call_openai_api_with_image(self, prompt: str, image_base64: str) -> Optional[str]:
        """Call OpenAI API with text and image prompt."""
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4-vision-preview",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=150,
                temperature=0.7
            )
            
            return response.choices[0].message.content.strip()
        
        except Exception as e:
            self.get_logger().error(f'OpenAI API call with image failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    llm_node = LLMNode()
    
    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
