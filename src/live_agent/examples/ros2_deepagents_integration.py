#!/usr/bin/env python3
"""
Example integration between ROS2 Live Agent and Deep Agents UI
This shows how to bridge ROS2 communication with the Deep Agents framework
"""

import asyncio
import json
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Deep Agents imports
try:
    from deepagents import create_deep_agent
    from langchain_core.tools import tool
    from langchain_anthropic import ChatAnthropic
    DEEPAGENTS_AVAILABLE = True
except ImportError:
    DEEPAGENTS_AVAILABLE = False
    print("Warning: Deep Agents not available")


class ROS2DeepAgentsBridge(Node):
    """
    Bridge between ROS2 and Deep Agents framework
    This allows the Deep Agents UI to communicate with ROS2 systems
    """
    
    def __init__(self):
        super().__init__('ros2_deepagents_bridge')
        
        # Initialize Deep Agent
        self.agent = self._init_deep_agent()
        
        # ROS2 interfaces
        self._init_ros_interfaces()
        
        # Message storage for UI communication
        self.message_history = []
        
        self.get_logger().info("ROS2-Deep Agents Bridge initialized")
    
    def _init_deep_agent(self):
        """Initialize Deep Agent with ROS2 tools"""
        if not DEEPAGENTS_AVAILABLE:
            return None
        
        # ROS2 tools for the agent
        @tool
        def ros2_publish_tool(topic: str, message: str) -> str:
            """Publish a message to a ROS2 topic"""
            try:
                msg = String()
                msg.data = message
                
                # Create publisher for the topic
                pub = self.create_publisher(String, topic, 10)
                pub.publish(msg)
                
                return f"Published to {topic}: {message}"
            except Exception as e:
                return f"Error publishing to {topic}: {e}"
        
        @tool
        def ros2_subscribe_tool(topic: str) -> str:
            """Subscribe to a ROS2 topic and get latest message"""
            try:
                # This is a simplified version - in practice you'd want to store
                # the latest message from each topic
                return f"Subscribed to {topic} - latest message would be returned here"
            except Exception as e:
                return f"Error subscribing to {topic}: {e}"
        
        @tool
        def get_robot_status_tool() -> str:
            """Get current robot status from ROS2 topics"""
            try:
                status = {
                    "timestamp": time.time(),
                    "node_status": "active",
                    "topics_available": [
                        "/live_agent/user_text",
                        "/live_agent/response",
                        "/live_agent/status"
                    ]
                }
                return json.dumps(status, indent=2)
            except Exception as e:
                return f"Error getting status: {e}"
        
        # Create the Deep Agent
        agent = create_deep_agent(
            model="claude-sonnet-4-5-20250929",
            tools=[ros2_publish_tool, ros2_subscribe_tool, get_robot_status_tool],
            system_prompt="""You are a ROS2-Deep Agents bridge. You can:
            - Publish messages to ROS2 topics
            - Subscribe to ROS2 topics
            - Get robot status information
            - Help users interact with ROS2 systems through natural language
            
            Always be helpful and provide clear feedback about ROS2 operations.""",
            name="ROS2-Bridge-Agent"
        )
        
        return agent
    
    def _init_ros_interfaces(self):
        """Initialize ROS2 publishers and subscribers"""
        # Subscribers
        self.sub_user_input = self.create_subscription(
            String,
            '/live_agent/user_text',
            self.on_user_input,
            10
        )
        
        self.sub_agent_response = self.create_subscription(
            String,
            '/live_agent/response',
            self.on_agent_response,
            10
        )
        
        # Publishers
        self.pub_bridge_response = self.create_publisher(
            String,
            '/bridge/response',
            10
        )
        
        self.pub_ui_data = self.create_publisher(
            String,
            '/ui/message_history',
            10
        )
    
    def on_user_input(self, msg: String):
        """Handle user input from ROS2"""
        user_text = msg.data
        self.get_logger().info(f"Received user input: {user_text}")
        
        # Store in message history
        self.message_history.append({
            "role": "user",
            "content": user_text,
            "timestamp": time.time()
        })
        
        # Process with Deep Agent if available
        if self.agent:
            try:
                response = self.agent.invoke({
                    "messages": [{"role": "user", "content": user_text}]
                })
                
                # Extract response
                if "messages" in response and response["messages"]:
                    last_message = response["messages"][-1]
                    if hasattr(last_message, 'content'):
                        response_text = last_message.content
                    else:
                        response_text = str(last_message)
                else:
                    response_text = "I processed your request but couldn't generate a response."
                
                # Store response
                self.message_history.append({
                    "role": "assistant",
                    "content": response_text,
                    "timestamp": time.time()
                })
                
                # Publish response
                self._publish_response(response_text)
                
            except Exception as e:
                self.get_logger().error(f"Error processing with Deep Agent: {e}")
                self._publish_response(f"Error: {e}")
        else:
            self._publish_response("Deep Agent not available")
    
    def on_agent_response(self, msg: String):
        """Handle agent response from ROS2"""
        response_text = msg.data
        self.get_logger().info(f"Received agent response: {response_text}")
        
        # Store in message history
        self.message_history.append({
            "role": "agent",
            "content": response_text,
            "timestamp": time.time()
        })
        
        # Publish to UI
        self._publish_ui_data()
    
    def _publish_response(self, text: str):
        """Publish response to ROS2"""
        try:
            msg = String()
            msg.data = text
            self.pub_bridge_response.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish response: {e}")
    
    def _publish_ui_data(self):
        """Publish message history for UI consumption"""
        try:
            ui_data = {
                "message_history": self.message_history,
                "timestamp": time.time()
            }
            
            msg = String()
            msg.data = json.dumps(ui_data)
            self.pub_ui_data.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish UI data: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ROS2DeepAgentsBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Bridge error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
