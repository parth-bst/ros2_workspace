#!/usr/bin/env python3
"""
Main ROS node for the live agent with Deep Agents integration
"""

import os
import time
import json
import yaml
from typing import Dict, Any, Optional, List
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from led_controller import light_led_on_gpio21, cleanup_gpio, init_led_controller

# Deep Agents imports
from deepagents import create_deep_agent
from deepagents.middleware.filesystem import FilesystemMiddleware
from deepagents.middleware.subagents import SubAgent, SubAgentMiddleware
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.store.memory import InMemoryStore
DEEPAGENTS_AVAILABLE = True



class LiveAgentNode(Node):
    """Main ROS node for the live agent with Deep Agents integration"""
    
    def __init__(self):
        super().__init__('live_agent')
        
        # Load configuration
        self._load_config()
        
        # Initialize context storage
        self.agent_context = {
            "indoor": True,
            "proximity_alert": False,
            "emergency_stop": False,
            "last_heartbeat": time.time()
        }
        
        # Initialize LED controller
        init_led_controller()
        
        # Initialize ROS interfaces
        self._init_ros_interfaces()
        
        # Initialize Deep Agent
        self.agent = self._init_deep_agent()
        
        # Initialize timers
        self._init_timers()
        
        self.get_logger().info("Live Agent initialized successfully with Deep Agents integration")
    
    def _load_config(self):
        """Load configuration from YAML files"""
        try:
            # Load topics config
            topics_config_path = os.path.join(
                os.path.dirname(__file__), 'config', 'topics.yaml'
            )
            if os.path.exists(topics_config_path):
                with open(topics_config_path, 'r') as f:
                    self.topics_config = yaml.safe_load(f)
            else:
                self.topics_config = {
                    'input_topics': {
                        'user_text': '/live_agent/user_text'
                    },
                    'output_topics': {
                        'response': '/live_agent/response',
                        'status': '/live_agent/status'
                    }
                }
            
            # Load agent config
            agent_config_path = os.path.join(
                os.path.dirname(__file__), 'config', 'agent.yaml'
            )
            if os.path.exists(agent_config_path):
                with open(agent_config_path, 'r') as f:
                    self.agent_config = yaml.safe_load(f)
            else:
                self.agent_config = {
                    'name': 'LiveAgent',
                    'model': 'gpt-5-nano',
                    'system_prompt': 'You are a helpful AI assistant with access to text-to-speech, system status and safety monitoring capabilities.',
                    'tools_enabled': True
                }
            
            # Override model from environment variable if set
            if 'OPENAI_MODEL' in os.environ:
                self.agent_config['model'] = os.environ['OPENAI_MODEL']
                
        except Exception as e:
            self.get_logger().warn(f"Failed to load config: {e}")
            self.topics_config = {}
            self.agent_config = {}
    
    
    def _init_deep_agent(self):
        """Initialize the Deep Agent"""
        if not DEEPAGENTS_AVAILABLE:
            self.get_logger().error("Deep Agents not available, cannot initialize agent")
            return None
            
        try:
            # Create basic tools for the agent
            @tool
            def get_status_tool() -> str:
                """Get current system status"""
                try:
                    status = {
                        "indoor": self.agent_context.get("indoor", True),
                        "proximity_alert": self.agent_context.get("proximity_alert", False),
                        "emergency_stop": self.agent_context.get("emergency_stop", False),
                        "last_heartbeat": self.agent_context.get("last_heartbeat", time.time())
                    }
                    return json.dumps(status, indent=2)
                except Exception as e:
                    return f"Status error: {e}"
            
            @tool
            def safety_check_tool() -> str:
                """Perform safety check"""
                try:
                    if self.agent_context.get("emergency_stop", False):
                        return "SAFETY VIOLATION: Emergency stop is active"
                    
                    if self.agent_context.get("proximity_alert", False):
                        return "SAFETY VIOLATION: Proximity alert is active"
                    
                    return "Safety check passed"
                except Exception as e:
                    return f"Safety check error: {e}"
            
            # Create TTS tool that can access the node instance
            def ros_tts_tool(node_instance):
                @tool
                def tts_speak_tool(text: str) -> str:
                    """Make the robot speak using text-to-speech"""
                    try:
                        msg = String()
                        msg.data = text
                        node_instance.pub_tts.publish(msg)
                        node_instance.get_logger().info(f"TTS: {text}")
                        return f"Successfully spoke: {text}"
                    except Exception as e:
                        node_instance.get_logger().error(f"TTS error: {e}")
                        return f"TTS error: {e}"
                return tts_speak_tool
            
            # Create simple LED tool
            @tool
            def led_control_tool() -> str:
                """Turn on LED on GPIO pin 21"""
                return light_led_on_gpio21()
            
            led_tools = [led_control_tool]
            
            # Create OpenAI model instance
            model_name = self.agent_config.get('model', 'gpt-5-nano')
            openai_model = ChatOpenAI(
                model=model_name,
                max_tokens=2000
            )
            
            # Create store for longterm memory
            store = InMemoryStore()
            
            # Create the deep agent with memory support
            all_tools = [ros_tts_tool(self), get_status_tool, safety_check_tool] + led_tools
            agent = create_deep_agent(
                model=openai_model,
                tools=all_tools,
                system_prompt=self.agent_config.get('system_prompt', 
                    "You are a helpful AI assistant with access to text-to-speech, LED control, system status and safety monitoring. "
                    "When users ask you to say something, use the tts_speak_tool to speak it directly. "
                    "When users ask you to turn on the LED, use the led_control_tool to turn on the LED on GPIO pin 21. "
                    "Keep responses brief and focused on using tools rather than lengthy text explanations. "
                    "For TTS requests, just use the tool and confirm with a short message like 'Done' or 'Spoken'. "
                    "For LED requests, use the tool and confirm with a short message like 'LED turned on'. "
                    "Always prioritize safety and use tools efficiently."),
                use_longterm_memory=self.agent_config.get('use_longterm_memory', True),
                store=store,
                name=self.agent_config.get('name', 'LiveAgent')
            )
            
            return agent
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Deep Agent: {e}")
            return None
    
    def _init_ros_interfaces(self):
        """Initialize ROS publishers and subscribers"""
        # Subscribers
        self.sub_text = self.create_subscription(
            String, 
            self.topics_config.get('input_topics', {}).get('user_text', '/live_agent/user_text'), 
            self.on_user_text, 
            10
        )
        
        # Publishers
        self.pub_response = self.create_publisher(
            String,
            self.topics_config.get('output_topics', {}).get('response', '/live_agent/response'),
            10
        )
        
        self.pub_status = self.create_publisher(
            String,
            self.topics_config.get('output_topics', {}).get('status', '/live_agent/status'),
            10
        )
        
        self.pub_tts = self.create_publisher(
            String,
            '/tts/speak',
            10
        )
    
    def _init_timers(self):
        """Initialize ROS timers"""
        # Heartbeat timer (2 seconds)
        self.create_timer(2.0, self.heartbeat)
        
        # Status publish timer (5 seconds)
        self.create_timer(5.0, self.publish_status)
    
    def on_user_text(self, msg: String):
        """Handle user text input using Deep Agent"""
        text = msg.data
        self.get_logger().info(f"Received user text: {text}")
        
        if not self.agent:
            self.get_logger().error("Deep Agent not available")
            self._publish_response("I'm sorry, my AI system is not available right now.")
            return
        
        try:
            # Use Deep Agent to process the request
            response = self.agent.invoke({
                "messages": [{"role": "user", "content": text}]
            })
            
            # Extract response from agent output
            if "messages" in response and response["messages"]:
                last_message = response["messages"][-1]
                if hasattr(last_message, 'content'):
                    response_text = last_message.content
                else:
                    response_text = str(last_message)
            else:
                response_text = "I processed your request but couldn't generate a response."
            
            self.get_logger().info(f"Agent response: {response_text}")
            self._publish_response(response_text)
            
        except Exception as e:
            self.get_logger().error(f"Error processing user text: {e}")
            self._publish_response("I'm sorry, I encountered an error processing your request.")
    
    
    def _publish_response(self, text: str):
        """Publish response to ROS topic"""
        try:
            msg = String()
            msg.data = text
            self.pub_response.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish response: {e}")
    
    def heartbeat(self):
        """Publish heartbeat status"""
        self.get_logger().info("Heartbeat: Live Agent alive")
        
        # Update context with current status
        self.agent_context["last_heartbeat"] = time.time()
    
    def publish_status(self):
        """Publish current status"""
        try:
            status = {
                "indoor": self.agent_context.get("indoor", True),
                "proximity_alert": self.agent_context.get("proximity_alert", False),
                "emergency_stop": self.agent_context.get("emergency_stop", False),
                "last_heartbeat": self.agent_context.get("last_heartbeat", time.time())
            }
            msg = String()
            msg.data = json.dumps(status, indent=2)
            self.pub_status.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish status: {e}")
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        cleanup_gpio()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = LiveAgentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        if node:
            node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
