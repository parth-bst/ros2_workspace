# Live Agent ROS2 Package

A ROS2 node that integrates with [Deep Agents](https://github.com/langchain-ai/deepagents) to provide intelligent robotic assistance with advanced AI capabilities.

## Features

- **Deep Agents Integration**: Uses LangChain's Deep Agents framework with middleware for planning, filesystem access, and subagent spawning
- **ROS2 Communication**: Full ROS2 integration with publishers, subscribers, and service interfaces
- **System Monitoring**: Built-in status monitoring and safety checks
- **Long-term Memory**: Persistent memory storage for context and learning
- **Intelligent Conversation**: Advanced AI-powered responses using Deep Agents

## Architecture

The Live Agent uses the Deep Agents framework which provides:

1. **TodoListMiddleware**: Planning and task management
2. **FilesystemMiddleware**: File system access for memory and context storage
3. **SubAgentMiddleware**: Spawning specialized subagents for specific tasks
4. **SummarizationMiddleware**: Context management and conversation summarization

## Prerequisites

- ROS2 Jazzy
- Python 3.11+
- Virtual environment with required packages

## Required Environment Variables

Set these in your `.env` file:

```bash
OPENAI_API_KEY=your_openai_api_key  # Required
TAVILY_API_KEY=your_tavily_api_key  # Optional for web search
OPENAI_MODEL=gpt-5-nano  # Optional, defaults to gpt-5-nano
```

## Installation

1. **Build the package**:
   ```bash
   cd /workspace/M1_WiredUp/ros2_workspace
   ./scripts/live_agent/build_node.sh
   ```

2. **Run the agent**:
   ```bash
   ./scripts/live_agent/run_live_agent.sh
   ```

3. **Test the agent**:
   ```bash
   ./scripts/live_agent/test_live_agent.sh
   ```

## Usage

### ROS2 Topics

**Input Topics:**
- `/live_agent/user_text` (std_msgs/String): User text input

**Output Topics:**
- `/live_agent/response` (std_msgs/String): Agent responses
- `/live_agent/status` (std_msgs/String): Agent status information

**Robot Control Topics:**
- `/tts/speak` (std_msgs/String): Text-to-speech commands


### Example Usage

Send a text message to the agent:
```bash
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Hello, can you help me?'"
```

Check system status:
```bash
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'What is the current system status?'"
```

Request TTS:
```bash
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Say hello to me'"
```

Request safety check:
```bash
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Perform a safety check'"
```

## Configuration

The agent can be configured through YAML files:

- `config/agent.yaml`: Agent behavior and model settings
- `config/topics.yaml`: ROS2 topic mappings

## Available Tools

The agent has access to the following tools:

1. **tts_speak_tool**: Convert text to speech
2. **get_status_tool**: Get current system status
3. **safety_check_tool**: Perform safety checks

## Safety Features

- Emergency stop detection
- Proximity alert monitoring
- System status validation

## Development

The package structure:

```
live_agent/
├── live_agent/
│   ├── __init__.py
│   └── live_agent.py          # Main node implementation
├── config/
│   ├── agent.yaml            # Agent configuration
│   └── topics.yaml           # Topic configuration
├── launch/
│   └── live_agent.launch.py  # Launch file
├── deepagents/               # Deep Agents framework
├── deep-agents-ui/           # UI components
├── package.xml
├── setup.py
└── requirements.txt
```

## Troubleshooting

1. **Deep Agents not available**: Ensure all dependencies are installed via the build script
2. **API key errors**: Check that environment variables are properly set
3. **ROS2 communication issues**: Verify ROS_DOMAIN_ID and discovery settings
4. **Tool execution errors**: Check robot hardware connections and topic mappings

## License

MIT License - see LICENSE file for details.
