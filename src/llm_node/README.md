# LLM Node

A ROS2 node that listens to ROS event queues and calls the OpenAI API to get LLM-generated responses.

## Features

- Listens to ROS topics for text and image messages
- Calls OpenAI API (GPT-3.5-turbo for text, GPT-4-vision for images)
- Publishes LLM responses back to ROS topics
- Minimal and efficient implementation

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

3. Build the ROS2 package:
```bash
cd /path/to/ros2_workspace
colcon build --packages-select llm_node
source install/setup.bash
```

## Usage

### Running the LLM Node

```bash
# Using launch file
ros2 launch llm_node llm_node.launch.py

# Or directly
ros2 run llm_node llm_node
```

### Sending Text Messages

The node listens to `/llm_input` topic for text messages:

```bash
# Using command line
ros2 topic pub /llm_input std_msgs/String "data: 'Hello, how are you?'"

# Or use the CLI publisher
cd /workspace/M1_WiredUp/ros2_workspace
./scripts/cli_publisher/run_cli_publisher.sh
```

### Sending Image Messages

The node listens to `/camera/image_raw` topic for image messages:

```bash
# Publish image data to the topic
ros2 topic pub /camera/image_raw sensor_msgs/Image "..." --once
```

### Listening to Responses

```bash
# Listen to LLM responses
ros2 topic echo /llm_response
```

### Available Test Scripts

The workspace includes several test scripts for different use cases:

```bash
# Interactive CLI for sending messages
./scripts/cli_publisher/run_cli_publisher.sh

# Run LLM node
./scripts/llm_node/run_llm_node.sh

# Build the package
./scripts/llm_node/build_node.sh
```

## Topics

- **Subscribed:**
  - `/llm_input` (std_msgs/String): Text input for LLM processing
  - `/camera/image_raw` (sensor_msgs/Image): Image input for vision processing

- **Published:**
  - `/llm_response` (std_msgs/String): LLM-generated responses

## Configuration

The node uses the following environment variables:
- `OPENAI_API_KEY`: Your OpenAI API key (required)

## Dependencies

- openai>=1.0.0
- Pillow>=9.0.0
- rclpy
- std_msgs
- sensor_msgs
