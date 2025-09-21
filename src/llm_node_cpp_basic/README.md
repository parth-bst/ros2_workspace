# LLM Node C++ Basic

A C++ ROS2 node that provides OpenAI API integration using the `liboai` library.

## Features

- **Text Completions**: Uses GPT-5-nano for text processing
- **ROS2 Integration**: Subscribes to `/llm_input`, publishes to `/llm_response`
- **Clean Implementation**: Simple, standalone C++ node using liboai library
- **Error Handling**: Comprehensive error checking and logging
- **Self-Contained**: Automatically clones and builds the liboai library

## OpenAI Library Integration

### Library Details
- **Library**: `liboai` (https://github.com/D7EAD/liboai.git)
- **Type**: C++17 client library for OpenAI API
- **Integration**: Automatically cloned during build process
- **Location**: `liboai/` directory within the package

### Include Structure
```cpp
#include "liboai.h"  // Main header file
```

### CMake Integration
```cmake
# Automatic cloning during build
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/liboai")
    execute_process(COMMAND git clone https://github.com/D7EAD/liboai.git)
endif()

# Build as subdirectory
add_subdirectory(liboai/liboai liboai_build)

# Link the library
target_link_libraries(llm_node_cpp_basic oai ...)
```

### Usage Pattern
```cpp
// Initialize client
std::unique_ptr<liboai::OpenAI> openai_client_;
openai_client_ = std::make_unique<liboai::OpenAI>();

// Set API key
openai_client_->auth.SetKey(api_key);

// Create conversation and call API
liboai::Conversation convo;
convo.AddUserData(user_input);
auto response = openai_client_->ChatCompletion->create(...);
```

## Usage

### Building

```bash
# From ros2_workspace directory
./scripts/llm_node_cpp_basic/build_node.sh
```

### Running

```bash
# From ros2_workspace directory
./scripts/llm_node_cpp_basic/run_llm_node.sh
```

### Testing

```bash
# Send a text message
ros2 topic pub /llm_input std_msgs/msg/String "data: 'Hello, how are you?'"

# Listen for responses
ros2 topic echo /llm_response
```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key (required)

## Topics

- **Subscribes to**: `/llm_input` (std_msgs/String)
- **Publishes to**: `/llm_response` (std_msgs/String)

## Dependencies

- ROS2 Jazzy
- liboai (automatically cloned during build)
- nlohmann-json
- libcurl
- std_msgs

## Comparison with Python Version

This C++ implementation provides the same functionality as the Python `llm_node` but with potentially better performance and lower memory usage.
