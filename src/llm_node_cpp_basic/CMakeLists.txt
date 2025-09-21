cmake_minimum_required(VERSION 3.16)
project(llm_node_cpp_basic)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find system dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# Find nlohmann/json
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(${CURL_INCLUDE_DIRS})

# Clone and build liboai if not present
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/liboai")
    execute_process(
        COMMAND git clone https://github.com/D7EAD/liboai.git
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_RESULT
    )
    if(NOT GIT_RESULT EQUAL "0")
        message(FATAL_ERROR "Failed to clone liboai repository")
    endif()
endif()

# Add liboai as subdirectory
add_subdirectory(liboai/liboai liboai_build)

# Create the main ROS2 node executable
add_executable(llm_node_cpp_basic src/llm_node.cpp)

# Link libraries
target_link_libraries(llm_node_cpp_basic
    oai
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Install
install(TARGETS
    llm_node_cpp_basic
    DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_target_dependencies(llm_node_cpp_basic
    rclcpp
    std_msgs
)

ament_package()
