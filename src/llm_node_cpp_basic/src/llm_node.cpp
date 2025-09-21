#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "liboai.h"
#include <memory>
#include <string>
#include <cstdlib>

class LLMNode : public rclcpp::Node {
public:
    LLMNode() : Node("llm_node_cpp_basic") {
        // Initialize OpenAI client
        openai_client_ = nullptr;
        setup_openai();
        
        // Create subscribers
        string_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/llm_input", 10,
            std::bind(&LLMNode::string_callback, this, std::placeholders::_1));
        
        // Create publisher for responses
        response_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/llm_response", 10);
        
        RCLCPP_INFO(this->get_logger(), "LLM Node (C++ Basic) initialized and listening for events...");
    }

private:
    void setup_openai() {
        const char* api_key = std::getenv("OPENAI_API_KEY");
        if (!api_key) {
            RCLCPP_ERROR(this->get_logger(), "OPENAI_API_KEY environment variable not set!");
            return;
        }
        
        try {
            openai_client_ = std::make_unique<liboai::OpenAI>();
            bool key_set = openai_client_->auth.SetKey(std::string(api_key));
            if (!key_set) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set OpenAI API key");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OpenAI client initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenAI client: %s", e.what());
        }
    }
    
    void string_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!openai_client_) {
            RCLCPP_WARN(this->get_logger(), "OpenAI client not available, skipping request");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Received string message: %s", msg->data.c_str());
        
        try {
            // Create chat completion request
            liboai::Conversation convo;
            bool data_added = convo.AddUserData(msg->data);
            if (!data_added) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add user data to conversation");
                return;
            }
            
               // Call OpenAI API
               auto response = openai_client_->ChatCompletion->create(
                   "gpt-5-nano",
                convo,
                std::nullopt,  // function_call
                std::nullopt,  // temperature (eg: 0.7f, use default for GPT-5-nano)
                std::nullopt,  // top_p
                std::nullopt,  // n
                std::nullopt,  // stream
                std::nullopt,  // stop
                std::nullopt,  // max_tokens (removed for GPT-5-nano compatibility)
                std::nullopt,  // presence_penalty
                std::nullopt,  // frequency_penalty
                std::nullopt,  // logit_bias
                std::nullopt   // user
               );
            
            // Update conversation with response
            bool updated = convo.Update(response);
            if (!updated) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update conversation with response");
                return;
            }
            
            // Get the response text
            std::string response_text = convo.GetLastResponse();
            RCLCPP_INFO(this->get_logger(), "LLM Response: %s", response_text.c_str());
            
            // Publish response back to ROS
            auto response_msg = std_msgs::msg::String();
            response_msg.data = response_text;
            response_publisher_->publish(response_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing string message: %s", e.what());
        }
    }
    
    std::unique_ptr<liboai::OpenAI> openai_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LLMNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
