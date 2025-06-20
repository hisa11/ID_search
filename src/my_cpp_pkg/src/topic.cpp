#include "topic.hpp"
#include <iostream>
#include <string>

using namespace std::chrono_literals;

namespace custom_interfaces
{

// 静的メンバ変数の定義
const std::string PublisherNode::TOPIC_NAME = "topic_name";
const std::string SubscriberNode::TOPIC_NAME = "topic_name";

// PublisherNodeの実装
PublisherNode::PublisherNode()
: Node("simple_publisher"), count_(0)
{
    // Publisherの作成
    publisher_ = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME, QUEUE_SIZE);
    
    RCLCPP_INFO(this->get_logger(), "PublisherNode initialized. Ready to publish to topic: %s", TOPIC_NAME.c_str());
}

void PublisherNode::publish_message(const std::string& message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
}

void PublisherNode::publish_message_with_count(const std::string& base_message)
{
    auto message = std_msgs::msg::String();
    message.data = base_message + " Count: " + std::to_string(count_);
    
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    
    count_++;
}

size_t PublisherNode::get_count() const
{
    return count_;
}

// SubscriberNodeの実装
SubscriberNode::SubscriberNode()
: Node("simple_subscriber")
{
    // Subscriptionの作成
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        TOPIC_NAME,
        QUEUE_SIZE,
        std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "SubscriberNode initialized. Subscribed to topic: %s", TOPIC_NAME.c_str());
}

void SubscriberNode::listener_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}

// Publisher用の即座送信関数
int run_publisher_once(int argc, char * argv[], const std::string& message)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PublisherNode>();
    
    try {
        // 少し待ってから送信（接続確立のため）
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        if (message.empty()) {
            node->publish_message_with_count("Hello ROS2!");
        } else {
            node->publish_message(message);
        }
        
        // メッセージが送信されるまで少し待つ
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

// Publisher用の対話的関数
int run_publisher_interactive(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PublisherNode>();
    
    try {
        std::cout << "対話的Publisherモードです。メッセージを入力してください (quit で終了):" << std::endl;
        
        std::string input;
        while (rclcpp::ok()) {
            std::cout << "メッセージ> ";
            std::getline(std::cin, input);
            
            if (input == "quit" || input == "exit") {
                break;
            }
            
            if (!input.empty()) {
                node->publish_message(input);
            } else {
                node->publish_message_with_count("Hello ROS2!");
            }
            
            // 少し待つ
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

// Subscriber用のmain関数
int run_subscriber(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SubscriberNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

}  // namespace custom_interfaces

// グローバルなPublisherインスタンス（topic_send用）
namespace {
    std::shared_ptr<custom_interfaces::PublisherNode> g_publisher_node = nullptr;
    bool g_rclcpp_initialized = false;
}

// 簡単送信関数の実装（グローバル関数として）
bool custom_interfaces::topic_send(const std::string& message)
{
    if (!g_publisher_node) {
        std::cerr << "Error: topic_init() を先に呼び出してください" << std::endl;
        return false;
    }
    
    try {
        g_publisher_node->publish_message(message);
        // メッセージが確実に送信されるまで少し待つ
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error in topic_send: " << e.what() << std::endl;
        return false;
    }
}

bool custom_interfaces::topic_init(int argc, char * argv[])
{
    try {
        if (!g_rclcpp_initialized) {
            rclcpp::init(argc, argv);
            g_rclcpp_initialized = true;
        }
        
        g_publisher_node = std::make_shared<custom_interfaces::PublisherNode>();
        
        // 接続が確立されるまで少し待つ
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error in topic_init: " << e.what() << std::endl;
        return false;
    }
}

void custom_interfaces::topic_cleanup()
{
    g_publisher_node.reset();
    if (g_rclcpp_initialized) {
        rclcpp::shutdown();
        g_rclcpp_initialized = false;
    }
}

// メイン関数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    if (argc > 1 && std::string(argv[1]) == "pub") {
        auto publisher_node = std::make_shared<custom_interfaces::PublisherNode>();
        rclcpp::spin(publisher_node);
    } else if (argc > 1 && std::string(argv[1]) == "sub") {
        auto subscriber_node = std::make_shared<custom_interfaces::SubscriberNode>();
        rclcpp::spin(subscriber_node);
    } else {
        std::cout << "使用方法: " << std::endl;
        std::cout << "  " << argv[0] << " pub  # Publisherモード" << std::endl;
        std::cout << "  " << argv[0] << " sub  # Subscriberモード" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}