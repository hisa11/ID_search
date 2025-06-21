#include "topic.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @brief マルチトピックリスナーの使用例
 * 
 * この例では、複数のトピックからメッセージを受信する方法を示します。
 * - ジョイスティック入力 (/joy)
 * - コントローラーの制御信号 (/cmd_vel など)
 * - カスタムトピック (topic_name)
 */

// カスタムリスナークラス
class MultiTopicListener : public rclcpp::Node
{
public:
    MultiTopicListener() : Node("multi_topic_listener")
    {
        // 1. controller_signal_load/joy からのジョイスティック入力 (実際のトピック)
        controller_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/controller_signal_load/joy",
            10,
            std::bind(&MultiTopicListener::controller_joy_callback, this, std::placeholders::_1)
        );
        
        // 2. 一般的な /joy トピック (参考用)
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&MultiTopicListener::joy_callback, this, std::placeholders::_1)
        );
        
        // 3. Twist メッセージ (速度制御など) のSubscriber
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&MultiTopicListener::cmd_vel_callback, this, std::placeholders::_1)
        );
        
        // 4. カスタムStringメッセージのSubscriber
        string_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic_name",
            10,
            std::bind(&MultiTopicListener::string_callback, this, std::placeholders::_1)
        );
        
        // 5. controller_data (文字列データがある場合)
        controller_data_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/controller_signal_load/controller_data",
            10,
            std::bind(&MultiTopicListener::controller_data_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "MultiTopicListener initialized");
        RCLCPP_INFO(this->get_logger(), "Listening to:");
        RCLCPP_INFO(this->get_logger(), "  - /controller_signal_load/joy (sensor_msgs/msg/Joy) <- MAIN");
        RCLCPP_INFO(this->get_logger(), "  - /joy (sensor_msgs/msg/Joy)");
        RCLCPP_INFO(this->get_logger(), "  - /cmd_vel (geometry_msgs/msg/Twist)");
        RCLCPP_INFO(this->get_logger(), "  - topic_name (std_msgs/msg/String)");
        RCLCPP_INFO(this->get_logger(), "  - /controller_signal_load/controller_data (std_msgs/msg/String)");
    }

private:
    void controller_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Controller Joy received: %zu axes, %zu buttons (Frame: %s)", 
            msg->axes.size(), msg->buttons.size(), msg->header.frame_id.c_str());
        
        // タイムスタンプ表示
        std::cout << "Timestamp: " << msg->header.stamp.sec << "." 
                  << msg->header.stamp.nanosec << std::endl;
        
        // ジョイスティック軸の値を表示
        if (!msg->axes.empty()) {
            std::cout << "Axes: ";
            for (size_t i = 0; i < msg->axes.size(); ++i) {
                std::cout << "[" << i << "]=" << msg->axes[i] << " ";
            }
            std::cout << std::endl;
        }
        
        // ボタンの状態を表示
        if (!msg->buttons.empty()) {
            std::cout << "Buttons: ";
            bool any_pressed = false;
            for (size_t i = 0; i < msg->buttons.size(); ++i) {
                if (msg->buttons[i] == 1) {
                    std::cout << "[" << i << "]=ON ";
                    any_pressed = true;
                }
            }
            if (!any_pressed) {
                std::cout << "(No buttons pressed)";
            }
            std::cout << std::endl;
        }
        
        // ボタンの名前付き表示（PS4コントローラーの場合）
        if (msg->buttons.size() >= 13) {
            std::vector<std::string> button_names = {
                "X", "Circle", "Triangle", "Square", 
                "L1", "R1", "L2", "R2", 
                "Share", "Options", "PS", 
                "Left", "Right", "Up", "Down", "L3", "R3"
            };
            
            std::cout << "Named buttons: ";
            bool any_named_pressed = false;
            for (size_t i = 0; i < std::min(msg->buttons.size(), button_names.size()); ++i) {
                if (msg->buttons[i] == 1) {
                    std::cout << button_names[i] << " ";
                    any_named_pressed = true;
                }
            }
            if (!any_named_pressed) {
                std::cout << "(None)";
            }
            std::cout << std::endl;
        }
        
        std::cout << "----------------------------------------" << std::endl;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Joy received: %zu axes, %zu buttons", 
            msg->axes.size(), msg->buttons.size());
        
        // ジョイスティックの詳細情報を表示
        if (!msg->axes.empty()) {
            std::cout << "Axes: ";
            for (size_t i = 0; i < msg->axes.size(); ++i) {
                std::cout << "[" << i << "]=" << msg->axes[i] << " ";
            }
            std::cout << std::endl;
        }
        
        if (!msg->buttons.empty()) {
            std::cout << "Buttons: ";
            bool any_pressed = false;
            for (size_t i = 0; i < msg->buttons.size(); ++i) {
                if (msg->buttons[i] == 1) {  // ボタンが押されている場合
                    std::cout << "[" << i << "]=ON ";
                    any_pressed = true;
                }
            }
            if (!any_pressed) {
                std::cout << "(No buttons pressed)";
            }
            std::cout << std::endl;
        }
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Cmd_vel received: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
            msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "String topic received: '%s'", msg->data.c_str());
    }
    
    void controller_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Controller signal received: '%s'", msg->data.c_str());
    }
    
    void controller_data_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Controller data received: '%s'", msg->data.c_str());
        
        // データの解析例（提供されたコードの形式に基づく）
        std::string data = msg->data;
        std::cout << "Raw controller data: " << data << std::endl;
        
        // n:lx:ly:rx:ry| の形式を解析
        if (data.find("n:") == 0) {
            size_t pos = 2; // "n:" の後から開始
            std::vector<std::string> values;
            
            while (pos < data.length() && data[pos] != '|') {
                size_t next_colon = data.find(':', pos);
                if (next_colon == std::string::npos) {
                    next_colon = data.find('|', pos);
                }
                if (next_colon != std::string::npos) {
                    values.push_back(data.substr(pos, next_colon - pos));
                    pos = next_colon + 1;
                } else {
                    break;
                }
            }
            
            if (values.size() >= 4) {
                std::cout << "Joystick values - LX:" << values[0] 
                         << " LY:" << values[1] 
                         << " RX:" << values[2] 
                         << " RY:" << values[3] << std::endl;
            }
        }
        
        // ボタン情報の解析 (keys:p| or keys:no_p| の形式)
        std::vector<std::string> button_names = {"cr", "ci", "tri", "sq", "L1", "R1", "L2", "R2", 
                                               "SH", "OP", "PS", "l", "r", "u", "d", "L3", "R3"};
        
        for (const auto& button : button_names) {
            std::string pressed_pattern = button + ":p|";
            if (data.find(pressed_pattern) != std::string::npos) {
                std::cout << "Button " << button << " is PRESSED" << std::endl;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_data_subscription_;
};
int main(int argc, char * argv[])
{
    // ROS2を初期化
    rclcpp::init(argc, argv);
    
    std::cout << "=== ROS2 Multi-Topic Listener Example ===" << std::endl;
    std::cout << "複数のトピックからメッセージを受信します..." << std::endl;
    std::cout << "終了するには Ctrl+C を押してください" << std::endl;
    std::cout << "==========================================" << std::endl;
    
    try {
        // MultiTopicListenerのインスタンスを作成
        auto listener_node = std::make_shared<MultiTopicListener>();
        
        // ノードをスピンして、メッセージの受信を開始
        rclcpp::spin(listener_node);
        
    } catch (const std::exception& e) {
        std::cerr << "エラーが発生しました: " << e.what() << std::endl;
    }
    
    std::cout << "リスナーを終了します..." << std::endl;
    
    // ROS2をシャットダウン
    rclcpp::shutdown();
    return 0;
}

/**
 * 使用方法とトピック調査の説明:
 * 
 * 1. 実際のトピック名を調べる:
 *    ros2 topic list
 *    
 * 2. 特定のノードが発行するトピックを調べる:
 *    ros2 node info /controller_signal_load/joy_node
 *    ros2 node info /controller_signal_load/listener
 *    
 * 3. トピックのメッセージ型を調べる:
 *    ros2 topic info /joy
 *    ros2 topic info /cmd_vel
 *    
 * 4. メッセージの内容を直接確認:
 *    ros2 topic echo /joy
 *    ros2 topic echo /cmd_vel
 * 
 * 5. このプログラムを実行:
 *    ros2 run my_cpp_pkg topic_listener_example
 * 
 * 注意: 実際のトピック名やメッセージ型は、あなたのシステムの設定により異なる場合があります。
 * 上記のコマンドで実際の値を確認してから、必要に応じてコードを修正してください。
 */