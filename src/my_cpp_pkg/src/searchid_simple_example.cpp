#include "serial.hpp"
#include "serchID.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include "sensor_msgs/msg/joy.hpp"
#include <array>

/*
使用例:
1. 個別送信:
   SearchID::ID::send("nucleo1", "asagohan");
   SearchID::ID::send("nucleo2", "bangohan");

2. 複数デバイスに同じメッセージ:
   std::vector<std::string> devices = {"nucleo1", "nucleo2"};
   SearchID::ID::sendToMultiple(devices, "hello");

3. 利用可能なIDを確認:
   auto ids = SearchID::ID::getAvailableIDs();
   for (const auto& id : ids) {
       std::cout << "Device ID: " << id << std::endl;
   }

4. 戻り値での成功/失敗判定:
   if (SearchID::ID::send("nucleo1", "test")) {
       std::cout << "送信成功" << std::endl;
   } else {
       std::cout << "送信失敗" << std::endl;
   }
*/

// コントローラーデータを処理するクラス
class ControllerToNucleo : public rclcpp::Node
{
public:
    ControllerToNucleo() : Node("controller_to_nucleo"), send_counter_(0)
    {
        // controller_signal_load/joy からのジョイスティック入力を受信
        controller_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/controller_signal_load/joy",
            10,
            std::bind(&ControllerToNucleo::controller_joy_callback, this, std::placeholders::_1));

        // ボタン名の配列を初期化
        keys = {"cr", "ci", "tri", "sq", "L1", "R1", "L2", "R2", "SH", "OP", "PS", "l", "r", "u", "d", "L3", "R3"};

        RCLCPP_INFO(this->get_logger(), "ControllerToNucleo initialized");
        RCLCPP_INFO(this->get_logger(), "Listening to /controller_signal_load/joy");
        RCLCPP_INFO(this->get_logger(), "Sending data every 10th callback (1/10 frequency)");
    }

private:
    void controller_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // カウンターをインクリメント
        send_counter_++;

        // 10回に1回だけ送信処理を実行
        if (send_counter_ % 10 != 0)
        {
            return; // 送信をスキップ
        }

        RCLCPP_INFO(this->get_logger(), "Processing callback #%d (sending data)", send_counter_);

        // ジョイスティック軸データの処理
        // PS4コントローラーの軸マッピング: [0]=LX, [1]=LY, [2]=RX, [3]=RY
        std::string joy_msg;
        if (msg->axes.size() >= 4)
        {
            // 軸データを整数に変換（-32767 to 32767 の範囲）
            double lx = static_cast<double>(msg->axes[0] * 1.000);
            double ly = static_cast<double>(msg->axes[1] * 1.000);
            double rx = static_cast<double>(msg->axes[3] * 1.000);
            double ry = static_cast<double>(msg->axes[4] * 1.000);

            joy_msg = "n:" + std::to_string(lx) + ":" + std::to_string(ly) + ":" + std::to_string(rx) + ":" + std::to_string(ry) + "|\n";
        }
        else
        {
            joy_msg = "n:0:0:0:0|\n";
        }

        // ボタンデータの処理
        std::vector<std::string> key_messages;
        for (int i = 0; i < 17; i++)
        {
            std::string key_message;
            if (i < static_cast<int>(msg->buttons.size()) && msg->buttons[i])
            {
                key_message = keys[i] + ":p|\n";
            }
            else
            {
                key_message = keys[i] + ":no_p|\n";
            }
            key_messages.push_back(key_message);
        }

        // 完全なメッセージを構築
        std::string full_message = joy_msg;
        for (const auto &key_msg : key_messages)
        {
            full_message += key_msg;
        }

        // nucleo1に送信
        if (SearchID::ID::send("nucleo1", full_message))
        {
            RCLCPP_INFO(this->get_logger(), "Sent to nucleo1: %s", full_message.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send to nucleo1");
        }

        // デバッグ用: 押されたボタンを表示
        std::cout << "Pressed buttons: ";
        bool any_pressed = false;
        for (size_t i = 0; i < std::min(msg->buttons.size(), keys.size()); ++i)
        {
            if (msg->buttons[i])
            {
                std::cout << keys[i] << " ";
                any_pressed = true;
            }
        }
        if (!any_pressed)
        {
            std::cout << "(None)";
        }
        std::cout << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_joy_subscription_;
    std::array<std::string, 17> keys;
    int send_counter_; // 送信カウンター
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // シリアル通信を宣言 - 利用可能な全ポートを試行
    std::vector<std::string> ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};
    std::vector<std::shared_ptr<SerialCommunication>> connected_devices;

    // SerialCommunicationの静的メソッドを使用してデバイススキャン
    connected_devices = SerialCommunication::scanAndConnectDevices(ports, B115200, "|");

    if (connected_devices.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices connected");
        return 1;
    }

    // SearchID::DeviceManagerを使用してデバイスを一括登録
    SearchID::DeviceManager &manager = SearchID::DeviceManager::getInstance();
    manager.registerDevices(connected_devices);

    // PC名を設定
    manager.setDeviceName("PC1");

    // コントローラー処理ノードを作成
    auto controller_node = std::make_shared<ControllerToNucleo>();

    // ROSのエグゼキューターを使ってスピン（受信処理のため）
    rclcpp::executors::MultiThreadedExecutor executor;
    for (auto &device : connected_devices)
    {
        executor.add_node(device);
    }
    // コントローラーノードも追加
    executor.add_node(controller_node);

    // DeviceManagerのstartメソッドを使用してID探索を実行
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting device ID discovery...");
    manager.start(500); // 500msのタイムアウト

    // ID応答を待機
    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for ID responses...");

    // 1秒間待機してID応答を収集
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1))
    {
        executor.spin_some(std::chrono::milliseconds(10));

        // 全デバイスから応答があったかチェック
        auto device_id_map = manager.getDeviceIDMap();
        if (device_id_map.size() >= connected_devices.size())
        {
            RCLCPP_INFO(rclcpp::get_logger("main"), "All devices responded, proceeding early");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 利用可能なIDを表示
    auto available_ids = SearchID::ID::getAvailableIDs();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Available device IDs: %zu devices", available_ids.size());
    for (const auto &id : available_ids)
    {
        RCLCPP_INFO(rclcpp::get_logger("main"), "  - %s", id.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting controller data relay to nucleo1...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Press Ctrl+C to stop");

    // メインループ: コントローラーデータを継続的に受信・送信
    try
    {
        executor.spin();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "RCL Error: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}