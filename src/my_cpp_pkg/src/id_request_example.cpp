#include "serchID.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // デバイス名を設定
    SearchID::setDeviceName("PC1");

    // USBデバイスを登録
    auto device1 = SearchID::createAndRegisterDevice(
        "arduino1", "arduino1_node", "/dev/ttyACM0", B9600, "\r\n");

    auto device2 = SearchID::createAndRegisterDevice(
        "arduino2", "arduino2_node", "/dev/ttyACM1", B9600, "\r\n");

    if (!device1 && !device2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices initialized successfully");
        rclcpp::shutdown();
        return 1;
    }

    // 初期化成功したデバイスで受信を開始
    if (device1)
    {
        SearchID::DeviceManager::getInstance().startDeviceReceiving("arduino1");
        RCLCPP_INFO(rclcpp::get_logger("main"), "Arduino1 receiving started");
    }

    if (device2)
    {
        SearchID::DeviceManager::getInstance().startDeviceReceiving("arduino2");
        RCLCPP_INFO(rclcpp::get_logger("main"), "Arduino2 receiving started");
    }

    // ノードを作成してタイマーを設定
    auto node = rclcpp::Node::make_shared("id_request_example");

    // 5秒後にIDリクエストを送信
    auto id_timer = node->create_wall_timer(
        std::chrono::seconds(5),
        []()
        {
            static bool id_sent = false;
            if (!id_sent)
            {
                RCLCPP_INFO(rclcpp::get_logger("main"), "Sending ID requests...");

                // 各デバイスにIDリクエストを送信
                auto devices = SearchID::getDevices();
                for (const auto &device_name : devices)
                {
                    if (SearchID::isOpen(device_name))
                    {
                        SearchID::sendIDRequest(device_name, "ID");
                        RCLCPP_INFO(rclcpp::get_logger("main"),
                                    "ID request sent to: %s", device_name.c_str());
                    }
                }

                id_sent = true;
            }
        });

    // 定期的に別のIDリクエストを送信（テスト用）
    auto periodic_timer = node->create_wall_timer(
        std::chrono::seconds(15),
        []()
        {
            auto devices = SearchID::getDevices();
            for (const auto &device_name : devices)
            {
                if (SearchID::isOpen(device_name))
                {
                    SearchID::sendIDRequest(device_name, "STATUS");
                    RCLCPP_INFO(rclcpp::get_logger("main"),
                                "STATUS request sent to: %s", device_name.c_str());
                }
            }
        });

    RCLCPP_INFO(rclcpp::get_logger("main"), "ID Request Example started");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Will send ID requests in 5 seconds");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Expected response format: '101,value1,value2-value3'");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Will parse and display: value1,value2,value3");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
