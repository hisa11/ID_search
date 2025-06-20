#include "serial.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // シリアル通信デバイスを作成
    auto serial_device = std::make_shared<SerialCommunication>("serial_test_node", "/dev/ttyACM0", 9600, "|");
    
    if (!serial_device->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize serial device");
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Serial device initialized successfully");

    // 受信コールバックを設定
    serial_device->setReceiveCallback([](const std::string& message) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "RECEIVED: '%s'", message.c_str());
        std::cout << "RECEIVED: " << message << std::endl;
    });

    // 受信を開始
    serial_device->startReceiving();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Started receiving");

    // ID探索メッセージを送信
    std::string id_message = "1,PC1,0,|";
    serial_device->sendRaw(id_message);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Sent ID request: %s", id_message.c_str());

    // ノードをスピン
    RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning node...");
    rclcpp::spin(serial_device);

    rclcpp::shutdown();
    return 0;
}
