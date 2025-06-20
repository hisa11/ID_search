#include "serial.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // カスタム区切り文字を使用してシリアル通信を初期化
    auto serial_node = std::make_shared<SerialCommunication>(
        "serial_example_node",      // ノード名
        "/dev/ttyUSB0",            // ポート
        B9600,                     // ボーレート
        "\r\n"                     // 区切り文字（CR+LF）
    );
    
    // 初期化
    if (!serial_node->initialize()) {
        RCLCPP_ERROR(serial_node->get_logger(), "Failed to initialize serial communication");
        return 1;
    }
    
    // 受信コールバックを設定
    serial_node->setReceiveCallback([&serial_node](const std::string& message) {
        RCLCPP_INFO(serial_node->get_logger(), "Callback received: '%s'", message.c_str());
        
        // エコーバック（受信したメッセージを送り返す）
        serial_node->send("Echo: " + message);
    });
    
    // 受信開始
    serial_node->startReceiving();
    
    // 定期的にメッセージを送信するタイマー
    auto timer = serial_node->create_wall_timer(
        std::chrono::seconds(5),
        [&serial_node]() {
            static int counter = 0;
            serial_node->send("asagohan_" + std::to_string(counter++));
        }
    );
    
    RCLCPP_INFO(serial_node->get_logger(), "Serial communication example started");
    RCLCPP_INFO(serial_node->get_logger(), "Sending messages every 5 seconds...");
    
    rclcpp::spin(serial_node);
    rclcpp::shutdown();
    return 0;
}
