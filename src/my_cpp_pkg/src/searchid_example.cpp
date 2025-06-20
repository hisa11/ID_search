#include "serchID.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // デバイスを作成して自動登録
    auto arduino = SearchID::createAndRegisterDevice(
        "arduino",                  // デバイス名
        "arduino_node",            // ノード名
        "/dev/ttyUSB0",           // ポート
        B9600,                    // ボーレート
        "|\n"                    // 区切り文字
    );
    
    auto sensor = SearchID::createAndRegisterDevice(
        "sensor",                  // デバイス名
        "sensor_node",            // ノード名
        "/dev/ttyUSB1",           // ポート
        B115200,                  // ボーレート
        "|\n"                      // 区切り文字
    );
    
    if (!arduino || !sensor) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize devices");
        return 1;
    }
    
    // 受信コールバックを設定
    SearchID::DeviceManager::getInstance().setDeviceCallback("arduino", 
        [](const std::string& message) {
            RCLCPP_INFO(rclcpp::get_logger("Arduino"), "Received: %s", message.c_str());
            
            // 特定の応答を送信
            if (message == "ping") {
                SearchID::sendByName("arduino", "pong");
            }
        });
    
    SearchID::DeviceManager::getInstance().setDeviceCallback("sensor", 
        [](const std::string& message) {
            RCLCPP_INFO(rclcpp::get_logger("Sensor"), "Sensor data: %s", message.c_str());
            
            // センサーデータを他のデバイスに転送
            SearchID::sendByName("arduino", "sensor_data:" + message);
        });
    
    // 受信開始
    SearchID::DeviceManager::getInstance().startDeviceReceiving("arduino");
    SearchID::DeviceManager::getInstance().startDeviceReceiving("sensor");
    
    // デバイス状態を表示
    SearchID::DeviceManager::getInstance().printDeviceStatus();
    
    // デバイス名を設定
    SearchID::setDeviceName("MyPC");
    
    // 5秒後に全デバイスを開始
    auto start_timer = arduino->create_wall_timer(
        std::chrono::seconds(5),
        []() {
            static bool started = false;
            if (!started) {
                RCLCPP_INFO(rclcpp::get_logger("main"), "Starting all devices...");
                SearchID::start(); // "1,MyPC,0," を全デバイスに送信
                started = true;
            }
        }
    );
    
    // 定期的にコマンドを送信
    auto timer = arduino->create_wall_timer(
        std::chrono::seconds(5),
        []() {
            static int counter = 0;
            
            // Arduinoに特定の値を送信
            SearchID::sendByName("arduino", "cmd:" + std::to_string(counter++));
            
            // 全デバイスにハートビートを送信
            if (counter % 3 == 0) {
                SearchID::sendToAll("heartbeat");
            }
            
            // デバイス一覧を表示
            auto devices = SearchID::getDevices();
            std::string device_list = "Devices: ";
            for (const auto& device : devices) {
                device_list += device + " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("main"), "%s", device_list.c_str());
        }
    );
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "SearchID example started");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Available commands:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "- Automatic ping/pong with arduino");
    RCLCPP_INFO(rclcpp::get_logger("main"), "- Sensor data forwarding");
    RCLCPP_INFO(rclcpp::get_logger("main"), "- Periodic commands and heartbeat");
    
    rclcpp::spin(arduino);
    rclcpp::shutdown();
    return 0;
}
