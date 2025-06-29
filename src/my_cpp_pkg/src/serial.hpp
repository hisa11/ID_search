#ifndef MY_CPP_PKG_SERIAL_HPP
#define MY_CPP_PKG_SERIAL_HPP

#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <vector>
#include <functional>
#include <memory>

class SerialCommunication : public rclcpp::Node {
public:
    using ReceiveCallback = std::function<void(const std::string&)>;

    // デフォルトの区切り文字は改行文字
    SerialCommunication(const std::string& node_name, 
                       const std::string& port = "/dev/ttyUSB0", 
                       speed_t baudrate = B9600,
                       const std::string& delimiter = "\n");
    
    ~SerialCommunication();

    // 初期化
    bool initialize();

    // 送信メソッド（区切り文字を自動で追加）
    bool send(const std::string& message);

    // 送信メソッド（区切り文字なし）
    bool sendRaw(const std::string& message);

    // 受信コールバックを設定
    void setReceiveCallback(ReceiveCallback callback);

    // 受信を開始
    void startReceiving();

    // 受信を停止
    void stopReceiving();

    // ポートの状態を取得
    bool isOpen() const;

    // 静的メソッド: 指定されたポートリストからデバイスをスキャンして接続
    static std::vector<std::shared_ptr<SerialCommunication>> scanAndConnectDevices(
        const std::vector<std::string>& ports,
        speed_t baudrate = B9600,
        const std::string& delimiter = "|"
    );

private:
    std::string port_;
    speed_t baudrate_;
    std::string delimiter_;
    int serial_fd_;
    bool is_receiving_;
    std::string receive_buffer_;
    ReceiveCallback receive_callback_;
    rclcpp::TimerBase::SharedPtr receive_timer_;

    // ポートの設定
    bool configure_port();

    // 受信処理
    void receive_process();

    // バッファから完全なメッセージを抽出
    std::vector<std::string> extract_messages();
};

#endif // MY_CPP_PKG_SERIAL_HPP