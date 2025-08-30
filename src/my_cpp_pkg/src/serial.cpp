#include "serial.hpp"
#include <chrono>
#include <sstream> // std::stringstream のために追加

speed_t convert_baudrate(const std::string& baudrate_str) {
    try {
        int baudrate = std::stoi(baudrate_str);
        switch (baudrate) {
            case 9600:    return B9600;
            case 19200:   return B19200;
            case 38400:   return B38400;
            case 57600:   return B57600;
            case 115200:  return B115200;
            case 230400:  return B230400;
            default:      return B115200;
        }
    } catch (const std::exception& e) {
        return B115200; // 変換失敗時はデフォルト
    }
}

SerialCommunication::SerialCommunication(const std::string& node_name, 
                                       const std::string& port, 
                                       speed_t baudrate,
                                       const std::string& delimiter)
    : Node(node_name), port_(port), baudrate_(baudrate), delimiter_(delimiter),
      serial_fd_(-1), is_receiving_(false) {
    
    RCLCPP_INFO(this->get_logger(), "SerialCommunication node created");
    RCLCPP_INFO(this->get_logger(), "Port: %s, Delimiter: '%s'", port_.c_str(), delimiter_.c_str());
}

SerialCommunication::~SerialCommunication() {
    stopReceiving();
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
}

bool SerialCommunication::initialize() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
        return false;
    }

    if (!configure_port()) {
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
    return true;
}

bool SerialCommunication::configure_port() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        return false;
    }

    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

    tty.c_cflag |= (CLOCAL | CREAD);    // 有効化
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8ビット
    tty.c_cflag &= ~PARENB;             // パリティなし
    tty.c_cflag &= ~CSTOPB;             // ストップビット1
    tty.c_cflag &= ~CRTSCTS;            // フロー制御なし

    tty.c_lflag = 0;                    // 非カノニカルモード
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;                 // ノンブロッキング読み取り
    tty.c_cc[VTIME] = 1;                // 0.1秒のタイムアウト

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        return false;
    }

    return true;
}

bool SerialCommunication::send(const std::string& message) {
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not initialized");
        return false;
    }

    std::string full_message = message + delimiter_;
    ssize_t bytes_written = write(serial_fd_, full_message.c_str(), full_message.length());
    
    if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Sent: '%s' (with delimiter)", message.c_str());
    return true;
}

bool SerialCommunication::sendRaw(const std::string& message) {
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not initialized");
        return false;
    }

    ssize_t bytes_written = write(serial_fd_, message.c_str(), message.length());
    
    if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Sent: '%s' (raw, no delimiter)", message.c_str());
    return true;
}

void SerialCommunication::setReceiveCallback(ReceiveCallback callback) {
    receive_callback_ = callback;
}

void SerialCommunication::startReceiving() {
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not initialized");
        return;
    }

    if (is_receiving_) {
        RCLCPP_WARN(this->get_logger(), "Already receiving");
        return;
    }

    is_receiving_ = true;
    receive_buffer_.clear();

    // 10msごとに受信をチェック
    receive_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SerialCommunication::receive_process, this)
    );

    RCLCPP_INFO(this->get_logger(), "Started receiving data - timer created");
    RCLCPP_INFO(this->get_logger(), "Serial fd: %d, is_receiving: %d", serial_fd_, is_receiving_);
}

void SerialCommunication::stopReceiving() {
    if (is_receiving_) {
        is_receiving_ = false;
        if (receive_timer_) {
            receive_timer_->cancel();
            receive_timer_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "Stopped receiving data");
    }
}

void SerialCommunication::receive_process() {
    if (!is_receiving_ || serial_fd_ < 0) {
        // より頻繁にデバッグログを出力
        static int debug_count = 0;
        if (debug_count++ % 100 == 0) {  // 1秒ごとに出力
            RCLCPP_INFO(this->get_logger(), "receive_process: not receiving or fd invalid (is_receiving_=%d, fd=%d)", 
                        is_receiving_, serial_fd_);
        }
        return;
    }

    char buffer[256];
    ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
    
    // すべての読み取り試行をログ出力
    static int read_count = 0;
    if (read_count++ % 100 == 0) {  // 1秒ごとに出力
        RCLCPP_INFO(this->get_logger(), "receive_process: read attempt %d, returned %zd bytes", read_count, bytes_read);
    }
    
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        RCLCPP_INFO(this->get_logger(), "Read %zd bytes: '%s'", bytes_read, buffer);
        
        // 16進ダンプも追加
        std::stringstream hex_dump;
        for (ssize_t i = 0; i < bytes_read; ++i) {
            hex_dump << "0x" << std::hex << std::uppercase << (unsigned char)buffer[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Hex dump: %s", hex_dump.str().c_str());
        
        receive_buffer_ += std::string(buffer);
        RCLCPP_INFO(this->get_logger(), "Buffer content: '%s'", receive_buffer_.c_str());
        
        // バッファから完全なメッセージを抽出
        std::vector<std::string> messages = extract_messages();
        
        // コールバックがセットされていれば、各メッセージに対して呼び出し
        if (receive_callback_) {
            for (const auto& message : messages) {
                RCLCPP_INFO(this->get_logger(), "Calling callback with: '%s'", message.c_str());
                receive_callback_(message);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No receive callback set");
        }
    } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "read() returned error: %s", strerror(errno));
        }
    } else {
        // bytes_read == 0
        static int zero_read_count = 0;
        if (zero_read_count++ % 1000 == 0) {  // 10秒ごとに出力
            RCLCPP_INFO(this->get_logger(), "read() returned 0 bytes (count: %d)", zero_read_count);
        }
    }
}

std::vector<std::string> SerialCommunication::extract_messages() {
    std::vector<std::string> messages;
    size_t pos = 0;
    
    while ((pos = receive_buffer_.find(delimiter_)) != std::string::npos) {
        std::string message = receive_buffer_.substr(0, pos + delimiter_.length()); // 区切り文字も含める
        if (!message.empty()) {
            messages.push_back(message);
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", message.c_str());
        }
        receive_buffer_.erase(0, pos + delimiter_.length());
    }
    
    return messages;
}

bool SerialCommunication::isOpen() const {
    return serial_fd_ >= 0;
}

bool SerialCommunication::reconnect() {
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to %s...", port_.c_str());
    
    // 既存の接続を閉じる
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(this->get_logger(), "Closed existing connection");
    }
    
    // 受信を停止
    bool was_receiving = is_receiving_;
    if (is_receiving_) {
        stopReceiving();
    }
    
    // バッファをクリア
    receive_buffer_.clear();
    
    // 再接続を試行
    bool success = initialize();
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Successfully reconnected to %s", port_.c_str());
        // 受信を再開
        if (was_receiving) {
            startReceiving();
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect to %s", port_.c_str());
    }
    
    return success;
}

bool SerialCommunication::checkConnection() {
    if (!isOpen()) {
        return false;
    }
    
    // 簡単な接続テスト（実際にデータを送信してみる）
    const std::string test_command = "";  // 空のコマンド
    ssize_t bytes_written = write(serial_fd_, test_command.c_str(), test_command.length());
    
    if (bytes_written < 0) {
        RCLCPP_WARN(this->get_logger(), "Connection check failed for %s", port_.c_str());
        return false;
    }
    
    return true;
}

void SerialCommunication::flushPort() {
    if (serial_fd_ >= 0) {
        // 送信バッファをフラッシュ
        tcflush(serial_fd_, TCIOFLUSH);
        // 受信バッファもクリア
        receive_buffer_.clear();
        RCLCPP_INFO(this->get_logger(), "シリアルポートをフラッシュしました: %s", port_.c_str());
    }
}

// 静的メソッド: 指定されたポートリストからデバイスをスキャンして接続
std::vector<std::shared_ptr<SerialCommunication>> SerialCommunication::scanAndConnectDevices(
    const std::vector<std::string>& ports,
    speed_t baudrate,
    const std::string& delimiter
) {
    std::vector<std::shared_ptr<SerialCommunication>> connected_devices;
    
    RCLCPP_INFO(rclcpp::get_logger("SerialCommunication"), "Scanning for serial devices...");
    
    // シリアルデバイスを接続
    for (const auto& port : ports) {
        RCLCPP_INFO(rclcpp::get_logger("SerialCommunication"), "Trying to connect to %s", port.c_str());
        
        std::string node_name = "device" + std::to_string(connected_devices.size()) + "_node";
        auto device = std::make_shared<SerialCommunication>(node_name, port, baudrate, delimiter);
            
        if (device->initialize()) {
            RCLCPP_INFO(rclcpp::get_logger("SerialCommunication"), "Successfully connected to %s", port.c_str());
            connected_devices.push_back(device);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("SerialCommunication"), "Failed to connect to %s", port.c_str());
        }
    }

    if (connected_devices.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), "No devices connected");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("SerialCommunication"), "Connected %zu device(s)", connected_devices.size());
    }
    
    return connected_devices;
}