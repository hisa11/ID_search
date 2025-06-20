#include "serial.hpp"
#include <chrono>

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
        std::string message = receive_buffer_.substr(0, pos);
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