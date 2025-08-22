#include "integration.hpp"
#include <chrono>
#include <sstream>
#include <algorithm>
#include <numeric>

IntegratedCommunicationSystem::IntegratedCommunicationSystem(
    const std::string& node_name, const std::vector<std::shared_ptr<SerialCommunication>>& serial_comms)
    : rclcpp::Node(node_name), self_node_name_(node_name), serial_comms_(serial_comms),
      shutdown_flag_(false),
      random_engine_(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
      distribution_(1, INT64_MAX)
{
    server_ = this->create_service<my_cpp_pkg::srv::DataExchange>(
        self_node_name_,
        std::bind(&IntegratedCommunicationSystem::handle_request, this, std::placeholders::_1, std::placeholders::_2));
        
    for(const auto& dev : serial_comms_) {
        dev->setReceiveCallback([this, port=dev->getPort()](const std::string& data){
            this->handle_serial_data(port, data);
        });
        dev->startReceiving();
    }
    worker_thread_ = std::thread([this](){ this->processRequestQueue(); });
    
    // 自動マイコン間通信中継機能を有効化
    setupAutomaticMicrocontrollerRelay();
}

IntegratedCommunicationSystem::~IntegratedCommunicationSystem()
{
    shutdown_flag_ = true;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
}

void IntegratedCommunicationSystem::setDataHandler(DataHandlerCallback callback) { data_handler_callback_ = callback; }

std::vector<std::string> IntegratedCommunicationSystem::discoverMicrocontrollerIDs(int timeout_ms)
{
    discovery_promise_ = std::make_shared<std::promise<bool>>();
    auto future = discovery_promise_->get_future();
    {
        std::lock_guard<std::mutex> lock(discovery_mutex_);
        discovered_microcontrollers_.clear();
        microcontroller_device_map_.clear(); 
    }
    std::string id_request_command = "1," + self_node_name_ + ",0,|";
    for(auto& dev : serial_comms_) {
        dev->sendRaw(id_request_command);
    }
    future.wait_for(std::chrono::milliseconds(timeout_ms));
    discovery_promise_ = nullptr;
    return getDiscoveredMicrocontrollers();
}

std::vector<std::string> IntegratedCommunicationSystem::getDiscoveredMicrocontrollers() const {
    std::lock_guard<std::mutex> lock(discovery_mutex_);
    return discovered_microcontrollers_;
}

// ★★★ 非同期送信の実装をこのシグネチャに統一 ★★★
void IntegratedCommunicationSystem::sendToNodeAsync(
    const std::string& target_node, const std::string& final_destination,
    const std::vector<std::string>& data, const std::string& message, int64_t request_type)
{
    auto client = get_client(target_node);
    if (!client->service_is_ready()) { 
        RCLCPP_ERROR(this->get_logger(), "サービス %s が準備されていません", target_node.c_str());
        return; 
    }

    auto request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
    request->request_type = request_type;
    request->source_node = self_node_name_;
    request->destination_node = final_destination;
    request->string_values = data;
    request->transaction_id = generate_transaction_id();
    request->message = message;
    
    RCLCPP_INFO(this->get_logger(), "非同期送信開始: %s -> request_type=%ld, data_size=%zu", 
                target_node.c_str(), request_type, data.size());
    
    client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "非同期送信完了: %s", target_node.c_str());
}

bool IntegratedCommunicationSystem::sendToMicrocontroller(const std::string& microcontroller_id, const std::string& data) {
    std::shared_ptr<SerialCommunication> serial_device = nullptr;
    {
        std::lock_guard<std::mutex> lock(discovery_mutex_);
        auto it = microcontroller_device_map_.find(microcontroller_id);
        if (it == microcontroller_device_map_.end()) {
            RCLCPP_ERROR(this->get_logger(), "マイコン '%s' に対応するシリアルデバイスが見つかりません。", microcontroller_id.c_str());
            return false;
        }
        serial_device = it->second;
    }
    return serial_device->sendRaw(data);
}

void IntegratedCommunicationSystem::handle_request(
    const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
    std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "サービスリクエスト受信: source=%s, request_type=%ld, data_size=%zu", 
                request->source_node.c_str(), request->request_type, request->string_values.size());
                
    // request_type=102の場合はデータハンドラーに直接渡す
    if (request->request_type == 102) {
        RCLCPP_INFO(this->get_logger(), "request_type=102 を検出。データハンドラーに転送します。");
        if (data_handler_callback_) {
            data_handler_callback_(request);
        } else {
            RCLCPP_WARN(this->get_logger(), "データハンドラーが設定されていません");
        }
    } else {
        // 通常のキューイング処理
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(request);
        }
        queue_cv_.notify_one();
    }
    
    response->response_type = 200; // Accepted
    response->return_node = self_node_name_;
    response->transaction_id = request->transaction_id;
}

void IntegratedCommunicationSystem::processRequestQueue()
{
    while(!shutdown_flag_) {
        std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]{ return !request_queue_.empty() || shutdown_flag_; });
            if (shutdown_flag_) return;
            request = request_queue_.front();
            request_queue_.pop();
        }

        const auto& req_data = request->string_values;
        if (req_data.size() < 5) continue;

        std::string destination_mc = req_data[2];
        std::stringstream ss;
        ss << req_data[0] << "," << req_data[1] << "," << req_data[2] << ","
           << request->transaction_id << "," << req_data[3] << ",";
        for (size_t i = 4; i < req_data.size(); ++i) {
            ss << req_data[i] << (i == req_data.size() - 1 ? "" : ",");
        }
        ss << "|";
        
        {
            std::lock_guard<std::mutex> lock(forward_map_mutex_);
            pending_forward_map_[request->transaction_id] = request->source_node;
        }

        RCLCPP_INFO(this->get_logger(), "マイコン %s にデータ送信: %s", destination_mc.c_str(), ss.str().c_str());
        bool sent = sendToMicrocontroller(destination_mc, ss.str());
        RCLCPP_INFO(this->get_logger(), "送信結果: %s", sent ? "成功" : "失敗");
    }
}

void IntegratedCommunicationSystem::handle_serial_data(const std::string& device_port, const std::string& raw_data) {
    std::string data = raw_data;
    if (data.empty() || data.back() != '|') return;
    data.pop_back();

    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string item;
    while(std::getline(ss, item, ',')) parts.push_back(item);

    // ID応答: "101,nucleo1,0"
    if (parts.size() >= 2 && parts[0] == "101") {
        std::string mc_id = parts[1];
        bool newly_discovered = false;
        {
            std::lock_guard<std::mutex> lock(discovery_mutex_);
            if (std::find(discovered_microcontrollers_.begin(), discovered_microcontrollers_.end(), mc_id) == discovered_microcontrollers_.end()) {
                discovered_microcontrollers_.push_back(mc_id);
                for(const auto& dev : serial_comms_) {
                    if(dev->getPort() == device_port) {
                        microcontroller_device_map_[mc_id] = dev;
                        break;
                    }
                }
                newly_discovered = true;
            }
        }
        if (newly_discovered) {
            RCLCPP_INFO(this->get_logger(), "✅ マイコンID '%s' をポート %s で発見・登録しました。", mc_id.c_str(), device_port.c_str());
            if (discovery_promise_) { discovery_promise_->set_value(true); }
        }
        return;
    }

    // 通常応答: "102,PC2,TID,0" またはマイコン間通信応答
    if (parts.size() >= 4 && parts[0] == "102") {
        try {
            int64_t tid = std::stoll(parts[2]);
            std::string original_sender;
            
            // まずPC経由の通信をチェック
            {
                std::lock_guard<std::mutex> lock(forward_map_mutex_);
                if (pending_forward_map_.count(tid)) {
                    original_sender = pending_forward_map_[tid];
                    pending_forward_map_.erase(tid);
                    
                    RCLCPP_INFO(this->get_logger(), "TID %ld の応答を %s に転送します。", tid, original_sender.c_str());
                    
                    std::vector<std::string> response_data(parts.begin(), parts.end());
                    RCLCPP_INFO(this->get_logger(), "応答データ: %s", 
                        std::accumulate(response_data.begin(), response_data.end(), std::string(),
                            [](const std::string& a, const std::string& b) { return a.empty() ? b : a + "," + b; }).c_str());
                    sendToNodeAsync(original_sender, "PC2", response_data, "マイコンからの応答", 102);
                    return;
                }
            }
            
            // マイコン間通信の応答をチェック
            {
                std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
                if (microcontroller_pending_map_.count(tid)) {
                    original_sender = microcontroller_pending_map_[tid];
                    microcontroller_pending_map_.erase(tid);
                    
                    RCLCPP_INFO(this->get_logger(), "🔄 マイコン間通信応答を %s に転送: TID %ld", 
                                original_sender.c_str(), tid);
                    
                    // 応答を元のマイコンに送信
                    std::stringstream response_message;
                    for (size_t i = 0; i < parts.size(); ++i) {
                        if (i > 0) response_message << ",";
                        response_message << parts[i];
                    }
                    response_message << "|";
                    
                    bool sent = sendToMicrocontroller(original_sender, response_message.str());
                    if (sent) {
                        RCLCPP_INFO(this->get_logger(), "✅ マイコン間通信応答転送成功: %s", original_sender.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "❌ マイコン間通信応答転送失敗: %s", original_sender.c_str());
                    }
                    return;
                }
            }
            
            RCLCPP_WARN(this->get_logger(), "TID %ld に対する待機中の要求が見つかりません", tid);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "TIDの解析に失敗: %s", e.what());
        }
        return;
    }
    
    // 上記以外のメッセージはデータハンドラに渡す
    if (data_handler_callback_) {
        std::string source_mc_id = "unknown_mc";
        {
             std::lock_guard<std::mutex> lock(discovery_mutex_);
             for(const auto& pair : microcontroller_device_map_) {
                 if (pair.second->getPort() == device_port) {
                     source_mc_id = pair.first;
                     break;
                 }
             }
        }
        // data_handler_callback_ は Request 型を期待するので、ダミーのRequestを作成
        auto dummy_request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
        dummy_request->source_node = source_mc_id;
        dummy_request->string_values = parts;
        data_handler_callback_(dummy_request);
    }
}

int64_t IntegratedCommunicationSystem::generate_transaction_id() { return distribution_(random_engine_); }

void IntegratedCommunicationSystem::setupAutomaticMicrocontrollerRelay() {
    // 自動マイコン間通信中継のためのデータハンドラーを設定
    setDataHandler([this](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
        this->handleMicrocontrollerToMicrocontrollerMessage(request);
    });
    RCLCPP_INFO(this->get_logger(), "🔗 自動マイコン間通信中継機能を有効化しました");
}

void IntegratedCommunicationSystem::handleMicrocontrollerToMicrocontrollerMessage(
    const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
    
    const auto& data = request->string_values;
    if (data.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "マイコンからの不正なメッセージ形式: データ不足");
        return;
    }
    
    std::string message_type = data[0];
    // メッセージ内の実際の送信者を使用
    std::string source_mc = data[1];
    std::string destination_mc = data[2];
    
    RCLCPP_INFO(this->get_logger(), "🔄 マイコン間通信を検出: %s → %s (タイプ: %s)", 
                source_mc.c_str(), destination_mc.c_str(), message_type.c_str());
    
    // 宛先マイコンが存在するかチェック
    {
        std::lock_guard<std::mutex> lock(discovery_mutex_);
        if (microcontroller_device_map_.find(destination_mc) == microcontroller_device_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "❌ 宛先マイコン '%s' が見つかりません。利用可能なマイコン: %zu個", 
                        destination_mc.c_str(), microcontroller_device_map_.size());
            return;
        }
    }
    
    // 元のマイコンからのメッセージを宛先マイコンに中継
    // メッセージ形式: "type,source,destination,tid,request_id,data..."
    // 例: "2,nucleo1,nucleo2,12345,1,apple42"
    
    // 新しいTransaction IDを生成して、元のTIDと置き換え
    int64_t new_transaction_id = generate_transaction_id();
    
    std::stringstream relay_message;
    relay_message << data[0]; // message_type
    relay_message << "," << data[1]; // source
    relay_message << "," << data[2]; // destination  
    relay_message << "," << new_transaction_id; // 新しいTID
    for (size_t i = 4; i < data.size(); ++i) {
        relay_message << "," << data[i];
    }
    relay_message << "|";
    
    // 応答を元のマイコンに返すためのマッピングを保存
    {
        std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
        microcontroller_pending_map_[new_transaction_id] = source_mc;
    }
    
    RCLCPP_INFO(this->get_logger(), "📤 マイコン %s にメッセージ中継: %s", 
                destination_mc.c_str(), relay_message.str().c_str());
    
    bool sent = sendToMicrocontroller(destination_mc, relay_message.str());
    if (sent) {
        RCLCPP_INFO(this->get_logger(), "✅ マイコン間通信中継成功: %s → %s (TID: %s → %s)", 
                    source_mc.c_str(), destination_mc.c_str(), data[3].c_str(), std::to_string(new_transaction_id).c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ マイコン間通信中継失敗: %s → %s", 
                     source_mc.c_str(), destination_mc.c_str());
        // 失敗時はマッピングを削除
        std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
        microcontroller_pending_map_.erase(new_transaction_id);
    }
}

rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr IntegratedCommunicationSystem::get_client(const std::string &service_name) {
    if (clients_.find(service_name) == clients_.end()) {
        clients_[service_name] = this->create_client<my_cpp_pkg::srv::DataExchange>(service_name);
    }
    return clients_[service_name];
}

std::shared_ptr<IntegratedCommunicationSystem> create_integrated_system(
    const std::string& node_name, bool use_serial, int baudrate)
{
    std::vector<std::shared_ptr<SerialCommunication>> serial_devices;
    if (use_serial) {
        speed_t speed = convert_baudrate(std::to_string(baudrate));
        serial_devices = SerialCommunication::scanAndConnectDevices(
            {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"}, speed, "|");
    }
    return std::make_shared<IntegratedCommunicationSystem>(node_name, serial_devices);
}