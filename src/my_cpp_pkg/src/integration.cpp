#include "integration.hpp"
#include <chrono>
#include <sstream>
#include <algorithm>
#include <numeric>

// コンストラクタ: 自動探索フラグを受け取り、有効なら探索スレッドを開始
IntegratedCommunicationSystem::IntegratedCommunicationSystem(
    const std::string& node_name, const std::vector<std::shared_ptr<SerialCommunication>>& serial_comms, bool auto_discover)
    // ★★★ 修正点1: メンバー変数の宣言順に合わせて初期化リストの順番を修正 ★★★
    : rclcpp::Node(node_name), self_node_name_(node_name), serial_comms_(serial_comms),
      auto_discover_enabled_(auto_discover), shutdown_flag_(false),
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
    
    setupAutomaticMicrocontrollerRelay();

    // 自動探索機能が有効な場合、バックグラウンドで探索を開始
    if (auto_discover_enabled_) {
        std::thread discovery_thread(&IntegratedCommunicationSystem::startAutoDiscovery, this, 2000, 3000);
        discovery_thread.detach();
    }
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

std::map<std::string, std::vector<std::string>> IntegratedCommunicationSystem::getNetworkMicrocontrollerMap() const {
    std::lock_guard<std::mutex> lock(node_map_mutex_);
    return node_to_microcontrollers_map_;
}

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
    RCLCPP_INFO(this->get_logger(), "🔍 サービスリクエスト受信: source=%s, request_type=%ld, data_size=%zu", 
                request->source_node.c_str(), request->request_type, request->string_values.size());
                
    if (request->request_type == 1) {
        RCLCPP_INFO(this->get_logger(), "📋 マイクロコントローラーリスト要求を処理中...");
        
        auto microcontroller_list = getDiscoveredMicrocontrollers();
        response->string_values = microcontroller_list;
        response->response_type = 101; // Info Response
        response->return_node = self_node_name_;
        response->transaction_id = request->transaction_id;
        
        RCLCPP_INFO(this->get_logger(), "✅ マイクロコントローラーリスト応答: %zu個", microcontroller_list.size());
        for (size_t i = 0; i < microcontroller_list.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  🔌 [%zu] %s", i, microcontroller_list[i].c_str());
        }
        return;
    }
    
    if (request->request_type == 102) {
        if (data_handler_callback_) {
            data_handler_callback_(request);
        } else {
            RCLCPP_WARN(this->get_logger(), "データハンドラーが設定されていません");
        }
    } else {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(request);
        }
        queue_cv_.notify_one();
    }
    
    response->response_type = 200;
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

    if (parts.size() >= 4 && parts[0] == "102") {
        try {
            int64_t tid = std::stoll(parts[2]);
            std::string original_sender;
            {
                std::lock_guard<std::mutex> lock(forward_map_mutex_);
                if (pending_forward_map_.count(tid)) {
                    original_sender = pending_forward_map_[tid];
                    pending_forward_map_.erase(tid);
                    
                    std::vector<std::string> response_data(parts.begin(), parts.end());
                    sendToNodeAsync(original_sender, "PC2", response_data, "マイコンからの応答", 102);
                    return;
                }
            }
            {
                std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
                if (microcontroller_pending_map_.count(tid)) {
                    original_sender = microcontroller_pending_map_[tid];
                    microcontroller_pending_map_.erase(tid);
                    
                    std::stringstream response_message;
                    for (size_t i = 0; i < parts.size(); ++i) {
                        if (i > 0) response_message << ",";
                        response_message << parts[i];
                    }
                    response_message << "|";
                    
                    sendToMicrocontroller(original_sender, response_message.str());
                    return;
                }
            }
            RCLCPP_WARN(this->get_logger(), "TID %ld に対する待機中の要求が見つかりません", tid);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "TIDの解析に失敗: %s", e.what());
        }
        return;
    }
    
    if (parts.size() == 3 && parts[0] == "1") {
        std::string target_node = parts[1];
        RCLCPP_INFO(this->get_logger(), "🛰️ マイコンからのネットワーク探索要求: ターゲット='%s'", target_node.c_str());

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

        if (source_mc_id == "unknown_mc") {
            RCLCPP_ERROR(this->get_logger(), "ポート %s からの要求ですが、送信元マイコンを特定できません。", device_port.c_str());
            return;
        }

        std::vector<std::string> mc_list;
        {
            std::lock_guard<std::mutex> lock(node_map_mutex_);
            auto it = node_to_microcontrollers_map_.find(target_node);
            if (it != node_to_microcontrollers_map_.end()) {
                mc_list = it->second;
                RCLCPP_INFO(this->get_logger(), "ノード '%s' に接続されたマイコン %zu 個を発見。", target_node.c_str(), mc_list.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "ノード '%s' はネットワークマップに見つかりません。", target_node.c_str());
            }
        }

        std::stringstream response_ss;
        response_ss << "101," << self_node_name_ << "," << target_node << "," << mc_list.size();
        for(const auto& mc_name : mc_list) {
            response_ss << "," << mc_name;
        }
        response_ss << "|";
        
        RCLCPP_INFO(this->get_logger(), "マイコン '%s' へ探索結果を返信: %s", source_mc_id.c_str(), response_ss.str().c_str());
        sendToMicrocontroller(source_mc_id, response_ss.str());
        
        return;
    }
    
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
        auto dummy_request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
        dummy_request->source_node = source_mc_id;
        dummy_request->string_values = parts;
        data_handler_callback_(dummy_request);
    }
}

int64_t IntegratedCommunicationSystem::generate_transaction_id() { return distribution_(random_engine_); }

void IntegratedCommunicationSystem::setupAutomaticMicrocontrollerRelay() {
    setDataHandler([this](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
        this->handleMicrocontrollerToMicrocontrollerMessage(request);
    });
    RCLCPP_INFO(this->get_logger(), "🔗 自動マイコン間通信中継機能を有効化しました");
}

void IntegratedCommunicationSystem::handleMicrocontrollerToMicrocontrollerMessage(
    const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
    
    const auto& data = request->string_values;
    if (data.size() < 4) { return; }
    
    std::string source_mc = data[1];
    std::string destination_mc = data[2];
    
    {
        std::lock_guard<std::mutex> lock(discovery_mutex_);
        if (microcontroller_device_map_.find(destination_mc) == microcontroller_device_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "❌ 宛先マイコン '%s' が見つかりません。", destination_mc.c_str());
            return;
        }
    }
    
    int64_t new_transaction_id = generate_transaction_id();
    
    std::stringstream relay_message;
    relay_message << data[0] << "," << data[1] << "," << data[2] << "," << new_transaction_id;
    for (size_t i = 4; i < data.size(); ++i) {
        relay_message << "," << data[i];
    }
    relay_message << "|";
    
    {
        std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
        microcontroller_pending_map_[new_transaction_id] = source_mc;
    }
    
    bool sent = sendToMicrocontroller(destination_mc, relay_message.str());
    if (!sent) {
        std::lock_guard<std::mutex> lock(microcontroller_relay_mutex_);
        microcontroller_pending_map_.erase(new_transaction_id);
    }
}

void IntegratedCommunicationSystem::startAutoDiscovery(int node_discovery_timeout_ms, int mc_discovery_timeout_ms) {
    RCLCPP_INFO(this->get_logger(), "🚀 自動探索プロセスを開始します...");
    
    // Step 1: 自身に接続されたマイクロコントローラを探索
    RCLCPP_INFO(this->get_logger(), "  (1/2) ローカルのマイクロコントローラを探索中...");
    discoverMicrocontrollerIDs(mc_discovery_timeout_ms);
    auto local_mcs = getDiscoveredMicrocontrollers();
    {
        std::lock_guard<std::mutex> lock(node_map_mutex_);
        node_to_microcontrollers_map_[self_node_name_] = local_mcs;
    }
    RCLCPP_INFO(this->get_logger(), "  ローカル探索完了。%zu個のマイコンを発見。", local_mcs.size());

    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    // ★ ここに待機処理を追加します ★
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    RCLCPP_INFO(this->get_logger(), "  ネットワーク接続の安定化のため1秒待機します...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 2: ネットワーク上の他のノードとそのマイクロコントローラを探索
    RCLCPP_INFO(this->get_logger(), "  (2/2) ネットワーク上の他ノードを探索中...");
    discoverNodesAndTheirMicrocontrollers(node_discovery_timeout_ms);
    
    RCLCPP_INFO(this->get_logger(), "✨ 自動探索プロセス完了。ネットワーク全体の構成:");
    auto network_map = getNetworkMicrocontrollerMap();
    for (const auto& pair : network_map) {
        std::string mc_list_str;
        if (pair.second.empty()) {
            mc_list_str = "(なし)";
        } else {
            mc_list_str = std::accumulate(pair.second.begin(), pair.second.end(), std::string(),
                [](const std::string& a, const std::string& b) { return a.empty() ? b : a + ", " + b; });
        }
        RCLCPP_INFO(this->get_logger(), "  - Node[%s] -> MCs: [%s]", pair.first.c_str(), mc_list_str.c_str());
    }
}

void IntegratedCommunicationSystem::discoverNodesAndTheirMicrocontrollers(int timeout_ms) {
    std::vector<std::string> node_names;
    const int max_retries = 5;
    const auto retry_interval = std::chrono::milliseconds(500);

    for (int i = 0; i < max_retries; ++i) {
        node_names = this->get_node_names();
        
        std::string current_nodes_str = std::accumulate(node_names.begin(), node_names.end(), std::string(),
            [](const std::string& a, const std::string& b) { return a.empty() ? b : a + ", " + b; });
        RCLCPP_INFO(this->get_logger(), "  [探索試行 %d/%d] 現在認識しているノード: [%s]", i + 1, max_retries, current_nodes_str.c_str());

        // 自分自身以外に1つでも "意味のある" ノードが見つかれば続行
        // (自分自身とros2cliデーモン以外のノードが1つでもあればOK)
        long meaningful_nodes = std::count_if(node_names.begin(), node_names.end(), [this](const std::string& name) {
            return name != ("/" + this->self_node_name_) && name.find("_ros2cli_daemon") == std::string::npos;
        });
        if (meaningful_nodes > 0) {
            break;
        }

        if (i < max_retries - 1) {
             RCLCPP_INFO(this->get_logger(), "  他ノードがまだ見つかりません。少し待って再試行します...");
            std::this_thread::sleep_for(retry_interval);
        }
    }

    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    // ★ ここからが修正されたフィルタリングロジックです ★
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    std::vector<std::string> target_nodes;
    std::string self_full_name = "/" + self_node_name_; // e.g., "/PC3"

    for (const auto& name : node_names) {
        // 自分自身のノードはスキップ
        if (name == self_full_name) {
            continue;
        }
        // ros2cliが内部的に使用するデーモンノードはスキップ
        if (name.find("_ros2cli_daemon") != std::string::npos) {
            continue;
        }
        // ROS 2の内部システムノードをスキップ (より堅牢に)
        if (name == "/rosout" || name == "/parameter_events") {
            continue;
        }
        
        // 上記のいずれでもなければ、ターゲットノードとして追加
        // ★注意: get_node_names()が返すのは /PC1 のような名前なので、/ を除去せずにそのまま使う
        target_nodes.push_back(name.substr(1)); // 先頭の'/'を除去して "PC1" の形式で追加
    }
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

    if (target_nodes.empty()) {
        RCLCPP_INFO(this->get_logger(), "  最終的に他ノードは見つかりませんでした。ROSネットワーク設定を確認してください。");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "  %zu個の他ノードを検出。サービスを確認し、マイコンリストを要求します...", target_nodes.size());

    using ServiceResponseFuture = rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedFuture;
    std::map<std::string, ServiceResponseFuture> futures;

    for (const auto& node_name : target_nodes) {
        auto client = get_client(node_name);
        if (!client->wait_for_service(std::chrono::milliseconds(200))) { // 少し待機時間を増やす
            RCLCPP_WARN(this->get_logger(), "  ノード '%s' のサービスが見つかりません。", node_name.c_str());
            continue;
        }

        auto request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
        request->request_type = 1;
        request->source_node = self_node_name_;
        request->destination_node = node_name;
        request->transaction_id = generate_transaction_id();
        
        futures[node_name] = client->async_send_request(request).future.share();
    }

    auto start_time = std::chrono::steady_clock::now();
    for (auto& pair : futures) {
        const std::string& node_name = pair.first;
        auto& future = pair.second;
        
        auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
        auto remaining_time = std::chrono::milliseconds(timeout_ms) - time_elapsed;
        if (remaining_time <= std::chrono::milliseconds(0)) break;

        if (future.wait_for(remaining_time) == std::future_status::ready) {
            auto response = future.get();
            if (response && response->response_type == 101) {
                std::lock_guard<std::mutex> lock(node_map_mutex_);
                node_to_microcontrollers_map_[node_name] = response->string_values;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "  ノード '%s' からの応答がタイムアウトしました。", node_name.c_str());
        }
    }
}




rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr IntegratedCommunicationSystem::get_client(const std::string &service_name) {
    if (clients_.find(service_name) == clients_.end()) {
        clients_[service_name] = this->create_client<my_cpp_pkg::srv::DataExchange>(service_name);
    }
    return clients_[service_name];
}

std::shared_ptr<IntegratedCommunicationSystem> create_integrated_system(
    const std::string& node_name, bool use_serial, int baudrate, bool auto_discover)
{
    std::vector<std::shared_ptr<SerialCommunication>> serial_devices;
    if (use_serial) {
        speed_t speed = convert_baudrate(std::to_string(baudrate));
        serial_devices = SerialCommunication::scanAndConnectDevices(
            {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"}, speed, "|");
    }
    return std::make_shared<IntegratedCommunicationSystem>(node_name, serial_devices, auto_discover);
}