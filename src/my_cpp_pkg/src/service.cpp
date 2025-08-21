#include "service.hpp"
#include <chrono>
#include <cctype>
#include <algorithm>
#include <future>
#include <thread>

#define ANSI_RED "\033[31m"
#define ANSI_RESET "\033[0m"

// コンストラクタ
Service::Service(const std::string &node_name, const std::vector<std::string> &initial_microcontrollers)
    : Node(node_name),
      self_node_name_(node_name),
      random_engine_(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
      distribution_(100000, 999999),
      last_cache_update_(std::chrono::steady_clock::time_point::min())
{
    my_microcontrollers_ = initial_microcontrollers;

    // ★★★ 自分のマイコン情報を新しいマップ構造で登録 ★★★
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (const auto &mc_name : my_microcontrollers_)
    {
        microcontroller_location_map_[mc_name].push_back(self_node_name_);
    }

    server_ = this->create_service<my_cpp_pkg::srv::DataExchange>(
        self_node_name_,
        std::bind(&Service::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "ノード '%s' が起動し、サービスリクエストを待機します。", self_node_name_.c_str());
    
    // 初回キャッシュ更新（非同期で実行）
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 少し待ってからキャッシュ更新
        refresh_node_cache();
    }).detach();
}

// 外部データハンドラ登録関数
void Service::set_data_handler(DataHandlerCallback callback)
{
    data_handler_callback_ = callback;
    RCLCPP_INFO(this->get_logger(), "外部データハンドラが登録されました。");
}

// 外部レスポンスハンドラ登録関数
void Service::set_response_handler(ResponseHandlerCallback callback)
{
    response_handler_callback_ = callback;
    RCLCPP_INFO(this->get_logger(), "外部レスポンスハンドラが登録されました。");
}

// リクエスト受信時の処理（ルーティング対応版）
void Service::handle_request(
    const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
    std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response)
{
    // --- 1. このメッセージをローカルで処理すべきか、転送すべきかを判断 ---
    bool process_locally = false;
    // --- 1. ルーティング（転送）処理 ---
    // 宛先が自分自身でなく、かつ自分がその宛先（マイコン）を保持していない場合
    if (request->destination_node != self_node_name_) {
        std::vector<std::string> my_devices = find_microcontroller_locations(request->destination_node);
        if (my_devices.empty() || std::find(my_devices.begin(), my_devices.end(), self_node_name_) == my_devices.end())
        {
            RCLCPP_INFO(this->get_logger(), "受信リクエストの宛先[%s]が自分ではないため、転送を試みます。", request->destination_node.c_str());

            // =================================================================
            // ▼▼▼ ここからが追加されたログ出力処理 ▼▼▼
            // =================================================================
            RCLCPP_INFO(this->get_logger(), "--- 転送メッセージ詳細 ---");
            RCLCPP_INFO(this->get_logger(), "  > 送信元      : %s", request->source_node.c_str());
            RCLCPP_INFO(this->get_logger(), "  > 最終宛先    : %s", request->destination_node.c_str());
            RCLCPP_INFO(this->get_logger(), "  > Type        : %ld", request->request_type);

            if (!request->numeric_values.empty()) {
                std::string numeric_str = "[";
                for (size_t i = 0; i < request->numeric_values.size(); ++i) {
                    numeric_str += std::to_string(request->numeric_values[i]);
                    if (i < request->numeric_values.size() - 1) numeric_str += ", ";
                }
                numeric_str += "]";
                RCLCPP_INFO(this->get_logger(), "  > 数値データ    : %s", numeric_str.c_str());
            }

            if (!request->string_values.empty()) {
                std::string string_str = "[";
                for (size_t i = 0; i < request->string_values.size(); ++i) {
                    string_str += "'" + request->string_values[i] + "'";
                    if (i < request->string_values.size() - 1) string_str += ", ";
                }
                string_str += "]";
                RCLCPP_INFO(this->get_logger(), "  > 文字列データ  : %s", string_str.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "  > メッセージ    : '%s'", request->message.c_str());
            RCLCPP_INFO(this->get_logger(), "--------------------------");
            // =================================================================
            // ▲▲▲ ここまでが追加されたログ出力処理 ▲▲▲
            // =================================================================

            std::vector<std::string> target_pcs = find_microcontroller_locations(request->destination_node);

            if (!target_pcs.empty()) {
                RCLCPP_INFO(this->get_logger(), "宛先['%s']を %zu 個のPCに転送します。", request->destination_node.c_str(), target_pcs.size());
                for (const auto& pc_name : target_pcs) {
                    if (pc_name == self_node_name_) continue;
                    RCLCPP_INFO(this->get_logger(), " -> %s", pc_name.c_str());
                    send_custom_request(pc_name, request->request_type, request->destination_node, request->numeric_values, request->string_values, request->message);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "転送先['%s']の所属PCが見つかりませんでした。", request->destination_node.c_str());
            }

            response->response_type = 103; // 103: Forwarded (転送済み)
            response->return_node = this->get_name();
            response->transaction_id = request->transaction_id;
            return;
        }
    }

    // --- 2. 自分宛のメッセージの通常処理 ---
    // (以降のコードは変更なし)
    RCLCPP_INFO(this->get_logger(), "自分宛のリクエスト(Type: %ld)を処理します。", request->request_type);
    if (data_handler_callback_) data_handler_callback_(request);

    // Type 1 マイコン情報交換処理（高速化）
    if (request->request_type == 1) {
        const std::string& from_node = request->source_node;
        const std::vector<std::string>& received_microcontrollers = request->string_values;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            for (const auto& mc_name : received_microcontrollers) {
                auto& node_list = microcontroller_location_map_[mc_name];
                if (std::find(node_list.begin(), node_list.end(), from_node) == node_list.end()) {
                    node_list.push_back(from_node);
                    RCLCPP_INFO(this->get_logger(), "マイコンロケーション追加: %s -> %s", mc_name.c_str(), from_node.c_str());
                }
            }
        }
    }

    // 高速レスポンス作成（タイムアウト防止）
    response->response_type = (request->request_type == 1) ? 101 : 102;
    response->return_node = this->get_name();
    response->transaction_id = request->transaction_id;
    response->numeric_values = {};
    response->string_values = my_microcontrollers_;  // 自分のマイコンリストを即座に返す
}


// データ送信関数（最適化版）
void Service::send_custom_request(
    const std::string &target_node_name,
    int64_t request_type,
    const std::string &destination_node,
    const std::vector<int64_t> &numeric_values,
    const std::vector<std::string> &string_values,
    const std::string &message)
{
    // 宛先に対応するクライアントオブジェクトを取得 (なければ作成)
    auto client = get_client(target_node_name);

    // サービスが利用可能かチェック (すぐにリターンするので待機は不要)
    if (!client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "サービス '%s' はまだ利用できません。送信を試みます...", target_node_name.c_str());
    }

    // 送信するリクエストオブジェクトを作成
    auto request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
    request->request_type = request_type;
    request->source_node = this->self_node_name_; // ★ 送信元として自身のノード名を設定
    request->destination_node = destination_node; // ★ メッセージの最終的な宛先を設定
    request->transaction_id = distribution_(random_engine_); // ランダムなIDを生成
    request->numeric_values = numeric_values;
    request->string_values = string_values;
    request->message = message;

    RCLCPP_INFO(this->get_logger(), "リクエスト送信 (TID: %ld, Type: %ld) -> %s (最終宛先: %s)", 
                request->transaction_id, request_type, target_node_name.c_str(), destination_node.c_str());

    // レスポンスを受信したときに実行されるコールバック関数を定義
    auto response_callback = [this](rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedFuture future) {
        try {
            // Futureからレスポンスオブジェクトを取得
            auto response = future.get();

            // 外部から登録されたレスポンスハンドラがあれば呼び出す
            if (response_handler_callback_) {
                response_handler_callback_(response);
            }

            // Type 101 (マイコン情報) レスポンスの場合、内部のマップを更新する
            if (response->response_type == 101) {
                const std::string& from_node = response->return_node;
                const std::vector<std::string>& received_microcontrollers = response->string_values;
                
                // mutexを使ってマップを安全に更新
                std::lock_guard<std::mutex> lock(map_mutex_);
                for (const auto& mc_name : received_microcontrollers) {
                    auto& node_list = microcontroller_location_map_[mc_name];
                    // 重複がなければノードリストに追加
                    if (std::find(node_list.begin(), node_list.end(), from_node) == node_list.end()) {
                        node_list.push_back(from_node);
                        RCLCPP_INFO(this->get_logger(), "  - Mapped(Resp): %s -> %s (新規追加)", mc_name.c_str(), from_node.c_str());
                    }
                }
            }
            // 他のレスポンスタイプ (102: OK, 103: Forwarded) の場合は特に内部処理はしない
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "サービス呼び出し中に例外が発生: %s", e.what());
        }
    };
    
    // 非同期でリクエストを送信し、応答は上記のコールバックで処理する
    client->async_send_request(request, response_callback);
}


// 動的ブロードキャスト関数（従来版、新しい高速版を推奨）
void Service::broadcast_to_all_nodes(
    int64_t request_type,
    const std::string &destination_node,
    const std::vector<int64_t> &numeric_values,
    const std::vector<std::string> &string_values,
    const std::string &message)
{
    // 新しい高速版を使用
    broadcast_to_all_nodes_fast(request_type, destination_node, numeric_values, string_values, message);
}

// クライアント取得ヘルパー
rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr Service::get_client(const std::string &service_name)
{
    if (clients_.find(service_name) == clients_.end())
    {
        clients_[service_name] = this->create_client<my_cpp_pkg::srv::DataExchange>(service_name);
    }
    return clients_[service_name];
}

// ★★★ 検索関数の実装を修正 ★★★
std::vector<std::string> Service::find_microcontroller_locations(const std::string &microcontroller_name)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = microcontroller_location_map_.find(microcontroller_name);
    if (it != microcontroller_location_map_.end())
    {
        return it->second; // ノード名のベクターを返す
    }
    return {}; // 見つからなければ空のベクターを返す
}

// ★★★ 新しい高速化機能 ★★★

// キャッシュを更新する関数
void Service::refresh_node_cache()
{
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    if (cache_valid_.load() && (now - last_cache_update_) < CACHE_LIFETIME) {
        return; // キャッシュが有効
    }

    RCLCPP_DEBUG(this->get_logger(), "ノードキャッシュを更新中...");
    
    std::vector<std::string> target_nodes = discover_target_nodes();
    cached_target_nodes_ = std::move(target_nodes);
    last_cache_update_ = now;
    cache_valid_.store(true);
    
    RCLCPP_DEBUG(this->get_logger(), "ノードキャッシュ更新完了: %zu ノード", cached_target_nodes_.size());
}

// ノード発見の内部実装
std::vector<std::string> Service::discover_target_nodes()
{
    std::vector<std::string> all_node_names_with_slash = this->get_node_names();
    std::string target_service_type = "my_cpp_pkg/srv/DataExchange";
    std::vector<std::string> target_nodes;
    
    // unordered_setを使って重複チェックを高速化
    std::unordered_set<std::string> processed_nodes;

    for (const auto &node_name_with_slash : all_node_names_with_slash)
    {
        std::string node_name = node_name_with_slash;
        // 先頭に / があれば取り除く
        if (!node_name.empty() && node_name[0] == '/')
        {
            node_name = node_name.substr(1);
        }

        // 自分自身には送信しない
        if (node_name == this->self_node_name_ || processed_nodes.count(node_name))
        {
            continue;
        }

        if (node_name.empty())
        {
            continue;
        }
        
        // 文字チェックを簡略化
        bool valid = true;
        for (char c : node_name)
        {
            if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_')
            {
                valid = false;
                break;
            }
        }
        
        if (!valid) {
            continue;
        }

        processed_nodes.insert(node_name);

        try
        {
            // APIにはスラッシュなしの名前を渡す
            auto services_map = this->get_service_names_and_types_by_node(node_name, "");
            for (const auto &pair : services_map)
            {
                for (const auto &type : pair.second)
                {
                    // サービス名はスラッシュ付きで比較
                    if (type == target_service_type && pair.first == "/" + node_name)
                    {
                        target_nodes.push_back(node_name);
                        goto next_node_loop;
                    }
                }
            }
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_DEBUG(this->get_logger(), "ノード '%s' のサービス取得中にエラー: %s", node_name.c_str(), e.what());
        }

    next_node_loop:;
    }

    return target_nodes;
}

// キャッシュされたノードリストを使った高速ブロードキャスト
void Service::broadcast_to_all_nodes_fast(
    int64_t request_type,
    const std::string &destination_node,
    const std::vector<int64_t> &numeric_values,
    const std::vector<std::string> &string_values,
    const std::string &message)
{
    RCLCPP_DEBUG(this->get_logger(), "高速ブロードキャストを開始...");

    // キャッシュを必要に応じて更新
    refresh_node_cache();

    std::vector<std::string> target_nodes;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        target_nodes = cached_target_nodes_;
    }

    if (target_nodes.empty())
    {
        RCLCPP_WARN(this->get_logger(), "ブロードキャスト対象のノードが見つかりませんでした。");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "高速ブロードキャスト実行！ 宛先: %zu ノード", target_nodes.size());

    // 並列送信を実行
    send_requests_parallel(target_nodes, request_type, destination_node, numeric_values, string_values, message);
}

// 並列送信の内部実装
void Service::send_requests_parallel(
    const std::vector<std::string> &target_nodes,
    int64_t request_type,
    const std::string &destination_node,
    const std::vector<int64_t> &numeric_values,
    const std::vector<std::string> &string_values,
    const std::string &message)
{
    // 小さなバッチに分けて並列処理
    const size_t batch_size = 4; // 4つずつ並列処理
    
    for (size_t i = 0; i < target_nodes.size(); i += batch_size)
    {
        std::vector<std::future<void>> futures;
        
        size_t end = std::min(i + batch_size, target_nodes.size());
        for (size_t j = i; j < end; ++j)
        {
            const auto &target_node_name = target_nodes[j];
            
            // 各送信を非同期で実行
            auto future = std::async(std::launch::async, [this, target_node_name, request_type, destination_node, numeric_values, string_values, message]() {
                RCLCPP_DEBUG(this->get_logger(), " -> %s", target_node_name.c_str());
                send_custom_request(target_node_name, request_type, destination_node, numeric_values, string_values, message);
            });
            
            futures.push_back(std::move(future));
        }
        
        // このバッチの完了を待つ
        for (auto &future : futures)
        {
            future.wait();
        }
    }
}