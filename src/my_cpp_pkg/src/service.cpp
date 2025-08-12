#include "service.hpp"
#include <chrono>

// コンストラクタ：ランダム数生成器も初期化
Service::Service(const std::string &node_name)
    : Node(node_name), 
      self_node_name_(node_name),
      // 乱数のシード（種）を設定。これは変更不要です。
      random_engine_(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
      // ★★★ ここを修正 ★★★
      // 生成する乱数の範囲を 100000 から 999999 まで（6桁）に設定
      distribution_(100000, 999999) 
{
    server_ = this->create_service<my_cpp_pkg::srv::DataExchange>(
        self_node_name_,
        std::bind(&Service::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "ノード '%s' が起動し、サービスリクエストを待機します。", self_node_name_.c_str());
}
void Service::set_data_handler(DataHandlerCallback callback)
{
    data_handler_callback_ = callback;
    RCLCPP_INFO(this->get_logger(), "外部データハンドラが登録されました。");
}
// リクエスト受信時の処理
void Service::handle_request(
    const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
    std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "リクエスト受信 (TID: %ld)", request->transaction_id);
    switch (request->request_type) {
        case 2: // PC2からの初期リクエスト
            RCLCPP_INFO(this->get_logger(), "  - Type 2: 初期リクエストを受信しました。");
            RCLCPP_INFO(this->get_logger(), "  - Message: '%s'", request->message.c_str());
            // ここで初期化処理などを実行
            break;

        case 4: // PC1からの定期連絡 (もし自分自身が受信した場合)
            RCLCPP_INFO(this->get_logger(), "  - Type 4: 定期連絡を受信しました。");
            break;
        
        case 10: // 緊急停止コマンド
            RCLCPP_INFO(this->get_logger(), "  - Type 10: 緊急停止コマンドを受信！");
            // (例) ロボットを停止させる関数を呼び出す
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "  - 未知のType %ld を受信しました。", request->request_type);
            break;
    }

    // ★★★ ここで外部から登録されたコールバックを呼び出す ★★★
    if (data_handler_callback_) {
        // コールバックが登録されていれば、受信したリクエストデータを渡して実行
        data_handler_callback_(request);
    } else {
        // コールバックが登録されていない場合のデフォルトの動作
        RCLCPP_WARN(this->get_logger(), "データハンドラが登録されていません。データをログ出力します。");
        RCLCPP_INFO(this->get_logger(), "  - Type: %ld", request->request_type);
        RCLCPP_INFO(this->get_logger(), "  - Message: '%s'", request->message.c_str());
    }
    
    // レスポンス作成 (共通処理)
    response->response_type = 102;
    response->return_node = this->get_name();
    response->transaction_id = request->transaction_id;
    response->values = {};
    
    RCLCPP_INFO(this->get_logger(), "レスポンスを返信します (TID: %ld)", response->transaction_id);
}
// データ送信関数
void Service::send_custom_request(
    const std::string &target_node_name,
    int64_t request_type,
    const std::string &destination_node,
    const std::vector<int64_t> &values,
    const std::string &message)
{
    auto client = get_client(target_node_name);

    if (!client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "サービス '%s' はまだ利用できません。", target_node_name.c_str());
        // 必要に応じてここでリターンするか、送信を試みるか選択
    }

    auto request = std::make_shared<my_cpp_pkg::srv::DataExchange::Request>();
    request->request_type = request_type;
    request->destination_node = destination_node;
    request->transaction_id = distribution_(random_engine_); // ランダムなIDを生成
    request->values = values;
    request->message = message;
    
    RCLCPP_INFO(this->get_logger(), "リクエスト送信 (TID: %ld) -> %s", request->transaction_id, target_node_name.c_str());

    auto response_callback = [this](rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "レスポンス受信:");
            RCLCPP_INFO(this->get_logger(), "  - Type: %ld", response->response_type);
            RCLCPP_INFO(this->get_logger(), "  - From: %s", response->return_node.c_str());
            RCLCPP_INFO(this->get_logger(), "  - TID: %ld", response->transaction_id);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "サービス呼び出し中に例外が発生: %s", e.what());
        }
    };
    
    client->async_send_request(request, response_callback);
}

// クライアント取得ヘルパー (変更なし)
rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr Service::get_client(const std::string &service_name)
{
    if (clients_.find(service_name) == clients_.end()) {
        clients_[service_name] = this->create_client<my_cpp_pkg::srv::DataExchange>(service_name);
    }
    return clients_[service_name];
}