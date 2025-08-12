#include "service.hpp"

using std::placeholders::_1; // bindで使用するプレースホルダー（第1引数用）
using std::placeholders::_2; // bindで使用するプレースホルダー（第2引数用）
using namespace std::chrono_literals; // 時間単位を使えるようにする

// コンストラクタ：サービスIDを受け取ってノードを初期化
Service::Service(const std::string &serviceID) 
    : Node(serviceID), service_id_(serviceID), is_running_(false)
{
    RCLCPP_INFO(this->get_logger(), "サービス '%s' を初期化しました", serviceID.c_str());
}

// サービスを開始する関数
void Service::start()
{
    if (!is_running_) {
        // サービスの作成（型：AddThreeInts、名前：サービスID、コールバック関数を登録）
        service_ = this->create_service<my_cpp_pkg::srv::AddThreeInts>(
            service_id_,                                                 // サービス名
            std::bind(&Service::handle_service, this, _1, _2)          // コールバックをbind
        );
        is_running_ = true;
        RCLCPP_INFO(this->get_logger(), "サービス '%s' を開始しました", service_id_.c_str());
    }
}

// サービスを停止する関数
void Service::stop()
{
    if (is_running_) {
        service_.reset(); // サービスオブジェクトをリセット
        is_running_ = false;
        RCLCPP_INFO(this->get_logger(), "サービス '%s' を停止しました", service_id_.c_str());
    }
}

// サービスの状態を取得する関数
bool Service::is_running() const
{
    return is_running_;
}

// サービスのコールバック関数：リクエストを受けてレスポンスを返す
void Service::handle_service(
    const std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Request> request,
    std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Response> response)
{
    response->sum = request->a + request->b + request->c; // 3つの値の足し算を実行してsumに代入
    
    RCLCPP_INFO(this->get_logger(), 
                "[%s] リクエスト: a = %ld, b = %ld, c = %ld",
                service_id_.c_str(), request->a, request->b, request->c);
    RCLCPP_INFO(this->get_logger(), 
                "[%s] 応答: sum = %ld", 
                service_id_.c_str(), response->sum);
}

// ========== Client クラスの実装 ==========

// コンストラクタ：クライアントIDとサービス名を受け取ってクライアントを初期化
Client::Client(const std::string &clientID, const std::string &service_name)
    : Node(clientID), client_id_(clientID), service_name_(service_name), last_result_(0)
{
    // クライアント作成（AddThreeInts型、指定されたサービス名）
    client_ = this->create_client<my_cpp_pkg::srv::AddThreeInts>(service_name_);
    
    RCLCPP_INFO(this->get_logger(), "クライアント '%s' を初期化しました（サービス: %s）", 
                clientID.c_str(), service_name_.c_str());
}

// サービスにリクエストを送信（3つの値）
bool Client::send_request(int64_t a, int64_t b, int64_t c)
{
    return send_request_internal(a, b, c);
}

// サービスにリクエストを送信（4つの値、dは使用されない）
bool Client::send_request(int64_t a, int64_t b, int64_t c, int64_t d)
{
    RCLCPP_WARN(this->get_logger(), "4番目のパラメータ d=%ld は無視されます", d);
    return send_request_internal(a, b, c);
}

// 内部的なリクエスト送信処理
bool Client::send_request_internal(int64_t a, int64_t b, int64_t c)
{
    // サービスが利用可能でない場合は失敗
    if (!is_service_available()) {
        RCLCPP_ERROR(this->get_logger(), "サービス '%s' が利用できません", service_name_.c_str());
        return false;
    }
    
    // リクエストオブジェクトを作成し、a、b、cに値を設定
    auto request = std::make_shared<my_cpp_pkg::srv::AddThreeInts::Request>();
    request->a = a;
    request->b = b;
    request->c = c;
    
    RCLCPP_INFO(this->get_logger(), "[%s] リクエスト送信: a=%ld, b=%ld, c=%ld", 
                client_id_.c_str(), a, b, c);
    
    // 非同期でリクエストを送信し、結果（Future）を取得
    auto result_future = client_->async_send_request(request);
    
    // Futureが完了するまで待機（成功したら結果を出力）
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        last_result_ = result_future.get()->sum;
        RCLCPP_INFO(this->get_logger(), "[%s] 結果: %ld + %ld + %ld = %ld", 
                    client_id_.c_str(), a, b, c, last_result_);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "[%s] サービス呼び出しに失敗しました", client_id_.c_str());
        return false;
    }
}

// 最後のレスポンス結果を取得
int64_t Client::get_last_result() const
{
    return last_result_;
}

// サービスが利用可能かチェック
bool Client::is_service_available()
{
    return client_->service_is_ready();
}

// サービスが利用可能になるまで待機
bool Client::wait_for_service(std::chrono::seconds timeout)
{
    auto start_time = std::chrono::steady_clock::now();
    
    // サーバーが利用可能になるまで待機（0.1秒ずつ確認）
    while (!client_->wait_for_service(100ms)) {
        if (!rclcpp::ok()) {  // 強制終了された場合
            RCLCPP_ERROR(this->get_logger(), "サービス待機中に中断されました");
            return false;
        }
        
        // タイムアウトチェック
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            RCLCPP_ERROR(this->get_logger(), "サービス '%s' の待機がタイムアウトしました", 
                        service_name_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "サービス '%s' 待機中...", service_name_.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "サービス '%s' が利用可能になりました", service_name_.c_str());
    return true;
}

