#ifndef SERVICE_HPP
#define SERVICE_HPP

#include "rclcpp/rclcpp.hpp"                        // ROS 2 の基本ヘッダー（ノード、ロガーなど）
#include "my_cpp_pkg/srv/add_three_ints.hpp"        // カスタムサービス型（AddThreeInts）
#include <memory>                                   // std::shared_ptr を使うために必要
#include <string>                                   // std::string を使うために必要
#include <vector>                                   // std::vector を使うために必要
#include <chrono>                                   // 時間処理用（wait用）

// サービス設定を格納する構造体
struct service_set
{
    std::string serviceID;      // サービスID
    std::vector<int8_t> values; // 値の配列
};

// サービス通信のサーバー側クラス
class Service : public rclcpp::Node
{
public:
    // コンストラクタ：サービスIDを受け取ってサーバーを初期化
    Service(const std::string &serviceID);
    
    // サービスを開始する関数
    void start();
    
    // サービスを停止する関数
    void stop();
    
    // サービスの状態を取得する関数
    bool is_running() const;

private:
    // サービスのコールバック関数：リクエストを受けてレスポンスを返す
    void handle_service(
        const std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Request> request,
        std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Response> response
    );
    
    std::string service_id_;                                                           // サービスID
    bool is_running_;                                                                  // サービス実行状態
    rclcpp::Service<my_cpp_pkg::srv::AddThreeInts>::SharedPtr service_;              // サービスオブジェクト
};

// サービス通信のクライアント側クラス
class Client : public rclcpp::Node
{
public:
    // コンストラクタ：クライアントIDとサービス名を受け取って初期化
    Client(const std::string &clientID, const std::string &service_name = "add_three_ints");
    
    // サービスにリクエストを送信（3つの値）
    bool send_request(int64_t a, int64_t b, int64_t c);
    
    // サービスにリクエストを送信（4つの値、dは使用されない）
    bool send_request(int64_t a, int64_t b, int64_t c, int64_t d);
    
    // 最後のレスポンス結果を取得
    int64_t get_last_result() const;
    
    // サービスが利用可能かチェック
    bool is_service_available();
    
    // サービスが利用可能になるまで待機
    bool wait_for_service(std::chrono::seconds timeout = std::chrono::seconds(10));

private:
    std::string client_id_;                                                           // クライアントID
    std::string service_name_;                                                        // サービス名
    int64_t last_result_;                                                            // 最後の結果
    rclcpp::Client<my_cpp_pkg::srv::AddThreeInts>::SharedPtr client_;              // クライアントオブジェクト
    
    // 内部的なリクエスト送信処理
    bool send_request_internal(int64_t a, int64_t b, int64_t c);
};

#endif
