#ifndef CUSTOM_INTERFACES_TOPIC_HPP_
#define CUSTOM_INTERFACES_TOPIC_HPP_

#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace custom_interfaces
{

/**
 * @brief シンプルなPublisherノードクラス
 * 
 * 指定されたデータを即座に送信するPublisherノード
 */
class PublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief コンストラクタ
     */
    PublisherNode();

    /**
     * @brief デストラクタ
     */
    ~PublisherNode() = default;

    /**
     * @brief 指定されたメッセージを送信する
     * 
     * @param message 送信するメッセージ
     */
    void publish_message(const std::string& message);

    /**
     * @brief カウンター付きメッセージを送信する
     * 
     * @param base_message ベースとなるメッセージ
     */
    void publish_message_with_count(const std::string& base_message);

    /**
     * @brief 現在のカウント値を取得する
     * 
     * @return size_t 現在のカウント値
     */
    size_t get_count() const;

private:
    // メンバ変数
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    
    // 設定可能なパラメータ
    static constexpr int QUEUE_SIZE = 10;
    static const std::string TOPIC_NAME;
};

/**
 * @brief シンプルなSubscriberノードクラス
 * 
 * メッセージを受信するSubscriberノード
 */
class SubscriberNode : public rclcpp::Node
{
public:
    /**
     * @brief コンストラクタ
     */
    SubscriberNode();

    /**
     * @brief デストラクタ
     */
    ~SubscriberNode() = default;

private:
    /**
     * @brief メッセージ受信コールバック関数
     * 
     * @param msg 受信したメッセージ
     */
    void listener_callback(const std_msgs::msg::String::SharedPtr msg);

    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    // 設定可能なパラメータ
    static constexpr int QUEUE_SIZE = 10;
    static const std::string TOPIC_NAME;
};

/**
 * @brief Publisher用の関数（指定データを即座に送信）
 * 
 * @param argc 引数の数
 * @param argv 引数の配列
 * @param message 送信するメッセージ（空の場合はデフォルトメッセージ）
 * @return int 実行結果
 */
int run_publisher_once(int argc, char * argv[], const std::string& message = "");

/**
 * @brief Publisher用のmain関数（対話的モード）
 * 
 * @param argc 引数の数
 * @param argv 引数の配列
 * @return int 実行結果
 */
int run_publisher_interactive(int argc, char * argv[]);

/**
 * @brief Subscriber用のmain関数
 * 
 * @param argc 引数の数
 * @param argv 引数の配列
 * @return int 実行結果
 */
int run_subscriber(int argc, char * argv[]);

/**
 * @brief 簡単にメッセージを送信する関数
 * ROS2を初期化し、メッセージを送信して終了する
 * 
 * @param message 送信するメッセージ
 * @return bool 送信成功時true、失敗時false
 */
bool topic_send(const std::string& message);

/**
 * @brief グローバルなPublisherインスタンスを初期化する
 * topic_send()を使用する前に一度だけ呼び出す必要がある
 * 
 * @param argc main関数の引数の数
 * @param argv main関数の引数の配列
 * @return bool 初期化成功時true、失敗時false
 */
bool topic_init(int argc, char * argv[]);

/**
 * @brief グローバルなPublisherインスタンスを終了する
 * プログラム終了時に呼び出す
 */
void topic_cleanup();

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES_TOPIC_HPP_