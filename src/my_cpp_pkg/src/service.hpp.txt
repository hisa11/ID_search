#ifndef SERVICE_HPP
#define SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "my_cpp_pkg/srv/data_exchange.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <random>
#include <functional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <future>
#include <atomic>

class Service : public rclcpp::Node
{
public:
    // 外部からデータ処理を注入するためのコールバック関数の型
    using DataHandlerCallback = std::function<void(const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request>)>;
    using ResponseHandlerCallback = std::function<void(const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response>)>;
    // コンストラクタ
    explicit Service(const std::string &node_name, const std::vector<std::string>& initial_microcontrollers = {});

    // データ受信時に呼ばれるコールバックを登録する関数
    void set_data_handler(DataHandlerCallback callback);
    
    // レスポンス受信時に呼ばれるコールバックを登録する関数
    void set_response_handler(ResponseHandlerCallback callback);

    // 特定のノードにリクエストを送信する関数
    void send_custom_request(
        const std::string &target_node_name,
        int64_t request_type,
        const std::string &destination_node,
        const std::vector<int64_t> &numeric_values,
        const std::vector<std::string> &string_values,
        const std::string &message);

    // ネットワーク上の全対象ノードにリクエストをブロードキャストする関数
    void broadcast_to_all_nodes(
        int64_t request_type,
        const std::string &destination_node,
        const std::vector<int64_t> &numeric_values,
        const std::vector<std::string> &string_values,
        const std::string &message);

    std::vector<std::string> find_microcontroller_locations(const std::string &microcontroller_name);

    // キャッシュを更新する関数
    void refresh_node_cache();

    // キャッシュされたノードリストを使った高速ブロードキャスト
    void broadcast_to_all_nodes_fast(
        int64_t request_type,
        const std::string &destination_node,
        const std::vector<int64_t> &numeric_values,
        const std::vector<std::string> &string_values,
        const std::string &message);

private:
    // サービスリクエストを受信したときの内部処理関数
    void handle_request(
        const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
        std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response);

    // 宛先ごとのクライアントオブジェクトを取得または作成するヘルパー関数
    rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr get_client(const std::string &service_name);

    // ノード発見の内部実装
    std::vector<std::string> discover_target_nodes();

    // 並列送信の内部実装
    void send_requests_parallel(
        const std::vector<std::string> &target_nodes,
        int64_t request_type,
        const std::string &destination_node,
        const std::vector<int64_t> &numeric_values,
        const std::vector<std::string> &string_values,
        const std::string &message);

    std::string self_node_name_;
    rclcpp::Service<my_cpp_pkg::srv::DataExchange>::SharedPtr server_;
    std::map<std::string, rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr> clients_;
    DataHandlerCallback data_handler_callback_;
    ResponseHandlerCallback response_handler_callback_;
    std::vector<std::string> my_microcontrollers_;
    std::unordered_map<std::string, std::vector<std::string>> microcontroller_location_map_;
    std::mutex map_mutex_;
    std::mt19937_64 random_engine_;
    std::uniform_int_distribution<int64_t> distribution_;

    // 高速化のためのキャッシュとフラグ
    std::vector<std::string> cached_target_nodes_;
    std::chrono::steady_clock::time_point last_cache_update_;
    std::mutex cache_mutex_;
    std::atomic<bool> cache_valid_{false};
    static constexpr std::chrono::milliseconds CACHE_LIFETIME{2000}; // 2秒間キャッシュを有効

    // サービス準備状態のキャッシュ
    std::unordered_map<std::string, bool> service_ready_cache_;
    std::mutex service_cache_mutex_;
};

#endif // SERVICE_HPP