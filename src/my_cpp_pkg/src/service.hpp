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
#include <functional> // std::function を使うために必要

class Service : public rclcpp::Node
{
public:
    using DataHandlerCallback = std::function<void(const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request>)>;

    explicit Service(const std::string &node_name);

    /**
     * @brief 新しいプロトコルでデータを非同期に送信します。
     * @param target_node_name サービスを呼び出す相手のノード名
     * @param request_type リクエストの種類
     * @param destination_node 最終的な宛先ノード名
     * @param values 送信する値の配列
     * @param message 追加の文字列メッセージ
     */
    void send_custom_request(
        const std::string &target_node_name,
        int64_t request_type,
        const std::string &destination_node,
        const std::vector<int64_t> &values,
        const std::string &message);
    
    void set_data_handler(DataHandlerCallback callback);

private:
    void handle_request(
        const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
        std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response);

    rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr get_client(const std::string &service_name);

    // メンバ変数
    std::string self_node_name_;
    rclcpp::Service<my_cpp_pkg::srv::DataExchange>::SharedPtr server_;
    std::map<std::string, rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr> clients_;

    // ランダムなトランザクションIDを生成するための装置
    std::mt19937_64 random_engine_;
    std::uniform_int_distribution<int64_t> distribution_;
    DataHandlerCallback data_handler_callback_;
};

#endif // SERVICE_HPP