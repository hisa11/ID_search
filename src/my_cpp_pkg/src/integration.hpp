#ifndef INTEGRATION_HPP
#define INTEGRATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "my_cpp_pkg/srv/data_exchange.hpp"
#include "serial.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <mutex>
#include <functional>
#include <random>
#include <future>
#include <condition_variable>
#include <queue>
#include <thread>
#include <atomic>

class IntegratedCommunicationSystem : public rclcpp::Node
{
public:
    using DataHandlerCallback = std::function<void(const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request>)>;
    
    // コンストラクタに自動探索フラグを追加
    explicit IntegratedCommunicationSystem(const std::string& node_name, const std::vector<std::shared_ptr<SerialCommunication>>& serial_comms, bool auto_discover);
    ~IntegratedCommunicationSystem();

    void setDataHandler(DataHandlerCallback callback);
    std::vector<std::string> discoverMicrocontrollerIDs(int timeout_ms = 2000);

    void sendToNodeAsync(
        const std::string& target_node,
        const std::string& final_destination,
        const std::vector<std::string>& data,
        const std::string& message,
        int64_t request_type);

    const std::vector<std::shared_ptr<SerialCommunication>>& getSerialDevices() const { return serial_comms_; }
    std::vector<std::string> getDiscoveredMicrocontrollers() const;
    std::map<std::string, std::vector<std::string>> getNetworkMicrocontrollerMap() const;

private:
    void handle_request(
        const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request,
        std::shared_ptr<my_cpp_pkg::srv::DataExchange::Response> response);
        
    void handle_serial_data(const std::string& device_port, const std::string& raw_data);

    void processRequestQueue();
    bool sendToMicrocontroller(const std::string& microcontroller_id, const std::string& data);
    rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr get_client(const std::string& service_name);
    int64_t generate_transaction_id();
    
    // マイコン間通信中継機能
    void setupAutomaticMicrocontrollerRelay();
    void handleMicrocontrollerToMicrocontrollerMessage(const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request);

    // 自動探索機能
    void startAutoDiscovery(int node_discovery_timeout_ms, int mc_discovery_timeout_ms);
    void discoverNodesAndTheirMicrocontrollers(int timeout_ms);

    std::string self_node_name_;
    std::vector<std::shared_ptr<SerialCommunication>> serial_comms_;
    rclcpp::Service<my_cpp_pkg::srv::DataExchange>::SharedPtr server_;
    std::map<std::string, rclcpp::Client<my_cpp_pkg::srv::DataExchange>::SharedPtr> clients_;
    DataHandlerCallback data_handler_callback_{nullptr};
    
    std::unordered_map<int64_t, std::string> pending_forward_map_;
    std::mutex forward_map_mutex_;
    
    std::unordered_map<int64_t, std::string> microcontroller_pending_map_;
    std::mutex microcontroller_relay_mutex_;
    
    std::vector<std::string> discovered_microcontrollers_;
    std::unordered_map<std::string, std::shared_ptr<SerialCommunication>> microcontroller_device_map_;
    mutable std::mutex discovery_mutex_;
    std::shared_ptr<std::promise<bool>> discovery_promise_;

    // ネットワーク探索結果
    std::map<std::string, std::vector<std::string>> node_to_microcontrollers_map_;
    mutable std::mutex node_map_mutex_;
    bool auto_discover_enabled_;

    std::queue<std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request>> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> shutdown_flag_{false};

    std::mt19937_64 random_engine_;
    std::uniform_int_distribution<int64_t> distribution_;
};

// グローバル関数に自動探索フラグを追加 (デフォルトはtrue)
std::shared_ptr<IntegratedCommunicationSystem> create_integrated_system(
    const std::string& node_name, bool use_serial, int baudrate, bool auto_discover = true);

#endif // INTEGRATION_HPP