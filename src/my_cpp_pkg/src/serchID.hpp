#ifndef SERCHID_HPP
#define SERCHID_HPP

#include "serial.hpp"
#include "topic.hpp"
#include "service.hpp"
#include <memory>
#include <unordered_map>
#include <string>
#include <functional>
#include <vector>
#include <sstream>
#include <iostream>
#include <set>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

namespace SearchID {

// 文字列分割ユーティリティ関数
inline std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string item;
    
    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }
    
    return result;
}

// デバイス管理クラス
class DeviceManager {
public:
    // シングルトンパターン
    static DeviceManager& getInstance() {
        static DeviceManager instance;
        return instance;
    }

    // デバイスを登録
    void registerDevice(const std::string& device_name, 
                       std::shared_ptr<SerialCommunication> device) {
        devices_[device_name] = device;
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Device registered: %s", device_name.c_str());
    }

    // 複数のデバイスを一括登録
    void registerDevices(const std::vector<std::shared_ptr<SerialCommunication>>& devices) {
        for (size_t i = 0; i < devices.size(); ++i) {
            std::string device_name = "device" + std::to_string(i);
            registerDevice(device_name, devices[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Registered %zu devices", devices.size());
    }

    // デバイス名でデバイスを取得
    std::shared_ptr<SerialCommunication> getDevice(const std::string& device_name) {
        auto it = devices_.find(device_name);
        if (it != devices_.end()) {
            return it->second;
        }
        return nullptr;
    }

    // 特定のデバイスにデータを送信
    bool sendToDevice(const std::string& device_name, const std::string& message) {
        auto device = getDevice(device_name);
        if (device && device->isOpen()) {
            return device->send(message);
        }
        RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                    "Device not found or not open: %s", device_name.c_str());
        return false;
    }

    // すべてのデバイスにデータを送信
    void broadcastToAllDevices(const std::string& message) {
        for (const auto& pair : devices_) {
            if (pair.second && pair.second->isOpen()) {
                pair.second->send(message);
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Sent to %s: %s", pair.first.c_str(), message.c_str());
            }
        }
    }

    // 登録されているデバイス一覧を取得
    std::vector<std::string> getDeviceList() const {
        std::vector<std::string> device_list;
        for (const auto& pair : devices_) {
            device_list.push_back(pair.first);
        }
        return device_list;
    }

    // デバイスの状態を確認
    bool isDeviceOpen(const std::string& device_name) const {
        auto it = devices_.find(device_name);
        if (it != devices_.end() && it->second) {
            return it->second->isOpen();
        }
        return false;
    }

    // 特定のデバイスにコールバックを設定
    void setDeviceCallback(const std::string& device_name, 
                          std::function<void(const std::string&)> callback) {
        auto device = getDevice(device_name);
        if (device) {
            device->setReceiveCallback(callback);
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                       "Callback set for device: %s", device_name.c_str());
        }
    }

    // デバイスの受信を開始
    void startDeviceReceiving(const std::string& device_name) {
        auto device = getDevice(device_name);
        if (device) {
            device->startReceiving();
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                       "Started receiving for device: %s", device_name.c_str());
        }
    }

    // デバイスの受信を停止
    void stopDeviceReceiving(const std::string& device_name) {
        auto device = getDevice(device_name);
        if (device) {
            device->stopReceiving();
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                       "Stopped receiving for device: %s", device_name.c_str());
        }
    }

    // パターンでデバイスを検索
    std::vector<std::string> findDevicesByPattern(const std::string& pattern);

    // デバイス状態を出力
    void printDeviceStatus();

    // デバイス名を設定（PC名として使用）
    void setDeviceName(const std::string& device_name) {
        device_name_ = device_name;
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Device name set to: %s", device_name_.c_str());
    }

    // デバイス名を取得
    std::string getDeviceName() const {
        return device_name_;
    }

    // すべてのUSBデバイスを開始（指定フォーマットでメッセージ送信）
    void start(int timeout_ms = 1000) {
        if (device_name_.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("DeviceManager"), 
                       "Device name not set. Using default 'PC1'");
            device_name_ = "PC1";
        }

        std::string message = "1," + device_name_ + ",0,|";
        
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Starting all devices with message: %s (timeout: %dms)", 
                   message.c_str(), timeout_ms);

        // グローバルタイマーノードが存在しない場合は作成
        if (!global_timer_node_) {
            global_timer_node_ = rclcpp::Node::make_shared("searchid_timer_node");
            global_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            global_executor_->add_node(global_timer_node_);
            
            // エグゼキューターをバックグラウンドで実行
            executor_thread_ = std::thread([this]() {
                global_executor_->spin();
            });
        }
        
        int sent_count = 0;
        for (const auto& pair : devices_) {
            if (pair.second && pair.second->isOpen()) {
                // デバイスのレスポンス待機状態を設定
                pending_start_responses_[pair.first] = true;
                start_response_time_[pair.first] = std::chrono::steady_clock::now();
                
                // レスポンス解析用のコールバックを設定
                pair.second->setReceiveCallback([this, device_name = pair.first](const std::string& message) {
                    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                               "Callback triggered for %s: '%s'", device_name.c_str(), message.c_str());
                    
                    // 空メッセージは無視
                    if (message.empty()) {
                        return;
                    }
                    
                    // 全ての受信データを処理
                    this->handleStartResponse(device_name, message);
                });
                
                // 受信開始を確実にする
                pair.second->startReceiving();
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Started receiving for device: %s", pair.first.c_str());
                
                if (pair.second->sendRaw(message)) {
                    sent_count++;
                    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                               "Start command sent to %s", pair.first.c_str());
                    
                    // タイムアウトタイマーを設定
                    auto timeout_timer = global_timer_node_->create_wall_timer(
                        std::chrono::milliseconds(timeout_ms),
                        [this, device_name = pair.first]() {
                            this->handleStartTimeout(device_name);
                        }
                    );
                    
                    // タイマーを保存
                    start_timeout_timers_[pair.first] = timeout_timer;
                    
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                                "Failed to send start command to %s", pair.first.c_str());
                    pending_start_responses_[pair.first] = false;
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("DeviceManager"), 
                           "Device %s is not open, skipping", pair.first.c_str());
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Start command sent to %d/%zu devices", sent_count, devices_.size());
    }

    // カスタムメッセージですべてのデバイスを開始
    void startWithMessage(const std::string& custom_message) {
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Starting all devices with custom message: %s", custom_message.c_str());
        
        for (const auto& pair : devices_) {
            if (pair.second && pair.second->isOpen()) {
                pair.second->send(custom_message);
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Custom start command sent to %s", pair.first.c_str());
            }
        }
    }

    // IDリクエストを送信してレスポンスを解析
    void sendIDRequest(const std::string& device_name, const std::string& id_command = "ID") {
        auto device = getDevice(device_name);
        if (!device || !device->isOpen()) {
            RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                        "Device %s not found or not open", device_name.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Sending ID request '%s' to device: %s", id_command.c_str(), device_name.c_str());

        // レスポンス解析用のコールバックを設定
        device->setReceiveCallback([device_name](const std::string& message) {
            DeviceManager::getInstance().parseIDResponse(device_name, message);
        });

        // IDコマンドを送信
        device->send(id_command);
    }

    // IDリクエストを送信してレスポンスを解析（タイムアウト付き）
    void sendIDRequestWithTimeout(const std::string& device_name, const std::string& id_command = "ID", int timeout_ms = 1000) {
        auto device = getDevice(device_name);
        if (!device || !device->isOpen()) {
            RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                        "Device %s not found or not open", device_name.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Sending ID request '%s' to device: %s (timeout: %dms)", 
                   id_command.c_str(), device_name.c_str(), timeout_ms);

        // レスポンス待機状態を設定
        pending_responses_[device_name] = true;
        response_start_time_[device_name] = std::chrono::steady_clock::now();
        
        // レスポンス解析用のコールバックを設定
        device->setReceiveCallback([this, device_name](const std::string& message) {
            this->handleIDResponse(device_name, message);
        });

        // IDコマンドを送信
        device->send(id_command);

        // グローバルタイマーノードが存在しない場合は作成
        if (!global_timer_node_) {
            global_timer_node_ = rclcpp::Node::make_shared("searchid_timer_node");
            global_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            global_executor_->add_node(global_timer_node_);
            
            // エグゼキューターをバックグラウンドで実行
            executor_thread_ = std::thread([this]() {
                global_executor_->spin();
            });
        }

        // タイムアウトタイマーを設定
        auto timeout_timer = global_timer_node_->create_wall_timer(
            std::chrono::milliseconds(timeout_ms),
            [this, device_name]() {
                this->cancelTimeout(device_name);
            }
        );

        // タイマーを保存
        timeout_timers_[device_name] = timeout_timer;
    }

    // IDレスポンスを解析
    void parseIDResponse(const std::string& device_name, const std::string& response) {
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Received from %s: %s", device_name.c_str(), response.c_str());

        // "101,maikon1,maikon2-maikon3" の形式を解析
        if (response.substr(0, 3) == "101") {
            std::vector<std::string> parts = splitString(response, ',');
            if (parts.size() >= 3) {
                std::string second_value = parts[1];  // maikon1
                std::string third_part = parts[2];    // maikon2-maikon3

                // 3番目の部分をさらに分割（"-"で区切られている場合）
                std::vector<std::string> third_parts = splitString(third_part, '-');
                
                std::string result = second_value;
                for (const auto& part : third_parts) {
                    if (!part.empty()) {
                        result += "," + part;
                    }
                }

                // ターミナルに結果を出力
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Parsed response from %s: %s", device_name.c_str(), result.c_str());
                
                // コンソールにも出力
                std::cout << "Device " << device_name << " response: " << result << std::endl;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("DeviceManager"), 
                           "Invalid response format from %s: %s", device_name.c_str(), response.c_str());
            }
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("DeviceManager"), 
                        "Non-ID response from %s: %s", device_name.c_str(), response.c_str());
        }
    }

    // IDレスポンスを処理
    void handleIDResponse(const std::string& device_name, const std::string& response) {
        // レスポンス待機中かチェック
        auto it = pending_responses_.find(device_name);
        if (it == pending_responses_.end() || !it->second) {
            // レスポンス待機中でない場合はそのまま通常ログ
            RCLCPP_DEBUG(rclcpp::get_logger("DeviceManager"), 
                        "Non-ID response from %s: %s", device_name.c_str(), response.c_str());
            return;
        }

        // レスポンス受信をマーク
        pending_responses_[device_name] = false;
        
        // タイムアウトタイマーをキャンセル
        cancelTimeout(device_name);

        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Received from %s: %s", device_name.c_str(), response.c_str());

        // "101,maikon1,maikon2-maikon3" の形式を解析
        if (response.substr(0, 3) == "101") {
            std::vector<std::string> parts = splitString(response, ',');
            if (parts.size() >= 3) {
                std::string second_value = parts[1];  // maikon1
                std::string third_part = parts[2];    // maikon2-maikon3

                // 3番目の部分をさらに分割（"-"で区切られている場合）
                std::vector<std::string> third_parts = splitString(third_part, '-');
                
                std::string result = second_value;
                for (const auto& part : third_parts) {
                    if (!part.empty()) {
                        result += "," + part;
                    }
                }

                // ターミナルに結果を出力
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Parsed response from %s: %s", device_name.c_str(), result.c_str());
                
                // コンソールにも出力
                std::cout << "Device " << device_name << " response: " << result << std::endl;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("DeviceManager"), 
                           "Invalid response format from %s: %s", device_name.c_str(), response.c_str());
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("DeviceManager"), 
                       "Unexpected response format from %s: %s", device_name.c_str(), response.c_str());
        }
    }

    // startコマンドのレスポンスを処理
    void handleStartResponse(const std::string& device_name, const std::string& response) {
        // 生の受信データをログ出力（デバッグ用）
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Raw response from %s: '%s' (length: %zu)", 
                   device_name.c_str(), response.c_str(), response.length());
        
        // レスポンス待機中かチェック
        auto it = pending_start_responses_.find(device_name);
        if (it == pending_start_responses_.end() || !it->second) {
            // レスポンス待機中でない場合もIDマッピングを試行
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                        "Processing response from %s: %s", device_name.c_str(), response.c_str());
        } else {
            // レスポンス受信をマーク
            pending_start_responses_[device_name] = false;
            
            // タイムアウトタイマーをキャンセル
            cancelStartTimeout(device_name);
        }

        // 応答フォーマットの検証と解析（101,device_id の形式を期待）
        if (response.find("101,") == 0 || response.find("101") == 0) {
            RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                       "Valid start response received from %s: %s", device_name.c_str(), response.c_str());
            
            // デバイスIDを抽出（101,nucleo1 の場合、nucleo1を抽出）
            std::vector<std::string> parts = splitString(response, ',');
            if (parts.size() >= 2) {
                std::string device_id = parts[1];  // nucleo1
                // デバイスIDをマップに登録
                mapDeviceID(device_id, device_name);
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Device ID '%s' mapped to device '%s'", device_id.c_str(), device_name.c_str());
            } else if (parts.size() == 1 && parts[0] == "101") {
                // "101"のみの場合、次の受信で残りを取得する可能性
                RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                           "Partial response received from %s: %s", device_name.c_str(), response.c_str());
            }
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("DeviceManager"), 
                       "Non-start response format from %s: %s", device_name.c_str(), response.c_str());
        }

        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Start response received from %s: %s", device_name.c_str(), response.c_str());
        
        // コンソールにも出力
        std::cout << "Device " << device_name << " start response: " << response << std::endl;
    }

    // startコマンドのタイムアウト処理
    void handleStartTimeout(const std::string& device_name) {
        auto it = pending_start_responses_.find(device_name);
        if (it != pending_start_responses_.end() && it->second) {
            // まだレスポンス待機中の場合はタイムアウト
            pending_start_responses_[device_name] = false;
            
            RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                        "Start command timeout for device %s - device is not usable", device_name.c_str());
            
            std::cout << "ERROR: Device " << device_name << " start timeout - device is not usable!" << std::endl;
            
            // デバイスを使用不可としてマーク
            unusable_devices_.insert(device_name);
        }
        
        // タイマーをクリーンアップ
        cancelStartTimeout(device_name);
    }

    // タイムアウトタイマーをキャンセル
    void cancelTimeout(const std::string& device_name) {
        auto timer_it = timeout_timers_.find(device_name);
        if (timer_it != timeout_timers_.end()) {
            timer_it->second->cancel();
            timeout_timers_.erase(timer_it);
        }
    }

    // startタイムアウトタイマーをキャンセル
    void cancelStartTimeout(const std::string& device_name) {
        auto timer_it = start_timeout_timers_.find(device_name);
        if (timer_it != start_timeout_timers_.end()) {
            timer_it->second->cancel();
            start_timeout_timers_.erase(timer_it);
        }
    }

    // デバイスが使用可能かチェック
    bool isDeviceUsable(const std::string& device_name) const {
        return unusable_devices_.find(device_name) == unusable_devices_.end();
    }

    // 使用不可デバイス一覧を取得
    std::set<std::string> getUnusableDevices() const {
        return unusable_devices_;
    }

    // デバイスIDをデバイス名にマップ
    void mapDeviceID(const std::string& device_id, const std::string& device_name) {
        device_id_map_[device_id] = device_name;
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Device ID mapped: %s -> %s", device_id.c_str(), device_name.c_str());
    }

    // デバイスIDでメッセージを送信
    bool sendByID(const std::string& device_id, const std::string& message) {
        auto it = device_id_map_.find(device_id);
        if (it != device_id_map_.end()) {
            auto device = getDevice(it->second);
            if (device && device->isOpen()) {
                return device->sendRaw(message + "|");  // デリミタを追加してRaw送信
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                            "Device %s not available", it->second.c_str());
                return false;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("DeviceManager"), 
                        "Device ID not found: %s", device_id.c_str());
            return false;
        }
    }

    // デバイスIDマッピングを取得
    std::unordered_map<std::string, std::string> getDeviceIDMap() const {
        return device_id_map_;
    }

private:
    DeviceManager() = default;
    ~DeviceManager() {
        // エグゼキューターを停止
        if (global_executor_) {
            global_executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }
    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;

    std::unordered_map<std::string, std::shared_ptr<SerialCommunication>> devices_;
    std::string device_name_ = "PC1";  // デフォルトのデバイス名
    std::unordered_map<std::string, bool> pending_responses_;  // IDレスポンス待機状態
    std::unordered_map<std::string, bool> pending_start_responses_;  // startレスポンス待機状態
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> response_start_time_;  // レスポンス開始時刻
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> start_response_time_;  // start開始時刻
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timeout_timers_;  // IDタイムアウトタイマー
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> start_timeout_timers_;  // startタイムアウトタイマー
    std::set<std::string> unusable_devices_;  // 使用不可デバイス一覧
    std::unordered_map<std::string, std::string> device_id_map_;  // デバイスID → デバイス名のマッピング
    
    // グローバルタイマーノードとエグゼキューター
    std::shared_ptr<rclcpp::Node> global_timer_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> global_executor_;
    std::thread executor_thread_;
};

// ヘルパー関数: デバイスを作成して自動登録
inline std::shared_ptr<SerialCommunication> createAndRegisterDevice(
    const std::string& device_name,
    const std::string& node_name,
    const std::string& port,
    speed_t baudrate = B9600,
    const std::string& delimiter = "\r\n") {
    
    auto device = std::make_shared<SerialCommunication>(node_name, port, baudrate, delimiter);
    
    if (device->initialize()) {
        DeviceManager::getInstance().registerDevice(device_name, device);
        RCLCPP_INFO(rclcpp::get_logger("SearchID"), 
                   "Device created and registered: %s -> %s", 
                   device_name.c_str(), port.c_str());
        return device;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SearchID"), 
                    "Failed to initialize device: %s", device_name.c_str());
        return nullptr;
    }
}

// ヘルパー関数: 複数デバイスの一括登録
inline void registerDevices(const std::vector<std::shared_ptr<SerialCommunication>>& devices) {
    DeviceManager::getInstance().registerDevices(devices);
}

// ヘルパー関数: デバイス名で送信
inline bool sendByName(const std::string& device_name, const std::string& message) {
    return DeviceManager::getInstance().sendToDevice(device_name, message);
}

// ヘルパー関数: 全デバイスに送信
inline void sendToAll(const std::string& message) {
    DeviceManager::getInstance().broadcastToAllDevices(message);
}

// ヘルパー関数: デバイス一覧取得
inline std::vector<std::string> getDevices() {
    return DeviceManager::getInstance().getDeviceList();
}

// ヘルパー関数: デバイス状態確認
inline bool isOpen(const std::string& device_name) {
    return DeviceManager::getInstance().isDeviceOpen(device_name);
}

// ヘルパー関数: デバイス名設定
inline void setDeviceName(const std::string& device_name) {
    DeviceManager::getInstance().setDeviceName(device_name);
}

// ヘルパー関数: デバイス名取得
inline std::string getDeviceName() {
    return DeviceManager::getInstance().getDeviceName();
}

// ヘルパー関数: 全デバイス開始
inline void start(int timeout_ms = 1000) {
    DeviceManager::getInstance().start(timeout_ms);
}

// ヘルパー関数: カスタムメッセージで全デバイス開始
inline void startWithMessage(const std::string& custom_message) {
    DeviceManager::getInstance().startWithMessage(custom_message);
}

// ヘルパー関数: IDリクエスト送信
inline void sendIDRequest(const std::string& device_name, const std::string& id_command = "ID") {
    DeviceManager::getInstance().sendIDRequest(device_name, id_command);
}

// ヘルパー関数: タイムアウト付きIDリクエスト送信
inline void sendIDRequestWithTimeout(const std::string& device_name, const std::string& id_command = "ID", int timeout_ms = 1000) {
    DeviceManager::getInstance().sendIDRequestWithTimeout(device_name, id_command, timeout_ms);
}

// ヘルパー関数: デバイス使用可能性チェック
inline bool isDeviceUsable(const std::string& device_name) {
    return DeviceManager::getInstance().isDeviceUsable(device_name);
}

// ヘルパー関数: デバイスIDで送信
inline bool sendByID(const std::string& device_id, const std::string& message) {
    return DeviceManager::getInstance().sendByID(device_id, message);
}

// ヘルパー関数: デバイスIDマップを取得
inline std::unordered_map<std::string, std::string> getDeviceIDMap() {
    return DeviceManager::getInstance().getDeviceIDMap();
}

// IDベース送信クラス
class ID {
private:
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> pending_responses_;
    static std::unordered_map<std::string, int> pending_random_nums_;
    
public:
    // 既存: IDを指定してメッセージ送信
    static bool send(const std::string& device_id, const std::string& message) {
        return DeviceManager::getInstance().sendByID(device_id, message);
    }
    
    // 追加: 可変長引数で複数データを非同期送信し、レスポンスをコールバックで受信（送信成功/失敗をboolで返す）
    // 例: if(SearchID::ID::send("nucleo1", "asagohan", "bangohan")) {...}
    template<typename... Args>
    static bool send(const std::string& device_id, Args&&... args) {
        std::vector<std::string> data = { std::forward<Args>(args)... };
        int random_num = std::rand() & 0xFFFF; // 16bit範囲に制限
        std::string pc_name = DeviceManager::getInstance().getDeviceName();
        if (pc_name.empty()) pc_name = "PC1";
        std::stringstream ss;
        ss << "2," << pc_name << "," << random_num << "," << data.size();
        for (const auto& d : data) ss << "," << d;
        ss << "|";
        std::string msg = ss.str();
        auto id_map = DeviceManager::getInstance().getDeviceIDMap();
        auto it = id_map.find(device_id);
        if (it == id_map.end()) {
            RCLCPP_ERROR(rclcpp::get_logger("ID"), "Device ID not found: %s", device_id.c_str());
            return false;
        }
        std::string dev_name = it->second;
        auto device = DeviceManager::getInstance().getDevice(dev_name);
        if (!device || !device->isOpen()) {
            RCLCPP_ERROR(rclcpp::get_logger("ID"), "Device not open: %s", dev_name.c_str());
            return false;
        }
        
        // レスポンス待機状態を記録
        std::string response_key = device_id + "_" + std::to_string(random_num);
        pending_responses_[response_key] = std::chrono::steady_clock::now();
        pending_random_nums_[response_key] = random_num;
        
        device->setReceiveCallback([device_id, random_num](const std::string& resp) {
            auto parts = splitString(resp, ',');
            if (parts.size() >= 4 && parts[0] == "102" && parts[1] == device_id) {
                int resp_rand = 0;
                try { resp_rand = std::stoi(parts[2]); } catch (...) {}
                if (resp_rand == random_num) {
                    std::string status = parts[3];
                    RCLCPP_INFO(rclcpp::get_logger("ID"), "Response from %s: status=%s", device_id.c_str(), status.c_str());
                    std::cout << "[ID] Response from " << device_id << ": status=" << status << std::endl;
                    
                    // レスポンス受信をマーク
                    std::string response_key = device_id + "_" + std::to_string(random_num);
                    pending_responses_.erase(response_key);
                    pending_random_nums_.erase(response_key);
                }
            }
        });
        
        return device->sendRaw(msg);
    }
    
    // レスポンスタイムアウトチェック（定期的に呼び出す）
    static void checkTimeouts(int timeout_ms = 2000) {
        auto now = std::chrono::steady_clock::now();
        std::vector<std::string> timed_out_keys;
        
        for (const auto& pair : pending_responses_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - pair.second).count();
            if (elapsed > timeout_ms) {
                timed_out_keys.push_back(pair.first);
            }
        }
        
        for (const auto& key : timed_out_keys) {
            // デバイスIDを抽出
            size_t underscore_pos = key.find_last_of('_');
            if (underscore_pos != std::string::npos) {
                std::string device_id = key.substr(0, underscore_pos);
                int random_num = pending_random_nums_[key];
                
                RCLCPP_ERROR(rclcpp::get_logger("ID"), "Response timeout for device %s (random: %d)", 
                            device_id.c_str(), random_num);
                std::cout << "[ERROR] Response timeout for device " << device_id 
                         << " (random: " << random_num << ")" << std::endl;
            }
            
            pending_responses_.erase(key);
            pending_random_nums_.erase(key);
        }
    }
    
    // 複数のIDに同じメッセージを送信
    static void sendToMultiple(const std::vector<std::string>& device_ids, const std::string& message) {
        auto& manager = DeviceManager::getInstance();
        for (const auto& id : device_ids) {
            if (manager.sendByID(id, message)) {
                RCLCPP_INFO(rclcpp::get_logger("ID"), "Sent '%s' to device ID: %s", 
                           message.c_str(), id.c_str());
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("ID"), "Failed to send to device ID: %s", id.c_str());
            }
        }
    }
    
    // 利用可能なデバイスIDのリストを取得
    static std::vector<std::string> getAvailableIDs() {
        auto device_map = DeviceManager::getInstance().getDeviceIDMap();
        std::vector<std::string> ids;
        for (const auto& pair : device_map) {
            ids.push_back(pair.first);
        }
        return ids;
    }
};

} // namespace SearchID

// 静的メンバ変数の定義
std::unordered_map<std::string, std::chrono::steady_clock::time_point> SearchID::ID::pending_responses_;
std::unordered_map<std::string, int> SearchID::ID::pending_random_nums_;

#endif // SERCHID_HPP

