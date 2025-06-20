#include "serchID.hpp"

// 実装は主にヘッダーファイルで完結しているため、
// 必要に応じて追加の実装をここに記述します。

// 例: より複雑な処理が必要な場合の実装例
namespace SearchID {

// より高度なデバイス検索機能
std::vector<std::string> DeviceManager::findDevicesByPattern(const std::string& pattern) {
    std::vector<std::string> matching_devices;
    for (const auto& pair : devices_) {
        if (pair.first.find(pattern) != std::string::npos) {
            matching_devices.push_back(pair.first);
        }
    }
    return matching_devices;
}

// デバイスの統計情報取得
void DeviceManager::printDeviceStatus() {
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "=== Device Status ===");
    for (const auto& pair : devices_) {
        const std::string& name = pair.first;
        bool is_open = pair.second ? pair.second->isOpen() : false;
        RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), 
                   "Device: %s, Status: %s", 
                   name.c_str(), is_open ? "OPEN" : "CLOSED");
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceManager"), "==================");
}

} // namespace SearchID

