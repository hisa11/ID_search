#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

/**
 * マイコンコード互換性テスト
 * 
 * 現在のマイコンコードと統合システムの通信をテストします
 */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // パラメータ取得
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto temp_node = std::make_shared<rclcpp::Node>("param_getter", options);
    
    int baudrate = temp_node->get_parameter_or("baudrate", 115200);
    
    std::cout << "\n╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     マイコンコード互換性テスト (nucleo1)      ║" << std::endl;
    std::cout << "║ 現在のマイコンコードとの通信を確認します       ║" << std::endl;
    std::cout << "║ ボーレート: " << baudrate << " bps                     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;

    // 指定ボーレートでシステム作成
    auto system = my_cpp_pkg::create_integrated_system("PC_TEST", {"nucleo1"}, true, baudrate);
    
    // コールバック設定
    system->setSerialDataHandler([&](const std::string& mc_id, const std::vector<std::string>& data) {
        std::cout << "\n📨 [マイコン応答] " << mc_id << " から受信:" << std::endl;
        std::cout << "   生データ: ";
        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << "[" << data[i] << "]";
            if (i < data.size() - 1) std::cout << ",";
        }
        std::cout << std::endl;
        
        // 応答の解析
        if (!data.empty()) {
            if (data[0] == "ID_RESPONSE" || data[0] == "101") {
                std::cout << "   ✅ ID応答: デバイスID = " << (data.size() > 1 ? data[1] : "不明") << std::endl;
            }
            else if (data[0] == "102") {
                std::cout << "   ✅ データ応答: ランダムID = " << (data.size() > 2 ? data[2] : "不明") 
                         << ", ステータス = " << (data.size() > 3 ? data[3] : "不明") << std::endl;
            }
            else if (data[0] == "SENSOR_DATA") {
                std::cout << "   📊 センサーデータ受信:" << std::endl;
                for (size_t i = 1; i < data.size(); i += 2) {
                    if (i + 1 < data.size()) {
                        std::cout << "     " << data[i] << " = " << data[i+1] << std::endl;
                    }
                }
            }
            else if (data[0] == "TASK_COMPLETE") {
                std::cout << "   ✅ タスク完了: " << (data.size() > 1 ? data[1] : "不明") 
                         << ", 結果 = " << (data.size() > 2 ? data[2] : "不明") << std::endl;
            }
            else if (data[0] == "FORWARD_TO_NODE") {
                std::cout << "   🔄 転送要求: " << (data.size() > 1 ? data[1] : "不明") << " への転送" << std::endl;
            }
            else if (data[0] == "STATUS") {
                std::cout << "   💡 ステータス情報:" << std::endl;
                for (size_t i = 1; i < data.size(); ++i) {
                    std::cout << "     - " << data[i] << std::endl;
                }
            }
            else {
                std::cout << "   ❓ 未知の応答形式" << std::endl;
            }
        }
    });
    
    // テストシーケンス
    std::thread test_thread([&]() {
        std::cout << "\n⏳ 3秒待機後、テストを開始します..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "\n🧪 === テストシーケンス開始 ===" << std::endl;
        
        // テスト1: ID探索（現在のマイコンコード対応版）
        std::cout << "\n📋 [テスト1] ID探索コマンド (マイコン互換形式)" << std::endl;
        std::vector<std::string> id_cmd = {"1", "PC_TEST", "0"};
        std::cout << "送信: ";
        for (const auto& d : id_cmd) std::cout << d << ",";
        std::cout << std::endl;
        
        if (system->sendToMicrocontroller("nucleo1", id_cmd)) {
            std::cout << "✅ ID探索コマンド送信成功" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // テスト2: データコマンド
        std::cout << "\n📋 [テスト2] データコマンド" << std::endl;
        std::vector<std::string> data_cmd = {"2", "PC_TEST", "nucleo1", "12345"};
        std::cout << "送信: ";
        for (const auto& d : data_cmd) std::cout << d << ",";
        std::cout << std::endl;
        
        if (system->sendToMicrocontroller("nucleo1", data_cmd)) {
            std::cout << "✅ データコマンド送信成功" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // テスト3: センサーコマンド（改善版マイコン用）
        std::cout << "\n📋 [テスト3] センサーコマンド" << std::endl;
        std::vector<std::string> sensor_cmd = {"GET_SENSOR", "temperature"};
        if (system->sendToMicrocontroller("nucleo1", sensor_cmd)) {
            std::cout << "✅ センサーコマンド送信成功" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // テスト4: タスク実行コマンド（改善版マイコン用）
        std::cout << "\n📋 [テスト4] タスク実行コマンド" << std::endl;
        std::vector<std::string> task_cmd = {"EXECUTE_TASK", "motion_control"};
        if (system->sendToMicrocontroller("nucleo1", task_cmd)) {
            std::cout << "✅ タスク実行コマンド送信成功" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // テスト5: ステータス確認
        std::cout << "\n📋 [テスト5] ステータス確認" << std::endl;
        std::vector<std::string> status_cmd = {"GET_STATUS"};
        if (system->sendToMicrocontroller("nucleo1", status_cmd)) {
            std::cout << "✅ ステータス確認コマンド送信成功" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n🎯 === テスト完了 ===" << std::endl;
        std::cout << "💡 期待される応答:" << std::endl;
        std::cout << "   テスト1: 101,nucleo1,0,|" << std::endl;
        std::cout << "   テスト2: 102,nucleo1,12345,OK,|" << std::endl;
        std::cout << "   テスト3: SENSOR_DATA,temperature,25.6|" << std::endl;
        std::cout << "   テスト4: TASK_COMPLETE,motion_control,success|" << std::endl;
        std::cout << "   テスト5: STATUS,online,led_ready,sensor_active|" << std::endl;
        std::cout << "\n🔄 継続監視中..." << std::endl;
    });
    
    // メインループ
    std::cout << "\n⚡ テストシステム起動中..." << std::endl;
    
    try {
        rclcpp::spin(system);
    } catch (const rclcpp::exceptions::RCLError& e) {
        std::cerr << "❌ RCLエラー: " << e.what() << std::endl;
    }
    
    if (test_thread.joinable()) {
        test_thread.join();
    }
    
    rclcpp::shutdown();
    std::cout << "\n👋 テスト終了" << std::endl;
    return 0;
}

/*
=== 使用方法 ===

1. マイコンに現在のコードを書き込み
2. シリアル接続確認
3. テスト実行:
   $ ros2 run my_cpp_pkg microcontroller_test

=== 期待される動作 ===

✅ 動作するテスト:
- テスト1: ID探索 (1,PC_TEST,0 → 101,nucleo1,0)
- テスト2: データ送信 (2,PC_TEST,nucleo1,12345 → 102,nucleo1,12345,OK)

⚠️ 改善版機能（マイコンコード拡張後）:
- テスト3-5: センサー、タスク、ステータス

=== トラブルシューティング ===

1. シリアル通信が無い場合:
   - デバイス接続確認 (/dev/ttyACM0など)
   - ボーレート確認 (115200)

2. 応答が無い場合:
   - マイコンの電源確認
   - LEDの点滅確認
   - シリアルモニターでの直接確認

3. 形式エラーの場合:
   - 区切り文字 '|' の確認
   - カンマ区切りの確認
*/
