#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

/**
 * ボーレート設定可能なマイコンテスト
 * 
 * 様々なボーレートでマイコンとの通信をテストします
 */

void test_with_baudrate(int baudrate) 
{
    std::cout << "\n╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     ボーレート " << baudrate << " bps でのテスト        ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;

    // 指定ボーレートでシステム作成
    auto system = my_cpp_pkg::create_integrated_system("PC_BAUDRATE_TEST", {"nucleo1"}, true, baudrate);
    
    // コールバック設定
    bool response_received = false;
    system->setSerialDataHandler([&](const std::string& mc_id, const std::vector<std::string>& data) {
        std::cout << "\n📨 [応答受信] " << mc_id << " から:" << std::endl;
        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << "   [" << i << "] " << data[i] << std::endl;
        }
        response_received = true;
    });
    
    // 少し待機後にテスト送信
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "\n📤 ID探索コマンド送信中..." << std::endl;
    std::vector<std::string> id_cmd = {"1", "PC_TEST", "0"};
    
    if (system->sendToMicrocontroller("nucleo1", id_cmd)) {
        std::cout << "✅ コマンド送信成功" << std::endl;
        
        // 応答待機
        auto start_time = std::chrono::steady_clock::now();
        while (!response_received && 
               std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
            // spin_someを使わずに、単純な待機
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (response_received) {
            std::cout << "✅ ボーレート " << baudrate << " で通信成功！" << std::endl;
        } else {
            std::cout << "❌ ボーレート " << baudrate << " で応答なし（タイムアウト）" << std::endl;
        }
    } else {
        std::cout << "❌ コマンド送信失敗" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // パラメータ取得
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto temp_node = std::make_shared<rclcpp::Node>("param_getter", options);
    
    int target_baudrate = temp_node->get_parameter_or("baudrate", 115200);
    bool test_multiple = temp_node->get_parameter_or("test_multiple", false);
    
    std::cout << "\n╔══════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║          ボーレート設定可能テスト               ║" << std::endl;
    std::cout << "║ マイコンとの通信速度を自動判定します             ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;
    
    if (test_multiple) {
        std::cout << "\n🔄 複数ボーレートでの自動判定テスト" << std::endl;
        
        // 一般的なボーレートを順次テスト
        std::vector<int> baudrates = {9600, 115200, 230400, 460800, 921600};
        
        for (int baud : baudrates) {
            std::cout << "\n" << std::string(50, '-') << std::endl;
            test_with_baudrate(baud);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        std::cout << "\n🎯 === 自動判定テスト完了 ===" << std::endl;
        std::cout << "💡 通信が成功したボーレートを使用してください。" << std::endl;
    } else {
        std::cout << "\n🎯 指定ボーレート: " << target_baudrate << " bps" << std::endl;
        test_with_baudrate(target_baudrate);
    }
    
    rclcpp::shutdown();
    return 0;
}

/*
=== 使用方法 ===

1. 指定ボーレートでテスト:
   $ ros2 run my_cpp_pkg baudrate_test --ros-args -p baudrate:=9600

2. 複数ボーレートで自動判定:
   $ ros2 run my_cpp_pkg baudrate_test --ros-args -p test_multiple:=true

3. デフォルト（115200）でテスト:
   $ ros2 run my_cpp_pkg baudrate_test

=== サポートされるボーレート ===

- 9600 bps    （低速、安定）
- 19200 bps
- 38400 bps
- 57600 bps
- 115200 bps  （標準）
- 230400 bps
- 460800 bps
- 921600 bps  （高速）
- 1000000 bps
- 2000000 bps （最高速）

=== トラブルシューティング ===

1. 通信できない場合:
   - マイコンのボーレート設定確認
   - ケーブル接続確認
   - デバイス権限確認

2. 応答がない場合:
   - 異なるボーレートで再試行
   - マイコンの電源再投入
   - /dev/ttyACM0 の存在確認

3. エラーメッセージの場合:
   - sudo usermod -a -G dialout $USER
   - ログアウト/ログイン
*/
