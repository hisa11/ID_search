#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <chrono>

/**
 * 統合通信システムの使用例
 * 
 * このサンプルは以下の機能を実演します：
 * 1. マイコンID探索
 * 2. 20ms後のノードID探索とマッピング
 * 3. ノード間通信（配列送信）
 * 4. マイコン通信（配列→区切り文字変換）
 * 5. マイコン経由の転送通信
 */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // パラメータ取得
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto temp_node = std::make_shared<rclcpp::Node>("param_getter", options);
    
    std::string node_name = temp_node->get_parameter_or("node_name", std::string("PC1"));
    std::vector<std::string> microcontrollers = temp_node->get_parameter_or("microcontrollers", 
                                                                            std::vector<std::string>({"nucleo1"}));
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "統合通信システム デモンストレーション" << std::endl;
    std::cout << "ノード名: " << node_name << std::endl;
    std::cout << "初期マイコン: ";
    for (const auto& mc : microcontrollers) {
        std::cout << mc << " ";
    }
    std::cout << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // === システム作成と初期化 ===
    std::cout << "\n[ステップ1] システム作成" << std::endl;
    auto system = my_cpp_pkg::create_integrated_system(node_name, microcontrollers, true);
    
    // コールバック設定
    system->setDataHandler([&](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
        std::cout << "[データ受信] " << request->source_node << " から受信:" << std::endl;
        std::cout << "  タイプ: " << request->request_type << std::endl;
        std::cout << "  メッセージ: " << request->message << std::endl;
        std::cout << "  データ数: " << request->string_values.size() << "個" << std::endl;
        for (size_t i = 0; i < request->string_values.size(); ++i) {
            std::cout << "    [" << i << "] " << request->string_values[i] << std::endl;
        }
    });
    
    system->setSerialDataHandler([&](const std::string& mc_id, const std::vector<std::string>& data) {
        std::cout << "[シリアル受信] マイコン " << mc_id << " から:" << std::endl;
        std::cout << "  データ数: " << data.size() << "個" << std::endl;
        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << "    [" << i << "] " << data[i] << std::endl;
        }
    });
    
    // === 初期化シーケンス実行 ===
    std::cout << "\n[ステップ2] 初期化シーケンス実行" << std::endl;
    
    std::thread init_thread([&]() {
        // 少し待機してからシステム初期化を開始
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n--- 初期化開始 ---" << std::endl;
        system->initializeSystem();
        
        // 初期化完了後、通信テストを実行
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // === 通信テスト ===
        std::cout << "\n[ステップ3] 通信テスト開始" << std::endl;
        
        // 3.1 マイコンに配列データ送信（区切り文字に変換される）
        std::cout << "\n--- マイコン通信テスト ---" << std::endl;
        std::vector<std::string> mc_data = {"sensor1", "100", "temperature", "25.6"};
        std::cout << "送信データ(配列): ";
        for (const auto& d : mc_data) std::cout << "[" << d << "] ";
        std::cout << std::endl;
        
        if (system->sendToMicrocontroller("nucleo1", mc_data)) {
            std::cout << "→ マイコンへの送信成功（区切り文字形式に変換）" << std::endl;
        } else {
            std::cout << "→ マイコンへの送信失敗" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 3.2 他ノードに配列データ送信（配列のまま送信）
        std::cout << "\n--- ノード間通信テスト ---" << std::endl;
        std::vector<std::string> node_data = {"task1", "execute", "param1", "value1"};
        std::cout << "送信データ(配列): ";
        for (const auto& d : node_data) std::cout << "[" << d << "] ";
        std::cout << std::endl;
        
        if (system->sendToNode("PC2", "nucleo2", node_data, "ノード間通信テスト")) {
            std::cout << "→ ノードへの送信成功（配列形式のまま）" << std::endl;
        } else {
            std::cout << "→ ノードへの送信失敗またはPC2が見つからない" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 3.3 発見結果の表示
        std::cout << "\n--- 発見結果表示 ---" << std::endl;
        auto discovered_mcs = system->getDiscoveredMicrocontrollers();
        std::cout << "発見されたマイコン (" << discovered_mcs.size() << "個):" << std::endl;
        for (const auto& mc : discovered_mcs) {
            std::cout << "  - " << mc << std::endl;
        }
        
        auto node_mappings = system->getNodeMicrocontrollerMapping();
        std::cout << "ノード-マイコンマッピング (" << node_mappings.size() << "個):" << std::endl;
        for (const auto& mapping : node_mappings) {
            std::cout << "  " << mapping.first << ": ";
            for (const auto& mc : mapping.second) {
                std::cout << mc << " ";
            }
            std::cout << std::endl;
        }
        
        std::cout << "\n=== デモンストレーション完了 ===" << std::endl;
        std::cout << "Ctrl+Cで終了してください。" << std::endl;
    });
    
    // === メインループ ===
    std::cout << "\n[メインループ] 通信待機中..." << std::endl;
    std::cout << "シリアル通信とROS2サービス通信を受信待機します。" << std::endl;
    
    try {
        rclcpp::spin(system);
    } catch (const rclcpp::exceptions::RCLError& e) {
        std::cerr << "RCLエラー: " << e.what() << std::endl;
    }
    
    if (init_thread.joinable()) {
        init_thread.join();
    }
    
    rclcpp::shutdown();
    std::cout << "\nシステム終了" << std::endl;
    return 0;
}

/*
=== 実行例 ===

端末1 (PC1):
$ ros2 run my_cpp_pkg integration_explain --ros-args -p node_name:=PC1 -p microcontrollers:="[nucleo1]"

端末2 (PC2):
$ ros2 run my_cpp_pkg integration_explain --ros-args -p node_name:=PC2 -p microcontrollers:="[nucleo2, nucleo3]"

=== 想定される通信フロー ===

1. 初期化フェーズ:
   PC1 -> シリアル: "ID_REQUEST|"
   シリアル -> PC1: "ID_RESPONSE,nucleo1|"
   
   (20ms後)
   
   PC1 -> PC2: ROS2サービス（マイコン情報問い合わせ）
   PC2 -> PC1: ROS2レスポンス（nucleo2, nucleo3の情報）

2. 通信フェーズ:
   a) マイコン通信:
      PC1 -> nucleo1: "sensor1,100,temperature,25.6|" (配列→区切り文字)
   
   b) ノード間通信:
      PC1 -> PC2: ["task1", "execute", "param1", "value1"] (配列のまま)
   
   c) マイコン経由転送:
      nucleo1 -> PC1: "FORWARD_TO_NODE,PC2,nucleo1,data1,data2|"
      PC1 -> PC2: ROS2サービス（転送）
      
      nucleo1 -> PC1: "FORWARD_TO_MICROCONTROLLER,nucleo2,nucleo1,data1,data2|"
      PC1 -> PC2: 転送要求
      PC2 -> nucleo2: "data1,data2|" (区切り文字)

=== データ形式変換ルール ===

1. ノード間通信:
   - 送信: std::vector<std::string> → ROS2サービス(string_values)
   - 受信: ROS2サービス(string_values) → std::vector<std::string>

2. マイコン通信:
   - 送信: std::vector<std::string> → "data1,data2,data3|"
   - 受信: "data1,data2,data3|" → std::vector<std::string>

3. 転送通信:
   - マイコン→ノード: 内容変更なし、宛先のみ変更
   - マイコン→マイコン: 配列→区切り文字変換のみ
   - レスポンス: 経由ノードを通って返送

=== 設定可能なパラメータ ===

--ros-args -p node_name:=PC1                    # ノード名
--ros-args -p microcontrollers:="[nucleo1]"     # 所有マイコン一覧

*/
