# 統合通信システム (Integration Communication System)

自作のサービス通信、マイコンID探索、シリアル通信を統合した包括的な通信システムです。

## 🌟 主な機能

### 1. 統合初期化シーケンス
- **マイコンID探索**: シリアル接続されたマイコンを自動発見
- **20ms待機**: システム安定化
- **ノードID探索**: ROS2ネットワーク上の他ノードを発見
- **マッピング**: どのノードがどのマイコンを制御するかをマッピング

### 2. データ形式変換
- **ノード間通信**: `std::vector<std::string>` (配列のまま送信)
- **マイコン通信**: 配列 → 区切り文字変換 (`"data1,data2,data3|"`)
- **受信時**: 区切り文字 → 配列変換

### 3. 転送機能
- **マイコン→ノード転送**: シリアル通信ノード経由でROS2ネットワークに転送
- **マイコン→マイコン転送**: 別ノード経由で他のマイコンに転送
- **レスポンス経由**: 応答も経由ノードを通って返送

## 📁 ファイル構成

```
src/
├── integration.hpp          # 統合システムのヘッダー
├── integration.cpp          # 統合システムの実装
├── integration_explain.cpp  # 基本的な使用例
├── integration_pc1_example.cpp  # PC1実用例
└── integration_pc2_example.cpp  # PC2実用例
```

## 🚀 使用方法

### 基本的な使用例

```bash
# 基本デモ
ros2 run my_cpp_pkg integration_explain

# パラメータ付きで実行
ros2 run my_cpp_pkg integration_explain --ros-args \
  -p node_name:=PC1 \
  -p microcontrollers:="[nucleo1]"
```

### 実用的な使用例

```bash
# 端末1: PC1として起動
ros2 run my_cpp_pkg integration_pc1_example

# 端末2: PC2として起動
ros2 run my_cpp_pkg integration_pc2_example
```

## 💻 プログラムでの使用方法

### 1. システム作成と初期化

```cpp
#include "integration.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // システム作成（シリアルデバイス自動検出）
    std::vector<std::string> my_microcontrollers = {"nucleo1"};
    auto system = create_integrated_system("PC1", my_microcontrollers, true);
    
    // 完全な初期化（マイコンID探索→20ms待機→ノード探索）
    system->initializeSystem();
    
    rclcpp::spin(system);
    rclcpp::shutdown();
    return 0;
}
```

### 2. コールバック設定

```cpp
// ノード間通信受信
system->setDataHandler([](const auto& request) {
    std::cout << "受信: " << request->source_node << " から" << std::endl;
    // request->string_values が配列のまま受信される
});

// シリアル通信受信
system->setSerialDataHandler([](const std::string& mc_id, const auto& data) {
    std::cout << "マイコン " << mc_id << " から受信" << std::endl;
    // data は区切り文字から配列に変換済み
});
```

### 3. データ送信

```cpp
// マイコンにデータ送信（配列→区切り文字変換）
std::vector<std::string> mc_data = {"GET_SENSOR", "temperature", "humidity"};
system->sendToMicrocontroller("nucleo1", mc_data);
// → "GET_SENSOR,temperature,humidity|" として送信

// ノードにデータ送信（配列のまま）
std::vector<std::string> node_data = {"EXECUTE_TASK", "param1", "value1"};
system->sendToNode("PC2", "nucleo2", node_data, "メッセージ");
// → ROS2サービスで配列のまま送信
```

### 4. 転送処理

```cpp
// マイコンからの転送要求を受信した場合
if (data[0] == "FORWARD_TO_NODE") {
    std::string target_node = data[1];
    std::vector<std::string> forward_data(data.begin() + 2, data.end());
    system->forwardMicrocontrollerToNode(source_mc, target_node, forward_data, "転送");
}

if (data[0] == "FORWARD_TO_MICROCONTROLLER") {
    std::string target_mc = data[1];
    std::vector<std::string> forward_data(data.begin() + 2, data.end());
    system->forwardMicrocontrollerToMicrocontroller(source_mc, target_mc, forward_data);
}
```

## 🔄 通信フロー例

### 初期化フェーズ
```
1. マイコンID探索:
   PC1 → シリアル: "ID_REQUEST|"
   シリアル → PC1: "ID_RESPONSE,nucleo1|"

2. 20ms待機

3. ノード探索:
   PC1 → PC2: ROS2サービス（マイコン情報問い合わせ）
   PC2 → PC1: レスポンス（nucleo2, nucleo3の情報）
```

### 通常通信
```
1. マイコン制御:
   PC1 → nucleo1: "GET_SENSOR,temperature,humidity|"
   nucleo1 → PC1: "SENSOR_DATA,25.6,60.2|"

2. ノード間通信:
   PC1 → PC2: ["EXECUTE_TASK", "motion_control", "speed:50"]
   PC2 → nucleo2: "EXECUTE_TASK,motion_control,speed:50|"
```

### 転送通信
```
1. マイコン→ノード転送:
   nucleo1 → PC1: "FORWARD_TO_NODE,PC2,sensor_data,25.6|"
   PC1 → PC2: ROS2サービス転送

2. マイコン→マイコン転送:
   nucleo1 → PC1: "FORWARD_TO_MICROCONTROLLER,nucleo2,cmd,start|"
   PC1 → PC2: 転送要求
   PC2 → nucleo2: "cmd,start|"

3. レスポンス経由:
   nucleo2 → PC2 → PC1 → nucleo1: レスポンス配信
```

## 🛠️ カスタマイズ

### シリアルポート設定
```cpp
// 自動検出ポートリストを変更
std::vector<std::string> custom_ports = {"/dev/ttyUSB0", "/dev/ttyUSB1"};
auto devices = auto_connect_serial_devices(custom_ports, B115200, "|");
auto system = std::make_shared<IntegratedCommunicationSystem>("PC1", microcontrollers, devices);
```

### マイコンプロトコル
```cpp
// マイコンから送信するデータ形式
"ID_RESPONSE,nucleo1|"                    // ID応答
"SENSOR_DATA,temperature,25.6|"           // データ送信
"FORWARD_TO_NODE,PC2,data1,data2|"        // ノード転送要求
"FORWARD_TO_MICROCONTROLLER,nucleo2,cmd|" // マイコン転送要求
```

## 🐛 デバッグ

### ログ出力の見方
- `📨 [ノード通信受信]`: ROS2サービス経由の受信
- `🔌 [シリアル通信受信]`: マイコンからの受信
- `🔄`: 転送処理の実行
- `✅/❌`: 処理の成功/失敗

### よくある問題
1. **シリアル接続失敗**: デバイス権限確認 `sudo chmod 666 /dev/ttyACM*`
2. **マイコンID応答なし**: マイコンファームウェアの応答実装確認
3. **ノード探索失敗**: ROS2ネットワーク設定確認

## 📈 拡張可能性

- **新しいマイコンタイプ**: ID応答プロトコルに対応すれば自動発見
- **カスタムデータ形式**: コールバック内で独自解析を実装
- **複雑なルーティング**: 転送ロジックをカスタマイズ
- **セキュリティ**: 認証・暗号化機能を追加

このシステムにより、複数のPC、マイコン、通信プロトコルを統一的に管理できます。
