# ROS2 C++ Package - API Reference

このパッケージでは、シリアル通信、デバイス管理、ROSトピック/サービス通信の機能を提供します。

## ヘッダーファイル一覧

### 1. `serial.hpp` - シリアル通信

#### SerialCommunicationクラス
基本的なシリアル通信機能を提供するROSノードクラス

**コンストラクタ**
```cpp
SerialCommunication(const std::string& node_name, 
                   const std::string& port = "/dev/ttyUSB0", 
                   speed_t baudrate = B9600,
                   const std::string& delimiter = "\n");
```

**主要メソッド**
- `bool initialize()` - シリアルポートを初期化
- `bool send(const std::string& message)` - メッセージ送信（区切り文字自動追加）
- `bool sendRaw(const std::string& message)` - メッセージ送信（区切り文字なし）
- `void setReceiveCallback(ReceiveCallback callback)` - 受信コールバック設定
- `void startReceiving()` - 受信開始
- `void stopReceiving()` - 受信停止
- `bool isOpen() const` - ポート状態確認

**静的メソッド**
- `static std::vector<std::shared_ptr<SerialCommunication>> scanAndConnectDevices(const std::vector<std::string>& ports, speed_t baudrate = B9600, const std::string& delimiter = "|")` - 複数ポートを自動スキャンして接続

---

### 2. `serchID.hpp` - デバイス検索・ID管理

#### SearchID::DeviceManagerクラス（Singleton）
複数デバイスの統合管理機能

**基本的なデバイス管理**
- `static DeviceManager& getInstance()` - インスタンス取得
- `void registerDevice(const std::string& device_name, std::shared_ptr<SerialCommunication> device)` - デバイス登録
- `void registerDevices(const std::vector<std::shared_ptr<SerialCommunication>>& devices)` - 複数デバイス一括登録
- `std::shared_ptr<SerialCommunication> getDevice(const std::string& device_name)` - デバイス取得
- `std::vector<std::string> getDeviceList() const` - 登録デバイス一覧取得

**メッセージ送信機能**
- `bool sendToDevice(const std::string& device_name, const std::string& message)` - 指定デバイスに送信
- `void broadcastToAllDevices(const std::string& message)` - 全デバイスに送信
- `bool sendByID(const std::string& device_id, const std::string& message)` - デバイスIDで送信

**デバイス状態管理**
- `bool isDeviceOpen(const std::string& device_name) const` - デバイス状態確認
- `bool isDeviceUsable(const std::string& device_name) const` - デバイス使用可能性確認
- `std::set<std::string> getUnusableDevices() const` - 使用不可デバイス一覧

**ID探索・管理機能**
- `void setDeviceName(const std::string& device_name)` - PC名設定
- `std::string getDeviceName() const` - PC名取得
- `void start(int timeout_ms = 1000)` - ID探索開始（自動コールバック設定）
- `void startWithMessage(const std::string& custom_message)` - カスタムメッセージで探索開始
- `std::unordered_map<std::string, std::string> getDeviceIDMap() const` - IDマッピング取得

**受信・コールバック管理**
- `void setDeviceCallback(const std::string& device_name, std::function<void(const std::string&)> callback)` - 受信コールバック設定
- `void startDeviceReceiving(const std::string& device_name)` - デバイス受信開始
- `void stopDeviceReceiving(const std::string& device_name)` - デバイス受信停止

#### SearchID::IDクラス（静的メソッド）
簡単なID指定送信機能

**主要メソッド**
- `static bool send(const std::string& device_id, const std::string& message)` - IDを指定してメッセージ送信
- `static void sendToMultiple(const std::vector<std::string>& device_ids, const std::string& message)` - 複数IDに同じメッセージ送信
- `static std::vector<std::string> getAvailableIDs()` - 利用可能なデバイスID一覧取得

#### SearchIDヘルパー関数
名前空間内で直接利用可能な便利関数

**デバイス作成・登録**
- `createAndRegisterDevice(device_name, node_name, port, baudrate, delimiter)` - デバイス作成と登録を同時実行
- `registerDevices(devices)` - 複数デバイス一括登録

**メッセージ送信**
- `sendByName(device_name, message)` - デバイス名で送信
- `sendToAll(message)` - 全デバイスに送信
- `sendByID(device_id, message)` - デバイスIDで送信

**状態確認**
- `getDevices()` - デバイス一覧取得
- `isOpen(device_name)` - デバイス状態確認
- `isDeviceUsable(device_name)` - デバイス使用可能性確認

**設定・制御**
- `setDeviceName(device_name)` - PC名設定
- `getDeviceName()` - PC名取得
- `start(timeout_ms)` - ID探索開始
- `startWithMessage(custom_message)` - カスタムメッセージで探索開始

---

### 3. `topic.hpp` - ROSトピック通信

#### custom_interfaces::PublisherNodeクラス
メッセージ送信用ノード

**主要メソッド**
- `void publish_message(const std::string& message)` - 指定メッセージ送信
- `void publish_message_with_count(const std::string& base_message)` - カウンター付きメッセージ送信
- `size_t get_count() const` - 現在のカウント値取得

#### custom_interfaces::SubscriberNodeクラス
メッセージ受信用ノード

#### custom_interfaces ユーティリティ関数
- `int run_publisher_once(argc, argv, message)` - 一回だけメッセージ送信
- `int run_publisher_interactive(argc, argv)` - 対話的モードでメッセージ送信
- `int run_subscriber(argc, argv)` - メッセージ受信プログラム実行
- `bool topic_send(const std::string& message)` - 簡単メッセージ送信
- `bool topic_init(argc, argv)` - グローバルPublisher初期化
- `void topic_cleanup()` - グローバルPublisher終了

---

### 4. `service.hpp` - ROSサービス通信

#### Serviceクラス（サーバー側）
整数加算サービスのサーバー実装

**主要メソッド**
- `Service(const std::string &serviceID)` - コンストラクタ（サービスID指定）
- `void start()` - サービス開始
- `void stop()` - サービス停止
- `bool is_running() const` - サービス実行状態確認

#### Clientクラス（クライアント側）
整数加算サービスのクライアント実装

**主要メソッド**
- `Client(const std::string &clientID, const std::string &service_name)` - コンストラクタ
- `bool send_request(int64_t a, int64_t b, int64_t c)` - 3つの整数でリクエスト送信
- `bool send_request(int64_t a, int64_t b, int64_t c, int64_t d)` - 4つの整数でリクエスト送信（dは未使用）
- `int64_t get_last_result() const` - 最後のレスポンス結果取得
- `bool is_service_available()` - サービス利用可能性確認
- `bool wait_for_service(std::chrono::seconds timeout)` - サービス利用可能まで待機

---

## 実行可能ファイル一覧

### シリアル通信関連
- **serial_example** - 基本的なシリアル通信のサンプル実装
- **serial_test** - シリアル通信機能のテスト用プログラム

### デバイス検索・ID管理関連
- **searchid_example** - デバイス検索機能の基本サンプル
- **searchid_simple_example** - シンプルなデバイス検索の実装例
- **id_request_example** - デバイスIDリクエスト機能のサンプル
- **id_request_timeout_example** - タイムアウト機能付きIDリクエスト
- **id_send_example** - デバイスID送信機能のテスト
- **id_send_example_v2** - ID送信機能の安定版実装

### タイムアウト・テスト関連
- **timeout_test_example** - タイムアウト機能のテスト用プログラム

### ROS通信関連
- **topic_node** - ROSトピック通信のノード実装
- **add_server** - 整数加算サービスのサーバー側実装
- **add_client** - 整数加算サービスのクライアント側実装

## 基本的な使用例

### 1. シリアルデバイスの自動スキャンと接続
```cpp
#include "serial.hpp"
#include "serchID.hpp"

// ポートを指定してデバイスをスキャン
std::vector<std::string> ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};
auto devices = SerialCommunication::scanAndConnectDevices(ports, B9600, "|");

// DeviceManagerに一括登録
SearchID::registerDevices(devices);
```

### 2. ID探索と個別メッセージ送信
```cpp
// PC名設定とID探索開始
SearchID::setDeviceName("PC1");
SearchID::start(1000);  // 1秒タイムアウト

// 個別送信
SearchID::ID::send("nucleo1", "asagohan");
SearchID::ID::send("nucleo2", "bangohan");

// 複数デバイスに同じメッセージ
std::vector<std::string> targets = {"nucleo1", "nucleo2"};
SearchID::ID::sendToMultiple(targets, "hello");
```

### 3. ROSトピック通信
```cpp
#include "topic.hpp"

// 簡単なメッセージ送信
custom_interfaces::topic_init(argc, argv);
custom_interfaces::topic_send("Hello World");
custom_interfaces::topic_cleanup();
```

### 4. ROSサービス通信
```cpp
#include "service.hpp"

// サーバー側
Service server("my_service");
server.start();

// クライアント側
Client client("my_client", "add_three_ints");
client.send_request(1, 2, 3);
int64_t result = client.get_last_result();
```
