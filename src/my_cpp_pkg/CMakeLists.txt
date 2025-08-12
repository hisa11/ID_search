cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# サービス定義を生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
  "msg/Controller.msg"
  "srv/DataExchange.srv" 
)

# カスタムインターフェースのタイプサポートを取得
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ヘッダーファイルのインクルードディレクトリを追加
include_directories(src)

# シリアル通信ライブラリの作成
add_library(serial_lib
  src/serial.cpp
)

target_include_directories(serial_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_lib
  rclcpp
)

# サンプル実行ファイルの作成
add_executable(serial_example src/serial_example.cpp)
target_link_libraries(serial_example serial_lib)
ament_target_dependencies(serial_example
  rclcpp
)

# SearchID ライブラリとサンプル
add_library(searchid_lib src/serchID.cpp)
target_link_libraries(searchid_lib serial_lib)
ament_target_dependencies(searchid_lib
  rclcpp
  std_msgs
  std_srvs
)
# カスタムインターフェースとのリンク
target_link_libraries(searchid_lib "${cpp_typesupport_target}")

add_executable(searchid_simple_example src/searchid_simple_example.cpp)
target_link_libraries(searchid_simple_example searchid_lib serial_lib)
ament_target_dependencies(searchid_simple_example
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
)
target_link_libraries(searchid_simple_example "${cpp_typesupport_target}")

# sendid_simple_example の実行ファイル
add_executable(sendid_simple_example src/sendid_simple_example.cpp)
target_link_libraries(sendid_simple_example searchid_lib serial_lib)
ament_target_dependencies(sendid_simple_example
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
)
target_link_libraries(sendid_simple_example "${cpp_typesupport_target}")

# タイムアウトテスト用の実行ファイル
add_executable(timeout_test_example src/timeout_test_example.cpp)
target_link_libraries(timeout_test_example searchid_lib serial_lib)
ament_target_dependencies(timeout_test_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(timeout_test_example "${cpp_typesupport_target}")

# topic ライブラリの作成
add_library(topic_lib src/topic.cpp)
ament_target_dependencies(topic_lib
  rclcpp
  std_msgs
)

# topic_node に main 関数を追加
add_executable(topic_node src/topic.cpp)
ament_target_dependencies(topic_node
  rclcpp
  std_msgs
)

# topic_listener_example の実行ファイル
add_executable(topic_listener_example src/topic_listener_example.cpp)
target_link_libraries(topic_listener_example topic_lib)
ament_target_dependencies(topic_listener_example
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# サンプル用の実行ファイル
add_executable(add_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_server
  rclcpp
)
# 新しい方法でリンク
target_link_libraries(add_server "${cpp_typesupport_target}")

add_executable(add_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_client
  rclcpp
)
# 新しい方法でリンク
target_link_libraries(add_client "${cpp_typesupport_target}")

# シリアル通信テスト用の実行ファイル
add_executable(serial_test src/serial_test.cpp)
target_link_libraries(serial_test serial_lib)
ament_target_dependencies(serial_test
  rclcpp
)

# service ライブラリ
add_library(service_lib src/service.cpp)
target_link_libraries(service_lib topic_lib)
ament_target_dependencies(service_lib rclcpp)
target_link_libraries(service_lib ${cpp_typesupport_target})

target_include_directories(service_lib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)

# サービスサンプル
add_executable(service_server_example src/service_server_example.cpp)
target_link_libraries(service_server_example service_lib topic_lib)
ament_target_dependencies(service_server_example rclcpp)
add_dependencies(service_server_example my_cpp_pkg__cpp)
target_include_directories(service_server_example PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(service_server_example ${cpp_typesupport_target})

add_executable(service_client_example src/service_client_example.cpp)
target_link_libraries(service_client_example service_lib topic_lib)
ament_target_dependencies(service_client_example rclcpp)
add_dependencies(service_client_example my_cpp_pkg__cpp)
target_include_directories(service_client_example PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(service_client_example ${cpp_typesupport_target})

# ヘッダーファイルのインストール
install(
  DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# インストール設定
install(TARGETS
  serial_lib
  searchid_lib
  topic_lib
  serial_example
  searchid_simple_example
  timeout_test_example
  topic_node
  topic_listener_example
  add_server
  add_client
  service_server_example
  service_client_example
  DESTINATION lib/${PROJECT_NAME}
)

# ライブラリのエクスポート
ament_export_targets(serial_lib_targets HAS_LIBRARY_TARGET)
ament_export_targets(searchid_lib_targets HAS_LIBRARY_TARGET)
ament_export_targets(topic_lib_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
)

install(
  TARGETS serial_lib
  EXPORT serial_lib_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS searchid_lib
  EXPORT searchid_lib_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS topic_lib
  EXPORT topic_lib_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 実行ファイルのインストール
install(TARGETS
  serial_example
  serial_test
  searchid_simple_example
  timeout_test_example
  topic_listener_example
  add_server
  add_client
  service_server_example
  service_client_example
  sendid_simple_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()