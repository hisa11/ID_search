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
find_package(rosidl_default_generators REQUIRED)

# サービス定義を生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
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

add_executable(searchid_example src/searchid_example.cpp)
target_link_libraries(searchid_example searchid_lib serial_lib)
ament_target_dependencies(searchid_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(searchid_example "${cpp_typesupport_target}")

add_executable(searchid_simple_example src/searchid_simple_example.cpp)
target_link_libraries(searchid_simple_example searchid_lib serial_lib)
ament_target_dependencies(searchid_simple_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(searchid_simple_example "${cpp_typesupport_target}")

add_executable(id_request_example src/id_request_example.cpp)
target_link_libraries(id_request_example searchid_lib serial_lib)
ament_target_dependencies(id_request_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(id_request_example "${cpp_typesupport_target}")

add_executable(id_request_timeout_example src/id_request_timeout_example.cpp)
target_link_libraries(id_request_timeout_example searchid_lib serial_lib)
ament_target_dependencies(id_request_timeout_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(id_request_timeout_example "${cpp_typesupport_target}")

# タイムアウトテスト用の実行ファイル
add_executable(timeout_test_example src/timeout_test_example.cpp)
target_link_libraries(timeout_test_example searchid_lib serial_lib)
ament_target_dependencies(timeout_test_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(timeout_test_example "${cpp_typesupport_target}")

# topic_node に main 関数を追加
add_executable(topic_node src/topic.cpp)
ament_target_dependencies(topic_node
  rclcpp
  std_msgs
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

# ID送信テスト用の実行ファイル
add_executable(id_send_example src/id_send_example.cpp)
target_link_libraries(id_send_example searchid_lib serial_lib)
ament_target_dependencies(id_send_example
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(id_send_example "${cpp_typesupport_target}")

# ID送信例（安定版）
add_executable(id_send_example_v2 src/id_send_example_v2.cpp)
target_link_libraries(id_send_example_v2 searchid_lib serial_lib)
ament_target_dependencies(id_send_example_v2
  rclcpp
)
target_link_libraries(id_send_example_v2 "${cpp_typesupport_target}")

# シリアル通信テスト用の実行ファイル
add_executable(serial_test src/serial_test.cpp)
target_link_libraries(serial_test serial_lib)
ament_target_dependencies(serial_test
  rclcpp
)

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
  serial_example
  searchid_example
  searchid_simple_example
  id_request_example
  id_request_timeout_example
  timeout_test_example
  topic_node
  add_server
  add_client
  DESTINATION lib/${PROJECT_NAME}
)

# ライブラリのエクスポート
ament_export_targets(serial_lib_targets HAS_LIBRARY_TARGET)
ament_export_targets(searchid_lib_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  rosidl_default_runtime
)

install(
  TARGETS serial_lib searchid_lib
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

# 実行ファイルのインストール
install(TARGETS
  serial_example
  serial_test
  searchid_example
  searchid_simple_example
  id_request_example
  id_request_timeout_example
  timeout_test_example
  add_server
  add_client
  id_send_example
  id_send_example_v2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()