#include "service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto temp_node_for_param = std::make_shared<rclcpp::Node>("param_getter", options);
    
    // ★★★ ここを修正 ★★★
    // "PC2" を std::string("PC2") に変更して、型を明示的にする
    std::string node_name = temp_node_for_param->get_parameter_or("node_name", std::string("PC2"));

    // こちらは元々正しい型なので変更不要
    std::vector<std::string> my_microcontrollers = temp_node_for_param->get_parameter_or("microcontrollers", std::vector<std::string>({}));

    auto node = std::make_shared<Service>(node_name, my_microcontrollers);

    node->set_data_handler([&](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request){
        RCLCPP_INFO(node->get_logger(), "[Handler] 自分宛のメッセージ(Type: %ld)を受信しました！", request->request_type);
        RCLCPP_INFO(node->get_logger(), "[Handler]   - 送信元: %s", request->source_node.c_str());
        RCLCPP_INFO(node->get_logger(), "[Handler]   - メッセージ: %s", request->message.c_str());
    });

    RCLCPP_INFO(node->get_logger(), "ノード '%s' を起動しました。待機します...", node->get_name());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}