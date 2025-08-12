#include "service.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // サービスIDを指定してサーバーを作成
    auto service_node = std::make_shared<Service>("add_three_ints");

    // サービスを開始
    service_node->start();

    RCLCPP_INFO(service_node->get_logger(), "サービスノードをスピンします");
    rclcpp::spin(service_node);

    // サービスを停止
    service_node->stop();

    rclcpp::shutdown();
    return 0;
}
