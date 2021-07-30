#include "rclcpp/rclcpp.hpp"
// #include "rpi_robot_bringup/basic_data.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("basic_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}