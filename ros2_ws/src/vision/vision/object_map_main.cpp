#include "vision/object_map.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}