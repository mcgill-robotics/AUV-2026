#include "controls/superimposer.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto superimposer_node = std::make_shared<controls::superimposer>();
    rclcpp::spin(superimposer_node);
    rclcpp::shutdown();
    return 0;
}