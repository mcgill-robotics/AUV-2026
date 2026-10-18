#include "controls/attitude_controller.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto AttitudeControllerNode = std::make_shared<controls::AttitudeController>();
    rclcpp::spin(AttitudeControllerNode);
    rclcpp::shutdown();
    return 0;
}