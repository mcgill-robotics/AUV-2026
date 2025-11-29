#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include "imuprocessing2.hpp" // Your processor header

class ImuTestNode : public rclcpp::Node
{
public:
    ImuTestNode() : Node("imu_test_node")
    {
        // 1. Initialize the Processor
        // In this test setup, 'handleZeroRequest' uses the stubbed quaternion (Identity),
        // so it sets the reference frame to align with the sensor's current frame.
        processor_.handleZeroRequest();
        RCLCPP_INFO(this->get_logger(), "Processor Initialized and Zeroed.");

        // 2. Create Subscription
        // We listen to 'test/imu_input'. You will publish to this from the terminal.
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "test/imu_input", 
            10, 
            std::bind(&ImuTestNode::topic_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Waiting for data on topic: /test/imu_input");
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // A. Extract Data from ROS Message
        Eigen::Vector4d live_quat(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );

        Eigen::Vector3d live_accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        Eigen::Vector3d dummy_gyro(0,0,0);

        // B. Run Your Logic
        Eigen::Vector3d result = processor_.processIMUData(live_quat, live_accel, dummy_gyro);

        // C. Print Result to Console
        // We use std::cout so you see it immediately in your terminal
        std::cout << "\n--- Recieved Input ---" << std::endl;
        std::cout << "Input Accel: " << live_accel.transpose() << std::endl;
        std::cout << "Input Quat:  " << live_quat.transpose() << std::endl;
        std::cout << ">> PROCESSED RESULT (Pool Frame): " << result.transpose() << std::endl;
    }

    PoolIMUProcessor processor_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuTestNode>());
    rclcpp::shutdown();
    return 0;
}