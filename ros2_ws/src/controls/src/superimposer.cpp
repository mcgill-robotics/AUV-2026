#include "controls/superimposer.hpp"

namespace controls
{ 
   superimposer::superimposer(): Node("superimposer")
   {
       // Initialize subscriptions
       sub_imu_ = this->create_subscription<imu_msg>(
           "processed/imu",
           1,
           std::bind(&superimposer::imu_callback, this, std::placeholders::_1)
       );

       sub_depth_effort_ = this->create_subscription<wrench_msg>(
           "/controls/depth_effort",
           1,
           std::bind(&superimposer::depth_effort_callback, this, std::placeholders::_1)
       );

       sub_attitude_effort_ = this->create_subscription<wrench_msg>(
           "/controls/attitude_effort",
           1,
           std::bind(&superimposer::attitude_effort_callback, this, std::placeholders::_1)
       );

       // Initialize publisher
       pub_effort_ = this->create_publisher<wrench_msg>("/controls/combined_effort", 1);

       // Initialize effort bias if needed
       this->declare_parameter<double>("effort_bias_force_x", 0.0);
       this->declare_parameter<double>("effort_bias_force_y", 0.0);
       this->declare_parameter<double>("effort_bias_force_z", 0.0);
       this->declare_parameter<double>("effort_bias_torque_x", 0.0);
       this->declare_parameter<double>("effort_bias_torque_y", 0.0);
       this->declare_parameter<double>("effort_bias_torque_z", 0.0);
       effort_bias_force_x = std::make_unique<double>(this->get_parameter("effort_bias_force_x").as_double());
       effort_bias_force_y = std::make_unique<double>(this->get_parameter("effort_bias_force_y").as_double());
       effort_bias_force_z = std::make_unique<double>(this->get_parameter("effort_bias_force_z").as_double());
       effort_bias_torque_x = std::make_unique<double>(this->get_parameter("effort_bias_torque_x").as_double());
       effort_bias_torque_y = std::make_unique<double>(this->get_parameter("effort_bias_torque_y").as_double());
       effort_bias_torque_z = std::make_unique<double>(this->get_parameter("effort_bias_torque_z").as_double());

   }

   void superimposer::imu_callback(const imu_msg::SharedPtr msg)
   {
       // Extract orientation quaternion from IMU message
       q_iv_ = quatd(
           msg->orientation.w,
           msg->orientation.x,
           msg->orientation.y,
           msg->orientation.z
       );
   }

   void superimposer::depth_effort_callback(const wrench_msg::SharedPtr msg)
   {
       depth_effort_ = *msg;
   }

   void superimposer::attitude_effort_callback(const wrench_msg::SharedPtr msg)
   {
       attitude_effort_ = *msg;
   }

   void superimposer::publish_combined_effort()
   {
       wrench_msg combined_effort;

       // Combine efforts (simple summation)
       combined_effort.force.x = depth_effort_.force.x + attitude_effort_.force.x +
                                 *effort_bias_force_x;
       combined_effort.force.y = depth_effort_.force.y + attitude_effort_.force.y +
                                 *effort_bias_force_y;
       combined_effort.force.z = depth_effort_.force.z + attitude_effort_.force.z +
                                 *effort_bias_force_z;
       combined_effort.torque.x = depth_effort_.torque.x + attitude_effort_.torque.x +
                                  *effort_bias_torque_x;
       combined_effort.torque.y = depth_effort_.torque.y + attitude_effort_.torque.y +
                                  *effort_bias_torque_y;
       combined_effort.torque.z = depth_effort_.torque.z + attitude_effort_.torque.z +
                                  *effort_bias_torque_z;

       // Publish combined effort
       pub_effort_->publish(combined_effort);
   }

} 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto superimposer_node = std::make_shared<controls::superimposer>();
    rclcpp::spin(superimposer_node);
    rclcpp::shutdown();
    return 0;
}