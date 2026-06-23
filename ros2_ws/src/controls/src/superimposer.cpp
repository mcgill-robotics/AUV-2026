#include "controls/superimposer.hpp"

namespace controls
{ 
   superimposer::superimposer(): Node("superimposer")
   {
       // Initialize subscriptions
       sub_imu_ = this->create_subscription<imu_msg>(
           "auv_frame/imu",
           rclcpp::SensorDataQoS().keep_last(1),
           std::bind(&superimposer::imu_callback, this, std::placeholders::_1)
       );

       sub_depth_effort_ = this->create_subscription<wrench_msg>(
           "/controls/depth_effort",
           rclcpp::SensorDataQoS().keep_last(1),
           std::bind(&superimposer::depth_effort_callback, this, std::placeholders::_1)
       );

       sub_x_effort_ = this->create_subscription<wrench_msg>(
           "/controls/x_effort",
           rclcpp::SensorDataQoS().keep_last(1),
           std::bind(&superimposer::x_effort_callback, this, std::placeholders::_1)
       );

        sub_y_effort_ = this->create_subscription<wrench_msg>(
           "/controls/y_effort",
           rclcpp::SensorDataQoS().keep_last(1),
           std::bind(&superimposer::y_effort_callback, this, std::placeholders::_1)
       );

       sub_attitude_effort_ = this->create_subscription<wrench_msg>(
           "/controls/attitude_effort",
           rclcpp::SensorDataQoS().keep_last(1),
           std::bind(&superimposer::attitude_effort_callback, this, std::placeholders::_1)
       );

       // Initialize publisher
       pub_effort_ = this->create_publisher<wrench_msg>("/controls/total_effort", rclcpp::SensorDataQoS().keep_last(1));

       // Initialize effort bias if needed
       this->declare_parameter<double>("effort_bias_force_x", 0.0);
       this->declare_parameter<double>("effort_bias_force_y", 0.0);
       this->declare_parameter<double>("effort_bias_force_z", 0.0);
       this->declare_parameter<double>("effort_bias_torque_x", 0.0);
       this->declare_parameter<double>("effort_bias_torque_y", 0.0);
       this->declare_parameter<double>("effort_bias_torque_z", 0.0);
       this->declare_parameter<double>("publish_hz", 50.0); // publish frequency
       this->declare_parameter<double>("max_planar_effort", 0.0);
       this->declare_parameter<double>("max_depth_effort", 0.0);
       this->declare_parameter<double>("max_attitude_effort", 0.0);
       
       effort_bias_force_x = this->get_parameter("effort_bias_force_x").as_double();
       effort_bias_force_y = this->get_parameter("effort_bias_force_y").as_double();
       effort_bias_force_z = this->get_parameter("effort_bias_force_z").as_double();
       effort_bias_torque_x = this->get_parameter("effort_bias_torque_x").as_double();
       effort_bias_torque_y = this->get_parameter("effort_bias_torque_y").as_double();
       effort_bias_torque_z = this->get_parameter("effort_bias_torque_z").as_double();
       publish_hz_ = this->get_parameter("publish_hz").as_double();
       max_planar_effort_ = this->get_parameter("max_planar_effort").as_double();
       max_depth_effort_ = this->get_parameter("max_depth_effort").as_double();
       max_attitude_effort_ = this->get_parameter("max_attitude_effort").as_double();

       this->declare_parameter<bool>("enabled", true);
       enabled_ = this->get_parameter("enabled").as_bool();

       parameter_callback_handle_ = this->add_on_set_parameters_callback(
           std::bind(&superimposer::parameters_callback, this, std::placeholders::_1)
       );

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000 / publish_hz_)),   // Control loop frequency
            std::bind(&superimposer::publish_combined_effort, this)
            );

        // Initialize orientation quaternion to identity
        q_iv_ = quatd(1.0, 0.0, 0.0, 0.0);

        // Initialize effort messages
        depth_effort_ = wrench_msg();
        depth_effort_.force.x = 0.0;
        depth_effort_.force.y = 0.0;
        depth_effort_.force.z = 0.0;
        depth_effort_.torque.x = 0.0;
        depth_effort_.torque.y = 0.0;
        depth_effort_.torque.z = 0.0;

        attitude_effort_ = wrench_msg();
        attitude_effort_.force.x = 0.0;
        attitude_effort_.force.y = 0.0;
        attitude_effort_.force.z = 0.0;
        attitude_effort_.torque.x = 0.0;
        attitude_effort_.torque.y = 0.0;
        attitude_effort_.torque.z = 0.0;    

        x_effort_ = wrench_msg();
        x_effort_.force.x = 0.0;
        x_effort_.force.y = 0.0;
        x_effort_.force.z = 0.0;
        x_effort_.torque.x = 0.0;
        x_effort_.torque.y = 0.0;
        x_effort_.torque.z = 0.0;

        y_effort_ = wrench_msg();
        y_effort_.force.x = 0.0;
        y_effort_.force.y = 0.0;
        y_effort_.force.z = 0.0;
        y_effort_.torque.x = 0.0;
        y_effort_.torque.y = 0.0;
        y_effort_.torque.z = 0.0;

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
       depth_effort_ = *msg; // This is in the pool frame
   }

    void superimposer::x_effort_callback(const wrench_msg::SharedPtr msg)
   {
       x_effort_ = *msg; // This is in the pool frame
   }

    void superimposer::y_effort_callback(const wrench_msg::SharedPtr msg)
   {
       y_effort_ = *msg; // This is in the pool frame
   }

   void superimposer::attitude_effort_callback(const wrench_msg::SharedPtr msg)
   {
        attitude_effort_ = *msg;
    }

    rcl_interfaces::msg::SetParametersResult superimposer::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "enabled")
            {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    result.successful = false;
                    result.reason = "'enabled' must be a boolean";
                    return result;
                }
                enabled_ = parameter.as_bool();
                RCLCPP_INFO(this->get_logger(), "Superimposer enabled: %s", enabled_ ? "true" : "false");
            }
        }

        return result;
    }

   void superimposer::publish_combined_effort()
   {
       if (!enabled_) {
           return;
       }

       wrench_msg combined_effort;

       // Transform depth effort from pool frame to body frame. 
       Vec3 depth_force_pool(depth_effort_.force.x, depth_effort_.force.y, depth_effort_.force.z);
       if (max_depth_effort_ > 0.0 && depth_force_pool.norm() > max_depth_effort_) {
           depth_force_pool = depth_force_pool * (max_depth_effort_ / depth_force_pool.norm());
       }

       Vec3 x_force_pool(x_effort_.force.x, x_effort_.force.y, x_effort_.force.z);
       Vec3 y_force_pool(y_effort_.force.x, y_effort_.force.y, y_effort_.force.z);
       Vec3 planar_force_pool = x_force_pool + y_force_pool;
       if (max_planar_effort_ > 0.0 && planar_force_pool.norm() > max_planar_effort_) {
           planar_force_pool = planar_force_pool * (max_planar_effort_ / planar_force_pool.norm());
       }

       Vec3 total_force_pool = depth_force_pool + planar_force_pool;
       Vec3 total_force_body = q_iv_.inverse() * total_force_pool; // Rotate to body frame

       Vec3 attitude_torque(attitude_effort_.torque.x, attitude_effort_.torque.y, attitude_effort_.torque.z);
       if (max_attitude_effort_ > 0.0 && attitude_torque.norm() > max_attitude_effort_) {
           attitude_torque = attitude_torque * (max_attitude_effort_ / attitude_torque.norm());
       }

       // Combine efforts (simple summation)
       combined_effort.force.x = total_force_body.x() + effort_bias_force_x;
       combined_effort.force.y = total_force_body.y() + effort_bias_force_y;
       combined_effort.force.z = total_force_body.z() + effort_bias_force_z;
       combined_effort.torque.x = attitude_torque.x() + effort_bias_torque_x;
       combined_effort.torque.y = attitude_torque.y() + effort_bias_torque_y;
       combined_effort.torque.z = attitude_torque.z() + effort_bias_torque_z;
       
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