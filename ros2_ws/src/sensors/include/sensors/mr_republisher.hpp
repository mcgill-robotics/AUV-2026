#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>

#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"


namespace sensors
{
 class MrRepublisher: rclcpp::Node
        {
        public:
                explicit MrRepublisher(const quatd& q_sv, const quatd& q_in, const Vec3& r_vs);
                void imu_callback(const imu_msg::SharedPtr imu_in);
                void depth_callback(const float64_msg::SharedPtr depth_in);
        
        private:
                ImuRepublisher imu_repub_;
                DepthRepublisher depth_repub_;
                rclcpp::Publisher<imu_msg>::SharedPtr imu_pub_;
                rclcpp::Publisher<float64_msg>::SharedPtr depth_pub_;
        };
        



}