#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>

#include <memory>
#include <vector>
#include <functional>


#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"


namespace sensors
{
 class Sensor_Node: public rclcpp::Node
        {
        public:
                Sensor_Node();
                ~Sensor_Node() = default;
                void imu_callback(const imu_msg::SharedPtr imu_in);
                void depth_callback(const float64_msg::SharedPtr depth_in);
        
        private:
                std::unique_ptr<ImuRepublisher> imu_repub_;
                std::unique_ptr<DepthRepublisher> depth_repub_;

                rclcpp::Publisher<imu_msg>::SharedPtr imu_pub_;
                rclcpp::Publisher<float64_msg>::SharedPtr depth_pub_;

                rclcpp::Subscription<imu_msg>::SharedPtr imu_sub_;
                rclcpp::Subscription<float64_msg>::SharedPtr depth_sub_;

                quatd q_sv_;
                quatd q_in_;
                Vec3 r_vs_v_; // Vector from depth sensor to vehicle, expressed in vehicle frame
        };
        



}