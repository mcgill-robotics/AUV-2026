#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>
#include <functional>
#include <string>


#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"


namespace sensors
{
 class State_aggregator: public rclcpp::Node
        {
        public:
                State_aggregator();
                ~State_aggregator() = default;

        private:
 
                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
                rclcpp::TimerBase::SharedPtr publish_timer_;


                rclcpp::Subscription<imu_msg>::SharedPtr imu_sub_;
                rclcpp::Subscription<float64_msg>::SharedPtr depth_sub_;
                rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr dvl_position_sub_;
                rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_velocity_sub_;

                void imu_callback(const imu_msg::SharedPtr imu_in);
                void depth_callback(const float64_msg::SharedPtr depth_in);
                void dvl_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr position_in);
                void dvl_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr velocity_in);
                void publish_state();                

                geometry_msgs::msg::PoseStamped current_pose_; //Final aggregated pose message

                double publish_frequency_; // Hz

                double current_depth_; // Depth Sensor
                Vec3 current_position_dvl_; // X,Y,Z postion from DVL
                Vec3 current_velocity_; // DVL
                geometry_msgs::msg::Quaternion current_orientation_; // Imu

                std::string frame_id_auv_;
		std::string frame_id_global_;			

        };
        



}