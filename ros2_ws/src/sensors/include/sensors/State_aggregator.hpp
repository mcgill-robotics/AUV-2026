#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
                std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


                rclcpp::Subscription<imu_msg>::SharedPtr imu_sub_;
                rclcpp::Subscription<float64_msg>::SharedPtr depth_sub_;
                rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr dvl_position_sub_;
                rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_velocity_sub_;
                rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vio_pose_sub_;

                void imu_callback(const imu_msg::SharedPtr imu_in);
                void depth_callback(const float64_msg::SharedPtr depth_in);
                void dvl_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr position_in);
                void dvl_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr velocity_in);
                void vio_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_in);
                void publish_state();                

                geometry_msgs::msg::PoseStamped current_pose_; //Final aggregated pose message

                double publish_frequency_; // Hz

                double current_depth_; // Depth Sensor
                Vec3 current_position_dvl_; // X,Y,Z postion from DVL
                Vec3 current_velocity_; // DVL
                geometry_msgs::msg::Quaternion current_orientation_imu_; // Imu
                
                Vec3 current_position_vio_; // X,Y,Z position from VIO
                geometry_msgs::msg::Quaternion current_orientation_vio_; // VIO orientation

                std::string frame_id_auv_;
		        std::string frame_id_global_;			
                bool publish_pose_tf_;
                
                bool use_vio_for_position_;
                bool use_vio_for_orientation_;

        };
        



}
