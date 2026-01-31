#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sensors
{
    // Standard Eigen Typedefs for cleaner code
    using Vec3  = Eigen::Vector3d;
    using quatd = Eigen::Quaterniond;

    // 1. The Mathematical Logic
    class DvlProcessor {
    public:
        // r_sd_v: Vector from Sensor to Vehicle CM in Body Frame
        explicit DvlProcessor(const Vec3& r_sd_v);

        // Process: Returns the corrected position of the Center of Mass
        Vec3 process(const Vec3& p_dvl_i, const quatd& q_vi) const;

    private:
        Vec3 r_sd_v_; // The fixed physical offset
    };

    // 2. The ROS Node Implementation
    
    class DvlNode : public rclcpp::Node {
    public:
        DvlNode();

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void dvl_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

        // ROS Interfaces
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr dvl_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr state_pub_;

        // Member Variables
        std::unique_ptr<DvlProcessor> processor_;
        quatd current_orientation_;
    };

} // namespace sensors