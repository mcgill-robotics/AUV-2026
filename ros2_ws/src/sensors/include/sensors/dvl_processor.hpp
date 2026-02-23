#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sensors
{
    using Vec3  = Eigen::Vector3d;
    using Quatd = Eigen::Quaterniond;

    class DvlProcessor {
    public:
        explicit DvlProcessor(const Vec3& r_dv_v);

        // Calculates Vehicle Center of Mass position
        Vec3 process(const Vec3& r_di_i, const Quatd& q_iv) const;

    private:
        Vec3 r_dv_v_; 
    };

    class DvlNode : public rclcpp::Node {
    public:
        DvlNode();

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void dvl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr state_pub_;

        std::unique_ptr<DvlProcessor> processor_;
        Quatd q_iv_; 
    };

} 