#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sensors
{
    using Vec3  = Eigen::Vector3d;
    using Quatd = Eigen::Quaterniond;

    // Struct to hold the raw data from the DVL sensor
    struct DvlData_DvlFrame {
        Vec3 r_di2_i2; 
        Vec3 v_di2_d;  
    };

    // Struct to hold the transformed dvl data in the pool inertial frame
    struct DvlData_InertialFrame {
        Vec3 r_vp_p; 
        Vec3 v_vp_p; 
    };

    class DvlProcessor : public rclcpp::Node {
    public:
        DvlProcessor();

    private:
        
        //Updates our stored vehicle orientation whenever the IMU publishes new data.
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); 
        
        //Takes new PoseWithCovarianceStamped message and pushes it through the data pipeline below.
        void dvl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg); 

        
        // Puts dvl position and velocity data in dvl frame from Pose message and puts into C++ struct.
        DvlData_DvlFrame parse_dvl(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const;
        
        // Takes in dvl position and velocity data in dvl frame, transforms it to inertial frame, and returns transformed data
        DvlData_InertialFrame process_dvl(const DvlData_DvlFrame& dvl_raw) const;
        
        // Takes transformed position values and builds a message to be published
        geometry_msgs::msg::PointStamped compose_position_msg(const DvlData_InertialFrame& dvl_inertial) const;
        
        // Takes transformed velocity values and builds a message to be published
        geometry_msgs::msg::TwistStamped compose_velocity_msg(const DvlData_InertialFrame& dvl_inertial) const;


        
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_sub_;
        
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

        Vec3 r_dv_v_; 
        std::string frame_id_global_;
        Quatd q_iv_; 
        Vec3 w_v_;
        Quatd q_pi2_;
        Quatd q_vd_;
    };

} 