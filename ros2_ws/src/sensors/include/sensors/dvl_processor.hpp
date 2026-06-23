#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <dvl_msgs/msg/dvl.hpp>
#include <dvl_msgs/msg/dvldr.hpp>
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
        
        //Takes new dvl velocity message and pushes it through the data pipeline below.
        void dvl_velocity_callback(const dvl_msgs::msg::DVL::SharedPtr msg);

        //Takes new dvl position message and pushes it through the data pipeline below.
        void dvl_position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg); 

        // Takes in raw dvl position data in dvl frame, transforms it to inertial frame, and returns transformed data
        Vec3 process_dvl_position(const Vec3& r_di2_i2) const;
                
        // Takes in raw dvl velocity data in dvl frame, transforms it to inertial frame, and returns transformed data
        Vec3 process_dvl_velocity(const Vec3& v_di2_d) const;

        // Publishes transformed position values as a PointStamped message
        void publish_position_msg(const Vec3& r_vp_p) const;
        
        // Publishes transformed velocity values as a TwistStamped message
        void publish_velocity_msg(const Vec3& v_vp_p) const;


        
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_vel_sub_;
        rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvl_pos_sub_;
        
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