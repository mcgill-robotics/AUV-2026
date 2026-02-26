#include "sensors/dvl_processor.hpp"

namespace sensors {

DvlProcessor::DvlProcessor() : Node("dvl_processor") {
    q_iv_.setIdentity();

    this->declare_parameter<std::vector<double>>("r_dv_v", {0.0, 0.0, 0.0});
    std::vector<double> r_dv_v_vec;
    this->get_parameter("r_dv_v", r_dv_v_vec);
    r_dv_v_ = Vec3(r_dv_v_vec[0], r_dv_v_vec[1], r_dv_v_vec[2]);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu", 10, std::bind(&DvlProcessor::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "dvl/odometry", 10, std::bind(&DvlProcessor::dvl_callback, this, std::placeholders::_1));
    
// REVIEW TOPIC NAMES 
    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/dvl/position", 10); 
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("auv_frame/dvl/velocity", 10); 
}


//Function updates stored vehicle orientation whenever the IMU publishes new data.
void DvlProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_iv_.w() = msg->orientation.w;
    q_iv_.x() = msg->orientation.x;
    q_iv_.y() = msg->orientation.y;
    q_iv_.z() = msg->orientation.z;
}


//Function takes new odometry messages and pushes them through the data pipeline.
void DvlProcessor::dvl_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    DvlData_DvlFrame dvl_raw = parse_dvl(*msg);
    
    DvlData_InertialFrame dvl_inertial = process_dvl(dvl_raw);
    
    geometry_msgs::msg::PointStamped pos_msg = compose_position_msg(dvl_inertial);
    geometry_msgs::msg::TwistStamped vel_msg = compose_velocity_msg(dvl_inertial);

    position_pub_->publish(pos_msg);
    velocity_pub_->publish(vel_msg);
}

DvlData_DvlFrame DvlProcessor::parse_dvl(const nav_msgs::msg::Odometry& msg) const {
    DvlData_DvlFrame dvl_raw;

    dvl_raw.r_di_i = Vec3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    //Check what frame velocity data is in and update line below accordingly if necessary
    dvl_raw.v_di_d = Vec3(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    
    return dvl_raw; 
}

DvlData_InertialFrame DvlProcessor::process_dvl(const DvlData_DvlFrame& dvl_raw) const {
    DvlData_InertialFrame dvl_inertial;
    
    dvl_inertial.r_vi_i = dvl_raw.r_di_i - (q_iv_ * r_dv_v_);
    
    // Check what frame velocity data is in and update line below accordingly if necessary
    dvl_inertial.v_vi_i = q_iv_ * dvl_raw.v_di_d;
    
    return dvl_inertial;
}

geometry_msgs::msg::PointStamped DvlProcessor::compose_position_msg(const DvlData_InertialFrame& dvl_inertial) const {
    geometry_msgs::msg::PointStamped msg_out;
    msg_out.header.frame_id = "pool_link"; //REVIEW, not sure what the frame_id is   
    msg_out.point.x = dvl_inertial.r_vi_i.x();
    msg_out.point.y = dvl_inertial.r_vi_i.y();
    msg_out.point.z = dvl_inertial.r_vi_i.z();
    
    return msg_out;
}

geometry_msgs::msg::TwistStamped DvlProcessor::compose_velocity_msg(const DvlData_InertialFrame& dvl_inertial) const {
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = "pool_link"; //REVIEW, not sure what the frame_id is 
    
    msg_out.twist.linear.x = dvl_inertial.v_vi_i.x();
    msg_out.twist.linear.y = dvl_inertial.v_vi_i.y();
    msg_out.twist.linear.z = dvl_inertial.v_vi_i.z();
    
    return msg_out;
}

} 

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlProcessor>());
    rclcpp::shutdown();
    return 0;
}