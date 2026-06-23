#include "sensors/dvl_processor.hpp"
#include "sensors/utils.hpp"

namespace sensors {

DvlProcessor::DvlProcessor() : Node("dvl_processor") {
    q_iv_.setIdentity();

    this->declare_parameter<std::vector<double>>("r_dv_v", {0.0, 0.0, 0.0});
    std::vector<double> r_dv_v_vec;
    this->get_parameter("r_dv_v", r_dv_v_vec);
    r_dv_v_ = Vec3(r_dv_v_vec[0], r_dv_v_vec[1], r_dv_v_vec[2]);

    //quarternion rotation from dvl inertial frame to pool inertial frame
    //DVL posts data with +X forward, +Y right, and +Z downward. 
    //Pool inertial uses +X forward, +Y left, and +Z up.
    this->declare_parameter<std::vector<double>>("q_pi2", {1.0, 0.0, 0.0, 0.0});
    std::vector<double> q_pi2_vec;
    this->get_parameter("q_pi2", q_pi2_vec);
    q_pi2_ = sensors::math::quatFromParamWxyz(q_pi2_vec);

    this->declare_parameter<std::vector<double>>("q_vd", {1.0, 0.0, 0.0, 0.0});
    std::vector<double> q_vd_vec;
    this->get_parameter("q_vd", q_vd_vec);
    q_vd_ = sensors::math::quatFromParamWxyz(q_vd_vec);


    this->declare_parameter<std::string>("frame_id_global", "pool_link");
    this->get_parameter("frame_id_global", frame_id_global_);


    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::imu_callback, this, std::placeholders::_1));

    dvl_pos_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/dead_reckoning", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_position_callback, this, std::placeholders::_1));
    dvl_vel_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
        "dvl/velocity", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_velocity_callback, this, std::placeholders::_1));
    

    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/dvl/position", rclcpp::SensorDataQoS().keep_last(1)); 
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("auv_frame/dvl/velocity", rclcpp::SensorDataQoS().keep_last(1)); 
}


//Function updates stored vehicle orientation whenever the IMU publishes new data.
void DvlProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_iv_.w() = msg->orientation.w;
    q_iv_.x() = msg->orientation.x;
    q_iv_.y() = msg->orientation.y;
    q_iv_.z() = msg->orientation.z;

    w_v_.x() = msg->angular_velocity.x;
    w_v_.y() = msg->angular_velocity.y;
    w_v_.z() = msg->angular_velocity.z;
}


//Function takes new odometry messages and pushes them through the data pipeline.
void DvlProcessor::dvl_velocity_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {
    Vec3 v_di2_d = Vec3(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    Vec3 v_vp_p = process_dvl_velocity(v_di2_d);
    
    publish_velocity_msg(v_vp_p);
}

void DvlProcessor::dvl_position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg) {
    Vec3 r_di2_i2 = Vec3(msg->position.x, msg->position.y, msg->position.z);

    Vec3 r_vp_p = process_dvl_position(r_di2_i2);
    
    publish_position_msg(r_vp_p);
}


Vec3 DvlProcessor::process_dvl_position(const Vec3& r_di2_i2) const 
{
    DvlData_InertialFrame dvl_inertial;
    Vec3 r_i2p_p = r_dv_v_;
    Vec3 r_di2_p = q_pi2_ * r_di2_i2;
    Vec3 r_vd_v = -r_dv_v_;
    Vec3 r_vd_p = q_iv_ * r_vd_v;

    Vec3 r_vp_p = r_i2p_p + r_di2_p + r_vd_p;
    return r_vp_p;
}

Vec3 DvlProcessor::process_dvl_velocity(const Vec3& v_di2_d) const {  
    Quatd q_id = q_iv_ * q_vd_;
    Vec3 v_di_p = q_id * v_di2_d;
    Vec3 v_vp_p = v_di_p - q_iv_ * (w_v_.cross(r_dv_v_));

    return v_vp_p;
}

void DvlProcessor::publish_position_msg(const Vec3& r_vp_p) const {
    geometry_msgs::msg::PointStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;
    msg_out.point.x = r_vp_p.x();
    msg_out.point.y = r_vp_p.y();
    msg_out.point.z = r_vp_p.z();
    
    position_pub_->publish(msg_out);
}

void DvlProcessor::publish_velocity_msg(const Vec3& v_vp_p) const {
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;    
    msg_out.twist.linear.x = v_vp_p.x();
    msg_out.twist.linear.y = v_vp_p.y();
    msg_out.twist.linear.z = v_vp_p.z();
    
    velocity_pub_->publish(msg_out);
}

} 

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlProcessor>());
    rclcpp::shutdown();
    return 0;
}