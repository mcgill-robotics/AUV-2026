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
        "auv_frame/imu", 10, std::bind(&DvlProcessor::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "dvl/dead_reckoning", 10, std::bind(&DvlProcessor::dvl_callback, this, std::placeholders::_1));
    

    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/dvl/position", 10); 
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("auv_frame/dvl/velocity", 10); 
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


//Function takes new dead reckoning messages and pushes them through the data pipeline.
void DvlProcessor::dvl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    DvlData_DvlFrame dvl_raw = parse_dvl(*msg);
    
    DvlData_InertialFrame dvl_inertial = process_dvl(dvl_raw);
    
    geometry_msgs::msg::PointStamped pos_msg = compose_position_msg(dvl_inertial);
    geometry_msgs::msg::TwistStamped vel_msg = compose_velocity_msg(dvl_inertial);

    position_pub_->publish(pos_msg);
    velocity_pub_->publish(vel_msg);
}

DvlData_DvlFrame DvlProcessor::parse_dvl(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const {
    DvlData_DvlFrame dvl_raw;

    dvl_raw.r_di2_i2 = Vec3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    dvl_raw.v_di2_d = Vec3(0.0, 0.0, 0.0);
    
    return dvl_raw; 
}

DvlData_InertialFrame DvlProcessor::process_dvl(const DvlData_DvlFrame& dvl_raw) const {
    DvlData_InertialFrame dvl_inertial;
    Vec3 r_i2p_p = r_dv_v_;
    Vec3 r_di2_p = q_pi2_ * dvl_raw.r_di2_i2;
    Vec3 r_vd_v = -r_dv_v_;
    Vec3 r_vd_p = q_iv_ * r_vd_v;


    dvl_inertial.r_vp_p = r_i2p_p + r_di2_p + r_vd_p;
    
    // Velocity transformation
    Quatd q_id = q_iv_ * q_vd_;
    Vec3 v_di_p = q_id * dvl_raw.v_di2_d;
    
    dvl_inertial.v_vp_p = v_di_p - q_iv_ * (w_v_.cross(r_dv_v_));
    return dvl_inertial;
}

geometry_msgs::msg::PointStamped DvlProcessor::compose_position_msg(const DvlData_InertialFrame& dvl_inertial) const {
    geometry_msgs::msg::PointStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;
    msg_out.point.x = dvl_inertial.r_vp_p.x();
    msg_out.point.y = dvl_inertial.r_vp_p.y();
    msg_out.point.z = dvl_inertial.r_vp_p.z();
    
    return msg_out;
}

geometry_msgs::msg::TwistStamped DvlProcessor::compose_velocity_msg(const DvlData_InertialFrame& dvl_inertial) const {
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;    
    msg_out.twist.linear.x = dvl_inertial.v_vp_p.x();
    msg_out.twist.linear.y = dvl_inertial.v_vp_p.y();
    msg_out.twist.linear.z = dvl_inertial.v_vp_p.z();
    
    return msg_out;
}

} 

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlProcessor>());
    rclcpp::shutdown();
    return 0;
}