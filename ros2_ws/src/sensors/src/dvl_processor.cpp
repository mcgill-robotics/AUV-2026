#include "sensors/dvl_processor.hpp"
#include "sensors/utils.hpp"

namespace sensors {

/// Creates the DVL processor, loads frame and mounting parameters, and sets up
/// subscriptions and publishers for DVL, IMU, position, velocity, and odometry.
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
    this->declare_parameter<std::string>("frame_id_auv", "auv_link");
    this->get_parameter("frame_id_auv", frame_id_auv_);


    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::imu_callback, this, std::placeholders::_1));

    dvl_pos_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/dead_reckoning", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_position_callback, this, std::placeholders::_1));
    dvl_vel_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
        "dvl/velocity", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_velocity_callback, this, std::placeholders::_1));
    

    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/dvl/position", rclcpp::SensorDataQoS().keep_last(1)); 
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("auv_frame/dvl/velocity", rclcpp::SensorDataQoS().keep_last(1)); 
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("state/odometry", rclcpp::SensorDataQoS().keep_last(1));
    
    orientation_covariance_ = Eigen::Matrix3d::Zero();
    angular_velocity_covariance_ = Eigen::Matrix3d::Zero();
    velocity_covariance_ = Eigen::Matrix3d::Zero();
    position_covariance_ = Eigen::Matrix3d::Zero();
}


//Function updates stored vehicle orientation whenever the IMU publishes new data.
/// Caches the latest vehicle orientation, angular rate, and IMU covariances.
///
/// Args:
///   msg: IMU data expressed in the vehicle frame, including orientation
///     relative to the pool frame.
void DvlProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_iv_.w() = msg->orientation.w;
    q_iv_.x() = msg->orientation.x;
    q_iv_.y() = msg->orientation.y;
    q_iv_.z() = msg->orientation.z;

    w_v_.x() = msg->angular_velocity.x;
    w_v_.y() = msg->angular_velocity.y;
    w_v_.z() = msg->angular_velocity.z;

    orientation_covariance_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->orientation_covariance.data());
    angular_velocity_covariance_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->angular_velocity_covariance.data());
}


//Function takes new odometry messages and pushes them through the data pipeline.
/// Converts DVL-frame velocity and covariance to pool-frame values and publishes
/// the velocity; the transformed covariance is also stored for odometry output.
///
/// Args:
///   msg: DVL velocity and its 3x3 covariance, expressed in DVL axes.
void DvlProcessor::dvl_velocity_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {
    Vec3 v_di2_d = Vec3(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    //Convert from row-major array to Eigen::Matrix3d
    Eigen::Matrix3d covariance = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->covariance.data());

    std::pair<Vec3, Eigen::Matrix3d> result = process_dvl_velocity(v_di2_d, covariance);
    velocity_covariance_ = result.second;
    current_velocity_p_ = result.first;
    velocity_received_ = true;
    publish_velocity_msg(result.first);

    if (position_received_) {
        publish_odometry_msg(current_position_p_, current_velocity_p_);
    }
}

/// Converts DVL dead-reckoning position to the vehicle-origin position and
/// updates the diagonal position covariance from the driver's standard deviation.
///
/// Args:
///   msg: DVL dead-reckoning position and scalar position standard deviation.
void DvlProcessor::dvl_position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg) {
    Vec3 r_di2_i2 = Vec3(msg->position.x, msg->position.y, msg->position.z);

    Vec3 r_vp_p = process_dvl_position(r_di2_i2);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = msg->pos_std * msg->pos_std;
    covariance(1, 1) = msg->pos_std * msg->pos_std;

    Eigen::Matrix3d rotation_matrix = q_pi2_.toRotationMatrix();
    Eigen::Matrix3d rotated_covariance = rotation_matrix * covariance * rotation_matrix.transpose();
    position_covariance_ = rotated_covariance;
    current_position_p_ = r_vp_p;
    position_received_ = true;
    //Don't set z, since depth is better from the pressure sensor
    publish_position_msg(r_vp_p);

    if (velocity_received_) {
        publish_odometry_msg(current_position_p_, current_velocity_p_);
    }
}


/// Transforms DVL dead-reckoning position into a pool-frame vehicle position.
///
/// Args:
///   r_di2_i2: DVL displacement from its startup origin, expressed in the
///     DVL's startup frame.
///
/// Returns:
///   Vehicle-origin position expressed in pool axes.
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


/// Transforms DVL velocity and covariance into pool axes and removes the
/// rotational velocity at the DVL mounting point to estimate vehicle velocity.
///
/// Args:
///   v_di2_d: Measured DVL velocity expressed in DVL axes.
///   covariance: 3x3 DVL velocity covariance, expressed in DVL axes.
///
/// Returns:
///   Pair containing pool-frame vehicle velocity and its pool-frame covariance.
std::pair<Vec3, Eigen::Matrix3d> DvlProcessor::process_dvl_velocity(const Vec3& v_di2_d, const Eigen::Matrix3d& covariance) const {  
    Quatd q_id = q_iv_ * q_vd_;
    Vec3 v_di_p = q_id * v_di2_d;
    Vec3 v_vp_p = v_di_p - q_iv_ * (w_v_.cross(r_dv_v_));

    const Eigen::Matrix3d imu_rotation = q_id.toRotationMatrix();
    const Eigen::Matrix3d transformed_covariance = imu_rotation * covariance * imu_rotation.transpose();

    return std::make_pair(v_vp_p, transformed_covariance);
}

/// Publishes a pool-frame vehicle position as a stamped point.
///
/// Args:
///   r_vp_p: Vehicle-origin position expressed in pool axes.
void DvlProcessor::publish_position_msg(const Vec3& r_vp_p) const {
    geometry_msgs::msg::PointStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;
    msg_out.point.x = r_vp_p.x();
    msg_out.point.y = r_vp_p.y();
    msg_out.point.z = r_vp_p.z();
    
    position_pub_->publish(msg_out);
}

/// Publishes a pool-frame vehicle linear velocity as a stamped twist.
///
/// Args:
///   v_vp_p: Vehicle-origin linear velocity expressed in pool axes.
void DvlProcessor::publish_velocity_msg(const Vec3& v_vp_p) const {
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;    
    msg_out.twist.linear.x = v_vp_p.x();
    msg_out.twist.linear.y = v_vp_p.y();
    msg_out.twist.linear.z = v_vp_p.z();

    velocity_pub_->publish(msg_out);
}

/// Publishes vehicle position in the pool frame and twist values in the AUV
/// frame, as required by nav_msgs/Odometry.
///
/// Args:
///   r_vp_p: Vehicle-origin position expressed in pool axes.
///   v_vp_p: Vehicle-origin linear velocity expressed in pool axes.
void DvlProcessor::publish_odometry_msg(const Vec3& r_vp_p, const Vec3& v_vp_p) const {
    nav_msgs::msg::Odometry msg_out;
    msg_out.header.stamp = this->now();
    msg_out.header.frame_id = frame_id_global_;
    msg_out.child_frame_id = frame_id_auv_;
    msg_out.pose.pose.position.x = r_vp_p.x();
    msg_out.pose.pose.position.y = r_vp_p.y();
    msg_out.pose.pose.position.z = r_vp_p.z();
    msg_out.pose.pose.orientation.w = q_iv_.w();
    msg_out.pose.pose.orientation.x = q_iv_.x();
    msg_out.pose.pose.orientation.y = q_iv_.y();
    msg_out.pose.pose.orientation.z = q_iv_.z();

    const Vec3 v_vp_v = q_iv_.inverse() * v_vp_p;
    msg_out.twist.twist.linear.x = v_vp_v.x();
    msg_out.twist.twist.linear.y = v_vp_v.y();
    msg_out.twist.twist.linear.z = v_vp_v.z();

    Eigen::Matrix<double, 6, 6> twist_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix3d vehicle_rotation = q_iv_.inverse().toRotationMatrix();
    twist_covariance.block<3, 3>(0, 0) = vehicle_rotation * velocity_covariance_ * vehicle_rotation.transpose();
    twist_covariance.block<3, 3>(3, 3) = angular_velocity_covariance_;

    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            msg_out.twist.covariance[row * 6 + column] = twist_covariance(row, column);
        }
    }

    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    pose_covariance.block<3, 3>(0, 0) = position_covariance_; // Set the position covariance
    pose_covariance.block<3, 3>(3, 3) = orientation_covariance_;

    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            msg_out.pose.covariance[row * 6 + column] = pose_covariance(row, column);
        }
    }

    // Publish the odometry message
    // Assuming you have an odometry publisher defined as odometry_pub_
    odometry_pub_->publish(msg_out);

} 

}

/// Initializes ROS, runs the DVL processor, and shuts ROS down on exit.
///
/// Returns:
///   Zero after the node exits normally.
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlProcessor>());
    rclcpp::shutdown();
    return 0;
}
