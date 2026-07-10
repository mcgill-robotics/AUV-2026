#include "sensors/Imu_processor.hpp"
#include "sensors/utils.hpp"


// See ../README.md for explanation of what is being done by ImuProcessor
// and variable naming conventions. 

namespace sensors
{
ImuProcessor::ImuProcessor()
    : Node("imu_processor"), q_iv_(quatd::Identity())
{

    imu_pub_ = this->create_publisher<imu_msg>(
        "auv_frame/imu",
        rclcpp::SensorDataQoS().keep_last(1)
    );
    this->declare_parameter<std::string>("topic_in", "imu/data");
    this->declare_parameter<std::vector<double>>("q_vs", {1, 0.0, 0.0, 0.0}); // Default no rotation. 
    this->declare_parameter<std::vector<double>>("q_in", {1.0, 0.0, 0.0, 0.0}); // Default: no rotation
    this->declare_parameter<std::string>("frame_id_auv", "auv_link");
    this->declare_parameter<std::string>("frame_id_global", "pool_link");
    this->declare_parameter<bool>("use_dvl_yaw_correction", true);
    this->declare_parameter<double>("dvl_yaw_interpolation_rate", 0.01);

    std::string topic_in;
    this->get_parameter("topic_in", topic_in);
    this->get_parameter("use_dvl_yaw_correction", use_dvl_yaw_correction_);
    this->get_parameter("dvl_yaw_interpolation_rate", dvl_yaw_interpolation_rate_);

    imu_sub_ = this->create_subscription<imu_msg>(
        topic_in,
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ImuProcessor::imu_callback, this, std::placeholders::_1)
    );

    if (use_dvl_yaw_correction_) {
        dvl_odom_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
            "dvl/dead_reckoning",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&ImuProcessor::dvl_odom_callback, this, std::placeholders::_1)
        );
    }

    std::vector<double> q_vs_vec;
    std::vector<double> q_in_vec;
    this->get_parameter("q_vs", q_vs_vec);
    this->get_parameter("q_in", q_in_vec);
    this->get_parameter("frame_id_auv", frame_id_auv_);
    this->get_parameter("frame_id_global", frame_id_global_);

    q_vs_ = sensors::math::quatFromParamWxyz(q_vs_vec);
    q_in_ = sensors::math::quatFromParamWxyz(q_in_vec);

    // Gravity vector in inertial frame (down is negative z)
    g_i << 0.0, 0.0, -9.81;

    q_sv_ = q_vs_.inverse();
}

Vec3 ImuProcessor::compute_free_acc(const Vec3& specific_force, const quatd& q_si) const
{
    // Rotate gravity into sensor frame
    Vec3 g_s = q_si * g_i; // This is an overload of the quaternion operator* for vectors.

    // Free acceleration 
    Vec3 a_free = q_vs_ * (specific_force + g_s);
    return a_free;
}

Vec3 ImuProcessor::rotate_gyro(const Vec3& w_s) const
{
    // Rotate gyro measurements into vehicle frame
    Vec3 w_v = q_vs_ * w_s;
    return w_v;
}

ImuDataRawFrame ImuProcessor::parse_imu(const imu_msg& imu_in_msg) const
{
    ImuDataRawFrame imu_raw;
    imu_raw.f_s << imu_in_msg.linear_acceleration.x, imu_in_msg.linear_acceleration.y, imu_in_msg.linear_acceleration.z;
    imu_raw.w_s << imu_in_msg.angular_velocity.x, imu_in_msg.angular_velocity.y, imu_in_msg.angular_velocity.z;
    imu_raw.q_is = quatd(imu_in_msg.orientation.w, imu_in_msg.orientation.x, imu_in_msg.orientation.y, imu_in_msg.orientation.z);
    return imu_raw;
}

ImuDataAUVFrame ImuProcessor::process_imu(const ImuDataRawFrame& imu_raw) const
{
    ImuDataAUVFrame imu_auv;
    imu_auv.a_free_v = compute_free_acc(imu_raw.f_s, imu_raw.q_is.inverse());
    imu_auv.w_v = rotate_gyro(imu_raw.w_s);
    imu_auv.q_iv = imu_raw.q_is * q_sv_;
    imu_auv.q_iv = sensors::math::canonicalizeShortest(imu_auv.q_iv);
    return imu_auv;
}

imu_msg ImuProcessor::compose_imu_msg(const ImuDataAUVFrame& imu_auv) const
{
    imu_msg imu_out_msg;

    imu_out_msg.header.stamp = this->now();
    imu_out_msg.header.frame_id = frame_id_auv_;

    imu_out_msg.linear_acceleration.x = imu_auv.a_free_v.x();
    imu_out_msg.linear_acceleration.y = imu_auv.a_free_v.y();
    imu_out_msg.linear_acceleration.z = imu_auv.a_free_v.z();

    imu_out_msg.angular_velocity.x = imu_auv.w_v.x();
    imu_out_msg.angular_velocity.y = imu_auv.w_v.y();
    imu_out_msg.angular_velocity.z = imu_auv.w_v.z();

    imu_out_msg.orientation.w = imu_auv.q_iv.w();
    imu_out_msg.orientation.x = imu_auv.q_iv.x();
    imu_out_msg.orientation.y = imu_auv.q_iv.y();
    imu_out_msg.orientation.z = imu_auv.q_iv.z();

    return imu_out_msg;
}

void ImuProcessor::imu_callback(const imu_msg::SharedPtr imu_in) 
{
    imu_msg imu_in_msg = *imu_in;
    ImuDataRawFrame imu_raw = parse_imu(imu_in_msg);
    ImuDataAUVFrame imu_auv = process_imu(imu_raw);

    if (use_dvl_yaw_correction_) {
        // Smoothly interpolate current yaw offset towards target offset (Complementary filter)
        // Runs at IMU frequency (~100Hz)
        double error = sensors::math::normalizeAngle(target_yaw_offset_ - current_yaw_offset_);
        current_yaw_offset_ += dvl_yaw_interpolation_rate_ * error;
        current_yaw_offset_ = sensors::math::normalizeAngle(current_yaw_offset_);

        // Apply the drift correction offset to the final orientation
        quatd q_correction(Eigen::AngleAxisd(current_yaw_offset_, Vec3::UnitZ()));
        imu_auv.q_iv = sensors::math::canonicalizeShortest(q_correction * imu_auv.q_iv);
    }

    // Store the final orientation so the DVL callback can compute the offset
    q_iv_ = imu_auv.q_iv;

    imu_msg imu_msg_out = compose_imu_msg(imu_auv);

    // Publish processed message
    imu_pub_->publish(imu_msg_out);
}

void ImuProcessor::dvl_odom_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg)
{
    // Extract absolute true yaw from DVL dead reckoning (provided in degrees)
    // The DVL native frame is FRD (Z-down), so positive yaw is clockwise.
    // The AUV IMU frame is FLU (Z-up), so positive yaw is counter-clockwise.
    // We invert the DVL yaw to match the FLU convention.
    double yaw_dvl_frd = msg->yaw * M_PI / 180.0;
    double yaw_dvl = sensors::math::normalizeAngle(-yaw_dvl_frd);

    // Compute raw yaw from the IMU before offset was applied
    double yaw_corrected = sensors::math::yawFromQuat(q_iv_);
    double yaw_raw = yaw_corrected - current_yaw_offset_;

    // Set the new target offset for the IMU callback to smoothly track
    target_yaw_offset_ = sensors::math::normalizeAngle(yaw_dvl - yaw_raw);
}

}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::ImuProcessor>());
	rclcpp::shutdown();

	return 0;
}