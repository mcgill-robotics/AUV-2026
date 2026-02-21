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
        10
    );
    imu_sub_ = this->create_subscription<imu_msg>(
        "imu/data",
        10,
        std::bind(&ImuProcessor::imu_callback, this, std::placeholders::_1)
    );

    this->declare_parameter<std::vector<double>>("q_vs", {1, 0.0, 0.0, 0.0}); // Default no rotation. 
    this->declare_parameter<std::vector<double>>("q_in", {1.0, 0.0, 0.0, 0.0}); // Default: no rotation
    this->declare_parameter<std::string>("frame_id_auv", "auv_link");
    this->declare_parameter<std::string>("frame_id_global", "pool_link");

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
    imu_msg imu_msg_out = compose_imu_msg(imu_auv);

    // Publish processed message
    imu_pub_->publish(imu_msg_out);
}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::ImuProcessor>());
	rclcpp::shutdown();

	return 0;
}