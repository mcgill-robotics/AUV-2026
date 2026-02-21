#include "sensors/depth_processor.hpp"

// See ../README.md for explanation of what is being done by DepthProcessor
// and variable naming conventions. 

namespace sensors
{
DepthProcessor::DepthProcessor()
    : rclcpp::Node("depth_processor")
{
    this->declare_parameter<std::vector<double>>("r_vs_v", {0.0624, 0.0585, 0.0962}); // [m] Default: from CAD. 
    std::vector<double> r_vs_v_vec;
    this->get_parameter("r_vs_v", r_vs_v_vec);
    r_vs_v_ = Vec3(r_vs_v_vec[0], r_vs_v_vec[1], r_vs_v_vec[2]);

    depth_pub_ = this->create_publisher<float64_msg>("auv_frame/depth", 10);
    depth_sub_ = this->create_subscription<float64_msg>(
            "/sensors/depth/z",
            10,
            std::bind(&DepthProcessor::depth_callback, this, std::placeholders::_1)
        );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu",
        10,
        std::bind(&DepthProcessor::imu_callback, this, std::placeholders::_1)
    );

    calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/depth_processor/calibrate",
        std::bind(&DepthProcessor::calibrate_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    q_iv_ = quatd::Identity();
    zero_offset_ = 0.0;
    calibration_active_ = false;
    calibration_sample_count_ = 0;
    calibration_sample_sum_ = 0.0;

};

void DepthProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_in)
{ 
    quatd q_iv(imu_in->orientation.w, imu_in->orientation.x, imu_in->orientation.y, imu_in->orientation.z);
    q_iv_ = q_iv;
};




void DepthProcessor::depth_callback(const std_msgs::msg::Float64::SharedPtr depth_in)
{
    const Vec3 r_vs_i = q_iv_ * r_vs_v_;
    const double r_vi_i_z = -depth_in->data + r_vs_i(2); // Add z-component of r_vs_i to depth measurement
    const double uncalibrated_depth = -r_vi_i_z;

    if (calibration_active_) {
        calibration_sample_sum_ += uncalibrated_depth;
        calibration_sample_count_ += 1;

        if (calibration_sample_count_ >= calibration_window_size_) {
            zero_offset_ = calibration_sample_sum_ / static_cast<double>(calibration_window_size_);
            calibration_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Depth calibration complete. zero_offset=%.6f", zero_offset_);
        }
    }

    float64_msg depth_out ;
    depth_out.data = uncalibrated_depth - zero_offset_;

    depth_pub_->publish(depth_out);
}; 

void DepthProcessor::calibrate_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (calibration_active_) {
        response->success = false;
        response->message = "Calibration already in progress.";
        return;
    }

    calibration_active_ = true;
    calibration_sample_count_ = 0;
    calibration_sample_sum_ = 0.0;

    response->success = true;
    response->message = "Calibration started. Averaging next " + std::to_string(calibration_window_size_) + " depth samples.";
    RCLCPP_INFO(this->get_logger(), "Depth calibration started.");
}

}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::DepthProcessor>());
	rclcpp::shutdown();

	return 0;
}
