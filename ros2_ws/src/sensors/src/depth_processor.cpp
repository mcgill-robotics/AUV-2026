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

    depth_processed_pub_ = this->create_publisher<float64_msg>("auv_frame/depth", 10);
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
        calibration_service_name_,
        std::bind(&DepthProcessor::calibrate_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    q_iv_ = quatd::Identity();

    calibrate_depth_ = this->declare_parameter<bool>("calibrate_depth");
    
    if (calibrate_depth_) {
        // only time we allow defaults since this is an internal parameter
        allow_calibration_ = this->declare_parameter<bool>("allow_calibration",true);
        calibration_service_name_ = this->declare_parameter<std::string>("calibration_service_name");
        depth_offset_ = this->declare_parameter<double>("depth_offset");
        calibration_window_size_ = this->declare_parameter<int>("calibration_window_size");
    }
};

void DepthProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_in)
{ 
    quatd q_iv(imu_in->orientation.w, imu_in->orientation.x, imu_in->orientation.y, imu_in->orientation.z);
    q_iv_ = q_iv;
};

void DepthProcessor::depth_callback(const std_msgs::msg::Float64::SharedPtr depth_in) const
{
    const Vec3 r_vs_i = q_iv_ * r_vs_v_;
    const double r_vi_i_z = -depth_in->data + r_vs_i(2); // Add z-component of r_vs_i to depth measurement

    float64_msg depth_processed_out;
    double published_depth = -r_vi_i_z;
    if (calibrate_depth_) {
        if (calibration_active_) {
        }
        published_depth = get_calibrated_depth(published_depth);
    }
    depth_processed_out.data = published_depth;
    depth_processed_pub_->publish(depth_processed_out);
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

    if (!allow_calibration_) {
        response->success = false;
        response->message = "Calibration is not allowed as it has likely been called previously in this run. Set allow_calibration parameter to true to enable:\nros2 param set /depth_processor allow_calibration true.";
        return;
    }

    calibration_active_ = true;
    allow_calibration_ = false; // only allow calibration to be started once per run to prevent accidental resets of calibration

    response->success = true;
    response->message = "Calibration started. Averaging next " + std::to_string(calibration_window_size_) + " depth samples.";
    RCLCPP_INFO(this->get_logger(), "Depth calibration started.");
}

void DepthProcessor::add_depth_calibration_measurement(double depth_measurement) 
{
    calibration_sample_sum_ += depth_measurement;
    calibration_sample_count_++;
    if (calibration_sample_count_ < calibration_window_size_) {
        RCLCPP_INFO(this->get_logger(), "Adding depth measurement %.3f to calibration (sample %d of %d)", depth_measurement, calibration_sample_count_ + 1, calibration_window_size_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Depth calibration complete. Depth offset set to %.3f based on average of %d samples.", depth_offset_, calibration_sample_count_);
        depth_offset_ = calibration_sample_sum_ / calibration_sample_count_;
        calibration_active_ = false;
    }
}

double DepthProcessor::get_calibrated_depth(double uncalibrated_depth) const 
{
    return uncalibrated_depth + depth_offset_;
}
}
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::DepthProcessor>());
	rclcpp::shutdown();

	return 0;
}
