#include "sensors/depth_processor.hpp"
#include "sensors/depth_calibration.hpp"

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
    depth_calibrated_pub_ = this->create_publisher<float64_msg>("auv_frame/depth_calibrated", 10);
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
    
    q_iv_ = quatd::Identity();

    //initialize depth calibration
    load_depth_configuration();
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
    depth_processed_out.data = -r_vi_i_z;
    depth_processed_pub_->publish(depth_processed_out);

    float64_msg depth_calibrated_out;
    depth_calibrated_out.data = get_calibrated_depth(depth_processed_out.data);
    depth_calibrated_pub_->publish(depth_calibrated_out);
}; 

}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::DepthProcessor>());
	rclcpp::shutdown();

	return 0;
}