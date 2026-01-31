#include "sensors/dvl_processor.hpp"

namespace sensors {


// 1. Mathematical Implementation

DvlProcessor::DvlProcessor(const Vec3& r_sd_v) 
    : r_sd_v_(r_sd_v) {}

Vec3 DvlProcessor::process(const Vec3& p_dvl_i, const quatd& q_vi) const {
    // Logic: Center = Sensor_Pos + (Rotation * Lever_Arm)
    return p_dvl_i + (q_vi * r_sd_v_);
}


// 2. ROS Node Implementation

DvlNode::DvlNode() : Node("dvl_processor") {
    // A. Initialize Variables
    current_orientation_ = quatd::Identity(); // Default to 0 rotation until IMU data arrives

    // B. Load Parameters (Vector from Sensor -> Vehicle Center)
    this->declare_parameter("dvl_offset_x", 0.0);
    this->declare_parameter("dvl_offset_y", 0.0);
    this->declare_parameter("dvl_offset_z", 0.0);

    double x = this->get_parameter("dvl_offset_x").as_double();
    double y = this->get_parameter("dvl_offset_y").as_double();
    double z = this->get_parameter("dvl_offset_z").as_double();

    // C. Initialize Math Processor
    processor_ = std::make_unique<DvlProcessor>(Vec3(x, y, z));

    // D. Setup Pubs and Subs
    // NOTE: Check your topic names ("imu/data", "dvl/position") matches your other nodes
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&DvlNode::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "dvl/position", 10, std::bind(&DvlNode::dvl_callback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation", 10);

    RCLCPP_INFO(this->get_logger(), "DvlProcessor started with Offset: [%.3f, %.3f, %.3f]", x, y, z);
}

void DvlNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Store orientation for use when DVL data arrives
    current_orientation_.w() = msg->orientation.w;
    current_orientation_.x() = msg->orientation.x;
    current_orientation_.y() = msg->orientation.y;
    current_orientation_.z() = msg->orientation.z;
}

void DvlNode::dvl_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // 1. Extract Position
    Vec3 p_dvl(msg->point.x, msg->point.y, msg->point.z);

    // 2. Process (Apply Offset)
    Vec3 p_com = processor_->process(p_dvl, current_orientation_);

    // 3. Publish Corrected Position
    geometry_msgs::msg::PointStamped output;
    output.header = msg->header; // Preserve timestamp
    output.point.x = p_com.x();
    output.point.y = p_com.y();
    output.point.z = p_com.z();

    state_pub_->publish(output);
}

} // namespace sensors



// 3.  Entry Point

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlNode>());
    rclcpp::shutdown();
    return 0;
}

