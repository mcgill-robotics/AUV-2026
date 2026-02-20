#include "sensors/dvl_processor.hpp"

namespace sensors {

DvlProcessor::DvlProcessor(const Vec3& r_dv_v) 
    : r_dv_v_(r_dv_v) {}

Vec3 DvlProcessor::process(const Vec3& r_di_i, const Quatd& q_vi) const {
    return r_di_i - (q_vi * r_dv_v_);
}

DvlNode::DvlNode() : Node("dvl_processor") {
    q_vi_.setIdentity();

    this->declare_parameter("r_dv_v.x", 0.0);
    this->declare_parameter("r_dv_v.y", 0.0);
    this->declare_parameter("r_dv_v.z", 0.0);

    double x = this->get_parameter("r_dv_v.x").as_double();
    double y = this->get_parameter("r_dv_v.y").as_double();
    double z = this->get_parameter("r_dv_v.z").as_double();

    processor_ = std::make_unique<DvlProcessor>(Vec3(x, y, z));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&DvlNode::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "dvl/position", 10, std::bind(&DvlNode::dvl_callback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation", 10);

    RCLCPP_INFO(this->get_logger(), "DvlProcessor started with Offset: [%.3f, %.3f, %.3f]", x, y, z);
}

void DvlNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_vi_.w() = msg->orientation.w;
    q_vi_.x() = msg->orientation.x;
    q_vi_.y() = msg->orientation.y;
    q_vi_.z() = msg->orientation.z;
    q_vi_.normalize();
}

void DvlNode::dvl_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    Vec3 r_di_i(msg->point.x, msg->point.y, msg->point.z);

    Vec3 r_vi_i = processor_->process(r_di_i, q_vi_);

    geometry_msgs::msg::PointStamped output;
    output.header = msg->header;
    output.point.x = r_vi_i.x();
    output.point.y = r_vi_i.y();
    output.point.z = r_vi_i.z();

    state_pub_->publish(output);
}

} // namespace sensors

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlNode>());
    rclcpp::shutdown();
    return 0;
}