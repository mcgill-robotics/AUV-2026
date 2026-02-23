#include "sensors/dvl_processor.hpp"

namespace sensors {



DvlProcessor::DvlProcessor(const Vec3& r_dv_v) 
    : r_dv_v_(r_dv_v) {}

Vec3 DvlProcessor::process(const Vec3& r_di_i, const Quatd& q_iv) const {
    return r_di_i - (q_iv * r_dv_v_);
}



DvlNode::DvlNode() : Node("dvl_processor") {
    q_iv_.setIdentity();
// gets parameters from yaml file using ros2 node function declare_parameter, if yaml file is empty then it sets r_dv_v to 0,0,0
    this->declare_parameter<std::vector<double>>("r_dv_v", {0.0, 0.0, 0.0});

// creates a vector r_dv_v_vec and then sets it equal to r_dv_v
    std::vector<double> r_dv_v_vec;
    this->get_parameter("r_dv_v", r_dv_v_vec);

    processor_ = std::make_unique<DvlProcessor>(Vec3(r_dv_v_vec[0], r_dv_v_vec[1], r_dv_v_vec[2]));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu", 10, std::bind(&DvlNode::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "dvl/dead_reckoning", 10, std::bind(&DvlNode::dvl_callback, this, std::placeholders::_1));
    
    state_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/position", 10); 
}



void DvlNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_iv_.w() = msg->orientation.w;
    q_iv_.x() = msg->orientation.x;
    q_iv_.y() = msg->orientation.y;
    q_iv_.z() = msg->orientation.z;
}


void DvlNode::dvl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    Vec3 r_di_i(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    Vec3 r_vi_i = processor_->process(r_di_i, q_iv_);

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