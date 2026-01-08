#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"
#include "sensors/Sensor_Node.hpp"

// Main Node that republish processed IMU and Depth data
namespace sensors
{
    Sensor_Node::Sensor_Node()
        : Node("sensor_node")
    {
        this->declare_parameter<std::vector<double>>("q_sv", {1, 0.0, 0.0, 0.0}); // Default no rotation. TODO: Verify and change in MEDN. 
        this->declare_parameter<std::vector<double>>("q_in", {1.0, 0.0, 0.0, 0.0}); // Default: no rotation
        this->declare_parameter<std::vector<double>>("r_vs_v", {0.0624, 0.0585, 0.0962}); // [m] Default: from CAD. 

        std::vector<double> q_sv_vec;
        std::vector<double> q_in_vec;
        std::vector<double> r_vs_v_vec;

        this->get_parameter("q_sv", q_sv_vec);
        this->get_parameter("q_in", q_in_vec);
        this->get_parameter("r_vs_v", r_vs_v_vec);

        q_sv_ = quatd(q_sv_vec[0], q_sv_vec[1], q_sv_vec[2], q_sv_vec[3]);
        q_in_ = quatd(q_in_vec[0], q_in_vec[1], q_in_vec[2], q_in_vec[3]);
        r_vs_v_ = Vec3(r_vs_v_vec[0], r_vs_v_vec[1], r_vs_v_vec[2]);
        
        imu_repub_ = std::make_unique<ImuRepublisher>(q_sv_, q_in_);
        depth_repub_ = std::make_unique<DepthRepublisher>(r_vs_v_);

        imu_sub_ = this->create_subscription<imu_msg>(
            "imu/data",
            10,
            std::bind(&Sensor_Node::imu_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<float64_msg>(
            "/sensors/depth/z",
            10,
            std::bind(&Sensor_Node::depth_callback, this, std::placeholders::_1)
        );


        imu_pub_ = this->create_publisher<imu_msg>("processed/imu", 10);
        depth_pub_ = this->create_publisher<float64_msg>("processed/depth", 10);


    }

    // IMU callback
    void Sensor_Node::imu_callback(const imu_msg::SharedPtr imu_in)
    {
        imu_msg imu_out = imu_repub_->process(*imu_in);
        imu_pub_->publish(imu_out);
    }

    // Depth callback
    void Sensor_Node::depth_callback(const float64_msg::SharedPtr depth_in)
    {
      // Need current orientation to process depth
      // For simplicity, assume last published IMU message has the most recent orientation
        float64_msg depth_out = depth_repub_->process(*depth_in, imu_repub_->q_vi_); // Pass current vehicle orientation from IMU processor
        depth_pub_->publish(depth_out);
    }
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::Sensor_Node>());
	rclcpp::shutdown();

	return 0;
}
	