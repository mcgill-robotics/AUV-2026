#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class PidControllerNode : public rclcpp::Node {
public:
  PidControllerNode();
  ~PidControllerNode() override;

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void dvl_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_loop();

  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
  void declare_pid_params(const std::string& prefix, double p, double i, double d, double i_max, double max_out, double i_zone);
  void update_pid_params(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_dvl_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_setpoint_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_effort_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  bool has_pose_ = false;
  bool has_dvl_ = false;
  bool has_imu_ = false;
  bool has_target_ = false;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_dvl_;
  sensor_msgs::msg::Imu current_imu_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped last_target_pose_; // To detect setpoint changes

  struct PIDGains {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double i_max = 0.0;
    double max_out = 0.0; 
    double i_zone = 0.0; // Integrate only when abs(error) < i_zone. 0 means always integrate.
  };
  struct VectorPIDGains {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d i = Eigen::Vector3d::Zero();
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    Eigen::Vector3d i_max = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_out = Eigen::Vector3d::Zero();
    Eigen::Vector3d i_zone = Eigen::Vector3d::Zero();
  };
  
  VectorPIDGains pos_pid, rot_pid, vel_pid, rate_pid;

  // Integrators
  Eigen::Vector3d int_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d int_rate = Eigen::Vector3d::Zero();

  // Last Errors/Measurements for Derivative
  Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_rate = Eigen::Vector3d::Zero();
  
  // Stored D-terms (so they hold value if sensor hasn't updated)
  Eigen::Vector3d d_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_rate = Eigen::Vector3d::Zero();

  rclcpp::Time last_time_;
  rclcpp::Time last_dvl_time_;
  rclcpp::Time last_imu_time_;
  
  double clamp(double value, double max_val) {
    if (value > max_val) return max_val;
    if (value < -max_val) return -max_val;
    return value;
  }
};

#endif
