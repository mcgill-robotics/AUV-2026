#include "douglas_control/pid_controller_node.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

PidControllerNode::PidControllerNode() : Node("pid_controller_node")
{
  // PID Parameter Columns: (prefix, p_gain, i_gain, d_gain, i_max_windup, max_output_limit, i_zone_threshold)
  declare_pid_params("pos_x", 1.0, 0.0, 0.0, 0.0, 0.6, 0.0); // max 0.8 m/s
  declare_pid_params("pos_y", 1.0, 0.0, 0.0, 0.0, 0.6, 0.0); // max 0.5 m/s
  declare_pid_params("pos_z", 2.0, 0.0, 0.0, 0.0, 0.5, 0.0); // max 0.3 m/s
  declare_pid_params("rot_r", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0); // Reduced P to soften angular velocity commands
  declare_pid_params("rot_p", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  declare_pid_params("rot_y", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  // --- Inner Loop Gains (Velocity -> Wrench) ---
  // Adding D-terms for damping as requested
  declare_pid_params("vel_x", 40.0, 10.0, 0.0, 20.0, 50.0, 0.5); // Integrate only when position error < 0.5 m
  declare_pid_params("vel_y", 40.0, 10.0, 0.0, 20.0, 50.0, 0.5);
  declare_pid_params("vel_z", 50.0, 10.0, 0.0, 30.0, 50.0, 0.25);
  declare_pid_params("rate_r", 8.0, 1.0, 0.0, 5.0, 20.0, 0.5); // Reduced P and I, increased D for smoother damping
  declare_pid_params("rate_p", 8.0, 1.0, 0.0, 5.0, 20.0, 0.5);
  declare_pid_params("rate_y", 10.0, 2.0, 0.0, 10.0, 20.0, 0.5);
  
  this->declare_parameter("net_buoyancy", 5.25); // Buoyancy_N - Weight_N
  this->declare_parameter("ff_scale", 0.5); // Feedforward scale factor

  // Initial populate
  std::vector<rclcpp::Parameter> initial_params;
  update_pid_params(initial_params);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PidControllerNode::parameters_callback, this, _1));

  std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "/state/pose");
  std::string dvl_topic = this->declare_parameter<std::string>("dvl_topic", "/auv_frame/dvl/velocity");
  std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/auv_frame/imu");
  std::string setpoint_topic = this->declare_parameter<std::string>("setpoint_topic", "/controls/setpoint");
  std::string forces_topic = this->declare_parameter<std::string>("forces_topic", "/controls/total_effort");

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&PidControllerNode::pose_callback, this, _1));
  sub_dvl_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    dvl_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&PidControllerNode::dvl_callback, this, _1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&PidControllerNode::imu_callback, this, _1));
  sub_setpoint_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    setpoint_topic, rclcpp::SystemDefaultsQoS(), std::bind(&PidControllerNode::setpoint_callback, this, _1));

  pub_effort_ = this->create_publisher<geometry_msgs::msg::Wrench>(forces_topic, 10);

  last_time_ = this->now();
  last_dvl_time_ = this->now();
  last_imu_time_ = this->now();
  timer_ = this->create_wall_timer(20ms, std::bind(&PidControllerNode::control_loop, this));
}

PidControllerNode::~PidControllerNode() {}

void PidControllerNode::declare_pid_params(const std::string& prefix, double p, double i, double d, double i_max, double max_out, double i_zone) {
  this->declare_parameter(prefix + "_p", p);
  this->declare_parameter(prefix + "_i", i);
  this->declare_parameter(prefix + "_d", d);
  this->declare_parameter(prefix + "_i_max", i_max);
  this->declare_parameter(prefix + "_max_out", max_out);
  this->declare_parameter(prefix + "_i_zone", i_zone);
}

void PidControllerNode::update_pid_params(const std::vector<rclcpp::Parameter>& /*parameters*/) {
  auto get_gains = [this](const std::string& prefix) -> PIDGains {
    PIDGains g;
    this->get_parameter(prefix + "_p", g.p);
    this->get_parameter(prefix + "_i", g.i);
    this->get_parameter(prefix + "_d", g.d);
    this->get_parameter(prefix + "_i_max", g.i_max);
    this->get_parameter(prefix + "_max_out", g.max_out);
    this->get_parameter(prefix + "_i_zone", g.i_zone);
    return g;
  };
  auto fill_vec = [](const PIDGains& gx, const PIDGains& gy, const PIDGains& gz, VectorPIDGains& v) {
      v.p << gx.p, gy.p, gz.p;
      v.i << gx.i, gy.i, gz.i;
      v.d << gx.d, gy.d, gz.d;
      v.i_max << gx.i_max, gy.i_max, gz.i_max;
      v.max_out << gx.max_out, gy.max_out, gz.max_out;
      v.i_zone << gx.i_zone, gy.i_zone, gz.i_zone;
  };
  fill_vec(get_gains("pos_x"), get_gains("pos_y"), get_gains("pos_z"), pos_pid);
  fill_vec(get_gains("rot_r"), get_gains("rot_p"), get_gains("rot_y"), rot_pid);
  fill_vec(get_gains("vel_x"), get_gains("vel_y"), get_gains("vel_z"), vel_pid);
  fill_vec(get_gains("rate_r"), get_gains("rate_p"), get_gains("rate_y"), rate_pid);
}

rcl_interfaces::msg::SetParametersResult PidControllerNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  update_pid_params(parameters);
  return result;
}

void PidControllerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = *msg;
  has_pose_ = true;
}

void PidControllerNode::dvl_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  current_dvl_ = *msg;
  has_dvl_ = true;
}

void PidControllerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  current_imu_ = *msg;
  has_imu_ = true;
}

void PidControllerNode::setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // If target changed significantly, reset integral
  if (has_target_) {
      double dist = std::hypot(msg->pose.position.x - target_pose_.pose.position.x, 
                               msg->pose.position.y - target_pose_.pose.position.y, 
                               msg->pose.position.z - target_pose_.pose.position.z);
      if (dist > 0.05) { // 5cm threshold for reset
        int_vel.setZero();
        int_rate.setZero();
      }
  }
  target_pose_ = *msg;
  has_target_ = true;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received Target Setpoint!");
}

void PidControllerNode::control_loop() {
  if (!has_pose_ || !has_dvl_ || !has_imu_ || !has_target_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for topics...");
    return;
  }

  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;
  if (dt <= 0.0 || dt > 1.0) return; // Prevent huge jumps if paused

  // 1. Calculate position error in World Frame
  Eigen::Vector3d p_curr(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
  Eigen::Vector3d p_targ(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
  Eigen::Vector3d p_err_world = p_targ - p_curr;

  // 2. Rotate position error into Body Frame
  Eigen::Quaterniond q_curr(current_pose_.pose.orientation.w, current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z);
  Eigen::Vector3d p_err_body = q_curr.inverse() * p_err_world;

  // 3. Calculate orientation error in Body Frame
  Eigen::Quaterniond q_targ(target_pose_.pose.orientation.w, target_pose_.pose.orientation.x, target_pose_.pose.orientation.y, target_pose_.pose.orientation.z);
  Eigen::Quaterniond q_err = q_curr.inverse() * q_targ;
  if (q_err.w() < 0) { // Keep shortest path
    q_err.coeffs() = -q_err.coeffs();
  }
  // Invert the sign of the vector components so it represents Current - Target
  Eigen::Vector3d q_err_vec(-q_err.x(), -q_err.y(), -q_err.z());

  // 4. Outer Loop PID (Position -> Target Velocity)
  Eigen::Vector3d v_targ = pos_pid.p.cwiseProduct(p_err_body);
  Eigen::Vector3d w_targ = rot_pid.p.cwiseProduct(-q_err_vec);

  // 5. Target Velocity Saturation
  double v_targ_xy_norm = std::hypot(v_targ.x(), v_targ.y());
  double max_xy = std::min(pos_pid.max_out.x(), pos_pid.max_out.y()); 
  if (v_targ_xy_norm > max_xy && v_targ_xy_norm > 1e-6) {
    v_targ.x() *= (max_xy / v_targ_xy_norm);
    v_targ.y() *= (max_xy / v_targ_xy_norm);
  }
  v_targ.z() = clamp(v_targ.z(), pos_pid.max_out.z());
  w_targ = w_targ.cwiseMin(rot_pid.max_out).cwiseMax(-rot_pid.max_out);

  // 6. Calculate velocity error
  Eigen::Vector3d vel_world(current_dvl_.twist.linear.x, current_dvl_.twist.linear.y, current_dvl_.twist.linear.z);
  Eigen::Vector3d vel_body = q_curr.inverse() * vel_world;

  Eigen::Vector3d v_err = v_targ - vel_body;
  Eigen::Vector3d angular_velocity(current_imu_.angular_velocity.x, current_imu_.angular_velocity.y, current_imu_.angular_velocity.z);
  Eigen::Vector3d w_err = w_targ - angular_velocity;

  // 7. Inner Loop PID (Velocity -> Wrench) with Integral Anti-Windup
  for (int i=0; i<3; ++i) {
    if (vel_pid.i_zone(i) == 0.0 || std::abs(p_err_body(i)) <= vel_pid.i_zone(i)) {
      int_vel(i) = std::clamp(int_vel(i) + v_err(i) * dt, -vel_pid.i_max(i), vel_pid.i_max(i));
    }
    if (rate_pid.i_zone(i) == 0.0 || std::abs(q_err_vec(i)) <= rate_pid.i_zone(i)) {
      int_rate(i) = std::clamp(int_rate(i) + w_err(i) * dt, -rate_pid.i_max(i), rate_pid.i_max(i));
    }
  }

  // Derivative on Measurement using true sensor dt and low-pass filter (alpha = 0.5)
  double alpha = 0.5;
  rclcpp::Time current_dvl_time = rclcpp::Time(current_dvl_.header.stamp);
  double dvl_dt = (current_dvl_time - last_dvl_time_).seconds();
  if (dvl_dt > 0.001) {
    Eigen::Vector3d raw_d_vel = - (vel_body - last_vel) / dvl_dt;
    d_vel = alpha * raw_d_vel + (1.0 - alpha) * d_vel;
    last_vel = vel_body;
    last_dvl_time_ = current_dvl_time;
  }

  rclcpp::Time current_imu_time = rclcpp::Time(current_imu_.header.stamp);
  double imu_dt = (current_imu_time - last_imu_time_).seconds();
  if (imu_dt > 0.001) {
    Eigen::Vector3d raw_d_rate = - (angular_velocity - last_rate) / imu_dt;
    d_rate = alpha * raw_d_rate + (1.0 - alpha) * d_rate;
    last_rate = angular_velocity;
    last_imu_time_ = current_imu_time;
  }

  Eigen::Vector3d force = vel_pid.p.cwiseProduct(v_err) + vel_pid.i.cwiseProduct(int_vel) + vel_pid.d.cwiseProduct(d_vel);
  Eigen::Vector3d torque = rate_pid.p.cwiseProduct(w_err) + rate_pid.i.cwiseProduct(int_rate) + rate_pid.d.cwiseProduct(d_rate);

  // 8. Hydrodynamic Feedforward (Drag Cancellation / Feedback Linearization) 
  // if accurate enough and add a scale to make it smaller to prevent any margin of error
  // from pushing it too much. So that it helps PID and doesn't fight against it 
  // linear + quadratic drag terms
  // double ff_scale = this->get_parameter("ff_scale").as_double();
  // force += ff_scale * (Eigen::Vector3d(16.42, 10.00, 15.00).cwiseProduct(vel_body) + 
  //                      Eigen::Vector3d(27.04, 75.00, 125.00).cwiseProduct(vel_body.cwiseProduct(vel_body.cwiseAbs())));

  // 9. Buoyancy Feedforward
  double net_buoyancy = this->get_parameter("net_buoyancy").as_double();
  Eigen::Vector3d buoyancy_world(0.0, 0.0, net_buoyancy);
  Eigen::Vector3d buoyancy_body = q_curr.inverse() * buoyancy_world;
  force -= buoyancy_body;

  // Wrench saturation
  force = force.cwiseMin(vel_pid.max_out).cwiseMax(-vel_pid.max_out);
  torque = torque.cwiseMin(rate_pid.max_out).cwiseMax(-rate_pid.max_out);

  geometry_msgs::msg::Wrench effort_msg;
  effort_msg.force.x = force.x();
  effort_msg.force.y = force.y();
  effort_msg.force.z = force.z();
  effort_msg.torque.x = torque.x();
  effort_msg.torque.y = torque.y();
  effort_msg.torque.z = torque.z();

  pub_effort_->publish(effort_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
