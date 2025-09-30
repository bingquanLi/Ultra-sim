#include "four_wheeled_vehicle_plugin.h"
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace gazebo {

FourWheeledVehiclePlugin::FourWheeledVehiclePlugin() {
  left_steering_pid = common::PID(2000.0, 0.0, 300.0);
  right_steering_pid = common::PID(2000.0, 0.0, 300.0);
  left_steering_pid.SetCmdMin(-5000.0);
  left_steering_pid.SetCmdMax(5000.0);
  right_steering_pid.SetCmdMin(-5000.0);
  right_steering_pid.SetCmdMax(5000.0);

  rear_left_pid = common::PID(1000.0, 0.0, 1.0);
  rear_right_pid = common::PID(1000.0, 0.0, 1.0);
  rear_left_pid.SetCmdMin(-5000.0);
  rear_left_pid.SetCmdMax(5000.0);
  rear_right_pid.SetCmdMin(-5000.0);
  rear_right_pid.SetCmdMax(5000.0);

  target_speed = 0.0;
  target_steering_angle = 0.0;
  last_update_time = common::Time(0);
}

FourWheeledVehiclePlugin::~FourWheeledVehiclePlugin() {
  update_connection_.reset();
  ros_node_.reset();
}

void FourWheeledVehiclePlugin::Load(physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf) {
  model_ = _model;

  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("vehicle_plugin"), "ROS 2未初始化");
    return;
  }
  ros_node_ = rclcpp::Node::make_shared("vehicle_controller_node");
  RCLCPP_INFO(ros_node_->get_logger(), "车辆控制器插件加载成功");

  fl_steer_joint_ = model_->GetJoint("front_left_steering_joint");
  fr_steer_joint_ = model_->GetJoint("front_right_steering_joint");
  rl_wheel_joint_ = model_->GetJoint("rear_left_wheel_joint");
  rr_wheel_joint_ = model_->GetJoint("rear_right_wheel_joint");

  wheelbase_ = _sdf->Get<double>("wheelbase", 3.0).first;
  track_width_ = _sdf->Get<double>("track_width", 1.666).first;
  wheel_radius_ = _sdf->Get<double>("wheel_radius", 0.3).first;
  max_speed_ = _sdf->Get<double>("max_speed", 20.0).first;
  max_steering_angle_ = _sdf->Get<double>("max_steering_angle", 0.6).first;

  if (!fl_steer_joint_ || !fr_steer_joint_ || !rl_wheel_joint_ ||
      !rr_wheel_joint_) {
    RCLCPP_FATAL(ros_node_->get_logger(), "关节未找到,请检查SDF关节名");
    return;
  }

  cmd_sub_ = ros_node_->create_subscription<vehicle_msgs::msg::VehicleCmd>(
      "vehicle_cmd", 10,
      std::bind(&FourWheeledVehiclePlugin::CmdCallback, this,
                std::placeholders::_1));

  status_pub_ = ros_node_->create_publisher<vehicle_msgs::msg::VehicleStatus>(
      "vehicle_status", 10);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &FourWheeledVehiclePlugin::OnUpdate, this, std::placeholders::_1));
}

void FourWheeledVehiclePlugin::CmdCallback(
    const vehicle_msgs::msg::VehicleCmd::SharedPtr msg) {
  target_speed = msg->speed;
  target_steering_angle = msg->steering_angle;

  target_speed = std::clamp(target_speed, -max_speed_, max_speed_);
  target_steering_angle = std::clamp(target_steering_angle,
                                     -max_steering_angle_, max_steering_angle_);
}

void FourWheeledVehiclePlugin::OnUpdate(const common::UpdateInfo &_info) {
  rclcpp::spin_some(ros_node_);

  if (last_update_time == common::Time(0)) {
    last_update_time = _info.simTime;
    return;
  }
  double dt = (_info.simTime - last_update_time).Double();
  last_update_time = _info.simTime;

  UpdateSteering(dt);
  UpdateSpeed(dt);

  auto status_msg = vehicle_msgs::msg::VehicleStatus();

  auto base_link = model_->GetLink("base_link");
  auto pose = base_link->WorldPose();
  status_msg.position.x = pose.Pos().X();
  status_msg.position.y = pose.Pos().Y();
  status_msg.position.z = pose.Pos().Z();

  status_msg.yaw = pose.Rot().Yaw();

  auto linear_vel = base_link->WorldLinearVel();
  status_msg.speed = std::hypot(linear_vel.X(), linear_vel.Y());

  status_pub_->publish(status_msg);
}

void FourWheeledVehiclePlugin::UpdateSteering(double dt) {
  double tan_alph = tan(target_steering_angle);
  double target_fl_angle = atan(wheelbase_ * tan_alph /
                                (wheelbase_ - 0.5 * track_width_ * tan_alph));
  double target_fr_angle = atan(wheelbase_ * tan_alph /
                                (wheelbase_ + 0.5 * track_width_ * tan_alph));

  double current_fl_angle = fl_steer_joint_->Position(0);
  double current_fr_angle = fr_steer_joint_->Position(0);
  double fl_error = current_fl_angle - target_fl_angle;
  double fr_error = current_fr_angle - target_fr_angle;

  double fl_force = left_steering_pid.Update(fl_error, dt);
  double fr_force = right_steering_pid.Update(fr_error, dt);

  fl_steer_joint_->SetForce(0, fl_force);
  fr_steer_joint_->SetForce(0, fr_force);
}

void FourWheeledVehiclePlugin::UpdateSpeed(double dt) {
  double target_angular_vel = target_speed / wheel_radius_;

  double current_rl_vel = rl_wheel_joint_->GetVelocity(0);
  double current_rr_vel = rr_wheel_joint_->GetVelocity(0);

  double rl_error = current_rl_vel - target_angular_vel;
  double rr_error = current_rr_vel - target_angular_vel;

  double rl_force = rear_left_pid.Update(rl_error, dt);
  double rr_force = rear_right_pid.Update(rr_error, dt);

  rl_wheel_joint_->SetForce(0, rl_force);
  rr_wheel_joint_->SetForce(0, rr_force);
}

GZ_REGISTER_MODEL_PLUGIN(FourWheeledVehiclePlugin)

} // namespace gazebo
