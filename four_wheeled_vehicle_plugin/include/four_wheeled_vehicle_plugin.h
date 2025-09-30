#ifndef FOUR_WHEELED_VEHICLE_PLUGIN_H
#define FOUR_WHEELED_VEHICLE_PLUGIN_H

#include "vehicle_msgs/msg/vehicle_cmd.hpp"
#include "vehicle_msgs/msg/vehicle_status.hpp"
#include <gazebo/common/PID.hh> // 用于PID控制器
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo {
class FourWheeledVehiclePlugin : public ModelPlugin {
public:
  FourWheeledVehiclePlugin();
  ~FourWheeledVehiclePlugin() override;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate(const common::UpdateInfo &_info);

  void CmdCallback(const vehicle_msgs::msg::VehicleCmd::SharedPtr msg);

  void UpdateSteering(double dt);

  void UpdateSpeed(double dt);

  physics::ModelPtr model_;
  physics::JointPtr fl_steer_joint_;
  physics::JointPtr fr_steer_joint_;
  physics::JointPtr rl_wheel_joint_;
  physics::JointPtr rr_wheel_joint_;
  event::ConnectionPtr update_connection_;
  common::Time last_update_time;

  common::PID left_steering_pid;
  common::PID right_steering_pid;
  common::PID rear_left_pid;
  common::PID rear_right_pid;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<vehicle_msgs::msg::VehicleCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<vehicle_msgs::msg::VehicleStatus>::SharedPtr status_pub_;

  double target_speed;
  double target_steering_angle;
  double wheelbase_;
  double track_width_;
  double wheel_radius_;
  double max_speed_;
  double max_steering_angle_;
};
} // namespace gazebo

#endif // FOUR_WHEELED_VEHICLE_PLUGIN_H
