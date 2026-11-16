// SPDX-License-Identifier: Apache-2.0
#ifndef RBY1_OMNI_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_
#define RBY1_OMNI_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace rby1_omni_controller
{

// ROS body coordinates: +x forward, +y left, +yaw counterclockwise.
struct PlanarVector {double x; double y;};
PlanarVector worldToBody(double world_x, double world_y, double body_yaw);

class OmniPursuitController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose) const;
  void resetHeading();
  void checkCollision(
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & measured_velocity) const;
  bool footprintCollision(double x, double y, double yaw) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>> collision_checker_;
  std::string name_;
  nav_msgs::msg::Path plan_;
  geometry_msgs::msg::PoseStamped goal_;
  std::size_t nearest_segment_{0};
  std::mutex mutex_;

  double desired_linear_vel_{0.10};
  double max_x_vel_{0.12};
  double max_y_vel_{0.08};
  double max_angular_vel_{0.25};
  double linear_accel_{0.15};
  double linear_decel_{0.20};
  double angular_accel_{0.30};
  double angular_decel_{0.40};
  double lookahead_dist_{0.30};
  double approach_dist_{0.30};
  double hold_yaw_kp_{1.5};
  double final_yaw_kp_{1.5};
  double transform_tolerance_{0.20};
  double collision_horizon_{1.0};
  double inactive_reset_timeout_{1.0};
  double goal_change_position_tolerance_{0.001};
  double goal_change_yaw_tolerance_{0.001};
  double max_robot_pose_search_dist_{2.0};
  double controller_period_{0.05};
  double speed_limit_{0.10};
  double held_yaw_{0.0};
  bool heading_valid_{false};
  bool final_alignment_{false};
  bool active_{false};
  rclcpp::Time last_compute_{0, 0, RCL_ROS_TIME};
};

}  // namespace rby1_omni_controller
#endif  // RBY1_OMNI_CONTROLLER__OMNI_PURSUIT_CONTROLLER_HPP_
