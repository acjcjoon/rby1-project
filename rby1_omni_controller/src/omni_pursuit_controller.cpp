// SPDX-License-Identifier: Apache-2.0
#include "rby1_omni_controller/omni_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rby1_omni_controller
{
namespace
{
constexpr double kEpsilon = 1e-9;
double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
double distance(double ax, double ay, double bx, double by) {return std::hypot(ax - bx, ay - by);}

void validatePose(geometry_msgs::msg::PoseStamped & pose)
{
  auto & p = pose.pose.position;
  auto & q = pose.pose.orientation;
  if (pose.header.frame_id.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(p.z) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    throw nav2_core::PlannerException("Omni pursuit: non-finite pose or missing frame");
  }
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < kEpsilon) {
    throw nav2_core::PlannerException("Omni pursuit: invalid pose quaternion");
  }
  q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
}

// Preserve the requested planar direction while applying BOTH axis limits.
PlanarVector limitAxes(PlanarVector velocity, double max_x, double max_y, double max_norm)
{
  double scale = 1.0;
  if (std::abs(velocity.x) > max_x) {scale = std::min(scale, max_x / std::abs(velocity.x));}
  if (std::abs(velocity.y) > max_y) {scale = std::min(scale, max_y / std::abs(velocity.y));}
  const double norm = std::hypot(velocity.x, velocity.y);
  if (norm > max_norm) {scale = std::min(scale, max_norm / norm);}
  return {velocity.x * scale, velocity.y * scale};
}

PlanarVector limitAcceleration(
  PlanarVector target, const geometry_msgs::msg::Twist & measured,
  double accel, double decel, double dt)
{
  const double speed = std::hypot(measured.linear.x, measured.linear.y);
  const double wanted = std::hypot(target.x, target.y);
  const auto brake = [&]() -> PlanarVector {
      const double scale = speed > kEpsilon ? std::max(0.0, speed - decel * dt) / speed : 0.0;
      return {measured.linear.x * scale, measured.linear.y * scale};
    };
  if (wanted < kEpsilon) {return brake();}
  const double ux = target.x / wanted, uy = target.y / wanted;
  const double along = measured.linear.x * ux + measured.linear.y * uy;
  const double lateral_squared = std::max(0.0, speed * speed - along * along);
  const double delta = (wanted < speed ? decel : accel) * dt;
  // If momentum prevents a direction-preserving command, brake first. Independent
  // vx/vy clamps would silently turn a straight diagonal into a curved approach.
  if (lateral_squared > delta * delta) {return brake();}
  const double allowance = std::sqrt(std::max(0.0, delta * delta - lateral_squared));
  if (along + allowance < 0.0) {return brake();}
  const double result = std::clamp(wanted, std::max(0.0, along - allowance), along + allowance);
  return {ux * result, uy * result};
}

bool insidePolygon(double x, double y, const std::vector<geometry_msgs::msg::Point> & polygon)
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}
}  // namespace

PlanarVector worldToBody(double world_x, double world_y, double body_yaw)
{
  const double c = std::cos(body_yaw), s = std::sin(body_yaw);
  return {c * world_x + s * world_y, -s * world_x + c * world_y};
}

void OmniPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getCostmap()) {
    throw std::runtime_error("Omni pursuit: missing node, TF or costmap");
  }
  node_ = parent; name_ = std::move(name); tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros); costmap_ = costmap_ros_->getCostmap();
  const auto parameter = [&](const std::string & key, double & value) {
      const auto full_name = name_ + "." + key;
      if (!node->has_parameter(full_name)) {node->declare_parameter(full_name, value);}
      value = node->get_parameter(full_name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(full_name + " must be finite and positive");
      }
    };
  parameter("desired_linear_vel", desired_linear_vel_);
  parameter("max_x_vel", max_x_vel_); parameter("max_y_vel", max_y_vel_);
  parameter("max_angular_vel", max_angular_vel_);
  parameter("linear_accel", linear_accel_); parameter("linear_decel", linear_decel_);
  parameter("angular_accel", angular_accel_); parameter("angular_decel", angular_decel_);
  parameter("lookahead_dist", lookahead_dist_); parameter("approach_dist", approach_dist_);
  parameter("hold_yaw_kp", hold_yaw_kp_); parameter("final_yaw_kp", final_yaw_kp_);
  parameter("transform_tolerance", transform_tolerance_);
  parameter("collision_horizon", collision_horizon_);
  parameter("inactive_reset_timeout", inactive_reset_timeout_);
  parameter("goal_change_position_tolerance", goal_change_position_tolerance_);
  parameter("goal_change_yaw_tolerance", goal_change_yaw_tolerance_);
  parameter("max_robot_pose_search_dist", max_robot_pose_search_dist_);
  if (node->has_parameter("controller_frequency")) {
    const double frequency = node->get_parameter("controller_frequency").as_double();
    if (!std::isfinite(frequency) || frequency <= 0.0) {
      throw std::runtime_error("controller_frequency must be finite and positive");
    }
    controller_period_ = 1.0 / frequency;
  }
  if (inactive_reset_timeout_ <= controller_period_) {
    throw std::runtime_error("inactive_reset_timeout must exceed one controller period");
  }
  speed_limit_ = desired_linear_vel_;
  last_compute_ = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(costmap_);
  RCLCPP_INFO(node->get_logger(), "%s: omni pursuit; heading hold, then station yaw", name_.c_str());
}

void OmniPursuitController::resetHeading()
{
  heading_valid_ = false; final_alignment_ = false; nearest_segment_ = 0;
}

void OmniPursuitController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetHeading(); active_ = true;
}

void OmniPursuitController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetHeading(); active_ = false;
}

void OmniPursuitController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetHeading(); active_ = false; plan_.poses.clear();
  collision_checker_.reset(); costmap_ = nullptr; costmap_ros_.reset(); tf_.reset();
}

geometry_msgs::msg::PoseStamped OmniPursuitController::transformPose(
  const geometry_msgs::msg::PoseStamped & input) const
{
  auto pose = input;
  validatePose(pose);
  const auto frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != frame) {
    try {
      pose = tf_->transform(pose, frame, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & exception) {
      throw nav2_core::PlannerException(std::string("Omni pursuit TF: ") + exception.what());
    }
    validatePose(pose);
  }
  return pose;
}

void OmniPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw nav2_core::PlannerException("Omni pursuit: empty path or missing path frame");
  }
  auto validated = path;
  for (auto & pose : validated.poses) {
    if (pose.header.frame_id.empty()) {pose.header = path.header;}
    validatePose(pose);
  }
  auto goal_pose = validated.poses.back();
  // A path is persistent geometry, not a sensor observation at its creation time.
  goal_pose.header.stamp = rclcpp::Time(0);
  const auto new_goal = transformPose(goal_pose);
  if (plan_.poses.empty() ||
    distance(new_goal.pose.position.x, new_goal.pose.position.y,
    goal_.pose.position.x, goal_.pose.position.y) > goal_change_position_tolerance_ ||
    std::abs(wrap(tf2::getYaw(new_goal.pose.orientation) -
    tf2::getYaw(goal_.pose.orientation))) > goal_change_yaw_tolerance_)
  {
    resetHeading();
  }
  // A replanner updates path geometry every second; it must not reset heading.
  goal_ = new_goal; plan_ = std::move(validated); nearest_segment_ = 0;
}

void OmniPursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0 || (percentage && speed_limit > 100.0)) {
    throw nav2_core::PlannerException("Omni pursuit: invalid speed limit");
  }
  // Nav2 uses zero as NO_SPEED_LIMIT, including percentage mode.
  speed_limit_ = speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT ? desired_linear_vel_ :
    std::min(desired_linear_vel_, percentage ? desired_linear_vel_ * speed_limit / 100.0 : speed_limit);
}

geometry_msgs::msg::TwistStamped OmniPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & input,
  const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto node = node_.lock();
  if (!node || !active_ || plan_.poses.empty()) {
    throw nav2_core::PlannerException("Omni pursuit: controller inactive or no path");
  }
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
    !std::isfinite(velocity.angular.z) || std::hypot(velocity.linear.x, velocity.linear.y) > 2.0 ||
    std::abs(velocity.angular.z) > 5.0)
  {
    throw nav2_core::PlannerException("Omni pursuit: invalid measured body velocity");
  }
  const auto now = node->now();
  double dt = last_compute_.nanoseconds() == 0 ? controller_period_ : (now - last_compute_).seconds();
  if (dt < 0.0 || dt > inactive_reset_timeout_) {resetHeading();}
  dt = std::clamp(dt, 0.001, 2.0 * controller_period_);
  last_compute_ = now;
  const auto robot = transformPose(input);
  const double x = robot.pose.position.x, y = robot.pose.position.y;
  const double yaw = tf2::getYaw(robot.pose.orientation);
  if (!heading_valid_) {held_yaw_ = yaw; heading_valid_ = true;}

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(plan_.poses.size());
  for (auto pose : plan_.poses) {
    pose.header.stamp = robot.header.stamp;
    path.push_back(transformPose(pose));
  }
  const auto & goal = path.back().pose;
  double xy_tolerance = 0.02, yaw_tolerance = 0.035;
  double stopped_linear = 0.01, stopped_angular = 0.02;
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker && goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    if (std::isfinite(pose_tolerance.position.x) && pose_tolerance.position.x > 0.0) {
      xy_tolerance = pose_tolerance.position.x;
    }
    const auto & q = pose_tolerance.orientation;
    const double qnorm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (std::isfinite(qnorm) && qnorm > kEpsilon) {
      const double tolerance = std::abs(tf2::getYaw(q));
      if (std::isfinite(tolerance) && tolerance > 0.0) {yaw_tolerance = tolerance;}
    }
    if (std::isfinite(velocity_tolerance.linear.x) && velocity_tolerance.linear.x > 0.0) {
      stopped_linear = velocity_tolerance.linear.x;
    }
    if (std::isfinite(velocity_tolerance.angular.z) && velocity_tolerance.angular.z > 0.0) {
      stopped_angular = velocity_tolerance.angular.z;
    }
  }
  const double goal_distance = distance(x, y, goal.position.x, goal.position.y);
  const double measured_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  const bool position_reached = goal_distance <= 0.8 * xy_tolerance;
  if (position_reached && measured_speed <= stopped_linear) {final_alignment_ = true;}

  // Project onto the nearest local segment, then interpolate a carrot by arc length.
  // Restrict forward search so a looping path cannot jump to a later crossing.
  std::size_t segment = std::min(nearest_segment_, path.size() - 1);
  double px = path[segment].pose.position.x, py = path[segment].pose.position.y;
  double best_distance = std::numeric_limits<double>::infinity();
  double searched = 0.0;
  for (std::size_t i = segment; i + 1 < path.size(); ++i) {
    const auto & a = path[i].pose.position; const auto & b = path[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t = length_squared > kEpsilon ?
      std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length_squared, 0.0, 1.0) : 0.0;
    const double qx = a.x + t * dx, qy = a.y + t * dy;
    const double d = distance(x, y, qx, qy);
    if (d < best_distance) {best_distance = d; segment = i; px = qx; py = qy;}
    searched += std::sqrt(length_squared);
    if (searched >= max_robot_pose_search_dist_) {break;}
  }
  nearest_segment_ = segment;
  double remaining = distance(x, y, px, py);
  double carrot_x = px, carrot_y = py, lookahead = lookahead_dist_;
  double previous_x = px, previous_y = py;
  for (std::size_t i = segment + 1; i < path.size(); ++i) {
    const auto & p = path[i].pose.position;
    const double length = distance(previous_x, previous_y, p.x, p.y);
    remaining += length;
    if (lookahead > 0.0 && length > kEpsilon) {
      const double fraction = std::min(1.0, lookahead / length);
      carrot_x = previous_x + fraction * (p.x - previous_x);
      carrot_y = previous_y + fraction * (p.y - previous_y);
      lookahead -= length;
    }
    previous_x = p.x; previous_y = p.y;
  }

  PlanarVector target{0.0, 0.0};
  const bool wait_for_rotation_stop = final_alignment_ && !position_reached &&
    std::abs(velocity.angular.z) > stopped_angular;
  if (!position_reached && !wait_for_rotation_stop) {
    const double dx = carrot_x - x, dy = carrot_y - y, carrot_distance = std::hypot(dx, dy);
    if (carrot_distance > kEpsilon) {
      const double approach_speed = speed_limit_ * std::min(1.0, remaining / approach_dist_);
      const double braking_speed = std::sqrt(2.0 * linear_decel_ *
        std::max(0.0, remaining - 0.5 * xy_tolerance));
      const double speed = std::min(approach_speed, braking_speed);
      target = worldToBody(speed * dx / carrot_distance, speed * dy / carrot_distance, yaw);
    }
  }
  target = limitAxes(target, max_x_vel_, max_y_vel_, speed_limit_);
  auto linear = limitAcceleration(target, velocity, linear_accel_, linear_decel_, dt);
  linear = limitAxes(linear, max_x_vel_, max_y_vel_, speed_limit_);

  const double goal_yaw_error = wrap(tf2::getYaw(goal.orientation) - yaw);
  const double yaw_error = final_alignment_ ? goal_yaw_error : wrap(held_yaw_ - yaw);
  double angular_target = 0.0;
  // Translation stops before final alignment. If it drifts out, recover position
  // while preserving the current heading, then resume final alignment.
  if (!final_alignment_ || (position_reached && measured_speed <= stopped_linear)) {
    const double kp = final_alignment_ ? final_yaw_kp_ : hold_yaw_kp_;
    const double stop_error = final_alignment_ ? 0.5 * yaw_tolerance : 0.001;
    if (std::abs(yaw_error) > stop_error) {
      angular_target = std::copysign(std::min({max_angular_vel_, kp * std::abs(yaw_error),
            std::sqrt(2.0 * angular_decel_ * std::max(0.0, std::abs(yaw_error) - stop_error))}), yaw_error);
    }
  }
  const bool slowing_yaw = angular_target * velocity.angular.z < 0.0 ||
    std::abs(angular_target) < std::abs(velocity.angular.z);
  const double yaw_delta = (slowing_yaw ? angular_decel_ : angular_accel_) * dt;
  double angular = std::clamp(angular_target, velocity.angular.z - yaw_delta, velocity.angular.z + yaw_delta);
  angular = std::clamp(angular, -max_angular_vel_, max_angular_vel_);

  if (goal_distance <= xy_tolerance && std::abs(goal_yaw_error) <= yaw_tolerance &&
    measured_speed <= stopped_linear && std::abs(velocity.angular.z) <= stopped_angular)
  {
    linear = {0.0, 0.0}; angular = 0.0; resetHeading();
  }
  geometry_msgs::msg::TwistStamped result;
  result.header.stamp = now; result.header.frame_id = costmap_ros_->getBaseFrameID();
  result.twist.linear.x = linear.x; result.twist.linear.y = linear.y; result.twist.angular.z = angular;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
  checkCollision(robot.pose, result.twist, velocity);
  return result;
}

bool OmniPursuitController::footprintCollision(double x, double y, double yaw) const
{
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3) {return true;}
  const double cost = collision_checker_->footprintCostAtPose(x, y, yaw, footprint);
  if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
  // The standard checker samples polygon edges only. Also inspect filled interior
  // cells, including unknown cells, so an obstacle entirely under the base counts.
  std::vector<geometry_msgs::msg::Point> polygon;
  unsigned int min_x = costmap_->getSizeInCellsX(), min_y = costmap_->getSizeInCellsY();
  unsigned int max_x = 0, max_y = 0;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point transformed;
    transformed.x = x + c * p.x - s * p.y; transformed.y = y + s * p.x + c * p.y;
    unsigned int mx, my;
    if (!costmap_->worldToMap(transformed.x, transformed.y, mx, my)) {return true;}
    min_x = std::min(min_x, mx); min_y = std::min(min_y, my);
    max_x = std::max(max_x, mx); max_y = std::max(max_y, my);
    polygon.push_back(transformed);
  }
  for (unsigned int mx = min_x; mx <= max_x; ++mx) {
    for (unsigned int my = min_y; my <= max_y; ++my) {
      if (costmap_->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy; costmap_->mapToWorld(mx, my, wx, wy);
        if (insidePolygon(wx, wy, polygon)) {return true;}
      }
    }
  }
  return false;
}

void OmniPursuitController::checkCollision(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & command,
  const geometry_msgs::msg::Twist & measured_velocity) const
{
  double radius = 0.0;
  for (const auto & point : costmap_ros_->getRobotFootprint()) {
    radius = std::max(radius, std::hypot(point.x, point.y));
  }
  const auto sweep = [&](const geometry_msgs::msg::Twist & twist, bool braking) {
      double x = pose.position.x, y = pose.position.y, yaw = tf2::getYaw(pose.orientation);
      double vx = twist.linear.x, vy = twist.linear.y, wz = twist.angular.z;
      const double stop_time = std::max(std::hypot(vx, vy) / linear_decel_,
        std::abs(wz) / angular_decel_);
      const double duration = braking ? stop_time + controller_period_ :
        std::max(collision_horizon_, stop_time + controller_period_);
      double elapsed = 0.0;
      std::size_t steps = 0;
      while (true) {
        if (footprintCollision(x, y, yaw)) {
          throw nav2_core::PlannerException("Omni pursuit: collision, unknown space or footprint outside costmap");
        }
        if (elapsed >= duration) {break;}
        if (++steps > 10000) {throw nav2_core::PlannerException("Omni pursuit: collision horizon too large");}
        const double speed = std::hypot(vx, vy);
        const double dt = std::min({0.05, duration - elapsed,
            0.4 * costmap_->getResolution() / std::max(kEpsilon, speed + radius * std::abs(wz))});
        double next_vx = vx, next_vy = vy, next_wz = wz;
        if (braking) {
          const double scale = speed > kEpsilon ? std::max(0.0, speed - linear_decel_ * dt) / speed : 0.0;
          next_vx *= scale; next_vy *= scale;
          next_wz = std::copysign(std::max(0.0, std::abs(wz) - angular_decel_ * dt), wz);
        }
        const double mid_yaw = yaw + 0.25 * (wz + next_wz) * dt;
        const double mid_vx = 0.5 * (vx + next_vx), mid_vy = 0.5 * (vy + next_vy);
        x += (std::cos(mid_yaw) * mid_vx - std::sin(mid_yaw) * mid_vy) * dt;
        y += (std::sin(mid_yaw) * mid_vx + std::cos(mid_yaw) * mid_vy) * dt;
        yaw += 0.5 * (wz + next_wz) * dt;
        vx = next_vx; vy = next_vy; wz = next_wz; elapsed += dt;
      }
    };
  sweep(command, false);
  sweep(measured_velocity, true);
}

}  // namespace rby1_omni_controller
PLUGINLIB_EXPORT_CLASS(rby1_omni_controller::OmniPursuitController, nav2_core::Controller)
