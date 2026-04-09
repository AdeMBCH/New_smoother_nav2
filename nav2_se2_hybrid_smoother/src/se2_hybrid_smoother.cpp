#include "nav2_se2_hybrid_smoother/se2_hybrid_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_se2_hybrid_smoother
{

namespace
{
template<typename ParameterT>
void declareIfNotDeclared(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::string & parameter_name,
  const ParameterT & default_value)
{
  if (!node->has_parameter(parameter_name)) {
    node->declare_parameter(parameter_name, default_value);
  }
}
}  // namespace


double SE2HybridSmoother::wrapAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

std::vector<double> SE2HybridSmoother::unwrapAngles(const std::vector<double> & wrapped)
{
  if (wrapped.empty()) {
    return {};
  }

  std::vector<double> unwrapped(wrapped.size(), wrapped.front());
  for (size_t i = 1; i < wrapped.size(); ++i) {
    const double delta = wrapAngle(wrapped[i] - wrapped[i - 1]);
    unwrapped[i] = unwrapped[i - 1] + delta;
  }
  return unwrapped;
}

void SE2HybridSmoother::removeDuplicatePoses(nav_msgs::msg::Path & path, double eps)
{
  if (path.poses.size() < 2) {
    return;
  }

  nav_msgs::msg::Path filtered;
  filtered.header = path.header;
  filtered.poses.reserve(path.poses.size());
  filtered.poses.push_back(path.poses.front());

  auto sq_dist = [](const auto & a, const auto & b) {
      const double dx = a.pose.position.x - b.pose.position.x;
      const double dy = a.pose.position.y - b.pose.position.y;
      return dx * dx + dy * dy;
    };

  const double eps2 = eps * eps;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    if (sq_dist(path.poses[i], filtered.poses.back()) > eps2) {
      filtered.poses.push_back(path.poses[i]);
    }
  }

  if (filtered.poses.size() == 1) {
    filtered.poses.push_back(path.poses.back());
  }
  path = std::move(filtered);
}

void SE2HybridSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  parent_ = parent;
  name_ = std::move(name);

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Lifecycle node is expired in SE2HybridSmoother::configure");
  }

  logger_ = node->get_logger();

  declareIfNotDeclared(node, name_ + ".resample_distance", 0.05);
  declareIfNotDeclared(node, name_ + ".max_iterations", 30);
  declareIfNotDeclared(node, name_ + ".w_data_pos", 1.0);
  declareIfNotDeclared(node, name_ + ".w_xy_smooth", 0.25);
  declareIfNotDeclared(node, name_ + ".w_heading_tangent", 0.4);
  declareIfNotDeclared(node, name_ + ".w_heading_smooth", 0.2);
  declareIfNotDeclared(node, name_ + ".w_increment_smooth", 0.15);
  declareIfNotDeclared(node, name_ + ".w_curvature_var", 0.05);
  declareIfNotDeclared(node, name_ + ".preserve_start_orientation", true);
  declareIfNotDeclared(node, name_ + ".preserve_goal_orientation", true);
  declareIfNotDeclared(node, name_ + ".convergence_tol", 1e-4);

  node->get_parameter(name_ + ".resample_distance", resample_distance_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".w_data_pos", w_data_pos_);
  node->get_parameter(name_ + ".w_xy_smooth", w_xy_smooth_);
  node->get_parameter(name_ + ".w_heading_tangent", w_heading_tangent_);
  node->get_parameter(name_ + ".w_heading_smooth", w_heading_smooth_);
  node->get_parameter(name_ + ".w_increment_smooth", w_increment_smooth_);
  node->get_parameter(name_ + ".w_curvature_var", w_curvature_var_);
  node->get_parameter(name_ + ".preserve_start_orientation", preserve_start_orientation_);
  node->get_parameter(name_ + ".preserve_goal_orientation", preserve_goal_orientation_);
  node->get_parameter(name_ + ".convergence_tol", convergence_tol_);

  resample_distance_ = std::max(0.01, resample_distance_);
  max_iterations_ = std::max(1, max_iterations_);
  convergence_tol_ = std::max(1e-8, convergence_tol_);

  RCLCPP_INFO(logger_, "Configured %s smoother plugin", name_.c_str());
}

void SE2HybridSmoother::cleanup()
{
  RCLCPP_INFO(logger_, "Cleanup %s smoother plugin", name_.c_str());
}

void SE2HybridSmoother::activate()
{
  RCLCPP_INFO(logger_, "Activate %s smoother plugin", name_.c_str());
}

void SE2HybridSmoother::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivate %s smoother plugin", name_.c_str());
}

nav_msgs::msg::Path SE2HybridSmoother::resamplePath(const nav_msgs::msg::Path & path, double step) const
{
  nav_msgs::msg::Path out;
  out.header = path.header;

  if (path.poses.size() < 2) {
    out = path;
    return out;
  }

  std::vector<double> cumdist(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p0 = path.poses[i - 1].pose.position;
    const auto & p1 = path.poses[i].pose.position;
    const double seg = std::hypot(p1.x - p0.x, p1.y - p0.y);
    cumdist[i] = cumdist[i - 1] + seg;
  }

  const double total = cumdist.back();
  if (total < 1e-6) {
    out = path;
    return out;
  }

  out.poses.clear();
  out.poses.push_back(path.poses.front());

  for (double s = step; s < total; s += step) {
    auto it = std::lower_bound(cumdist.begin(), cumdist.end(), s);
    if (it == cumdist.end()) {
      break;
    }
    size_t idx = static_cast<size_t>(std::distance(cumdist.begin(), it));
    if (idx == 0) {
      continue;
    }

    const double s0 = cumdist[idx - 1];
    const double s1 = cumdist[idx];
    const double ratio = (s1 - s0 > 1e-9) ? (s - s0) / (s1 - s0) : 0.0;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    const auto & p0 = path.poses[idx - 1].pose.position;
    const auto & p1 = path.poses[idx].pose.position;

    pose.pose.position.x = p0.x + ratio * (p1.x - p0.x);
    pose.pose.position.y = p0.y + ratio * (p1.y - p0.y);
    pose.pose.position.z = p0.z + ratio * (p1.z - p0.z);

    tf2::Quaternion q0, q1;
    tf2::fromMsg(path.poses[idx - 1].pose.orientation, q0);
    tf2::fromMsg(path.poses[idx].pose.orientation, q1);
    tf2::Quaternion q = q0.slerp(q1, ratio);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);

    out.poses.push_back(pose);
  }

  out.poses.push_back(path.poses.back());
  return out;
}

std::vector<double> SE2HybridSmoother::computeTangentHeading(const nav_msgs::msg::Path & path) const
{
  const size_t n = path.poses.size();
  std::vector<double> theta(n, 0.0);
  if (n == 0) {
    return theta;
  }

  if (n == 1) {
    theta[0] = tf2::getYaw(path.poses[0].pose.orientation);
    return theta;
  }

  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & pm = path.poses[i - 1].pose.position;
    const auto & pp = path.poses[i + 1].pose.position;
    theta[i] = std::atan2(pp.y - pm.y, pp.x - pm.x);
  }

  theta[0] = std::atan2(
    path.poses[1].pose.position.y - path.poses[0].pose.position.y,
    path.poses[1].pose.position.x - path.poses[0].pose.position.x);
  theta[n - 1] = std::atan2(
    path.poses[n - 1].pose.position.y - path.poses[n - 2].pose.position.y,
    path.poses[n - 1].pose.position.x - path.poses[n - 2].pose.position.x);

  return unwrapAngles(theta);
}

std::vector<double> SE2HybridSmoother::computeCurvature(
  const nav_msgs::msg::Path & path,
  const std::vector<double> & unwrapped_heading) const
{
  const size_t n = path.poses.size();
  std::vector<double> kappa(n, 0.0);
  if (n < 2 || unwrapped_heading.size() != n) {
    return kappa;
  }

  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;
    const double ds = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (ds > 1e-6) {
      kappa[i] = (unwrapped_heading[i + 1] - unwrapped_heading[i]) / ds;
    }
  }
  kappa[n - 1] = kappa[n - 2];
  return kappa;
}

std::vector<SE2HybridSmoother::IncrementSE2> SE2HybridSmoother::computeRelativeIncrementsSE2(
  const nav_msgs::msg::Path & path,
  const std::vector<double> & wrapped_heading) const
{
  const size_t n = path.poses.size();
  std::vector<IncrementSE2> incs;
  if (n < 2 || wrapped_heading.size() != n) {
    return incs;
  }

  incs.resize(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double c = std::cos(wrapped_heading[i]);
    const double s = std::sin(wrapped_heading[i]);

    incs[i].dx_local = c * dx + s * dy;
    incs[i].dy_local = -s * dx + c * dy;
    incs[i].dtheta = wrapAngle(wrapped_heading[i + 1] - wrapped_heading[i]);
  }
  return incs;
}

void SE2HybridSmoother::applyIncrementCorrection(
  nav_msgs::msg::Path & path,
  std::vector<double> & wrapped_heading,
  const std::vector<IncrementSE2> & original,
  const std::vector<IncrementSE2> & smoothed,
  double alpha) const
{
  if (original.size() != smoothed.size() || path.poses.size() < 3) {
    return;
  }

  for (size_t i = 1; i + 1 < original.size(); ++i) {
    const double ddx = smoothed[i].dx_local - original[i].dx_local;
    const double ddy = smoothed[i].dy_local - original[i].dy_local;
    const double ddtheta = wrapAngle(smoothed[i].dtheta - original[i].dtheta);

    const double c = std::cos(wrapped_heading[i]);
    const double s = std::sin(wrapped_heading[i]);
    const double gx = c * ddx - s * ddy;
    const double gy = s * ddx + c * ddy;

    path.poses[i + 1].pose.position.x += alpha * gx;
    path.poses[i + 1].pose.position.y += alpha * gy;
    wrapped_heading[i + 1] = wrapAngle(wrapped_heading[i + 1] + alpha * ddtheta);
  }
}

bool SE2HybridSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 3) {
    return true;
  }

  removeDuplicatePoses(path);
  if (path.poses.size() < 3) {
    return true;
  }

  const std::string frame = path.header.frame_id;
  for (const auto & pose : path.poses) {
    if (pose.header.frame_id != frame) {
      RCLCPP_WARN(logger_, "Path contains mixed frame_id values, skipping smoothing");
      return false;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  const auto max_ns = std::chrono::nanoseconds(max_time.nanoseconds());

  nav_msgs::msg::Path resampled = resamplePath(path, resample_distance_);
  if (resampled.poses.size() < 3) {
    path = std::move(resampled);
    return true;
  }

  const nav_msgs::msg::Path anchor = resampled;
  const auto start_pose = anchor.poses.front();
  const auto goal_pose = anchor.poses.back();

  std::vector<double> heading(resampled.poses.size(), 0.0);
  for (size_t i = 0; i < resampled.poses.size(); ++i) {
    heading[i] = tf2::getYaw(resampled.poses[i].pose.orientation);
  }

  for (int iter = 0; iter < max_iterations_; ++iter) {
    if (std::chrono::steady_clock::now() - start > max_ns) {
      RCLCPP_DEBUG(logger_, "SE2 smoothing reached time budget at iteration %d", iter);
      break;
    }

    double max_delta = 0.0;

    // Step 3: conservative XY smoothing with strong data anchoring.
    for (size_t i = 1; i + 1 < resampled.poses.size(); ++i) {
      auto & p = resampled.poses[i].pose.position;
      const auto & p0 = anchor.poses[i].pose.position;
      const auto & pm = resampled.poses[i - 1].pose.position;
      const auto & pp = resampled.poses[i + 1].pose.position;

      const double dx = w_data_pos_ * (p0.x - p.x) + w_xy_smooth_ * (pm.x + pp.x - 2.0 * p.x);
      const double dy = w_data_pos_ * (p0.y - p.y) + w_xy_smooth_ * (pm.y + pp.y - 2.0 * p.y);

      p.x += dx;
      p.y += dy;
      max_delta = std::max(max_delta, std::hypot(dx, dy));
    }

    // Step 4 + 5: tangent heading and heading smoothing.
    std::vector<double> theta_tan = computeTangentHeading(resampled);
    std::vector<double> heading_unwrapped = unwrapAngles(heading);
    for (size_t i = 1; i + 1 < heading_unwrapped.size(); ++i) {
      const double d_tan = theta_tan[i] - heading_unwrapped[i];
      const double d_smooth = 0.5 * (heading_unwrapped[i - 1] + heading_unwrapped[i + 1]) - heading_unwrapped[i];
      const double dtheta = w_heading_tangent_ * d_tan + w_heading_smooth_ * d_smooth;
      heading_unwrapped[i] += dtheta;
      max_delta = std::max(max_delta, std::abs(dtheta));
    }

    // Step 6: local SE(2) increment correction (lightweight, local only).
    for (size_t i = 0; i < heading.size(); ++i) {
      heading[i] = wrapAngle(heading_unwrapped[i]);
    }
    auto inc = computeRelativeIncrementsSE2(resampled, heading);
    auto inc_smooth = inc;
    for (size_t i = 1; i + 1 < inc.size(); ++i) {
      inc_smooth[i].dx_local = (1.0 - w_increment_smooth_) * inc[i].dx_local +
        0.5 * w_increment_smooth_ * (inc[i - 1].dx_local + inc[i + 1].dx_local);
      inc_smooth[i].dy_local = (1.0 - w_increment_smooth_) * inc[i].dy_local +
        0.5 * w_increment_smooth_ * (inc[i - 1].dy_local + inc[i + 1].dy_local);
      inc_smooth[i].dtheta = wrapAngle(
        (1.0 - w_increment_smooth_) * inc[i].dtheta +
        0.5 * w_increment_smooth_ * (inc[i - 1].dtheta + inc[i + 1].dtheta));
    }
    applyIncrementCorrection(resampled, heading, inc, inc_smooth, 0.25);

    // Step 7: mild curvature-variation regularization.
    heading_unwrapped = unwrapAngles(heading);
    auto kappa = computeCurvature(resampled, heading_unwrapped);
    for (size_t i = 1; i + 2 < kappa.size(); ++i) {
      const double dk_prev = kappa[i] - kappa[i - 1];
      const double dk_next = kappa[i + 1] - kappa[i];
      const double dd = (dk_next - dk_prev);
      heading_unwrapped[i + 1] -= w_curvature_var_ * dd * 0.01;
    }

    if (preserve_start_orientation_) {
      heading_unwrapped.front() = tf2::getYaw(start_pose.pose.orientation);
    }
    if (preserve_goal_orientation_) {
      heading_unwrapped.back() = tf2::getYaw(goal_pose.pose.orientation);
    }

    for (size_t i = 0; i < heading.size(); ++i) {
      heading[i] = wrapAngle(heading_unwrapped[i]);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, heading[i]);
      q.normalize();
      resampled.poses[i].pose.orientation = tf2::toMsg(q);
    }

    // Step 8: preserve terminal positions.
    resampled.poses.front().pose.position = start_pose.pose.position;
    resampled.poses.back().pose.position = goal_pose.pose.position;

    if (max_delta < convergence_tol_) {
      RCLCPP_DEBUG(logger_, "SE2 smoothing converged at iteration %d", iter);
      break;
    }
  }

  path = std::move(resampled);
  return true;
}

}  // namespace nav2_se2_hybrid_smoother

PLUGINLIB_EXPORT_CLASS(nav2_se2_hybrid_smoother::SE2HybridSmoother, nav2_core::Smoother)
