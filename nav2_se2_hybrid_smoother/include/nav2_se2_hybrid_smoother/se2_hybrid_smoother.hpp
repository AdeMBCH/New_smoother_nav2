#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/smoother.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_se2_hybrid_smoother
{

class SE2HybridSmoother : public nav2_core::Smoother
{
public:
  SE2HybridSmoother() = default;
  ~SE2HybridSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

private:
  struct IncrementSE2
  {
    double dx_local{0.0};
    double dy_local{0.0};
    double dtheta{0.0};
  };

  static double wrapAngle(double a);
  static std::vector<double> unwrapAngles(const std::vector<double> & wrapped);
  static void removeDuplicatePoses(nav_msgs::msg::Path & path, double eps = 1e-6);

  nav_msgs::msg::Path resamplePath(const nav_msgs::msg::Path & path, double step) const;
  std::vector<double> computeTangentHeading(const nav_msgs::msg::Path & path) const;
  std::vector<double> computeCurvature(
    const nav_msgs::msg::Path & path,
    const std::vector<double> & unwrapped_heading) const;
  std::vector<IncrementSE2> computeRelativeIncrementsSE2(
    const nav_msgs::msg::Path & path,
    const std::vector<double> & wrapped_heading) const;
  void applyIncrementCorrection(
    nav_msgs::msg::Path & path,
    std::vector<double> & wrapped_heading,
    const std::vector<IncrementSE2> & original,
    const std::vector<IncrementSE2> & smoothed,
    double alpha) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("SE2HybridSmoother")};
  std::string name_;

  double resample_distance_{0.05};
  int max_iterations_{30};
  double w_data_pos_{1.0};
  double w_xy_smooth_{0.25};
  double w_heading_tangent_{0.4};
  double w_heading_smooth_{0.2};
  double w_increment_smooth_{0.15};
  double w_curvature_var_{0.05};
  bool preserve_start_orientation_{true};
  bool preserve_goal_orientation_{true};
  double convergence_tol_{1e-4};
};

}  // namespace nav2_se2_hybrid_smoother
