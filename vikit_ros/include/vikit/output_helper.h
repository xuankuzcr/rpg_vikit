/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *      Author: cforster
 */

#ifndef OUTPUT_HELPER_H_
#define OUTPUT_HELPER_H_

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vk {
namespace output_helper {

using namespace std;
using namespace Eigen;
using namespace Sophus;

void publishTfTransform(const SE3<double> &T, const rclcpp::Time &stamp,
                        const string &frame_id, const string &child_frame_id,
                        std::shared_ptr<tf2_ros::TransformBroadcaster> &br);

void publishPointMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos, const string &ns, const rclcpp::Time &timestamp,
    int id, int action, double marker_scale, const Vector3d &color,
    rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void publishLineMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &start, const Vector3d &end, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    const Vector3d &color, rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void publishArrowMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos, const Vector3d &dir, double scale, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    const Vector3d &color);

void publishHexacopterMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const string &frame_id, const string &ns, const rclcpp::Time &timestamp,
    int id, int action, double marker_scale, const Vector3d &color);

void publishCameraMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const string &frame_id, const string &ns, const rclcpp::Time &timestamp,
    int id, double marker_scale, const Vector3d &color);

void publishFrameMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Matrix3d &rot, const Vector3d &pos, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

/// Helper class to create visualization messages.
class OutputHelper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OutputHelper();
  ~OutputHelper();

  /// Create a marker message for a coordinate frame.
  visualization_msgs::msg::Marker markerFrame(const SE3<double> &T_w_f,
                                              const std::string &ns,
                                              const int id, const double scale,
                                              const double alpha = 1.0);

  /// Create a marker message for a point cloud.
  visualization_msgs::msg::Marker
  markerPoints(const std::vector<Eigen::Vector3d> &points,
               const std::string &ns, const int id, const std::string &frame_id,
               const double scale,
               const std::vector<std::array<double, 3>> &colors =
                   std::vector<std::array<double, 3>>());

  /// Create a marker message for a line strip.
  visualization_msgs::msg::Marker
  markerLineStrip(const std::vector<Eigen::Vector3d> &points,
                  const std::string &ns, const int id,
                  const std::string &frame_id, const double scale,
                  const std::array<double, 3> &color = {1.0, 0.0, 0.0});

  /// Create a marker message for a trajectory.
  visualization_msgs::msg::Marker
  markerTrajectory(const std::vector<SE3<double>> &trajectory,
                   const std::string &ns, const int id,
                   const std::string &frame_id, const double scale,
                   const std::array<double, 3> &color = {1.0, 0.0, 0.0});

  /// Create a marker array message for multiple coordinate frames.
  visualization_msgs::msg::MarkerArray
  markerFrames(const std::vector<SE3<double>> &frames, const std::string &ns,
               const int id_start, const double scale,
               const double alpha = 1.0);

private:
  std::string frame_id_;
};

} // namespace output_helper
} // namespace vk

#endif /* OUTPUT_HELPER_H_ */
