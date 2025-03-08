/*
 * output_helper.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: chrigi
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vikit/output_helper.h>
#include <visualization_msgs/msg/marker.hpp>

namespace vk {
namespace output_helper {

void publishTfTransform(const SE3<double> &T, const rclcpp::Time &stamp,
                        const string &frame_id, const string &child_frame_id,
                        std::shared_ptr<tf2_ros::TransformBroadcaster> &br) {
  Eigen::Matrix<double, 3, 1> t(T.translation());
  Eigen::Quaterniond q(T.rotationMatrix());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.translation.x = t[0];
  transform.transform.translation.y = t[1];
  transform.transform.translation.z = t[2];
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  br->sendTransform(transform);
}

void publishPointMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos, const string &ns, const rclcpp::Time &timestamp,
    int id, int action, double marker_scale, const Vector3d &color,
    rclcpp::Duration lifetime) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = action;
  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;
  marker.color.a = 1.0;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.lifetime = lifetime;
  pub->publish(marker);
}

void publishLineMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &start, const Vector3d &end, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    const Vector3d &color, rclcpp::Duration lifetime) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = action;
  marker.scale.x = marker_scale;
  marker.color.a = 1.0;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  geometry_msgs::msg::Point start_point;
  start_point.x = start[0];
  start_point.y = start[1];
  start_point.z = start[2];
  marker.points.push_back(start_point);
  geometry_msgs::msg::Point end_point;
  end_point.x = end[0];
  end_point.y = end[1];
  end_point.z = end[2];
  marker.points.push_back(end_point);
  marker.lifetime = lifetime;
  pub->publish(marker);
}

void publishArrowMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos, const Vector3d &dir, double scale, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    const Vector3d &color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = action;
  marker.scale.x = marker_scale;
  marker.scale.y = 2 * marker_scale;
  marker.scale.z = 2 * marker_scale;
  marker.color.a = 1.0;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  geometry_msgs::msg::Point start_point;
  start_point.x = pos[0];
  start_point.y = pos[1];
  start_point.z = pos[2];
  marker.points.push_back(start_point);
  geometry_msgs::msg::Point end_point;
  end_point.x = pos[0] + scale * dir[0];
  end_point.y = pos[1] + scale * dir[1];
  end_point.z = pos[2] + scale * dir[2];
  marker.points.push_back(end_point);
  pub->publish(marker);
}

void publishHexacopterMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const string &frame_id, const string &ns, const rclcpp::Time &timestamp,
    int id, int action, double marker_scale, const Vector3d &color) {
  /*
   * Function by Markus Achtelik from libsfly_viz.
   * Thank you.
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = 0;
  marker.id = id;

  // make rotors
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = 0.2 * marker_scale;
  marker.scale.y = 0.2 * marker_scale;
  marker.scale.z = 0.01 * marker_scale;
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 0.8;
  marker.pose.position.z = 0;

  // front left/right
  marker.pose.position.x = 0.19 * marker_scale;
  marker.pose.position.y = 0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = 0.19 * marker_scale;
  marker.pose.position.y = -0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // left/right
  marker.pose.position.x = 0;
  marker.pose.position.y = 0.22 * marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = 0;
  marker.pose.position.y = -0.22 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // back left/right
  marker.pose.position.x = -0.19 * marker_scale;
  marker.pose.position.y = 0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = -0.19 * marker_scale;
  marker.pose.position.y = -0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // make arms
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.44 * marker_scale;
  marker.scale.y = 0.02 * marker_scale;
  marker.scale.z = 0.01 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.015 * marker_scale;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;

  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);

  // 30 deg rotation  0.9659  0  0  0.2588
  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = 0.2588;
  marker.id--;
  pub->publish(marker);

  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = -0.2588;
  marker.id--;
  pub->publish(marker);
}

void publishCameraMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const string &frame_id, const string &ns, const rclcpp::Time &timestamp,
    int id, double marker_scale, const Vector3d &color) {
  /*
   * draw a pyramid as the camera marker
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = 0;
  marker.id = id;

  // make rectangles as frame
  double r_w = 1.0;
  double z_plane = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = 0;
  marker.pose.position.y = (r_w / 4.0) * marker_scale;
  marker.pose.position.z = z_plane;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = r_w * marker_scale;
  marker.scale.y = 0.04 * marker_scale;
  marker.scale.z = 0.04 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id--;
  pub->publish(marker);
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.y = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);
  marker.pose.position.x = -(r_w / 2.0) * marker_scale;
  marker.id--;
  pub->publish(marker);

  // make pyramid edges
  marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
  marker.pose.position.z = 0.5 * z_plane;

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  //  0.08198092, -0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  // -0.27395078, -0.22863284,  0.9091823 ,  0.21462883
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  //  -0.08198092,  0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  // -0.08198092, -0.34727674, -0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);
}

void publishFrameMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Matrix3d &rot, const Vector3d &pos, const string &ns,
    const rclcpp::Time &timestamp, int id, int action, double marker_scale,
    rclcpp::Duration lifetime) {
  Vector3d x = rot.col(0);
  Vector3d y = rot.col(1);
  Vector3d z = rot.col(2);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id++;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = action;
  marker.points.reserve(6);
  marker.scale.x = 0.2 * marker_scale;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = lifetime;

  geometry_msgs::msg::Point p;
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  marker.points.push_back(p);
  p.x = pos[0] + marker_scale * x[0];
  p.y = pos[1] + marker_scale * x[1];
  p.z = pos[2] + marker_scale * x[2];
  marker.points.push_back(p);
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  pub->publish(marker);

  marker.points.clear();
  marker.id = id++;
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  marker.points.push_back(p);
  p.x = pos[0] + marker_scale * y[0];
  p.y = pos[1] + marker_scale * y[1];
  p.z = pos[2] + marker_scale * y[2];
  marker.points.push_back(p);
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  pub->publish(marker);

  marker.points.clear();
  marker.id = id++;
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  marker.points.push_back(p);
  p.x = pos[0] + marker_scale * z[0];
  p.y = pos[1] + marker_scale * z[1];
  p.z = pos[2] + marker_scale * z[2];
  marker.points.push_back(p);
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  pub->publish(marker);
}

OutputHelper::OutputHelper() : frame_id_("world") {}

OutputHelper::~OutputHelper() {}

visualization_msgs::msg::Marker
OutputHelper::markerFrame(const SE3<double> &T_w_f, const std::string &ns,
                          const int id, const double scale,
                          const double alpha) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.2 * scale;

  const Matrix3d R = T_w_f.rotationMatrix();
  const Vector3d t = T_w_f.translation();
  const Vector3d x = R.col(0);
  const Vector3d y = R.col(1);
  const Vector3d z = R.col(2);

  geometry_msgs::msg::Point p;
  p.x = t[0];
  p.y = t[1];
  p.z = t[2];
  marker.points.push_back(p);
  p.x = t[0] + scale * x[0];
  p.y = t[1] + scale * x[1];
  p.z = t[2] + scale * x[2];
  marker.points.push_back(p);

  p.x = t[0];
  p.y = t[1];
  p.z = t[2];
  marker.points.push_back(p);
  p.x = t[0] + scale * y[0];
  p.y = t[1] + scale * y[1];
  p.z = t[2] + scale * y[2];
  marker.points.push_back(p);

  p.x = t[0];
  p.y = t[1];
  p.z = t[2];
  marker.points.push_back(p);
  p.x = t[0] + scale * z[0];
  p.y = t[1] + scale * z[1];
  p.z = t[2] + scale * z[2];
  marker.points.push_back(p);

  std_msgs::msg::ColorRGBA color;
  color.a = alpha;

  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  return marker;
}

visualization_msgs::msg::Marker
OutputHelper::markerPoints(const std::vector<Eigen::Vector3d> &points,
                           const std::string &ns, const int id,
                           const std::string &frame_id, const double scale,
                           const std::vector<std::array<double, 3>> &colors) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;
  marker.scale.y = scale;

  for (const auto &point : points) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    marker.points.push_back(p);
  }

  if (colors.empty()) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  } else {
    for (const auto &color : colors) {
      std_msgs::msg::ColorRGBA c;
      c.r = color[0];
      c.g = color[1];
      c.b = color[2];
      c.a = 1.0;
      marker.colors.push_back(c);
    }
  }

  return marker;
}

visualization_msgs::msg::Marker
OutputHelper::markerLineStrip(const std::vector<Eigen::Vector3d> &points,
                              const std::string &ns, const int id,
                              const std::string &frame_id, const double scale,
                              const std::array<double, 3> &color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;

  for (const auto &point : points) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    marker.points.push_back(p);
  }

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::msg::Marker
OutputHelper::markerTrajectory(const std::vector<SE3<double>> &trajectory,
                               const std::string &ns, const int id,
                               const std::string &frame_id, const double scale,
                               const std::array<double, 3> &color) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(trajectory.size());
  for (const auto &pose : trajectory) {
    points.push_back(pose.translation());
  }
  return markerLineStrip(points, ns, id, frame_id, scale, color);
}

visualization_msgs::msg::MarkerArray
OutputHelper::markerFrames(const std::vector<SE3<double>> &frames,
                           const std::string &ns, const int id_start,
                           const double scale, const double alpha) {
  visualization_msgs::msg::MarkerArray markers;
  int id = id_start;
  for (const auto &frame : frames) {
    markers.markers.push_back(markerFrame(frame, ns, id++, scale, alpha));
  }
  return markers;
}

} // namespace output_helper
} // namespace vk
