/*
 * math_utils.h
 *
 *  Created on: Jul 20, 2012
 *      Author: cforster
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>

namespace vk {

using namespace std;
// Remove the using namespace Eigen to avoid ambiguity
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using namespace Sophus;

Vector3d triangulateFeatureNonLin(const Matrix3d &R, const Vector3d &t,
                                  const Vector3d &feature1,
                                  const Vector3d &feature2);

/// Assumes the bearing vectors f_c and f_r are on the epipolar plane, i.e.
/// perfect triangulation without noise!
bool depthFromTriangulationExact(const Matrix3d &R_r_c, const Vector3d &t_r_c,
                                 const Vector3d &f_r, const Vector3d &f_c,
                                 double &depth_in_r, double &depth_in_c);

double reprojError(const Vector3d &f1, const Vector3d &f2,
                   double error_multiplier2);

double computeInliers(const vector<Vector3d> &features1,
                      const vector<Vector3d> &features2, const Matrix3d &R,
                      const Vector3d &t, const double reproj_thresh,
                      double error_multiplier2, vector<Vector3d> &xyz_vec,
                      vector<int> &inliers, vector<int> &outliers);

void computeInliersOneView(const vector<Vector3d> &feature_sphere_vec,
                           const vector<Vector3d> &xyz_vec, const Matrix3d &R,
                           const Vector3d &t, const double reproj_thresh,
                           const double error_multiplier2, vector<int> &inliers,
                           vector<int> &outliers);

//! Direct Cosine Matrix to Roll Pitch Yaw
Vector3d dcm2rpy(const Matrix3d &R);

//! Roll Pitch Yaw to Direct Cosine Matrix
Matrix3d rpy2dcm(const Vector3d &rpy);

//! Angle Axis parametrization to Quaternion
Quaterniond angax2quat(const Vector3d &n, const double &angle);

//! Angle Axis parametrization to Matrix representation
Matrix3d angax2dcm(const Vector3d &n, const double &angle);

double sampsonusError(const Vector2d &v2Dash, const Matrix3d &m3Essential,
                      const Vector2d &v2);

inline Matrix3d sqew(const Vector3d &v) {
  Matrix3d v_sqew;
  v_sqew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return v_sqew;
}

inline double norm_max(const VectorXd &v) {
  double max = -1;
  for (int i = 0; i < v.size(); i++) {
    double abs = fabs(v[i]);
    if (abs > max) {
      max = abs;
    }
  }
  return max;
}

inline Vector2d project2d(const Vector3d &v) { return v.head<2>() / v[2]; }

inline Vector3d unproject2d(const Vector2d &v) {
  return Vector3d(v[0], v[1], 1.0);
}

inline Vector3d project3d(const Vector4d &v) { return v.head<3>() / v[3]; }

inline Vector4d unproject3d(const Vector3d &v) {
  return Vector4d(v[0], v[1], v[2], 1.0);
}

template <class T> T getMedian(vector<T> &data_vec) {
  assert(!data_vec.empty());
  typename vector<T>::iterator it =
      data_vec.begin() + floor(data_vec.size() / 2);
  nth_element(data_vec.begin(), it, data_vec.end());
  return *it;
}

inline double pyrFromZero_d(double x_0, int level) {
  return x_0 / (1 << level); // = 1 / 2^level
}

inline Vector2d pyrFromZero_2d(const Vector2d &uv_0, int level) {
  return Vector2d(pyrFromZero_d(uv_0[0], level), pyrFromZero_d(uv_0[1], level));
}

inline void frameJac_xyz2uv(const Vector3d &xyz, const double &focal_length,
                            Eigen::Matrix<double, 2, 6> &frame_jac) {
  const double x = xyz[0];
  const double y = xyz[1];
  const double z = xyz[2];
  const double z_2 = z * z;

  frame_jac(0, 0) = -1. / z * focal_length;
  frame_jac(0, 1) = 0;
  frame_jac(0, 2) = x / z_2 * focal_length;
  frame_jac(0, 3) = x * y / z_2 * focal_length;
  frame_jac(0, 4) = -(1 + (x * x / z_2)) * focal_length;
  frame_jac(0, 5) = y / z * focal_length;

  frame_jac(1, 0) = 0;
  frame_jac(1, 1) = -1. / z * focal_length;
  frame_jac(1, 2) = y / z_2 * focal_length;
  frame_jac(1, 3) = (1 + y * y / z_2) * focal_length;
  frame_jac(1, 4) = -x * y / z_2 * focal_length;
  frame_jac(1, 5) = -x / z * focal_length;
}

/// Compute the skew symmetric matrix of a 3D vector.
Eigen::Matrix3d skew(const Eigen::Vector3d &v);

/// Compute the cross product matrix of two 3D vectors.
Eigen::Matrix3d cross(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

/// Compute the rotation matrix from a rotation vector using Rodrigues formula.
Eigen::Matrix3d rodrigues2matrix(const Eigen::Vector3d &v);

/// Compute the rotation vector from a rotation matrix using Rodrigues formula.
Eigen::Vector3d matrix2rodrigues(const Eigen::Matrix3d &R);

/// Compute the rotation matrix from euler angles.
Eigen::Matrix3d rpy2dcm(const Eigen::Vector3d &rpy);

/// Compute euler angles from a rotation matrix.
Eigen::Vector3d dcm2rpy(const Eigen::Matrix3d &R);

/// Compute the rotation matrix from a quaternion.
Eigen::Matrix3d quaternion2dcm(const Eigen::Vector4d &q);

/// Compute a quaternion from a rotation matrix.
Eigen::Vector4d dcm2quaternion(const Eigen::Matrix3d &R);

/// Compute the SE3 transformation from a rotation matrix and translation
/// vector.
SE3<double> transformation(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

/// Compute the SE3 exponential map.
SE3<double> exp_se3(const Eigen::Matrix<double, 6, 1> &xi);

/// Compute the SE3 logarithmic map.
Eigen::Matrix<double, 6, 1> log_se3(const SE3<double> &T);

} // end namespace vk

#endif /* MATH_UTILS_H_ */
