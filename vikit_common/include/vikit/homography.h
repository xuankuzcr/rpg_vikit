/*
 * homography.cpp
 * Adaptation of PTAM-GPL HomographyInit class.
 * https://github.com/Oxford-PTAM/PTAM-GPL
 * Licence: GPLv3
 * Copyright 2008 Isis Innovation Limited
 *
 *  Created on: Sep 2, 2012
 *      by: cforster
 *
 * This class implements the homography decomposition of Faugeras and Lustman's
 * 1988 tech report. Code converted to Eigen from PTAM.
 *
 */

#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <vikit/math_utils.h>

namespace vk {

using namespace Eigen;
using namespace std;
using namespace Sophus;

struct HomographyDecomposition {
  Vector3d t;
  Matrix3d R;
  double d;
  Vector3d n;

  // Resolved  Composition
  SE3<double> T; //!< second from first
  int score;
};

/// Homography parameterized as position (center) and rotation.
class Homography {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Construct homography from position and rotation.
  static Eigen::Matrix3d from_position_and_rotation(const Eigen::Vector2d &pos,
                                                    double rotation);

  /// Construct homography from pose.
  static Eigen::Matrix3d from_pose(const SE3<double> &T_f_w,
                                   const double focal_length);

  /// Construct homography from pose and plane.
  static Eigen::Matrix3d from_pose_and_plane(const SE3<double> &T_f_w,
                                             const Eigen::Vector3d &normal,
                                             const double distance);

  Homography(const vector<Vector2d, aligned_allocator<Vector2d>> &_fts1,
             const vector<Vector2d, aligned_allocator<Vector2d>> &_fts2,
             double _error_multiplier2, double _thresh_in_px);

  void calcFromPlaneParams(const Vector3d &normal,
                           const Vector3d &point_on_plane);

  void calcFromMatches();

  size_t computeMatchesInliers();

  bool computeSE3fromMatches();

  bool decompose();

  void findBestDecomposition();

  double thresh;
  double error_multiplier2;
  const vector<Vector2d, aligned_allocator<Vector2d>>
      &fts_c1; //!< Features on first image on unit plane
  const vector<Vector2d, aligned_allocator<Vector2d>>
      &fts_c2; //!< Features on second image on unit plane
  vector<bool> inliers;
  SE3<double> T_c2_from_c1; //!< Relative translation and rotation of two images
  Matrix3d H_c2_from_c1;    //!< Homography
  vector<HomographyDecomposition> decompositions;
};

} /* end namespace vk */

#endif /* HOMOGRAPHY_H_ */
