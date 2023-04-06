#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_map>

#include "utils.h"

using namespace std;
using namespace cv;


using point2D_t = uint32_t;
using point3D_t = uint64_t;
const point3D_t kInvalidPoint3DId = std::numeric_limits<point3D_t>::max();

class Point2D {
public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2D()
    : xy_(Eigen::Vector2d::Zero()), point3D_id_(kInvalidPoint3DId) {}

  // The coordinate in image space in pixels.
  inline const Eigen::Vector2d& XY() const;
  inline Eigen::Vector2d& XY();
  inline double X() const;
  inline double Y() const;
  inline void SetXY(const Eigen::Vector2d& xy);

  // The identifier of the observed 3D point. If the image point does not
  // observe a 3D point, the identifier is `kInvalidPoint3Did`.
  inline point3D_t Point3DId() const;
  inline bool HasPoint3D() const;
  inline void SetPoint3DId(const point3D_t point3D_id);

 private:
  // The image coordinates in pixels, starting at upper left corner with 0.
  Eigen::Vector2d xy_;

  // The identifier of the 3D point. If the 2D point is not part of a 3D point
  // track the identifier is `kInvalidPoint3DId` and `HasPoint3D() = false`.
  point3D_t point3D_id_;
};
////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector2d& Point2D::XY() const { return xy_; }

Eigen::Vector2d& Point2D::XY() { return xy_; }

double Point2D::X() const { return xy_.x(); }

double Point2D::Y() const { return xy_.y(); }

void Point2D::SetXY(const Eigen::Vector2d& xy) { xy_ = xy; }

point3D_t Point2D::Point3DId() const { return point3D_id_; }

bool Point2D::HasPoint3D() const { return point3D_id_ != kInvalidPoint3DId; }

void Point2D::SetPoint3DId(const point3D_t point3D_id) {
  point3D_id_ = point3D_id;
}



class Image{

public:


 private:
  // Identifier of the image, if not specified `kInvalidImageId`.
  image_t image_id_;

  // The name of the image, i.e. the relative path.
  std::string name_;

  // The identifier of the associated camera. Note that multiple images might
  // share the same camera. If not specified `kInvalidCameraId`.
  camera_t camera_id_;


  // The number of 3D points the image observes, i.e. the sum of its `points2D`
  // where `point3D_id != kInvalidPoint3DId`.
  point2D_t num_points3D_;

  // The number of image points that have at least one correspondence to
  // another image.
  point2D_t num_observations_;

  // The sum of correspondences per image point.
  point2D_t num_correspondences_;

  // The number of 2D points, which have at least one corresponding 2D point in
  // another image that is part of a 3D point track, i.e. the sum of `points2D`
  // where `num_tris > 0`.
  point2D_t num_visible_points3D_;

  // The pose of the image, defined as the transformation from world to image.
  Eigen::Vector4d qvec_;
  Eigen::Vector3d tvec_;

  // The pose prior of the image, e.g. extracted from EXIF tags.
  Eigen::Vector4d qvec_prior_;
  Eigen::Vector3d tvec_prior_;

  std::vector<class Point2D> points2D_;

  // Per image point, the number of correspondences that have a 3D point.
  std::vector<point2D_t> num_correspondences_have_point3D_;

  // Data structure to compute the distribution of triangulated correspondences
  // in the image. Note that this structure is only usable after `SetUp`.
//   VisibilityPyramid point3D_visibility_pyramid_;

};