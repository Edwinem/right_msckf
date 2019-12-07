#pragma once


#include <Eigen/Geometry>

namespace msckf {

using Vec3d=Eigen::Vector3d;
using Vec2d=Eigen::Vector2d;
using Vec6d=Eigen::Matrix<double, 6, 1>;
using VecXd=Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Mat33d=Eigen::Matrix3d;
using Mat44d=Eigen::Matrix4d;
using Mat66d=Eigen::Matrix<double, 6, 6>;
using MatXd=Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Mat15x15d=Eigen::Matrix<double, 15, 15>;
using Mat15x12d=Eigen::Matrix<double, 15, 12>;

const double deg2rad = M_PI / 180;

Mat33d CreateSkew(const Vec3d &input);

Eigen::Quaterniond RotVec2Quat(const Vec3d &rot_vec);

template<typename Scalar>
constexpr Scalar Square(Scalar val) {
  return val * val;
}


Mat44d BuildTransform(const Eigen::Quaterniond& quat,const Vec3d& trans);

Mat44d InvertTransform(const Mat44d& transform);

}
