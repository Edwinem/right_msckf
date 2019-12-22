#include "core.h"

namespace msckf{

Eigen::Quaterniond RotVec2Quat(const Vec3d &rot_vec) {
  double theta = rot_vec.norm();
  if (theta < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }

  Eigen::Quaterniond quat;
  quat.w() = cos(theta / 2);
  double scale = sin(theta / 2);
  Vec3d copy = rot_vec / theta;
  quat.x() = scale * copy[0];
  quat.y() = scale * copy[1];
  quat.z() = scale * copy[2];
  return quat;

}
Mat33d CreateSkew(const Vec3d &input) {
  Mat33d output = Mat33d::Zero();
  output(0, 0) = 0;
  output(0, 1) = -input(2);
  output(0, 2) = input(1);
  output(1, 0) = input(2);
  output(1, 1) = 0;
  output(1, 2) = -input(0);
  output(2, 0) = -input(1);
  output(2, 1) = input(0);
  output(2, 2) = 0;
  return output;
}
Mat44d BuildTransform(const Eigen::Quaterniond &quat, const Vec3d &trans) {
  Mat44d T=Mat44d::Identity();
  T.block<3,3>(0,0)=quat.toRotationMatrix();
  T.block<3,1>(0,3)=trans;
  return T;

}
Mat44d InvertTransform(const Mat44d &transform) {
  Mat44d Tinv = Mat44d::Identity();
  Tinv.block<3,3>(0,0) = transform.block<3,3>(0,0).transpose();
  Tinv.block<3,1>(0,3) = -Tinv.block<3,3>(0,0)*transform.block<3,1>(0,3);
  return Tinv;
}
Mat44d Omega(const Vec3d& gyro_meas) {
  Mat44d omega;
  double wx = gyro_meas(0);
  double wy = gyro_meas(1);
  double wz = gyro_meas(2);
  omega <<
        0, -wx, -wy, -wz,
      wx, 0, wz, -wy,
      wy, -wz, 0, wx,
      wz, wy, -wx, 0;
  return omega;

}

Mat44d OmegaEigenQuat(const Vec3d& gyro_meas) {
  Mat44d omega;
  double wx = gyro_meas(0);
  double wy = gyro_meas(1);
  double wz = gyro_meas(2);
  omega <<
        -wx, -wy, -wz, 0,
      0, wz, -wy, wx,
      -wz, 0, wx, wy,
      wy, -wx, 0, wz;
  return omega;

}

}