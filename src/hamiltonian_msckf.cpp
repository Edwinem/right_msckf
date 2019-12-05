//
// Created by niko on 11/26/19.
//

#include "hamiltonian_msckf.h"

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
State::State() {

}
State::State(int num_cam_clones) : num_cam_clones(num_cam_clones) {

  int state_size_clones = 7 * num_cam_clones;
  int state_size = state_size_clones + 13;

  state_vec.resize(state_size);
  state_vec.setZero();

  state_vec[3] = 1.0; //quat set to identity
}
Eigen::Quaterniond State::GetQuat() {
  return Eigen::Quaterniond(state_vec.data() + quat_idx);
}
Vec3d State::GetGyroBias() {
  return state_vec.segment(bg_idx, 3);
}
Vec3d State::GetAccelBias() {
  return state_vec.segment(ba_idx, 3);
}
Vec3d State::GetPos() {
  return state_vec.segment(pos_idx, 3);
}
Vec3d State::GetVel() {
  return state_vec.segment(vel_idx, 3);
}
Eigen::Quaterniond State::GetCloneQuat(int n_clone) {
  assert(n_clone <= num_cam_clones);

  return Eigen::Quaterniond(state_vec.data() + clone_start_idx + 7 * n_clone);
}
Vec3d State::GetClonePos(int n_clone) {
  assert(n_clone <= num_cam_clones);

  return state_vec.segment(clone_start_idx + 7 * n_clone + 4, 3);
}
}