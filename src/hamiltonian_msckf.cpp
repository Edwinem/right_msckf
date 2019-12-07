//
// Created by niko on 11/26/19.
//

#include "hamiltonian_msckf.h"

namespace msckf{

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




Vec2d ReprojectionError(const Eigen::Isometry3d &pose,
                        const Vec3d &pt_msckf,
                        const Vec2d &proj) {
  const double alpha = pt_msckf(0);
  const double beta = pt_msckf(1);
  const double rho = pt_msckf(2);

  Vec3d h = pose.linear() * Vec3d(alpha, beta, 1.0) + rho * pose.translation();

  Vec2d z_hat(h[0] / h[2], h[1] / h[2]);
  return (z_hat - proj);
}

void FeatureOptJacobian(const Eigen::Isometry3d &T_c0_ci, const Vec3d &x,
                        const Vec2d &z, Eigen::Matrix<double,2,3> &J) {

  // Compute hi1, hi2, and hi3 as Equation (37).
  const double &alpha = x(0);
  const double &beta = x(1);
  const double &rho = x(2);

  Vec3d h =
      T_c0_ci.linear() * Vec3d(alpha, beta, 1.0) + rho * T_c0_ci.translation();
  double &h1 = h(0);
  double &h2 = h(1);
  double &h3 = h(2);

  // Compute the Jacobian.
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  W.leftCols<2>() = T_c0_ci.linear().leftCols<2>();
  W.rightCols<1>() = T_c0_ci.translation();

  J = MatXd::Zero(2, 3);
  J.row(0) = 1 / h3 * W.row(0) - h1 / (h3 * h3) * W.row(2);
  J.row(1) = 1 / h3 * W.row(1) - h2 / (h3 * h3) * W.row(2);

//  // Compute the residual.
//  Vec2d z_hat(h1 / h3, h2 / h3);
//  r = z_hat - z;


//  // Compute the weight based on the residual.
//  double e             = r.norm();
//  double huber_epsilon = 0.01;
//  if (e <= huber_epsilon) {
//    w = 1.0;
//  } else {
//    w = huber_epsilon / (2 * e);
//  }
}

bool TriangulateNViewETH(const std::vector<Vec2d,Eigen::aligned_allocator<Vec2d>>& projs,
    const std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>>& cam_pos, Vec3d& final_pos) {

  int                                      num_projs = projs.size();
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  Eigen::VectorXd                          b;
  A.resize(2 * num_projs, Eigen::NoChange);
  b.resize(2 * num_projs);

  for (int i = 0; i < num_projs; ++i) {
    Eigen::Isometry3d C_T_G = cam_pos[i].inverse();
    Eigen::Matrix3d   C_R_G = C_T_G.rotation();
    const Eigen::Vector3d& C_p_G = C_T_G.translation();

    A.row(2 * i + 0) = projs[i](0) * C_R_G.row(2) - C_R_G.row(0);
    A.row(2 * i + 1) = projs[i](1) * C_R_G.row(2) - C_R_G.row(1);

    b(2 * i + 0) = -(projs[i](0) * C_p_G(2) - C_p_G(0));
    b(2 * i + 1) = -(projs[i](1) * C_p_G(2) - C_p_G(1));
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  final_pos = svd.solve(b);

  Eigen::MatrixXd singularValues;
  singularValues.resize(svd.singularValues().rows(), 1);
  singularValues = svd.singularValues();
  double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);
  if (std::abs(condA) > 1000) {
    return false;
  }

  return true;
}

bool TriangulateNViewRefine(const std::vector<Vec2d,Eigen::aligned_allocator<Vec2d>>& projs,
                         const std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>>& cam_pos, Vec3d& pos) {


  return true;
}


bool HamiltonianMSCKF::TriangulateFeature(Landmark_To_Residualize& track) {

  //Find cameras with the feature id

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      camera_poses;

  std::vector<Vec2d,Eigen::aligned_allocator<Vec2d>> valid_projs;


  auto& cam_states=track.connected_cams;

  for(int idx=0; idx<track.landmark.projs.size(); ++idx){
    auto& l=track.landmark;
    int64_t cam_id=l.camera_ids[idx];
    if(cam_states.find(cam_id)!=cam_states.end()){
      valid_projs.push_back(l.projs[idx]);
      //Convert Quat,pos to 4x4 rigid body Transform -> Eigen::Isometry3d
      Mat44d pose;
      pose.block<3,3>(0,0)=cam_states[cam_id].quat.toRotationMatrix();
      pose.block<3,1>(0,3)=cam_states[cam_id].pos;
      camera_poses.emplace_back(Eigen::Isometry3d(pose));
    }
  }

  //Convert the poses so that they are relative to the first pose so that
  // pose[0] is the identity matrix
  Eigen::Isometry3d T_c0_w = camera_poses[0];
  for (auto &pose : camera_poses) {
    pose = pose.inverse() * T_c0_w;
  }

  //First use SVD triangulation to get initial guess
  Vec3d initial_guess;
  bool valid=TriangulateNViewETH(valid_projs,camera_poses,initial_guess);
  if(!valid){
    return false;
  }


  //Now refine triangulation with Gauss Newton

  valid =TriangulateNViewRefine(valid_projs,camera_poses,initial_guess);

  track.landmark.pos_W=initial_guess;
  track.landmark.have_pos=valid;
  return valid;







}
}