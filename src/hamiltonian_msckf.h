

#pragma once

#include <Eigen/Geometry>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <list>
#include <deque>

#include "core.h"

namespace msckf {

struct IMUMeas {
  Vec3d acc;
  Vec3d gyro;
};

struct Feature2D {
  int64_t id = 0;

  Vec2d norm_coord; //img coordinates in normalized form
};

struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int64_t id = 0; // same as Feature
  std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> projs;
  std::vector<int64_t> camera_ids;

  int64_t cam_id_first;
  int64_t cam_id_last;

  int GetNumTracks() {
    return projs.size();
  }

  Vec3d pos_W; //Position in World
  bool have_pos = false;

};

struct CamState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int64_t cam_id = -1;
  double time = 0;
  std::vector<int64_t> landmark_ids;

  Vec3d pos;
  Eigen::Quaterniond quat;

};

struct Landmark_To_Residualize {
  Landmark landmark;
  std::unordered_map<int64_t, CamState> connected_cams;
};

struct State {

  int num_cam_clones;

  // order follows MSCKF paper
  // quat,b_g,vel,b_a,pos,clone1(quat,pos),clone2(quat,pos),...,cloneN(quat,pos)
  VecXd state_vec;

  MatXd imu_covar; //covariance of quat-imu_pos
  MatXd cams_covar;
  
  
  MatXd imu_cam_covar;

  MatXd P;

  std::deque<CamState> cam_states;

  //state indexes to get values
  const int quat_idx = 0;
  const int bg_idx = 4;
  const int vel_idx = 7;
  const int ba_idx = 10;
  const int pos_idx = 13;
  const int clone_start_idx = 16;

  //indexes in error state. Almost exact same except the quaternion error state is of size 3
  const int quat_error_idx = 0;
  const int bg_error_idx = 3;
  const int vel_error_idx = 6;
  const int ba_error_idx = 9;
  const int pos_error_idx = 12;

  State();

  State(int num_cam_clones);

  Eigen::Quaterniond GetQuat();

  Vec3d GetGyroBias();

  Vec3d GetAccelBias();

  Vec3d GetPos();

  Vec3d GetVel();

  Eigen::Quaterniond GetCloneQuat(int n_clone);

  Vec3d GetClonePos(int n_clone);

};

class HamiltonianMSCKF {
public:

  State state;

  int64_t latest_cam_id;

  //Camera offsets
  Eigen::Quaterniond q_IMU_C;
  Vec3d p_IMU_C;

  int max_track_length = 10;

  double total_time = 0;

  // ---------------------------- IMU ---------------------------------------------------
  double sigma_gyr_2 = Square(1e-4);
  double sigma_acc_2 = Square(1e-2);
  double sigma_bg_drift_2 = Square(3.6733e-5);
  double sigma_ba_drift_2 = Square(7e-4);
  double tau_bg{50.0};
  double tau_ba{100.0};

  // Initial Uncertainty
  double sigma_pos_init{1e-12};
  double sigma_vel_init{1e-6};
  double sigma_att_init{1e-6 * deg2rad};
  double sigma_ba_init{1e-6};
  double sigma_bg_init{3.6733e-5};

  double tau_acc_bias{0};
  double tau_gyr_bias{0};
  Vec3d pos_imu_b_B{Vec3d::Zero()};

  VecXd noise_vec;

  // Matrices
  Eigen::Matrix<double, 12, 12> R_imu{Eigen::Matrix<double, 12, 12>::Identity()};

  // ---------------------------- VO and LCVO ---------------------------------------------
  double sigma_vo_feature_u{0.01};  //FIXME get this value
  double sigma_vo_feature_v{0.01};  //FIXME get this value
  double sigma_lcvo_pos{0.05};
  double sigma_lcvo_att{2 * deg2rad};

  // Adaptation in MSC-EKF
  double sigma_vo_fu_ad{0.01};
  double sigma_vo_fv_ad{0.01};
  VecXd error_vo_pred{VecXd::Zero(2, 1)};


  //================================ MSC EKF Necessary Matrices ==========================================================

  //------------ Propagate --------------------------

  Mat15x15d F_k;

  Mat15x12d G_k;

  Mat15x15d Phi_k;

  Mat15x15d Q_k;

  Mat15x15d Phi_k_propagated;

  //--------------- Update --------------------------

  MatXd S_k;

  MatXd H_k;

  MatXd R_k;

  MatXd K_k;

  VecXd residual;

  VecXd delta_state;

  std::unordered_map<int64_t, Landmark> map_id_landmark;

  std::unordered_set<int64_t> feature_ids_to_remove;

  void Propogate(const IMUMeas &meas, double dt) {

    Vec3d acc_meas = meas.acc;
    Vec3d gyro_meas = meas.gyro;

    Vec3d unbiased_accel_B = acc_meas - state.GetAccelBias();
    Vec3d unbiased_gyro_B = gyro_meas - state.GetGyroBias();

    Mat33d rot_matrix = state.GetQuat().toRotationMatrix();






    //RK4







  }

  Vec3d grav;

  void Update();

  void
  CalcIMUDerivatives(State &x, Vec3d &pos_dx, Vec3d &vel_dx, Eigen::Quaterniond &quat_dx, Vec3d &unbiased_acc,
                     Vec3d &unbiased_gyro) {
    // From equations 236 in Sola Page 54

    pos_dx = x.GetVel();
    vel_dx = x.GetQuat().toRotationMatrix() * (unbiased_acc) + grav;
    //quat_dx=x.GetQuat()*unbiased_gyro*0.5;


  }

  void CalcProcessCovDerivatives(State &x, MatXd &F, Vec3d &unbiased_acc, Vec3d &unbiased_gyro) {

    MatXd F_k(18, 18);

    Mat33d gyro_skew = CreateSkew(unbiased_gyro);
    Mat33d acc_skew = CreateSkew(unbiased_acc);

    Mat33d R = x.GetQuat().toRotationMatrix();

    F_k.setZero();

    //All the Identity jacobians
    F_k.block<3, 3>(x.pos_error_idx, x.pos_error_idx) = Mat33d::Identity();
    F_k.block<3, 3>(x.vel_error_idx, x.vel_error_idx) = Mat33d::Identity();
    F_k.block<3, 3>(x.bg_error_idx, x.ba_error_idx) = Mat33d::Identity();
    F_k.block<3, 3>(x.ba_error_idx, x.ba_error_idx) = Mat33d::Identity();


    //Velocity Jacobians

    //Jacobian velocity w.r.t Rotation body to earth
    F_k.block<3, 3>(state.vel_error_idx, state.quat_error_idx) = -R * acc_skew;
    //Jacobian velocity w.r.t. bias_acc
    F_k.block<3, 3>(state.vel_error_idx, state.ba_error_idx) = -R;

    //Rotation Jacobians


    //Jacobian rotation w.r.t quat
    F_k.block<3, 3>(state.quat_error_idx, state.quat_error_idx) = -gyro_skew;

    F_k.block<3, 3>(state.quat_error_idx, state.bg_error_idx) = -Mat33d::Identity();

    double dt;// FIXEME
    MatXd Fdt = F * dt;
    auto Fdt2 = Fdt * Fdt;
    auto Fdt3 = Fdt2 * Fdt;

    MatXd Phi = Fdt + 0.5 * Fdt2 + (1. / 6.) * Fdt3;


    // See equation 246 in Sola. To see why we can ignore rotation in accelerometer noise
    MatXd G_k(15, 12);

    G_k.setZero();

    G_k.block<3, 3>(0, 0) = Mat33d::Identity();

    MatXd Qc(12, 12);

    double dt2 = dt * dt; //According to Sola
    Qc.block(0, 0, 3, 3) = sigma_gyr_2 / dt2 * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(3, 3, 3, 3) = sigma_bg_drift_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(6, 6, 3, 3) = sigma_acc_2 / dt2 * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(9, 9, 3, 3) = sigma_ba_drift_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();

    MatXd P = Phi * state.imu_covar * Phi.transpose() + G_k * Qc * G_k.transpose();

  }

  void UpdateFeatures(const std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> &projs,
                      const std::vector<int64_t> &ids) {

    assert(ids.size() == projs.size());

    for (int idx = 0; idx < projs.size(); ++idx) {
      const int64_t id = ids[idx];
      const Vec2d &proj = projs[idx];

      auto iter = map_id_landmark.find(id);

      //Means new feature. Landmark is created
      if (iter == map_id_landmark.end()) {
        Landmark l;
        l.id = id;
        l.projs.push_back(proj);
        l.cam_id_first = latest_cam_id + 1;
        l.cam_id_last = latest_cam_id + 1;

        auto &cur_cam_state = (state.cam_states.back());
        cur_cam_state.landmark_ids.emplace_back(id);

      } else {
        auto &landmark = iter->second;
        landmark.projs.push_back(proj);
        auto &cur_cam_state = (state.cam_states.back());
        cur_cam_state.landmark_ids.emplace_back(id);
        landmark.cam_id_last = cur_cam_state.cam_id;
      }

    }


    //Check if any Features have reached max length and mark them to be removed
    for (auto &iter: map_id_landmark) {
      auto &landmark = iter.second;

      if (landmark.projs.size() > max_track_length) {
        feature_ids_to_remove.insert(landmark.id);
      }

    }

  }

  bool CheckLandMarkValidity(const Landmark_To_Residualize &l) {
    return true;
  }

  void Marginilize() {

    int total_obs = 0;

    if (feature_ids_to_remove.size() == 0) {
      return;
    }

    std::vector<Landmark_To_Residualize> valid_tracks;

    for (const auto feature_id:feature_ids_to_remove) {
      auto iter = map_id_landmark.find(feature_id);
      assert(iter != map_id_landmark.end());

      Landmark_To_Residualize track;

      Landmark l = std::move(iter->second);
      map_id_landmark.erase(iter);





      //Find all cameras connected to this landmark. Also remove any projections that don't have a camera state.
      // This can happen if the camera state has been pruned
      std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> valid_projs;
      std::vector<int64_t> valid_cam_ids;
      for (int idx = 0; idx < l.camera_ids.size(); ++idx) {
        int cam_id = l.camera_ids[idx];
        for (auto &cam:state.cam_states) {
          if (cam_id == cam.cam_id) {
            valid_projs.emplace_back(l.projs[idx]);
            valid_cam_ids.emplace_back(cam_id);
            track.connected_cams[cam.cam_id] = cam;
            break;
          }

        }
      }

      //Copy over landmark info and valid projs to track
      track.landmark = l;
      track.landmark.projs = valid_projs;
      track.landmark.camera_ids = valid_cam_ids;

      bool valid = CheckLandMarkValidity(track);

      if (!valid) {
        continue;
      }

      bool triang_result = TriangulateFeature(track);

      if (triang_result) {
        valid_tracks.push_back(track);
      }

    }

    if (valid_tracks.empty()) {
      return;
    }

    int num_valid = valid_tracks.size();

    MatXd H_o = MatXd::Zero(2 * total_obs - 3 * num_valid,
                            15 + 6 * state.cam_states.size());
    MatXd R_o = MatXd::Zero(2 * total_obs - 3 * num_valid,
                            2 * total_obs - 3 * num_valid);
    VecXd r_o(2 * total_obs - 3 * num_valid);

    for (auto &track:valid_tracks) {

      //Calculate Residual

      std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> residuals = ComputeTrackResidual(track);

      MatXd H_o_j, A;

      ComputeJacobians(track);

    }

  }

  std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> ComputeTrackResidual(Landmark_To_Residualize &track) {

    std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> residuals(track.landmark.projs.size());

    int iter = 0;
    for (int idx = 0; idx < track.landmark.projs.size(); ++idx) {
      const auto &obs = track.landmark.projs[idx];
      auto cam_id = track.landmark.camera_ids[idx];

      const Vec3d &pt3D = track.landmark.pos_W;
      const auto &cam_state = track.connected_cams[cam_id];

      Mat44d T_wc = BuildTransform(cam_state.quat, cam_state.pos);
      Mat44d T_cw = InvertTransform(T_wc);
      Vec3d p_f_C = T_wc.block<3, 3>(0, 0) * pt3D + T_wc.block<3, 1>(0, 3);
      Vec2d projection = p_f_C.template head<2>() / p_f_C(2);

      residuals[idx] = obs - projection;
    }

    return residuals;
  }

  void ComputeJacobians(Landmark_To_Residualize &track) {

    int n_obs = track.landmark.projs.size();

    MatXd H_f_j = MatXd::Zero(2 * n_obs, 3);
    MatXd H_x_j =
        MatXd::Zero(2 * n_obs, 15 + 6 * state.cam_states.size());


    Mat33d R_wi=state.GetQuat().toRotationMatrix(); //Rotation of IMU from World
    Mat33d R_ic=q_IMU_C.toRotationMatrix(); //Rotation of Camera from IMU


    for (int c_i = 0; c_i < n_obs; c_i++) {
      const int64_t cam_id = track.landmark.camera_ids[c_i];

      const Vec3d &pt3D = track.landmark.pos_W;
      const auto &camera_state = track.connected_cams[cam_id];

      Vec3d p_f_C = camera_state.quat.toRotationMatrix() *
          (pt3D - camera_state.pos);

      double X, Y, Z;

      X = p_f_C(0);
      Y = p_f_C(1);
      Z = p_f_C(2);

      //See Jacobian in MSCKF paper. Also is same in VINS-Mono Projection Factor
      Eigen::Matrix<double, 2, 3> J_i;
      J_i << 1, 0, -X / Z,
             0, 1, -Y / Z;
      J_i *= 1 / Z;



      // P_c= T_cw*P_w and T_cw= T_ci*T_iw
      // P_c = R_cw*P_w+ t_wc
      //

      Eigen::Matrix<double, 3, 6> dPcam_dstate = Eigen::Matrix<double, 3, 6>::Zero();

      // Jacobian w.r.t. Camera Orientation
      dPcam_dstate.leftCols(3) = R_ic.transpose()*R_wi.transpose()* -CreateSkew(camera_state.pos);

      // Jacobian w.r.t. Camera Position
      dPcam_dstate.rightCols(3) = -(R_ic.transpose()*R_wi.transpose());


      MatXd Jac=J_i*dPcam_dstate;


//      // Enforce observability constraint, see propagation for citation
//      Eigen::Matrix<double,2,6> A;
//      A << J_i * vectorToSkewSymmetric(p_f_C),
//          -J_i * cam_states_[index].q_CG.toRotationMatrix();
//
//      Eigen::Matrix<double,6,1> u = Matrix<_S, 6, 1>::Zero();
//      u.head(3) = cam_states_[index].q_CG.toRotationMatrix() * imu_state_.g;
//      Vector3<_S> tmp = p_f_G - cam_states_[index].p_C_G;
//      u.tail(3) = vectorToSkewSymmetric(tmp) * imu_state_.g;
//
//      Matrix<_S, 2, 6> H_x =
//          A - A * u * (u.transpose() * u).inverse() * u.transpose();
//      Matrix<_S, 2, 3> H_f = -H_x.template block<2, 3>(0, 3);
//      H_f_j.template block<2, 3>(2 * c_i, 0) = H_f;
//
//      // Potential indexing problem zone
//      H_x_j.template block<2, 6>(2 * c_i, 15 + 6 * (index)) = H_x;
//    }
//
//    int jacobian_row_size = 2 * camStateIndices.size();
//
//    JacobiSVD<MatrixX<_S>> svd_helper(H_f_j, ComputeFullU | ComputeThinV);
//    A_j = svd_helper.matrixU().rightCols(jacobian_row_size - 3);
//
//    H_o_j = A_j.transpose() * H_x_j;
    }

  }

  bool TriangulateFeature(Landmark_To_Residualize &track);

  /*
  void MeasurementUpdate(const MatXd &H_o, const MatXd &r_o, const MatXd &R_o) {
    if (r_o.size() != 0) {
      // Build MSCKF covariance matrix
      MatXd P = MatXd::Zero(15 + state.cams_covar.rows(), 15 + state.cams_covar.cols());
      P.template block<15, 15>(0, 0) = state.imu_covar;
      if (state.cams_covar.rows() != 0) {
        P.block(0, 15, 15, state.imu_cam_covar.cols()) = state.imu_cam_covar;
        P.block(15, 0, state.imu_cam_covar.cols(), 15) = state.imu_cam_covar.transpose();
        P.block(15, 15, state.cams_covar.rows(), state.cams_covar.cols()) = state.cams_covar;
      }

      MatXd T_H, Q_1, R_n;
      VecXd r_n;

      // Put residuals in update-worthy form
      // Calculates T_H matrix according to Mourikis 2007
      Eigen::HouseholderQR <MatXd> qr(H_o);
      MatXd Q = qr.householderQ();
      MatXd R = qr.matrixQR().template triangularView<Eigen::Upper>();

      VecXd nonZeroRows = R.rowwise().any();
      int numNonZeroRows = nonZeroRows.sum();

      T_H = MatXd::Zero(numNonZeroRows, R.cols());
      Q_1 = MatXd::Zero(Q.rows(), numNonZeroRows);

      size_t counter = 0;
      for (size_t r_ind = 0; r_ind < R.rows(); r_ind++) {
        if (nonZeroRows(r_ind) == 1.0) {
          T_H.row(counter) = R.row(r_ind);
          Q_1.col(counter) = Q.col(r_ind);
          counter++;
          if (counter > numNonZeroRows) {
            //ROS_ERROR("More non zero rows than expected in QR decomp");
          }
        }
      }

      r_n = Q_1.transpose() * r_o;
      R_n = Q_1.transpose() * R_o * Q_1;

      // Calculate Kalman Gain
      MatXd temp = T_H * P * T_H.transpose() + R_n;
      MatXd K = (P * T_H.transpose()) * temp.inverse();

      // State Correction
      VecXd deltaX = K * r_n;

//      // Update IMU state (from updateState matlab function defined in MSCKF.m)
//      Eigen::Quaterniond q_IG_up = buildUpdateQuat(deltaX.template head<3>()) * state..q_IG;
//
//      imu_state_.q_IG = q_IG_up;
//
//      imu_state_.b_g += deltaX.template segment<3>(3);
//      imu_state_.b_a += deltaX.template segment<3>(9);
//      imu_state_.v_I_G += deltaX.template segment<3>(6);
//      imu_state_.p_I_G += deltaX.template segment<3>(12);
//
//      // Update Camera<_S> states
//      for (size_t c_i = 0; c_i < cam_states_.size(); c_i++) {
//        Quaterniond q_CG_up = buildUpdateQuat(deltaX.template segment<3>(15 + 6 * c_i)) *
//            cam_states_[c_i].q_CG;
//        cam_states_[c_i].q_CG = q_CG_up.normalized();
//        cam_states_[c_i].p_C_G += deltaX.template segment<3>(18 + 6 * c_i);
//      }
//
//      // Covariance correction
//      MatXd tempMat = MatXd::Identity(15 + 6 * cam_states_.size(),
//                                      15 + 6 * cam_states_.size()) -
//          K * T_H;
//
//      MatXd P_corrected, P_corrected_transpose;
//      P_corrected = tempMat * P * tempMat.transpose() + K * R_n * K.transpose();
//      // Enforce symmetry
//      P_corrected_transpose = P_corrected.transpose();
//      P_corrected += P_corrected_transpose;
//      P_corrected /= 2;
//
//      if (P_corrected.rows() - 15 != state.cams_covar.rows()) {
//        std::cout << "[P:" << P_corrected.rows() << "," << P_corrected.cols() << "]";
//        std::cout << "[state.cams_covar:" << state.cams_covar.rows() << "," << state.cams_covar.cols() << "]";
//        std::cout << std::endl;
//      }
//
//      // TODO : Verify need for eig check on P_corrected here (doesn't seem too
//      // important for now)
//      state.imu_covar = P_corrected.template block<15, 15>(0, 0);
//
//      // TODO: Check here
//      state.cams_covar = P_corrected.block(15, 15, P_corrected.rows() - 15,
//                                     P_corrected.cols() - 15);
//      state.imu_cam_covar = P_corrected.block(0, 15, 15, P_corrected.cols() - 15);

      return;
    } else
      return;
  }
*/
   
  void AugmentState() {

    int n_cams = state.cam_states.size();


    // T_wc = T_wi*T_ic
    // w=world,c=camera,i=imu/body
    // T=Rt

    // From above
    // Q_wc=Q_wi*Q_ic
    Eigen::Quaterniond cam_quat = state.GetQuat() * q_IMU_C; //
    cam_quat.normalize(); // ensure it is unit quaternion

    // t_wc= t_wi+ R_wi*t_ic
    Vec3d cam_pose = state.GetPos() + state.GetQuat() * p_IMU_C;

    CamState cam;

    cam.pos = cam_pose;
    cam.quat = cam_quat;

    state.cam_states.push_back(cam);

    if (state.cam_states.size() > 0) {
      state.P.resize(15 + state.cams_covar.rows(), 15 + state.cams_covar.cols());

      state.P.block<15, 15>(0, 0) = state.imu_covar;
      state.P.block(0, 15, 15, state.cams_covar.cols()) = state.imu_covar;
      state.P.block(15, 0, state.cams_covar.rows(), 15) = state.imu_covar.transpose();

      state.P.block(15, 15, state.cams_covar.rows(), state.cams_covar.cols()) = state.cams_covar;

    } else {
      state.P = state.imu_covar;
    }


    //Size is 6 since minimal Jacobian pose is 6(quat=3,pos=3)

    MatXd Jac = MatXd::Zero(6, 15 + 6 * state.cam_states.size());


    //Follows the format seen in Mourikis MSCKF Eq 16
    //Jacobians are with Body to Earth using Q_wc=Q_wi*Q_ic and t_wc= t_wi+ R_wi*t_ic




    //Quaternion Jacs
    // Jac of cam_quat w.r.t imu_quat
    Jac.block<3, 3>(0, state.quat_error_idx) = q_IMU_C.toRotationMatrix();


    //Position Jac

    //Jac of cam_position with respect to imu_quat d/dR(R_wi*t_ic)
    //derivative of this can be found in Bloesh primer
    Mat33d s = CreateSkew(p_IMU_C);
    Jac.block<3, 3>(3, state.quat_error_idx) = -state.GetQuat().toRotationMatrix() * s;

    //Jac of cam_position w.r.t position
    Jac.block<3, 3>(3, state.pos_error_idx) = Mat33d::Identity();

    MatXd tempMat = MatXd::Identity(15 + 6 * n_cams + 6,
                                    15 + 6 * n_cams);
    tempMat.block(15 + 6 * n_cams, 0, 6,
                  15 + 6 * n_cams) = Jac;

    MatXd P_aug = tempMat * state.P * tempMat.transpose();

    MatXd P_aug_sym = (P_aug + P_aug.transpose()) / 2.0;

    state.cam_states.push_back(cam);

    state.imu_covar = P_aug_sym.block<15, 15>(0, 0);

    state.cams_covar.resize(P_aug_sym.rows() - 15, P_aug_sym.cols() - 15);
    state.cams_covar = P_aug.block(15, 15, P_aug_sym.rows() - 15, P_aug_sym.cols() - 15);

  }

};



//void RungeKutta4(std::function<VecXd(const VecXd&,const VecXd&,double)> compute_xdot,VecXdVecXd& out){
//
//    VecXd k1=compute_xdot()
//
//
//}

}
