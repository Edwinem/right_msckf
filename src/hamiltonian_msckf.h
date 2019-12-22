

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

  bool use_observability_augmentation = false;


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

  double noise_pixel_sigma;

  void Propogate(const IMUMeas& meas, double dt) {

    Vec3d acc_meas = meas.acc;
    Vec3d gyro_meas = meas.gyro;

    Vec3d unbiased_accel_B = acc_meas - state.GetAccelBias();
    Vec3d unbiased_gyro_B = gyro_meas - state.GetGyroBias();

    Mat33d rot_matrix = state.GetQuat().toRotationMatrix();






    //RK4







  }

  Vec3d grav;

  void Update();

  void PropogateStateRK4(State& x, Vec3d& unbiased_acc, Vec3d& unbiased_gyro, Vec3d& gravity, double dt) {


    // Current state
    Eigen::Quaterniond q0 = state.GetQuat();
    Vec3d pos0 = state.GetPos();
    Vec3d vel0 = state.GetVel();
    Mat33d Rwb0 = q0.toRotationMatrix();

    //RK4

    //q_dot=0.5*q*w=0.5*omega(w)*q
    // v_dot=R*(acc_m-acc_b)+gravity
    // p_dot=v

    //k1
    Vec4d q1_dot = 0.5 * OmegaEigenQuat(unbiased_gyro) * q0.coeffs();
    Vec3d p1_dot = vel0;
    Vec3d v1_dot = Rwb0 * (unbiased_acc) + gravity;

    Vec4d q_k1 = q1_dot * dt;
    Vec3d p_k1 = p1_dot * dt;
    Vec3d v_k1 = v1_dot * dt;

    Vec4d q1 = q0.coeffs() + q_k1 * 0.5;
    Vec3d v1 = vel0 + v_k1 * 0.5;
//
    Eigen::Quaterniond q1_eig(q1.data());
    q1_eig.normalize();
//
//    //k2
    Vec4d q2_dot = 0.5 * OmegaEigenQuat(unbiased_gyro) * q1_eig.coeffs();
    Vec3d p2_dot = v1;
    Vec3d v2_dot = q1_eig.toRotationMatrix() * unbiased_acc + gravity;

    Vec4d q_k2 = q2_dot * dt;
    Vec3d p_k2 = p2_dot * dt;
    Vec3d v_k2 = v2_dot * dt;

    Vec4d q2 = q0.coeffs() + q_k2 * 0.5;
    Vec3d v2 = vel0 + v_k2 * 0.5;
//
    Eigen::Quaterniond q2_eig(q2.data());
    q2_eig.normalize();
    //k3
    Vec4d q3_dot = 0.5 * OmegaEigenQuat(unbiased_gyro) * q2_eig.coeffs();
    Vec3d p3_dot = v2;
    Vec3d v3_dot = q2_eig.toRotationMatrix() * unbiased_acc + gravity;

    Vec4d q_k3 = q3_dot * dt;
    Vec3d p_k3 = p3_dot * dt;
    Vec3d v_k3 = v3_dot * dt;

    Vec4d q3 = q0.coeffs() + q_k3;
    Vec3d v3 = vel0 + v_k3;

    Eigen::Quaterniond q3_eig(q3.data());
    q3_eig.normalize();

    Vec4d q4_dot = 0.5 * OmegaEigenQuat(unbiased_gyro) * q3_eig.coeffs();
    Vec3d p4_dot = v3;
    Vec3d v4_dot = q3_eig.toRotationMatrix() * unbiased_acc + gravity;

    Vec4d q_k4 = q4_dot * dt;
    Vec3d p_k4 = p4_dot * dt;
    Vec3d v_k4 = v4_dot * dt;


    // Sum

    Vec3d p = pos0 + (1 / 6.0) * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4);
    Vec3d v = vel0 + (1 / 6.0) * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4);
    Vec4d q = q0.coeffs() + (1 / 6.0) * (q_k1 + 2 * q_k2 + 2 * q_k3 + q_k4);
    Eigen::Quaterniond q_eig(q.data());
  }

  void
  CalcIMUDerivatives(State& x, Vec3d& pos_dx, Vec3d& vel_dx, Eigen::Quaterniond& quat_dx, Vec3d& unbiased_acc,
                     Vec3d& unbiased_gyro) {
    // From equations 236 in Sola Page 54

    pos_dx = x.GetVel();
    vel_dx = x.GetQuat().toRotationMatrix() * (unbiased_acc) + grav;
    //quat_dx=x.GetQuat()*unbiased_gyro*0.5;





//    Vec3 gyro = gyro_accel.head<3>();
//    Vec3 accel = gyro_accel.tail<3>();
//
//    Vec3 gyro_calib = imu_.Cg() * gyro - X.bg;   // \hat\omega in the doc
//    Vec3 accel_calib = imu_.Ca() * accel - X.ba; // \hat\alpha in the doc
//
//    // jacobian w.r.t. error state
//    Mat3 R = X.Rsb.matrix();
//
//    Eigen::Matrix<number_t, 3, 9> dW_dCg;
//    for (int i = 0; i < 3; ++i) {
//      // NOTE: use the raw measurement (gyro) here. NOT the calibrated one
//      // (gyro_calib)!!!
//      dW_dCg.block<1, 3>(i, 3 * i) = gyro;
//    }
//
//    Eigen::Matrix<number_t, 3, 9> dV_dRCa = dAB_dA<3, 3>(accel);
//    Eigen::Matrix<number_t, 9, 9> dRCa_dCafm = dAB_dB<3, 3>(R); // fm: full matrix
//    Eigen::Matrix<number_t, 9, 6> dCafm_dCa = dA_dAu<number_t, 3>(); // full matrix w.r.t. upper triangle
//    Eigen::Matrix<number_t, 3, 6> dV_dCa = dV_dRCa * dRCa_dCafm * dCafm_dCa;
//
//    Mat3 dW_dW = -hat(gyro_calib);
//    // static Mat3 dW_dbg = -I3;
//
//    // static Mat3 dT_dV = I3;
//
//    Mat3 dV_dW = -R * hat(accel_calib);
//    Mat3 dV_dba = -R;
//
//    Mat3 dV_dWg = -R * hat(g_); // effective dimension: 3x2, since Wg is 2-dim
//    // Mat2 dWg_dWg = Mat2::Identity();



  }

  void CalcProcessCovDerivatives(State& x, MatXd& F, Vec3d& unbiased_acc, Vec3d& unbiased_gyro) {

    MatXd F_k(18, 18);

    Mat33d gyro_skew = CreateSkew(unbiased_gyro);
    Mat33d acc_skew = CreateSkew(unbiased_acc);

    Mat33d R = x.GetQuat().toRotationMatrix();

    F_k.setZero();



    // Velocity Jacobians

    // Jacobian velocity w.r.t Rotation body to earth
    Mat33d dV_dtheta = -R * acc_skew;
    // Jacobian velocity w.r.t. bias_acc
    Mat33d dV_daccb = -R;
    // Rotation Jacobians

    Mat33d dQ_dtheta = -gyro_skew;
    Mat33d dQ_dgyrob = -Mat33d::Identity();

    // Position Jacobians
    Mat33d dP_dv = Mat33d::Identity();

    // Set the blocks in the F matrix
    F_k.block<3, 3>(x.pos_error_idx, x.vel_error_idx) = dP_dv;
    F_k.block<3, 3>(state.quat_error_idx, state.quat_error_idx) = dQ_dtheta;
    F_k.block<3, 3>(state.quat_error_idx, state.bg_error_idx) = dQ_dgyrob;
    F_k.block<3, 3>(state.vel_error_idx, state.quat_error_idx) = dV_dtheta;
    F_k.block<3, 3>(state.vel_error_idx, state.ba_error_idx) = dV_daccb;

    double dt;// FIXEME
    MatXd Fdt = F * dt;
    auto Fdt2 = Fdt * Fdt;
    auto Fdt3 = Fdt2 * Fdt;

    MatXd Phi = Fdt + 0.5 * Fdt2 + (1. / 6.) * Fdt3;

    MatXd G_k(15, 12);

    G_k.setZero();

    // Jacobians with respect to noise
    G_k.block<3, 3>(0, 0) = -Mat33d::Identity(); // dQ_dng
    G_k.block<3, 3>(3, 3) = Mat33d::Identity(); // dGyroBias_dng
    G_k.block<3, 3>(6, 6) = -R; // dV_dnacc
    G_k.block<3, 3>(9, 9) = Mat33d::Identity(); // dAccBias_dnacc


    MatXd Qc(12, 12);

    double dt2 = dt * dt; //According to Sola
    Qc.block(0, 0, 3, 3) = sigma_gyr_2 / dt2 * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(3, 3, 3, 3) = sigma_bg_drift_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(6, 6, 3, 3) = sigma_acc_2 / dt2 * Eigen::Matrix<double, 3, 3>::Identity();
    Qc.block(9, 9, 3, 3) = sigma_ba_drift_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();

    MatXd P = Phi * state.imu_covar * Phi.transpose() + G_k * Qc * G_k.transpose();

  }

  void UpdateFeatures(const std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>>& projs,
                      const std::vector<int64_t>& ids) {

    assert(ids.size() == projs.size());

    for (int idx = 0; idx < projs.size(); ++idx) {
      const int64_t id = ids[idx];
      const Vec2d& proj = projs[idx];

      auto iter = map_id_landmark.find(id);

      //Means new feature. Landmark is created
      if (iter == map_id_landmark.end()) {
        Landmark l;
        l.id = id;
        l.projs.push_back(proj);
        l.cam_id_first = latest_cam_id + 1;
        l.cam_id_last = latest_cam_id + 1;

        auto& cur_cam_state = (state.cam_states.back());
        cur_cam_state.landmark_ids.emplace_back(id);

      } else {
        auto& landmark = iter->second;
        landmark.projs.push_back(proj);
        auto& cur_cam_state = (state.cam_states.back());
        cur_cam_state.landmark_ids.emplace_back(id);
        landmark.cam_id_last = cur_cam_state.cam_id;
      }

    }


    //Check if any Features have reached max length and mark them to be removed
    for (auto& iter: map_id_landmark) {
      auto& landmark = iter.second;

      if (landmark.projs.size() > max_track_length) {
        feature_ids_to_remove.insert(landmark.id);
      }

    }

  }

  bool CheckLandMarkValidity(const Landmark_To_Residualize& l) {
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
        for (auto& cam:state.cam_states) {
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

    int matrix_index = 0; // The index to navigate the above matrices
    for (auto& track:valid_tracks) {

      int num_obs = track.landmark.projs.size();

      //Calculate Residual

      VecXd residuals = ComputeTrackResidual(track);

      // Noise Matrix
      MatXd R_j = MatXd::Identity(residuals.size() / 2, residuals.size() / 2) * noise_pixel_sigma;

      MatXd H_x_j, H_f_j;

      ComputeJacobians(track, H_x_j, H_f_j);

      // Perform the NullSpace Marginilization. Computes A as seen in MSCKF Eq 23
      Eigen::JacobiSVD<MatXd> svd_helper(H_f_j, Eigen::ComputeFullU | Eigen::ComputeThinV);
      MatXd A = svd_helper.matrixU().rightCols(H_f_j.rows() - 3);

      MatXd H_o_j = A.transpose() * H_x_j;
      VecXd r_o_j = A.transpose() * residuals;
      MatXd R_o_j = A.transpose() * R_j * A;

      assert(r_o_j.size() == (2 * num_obs - 3)); // See MSCKF paper right after Eq 24
      assert(H_o_j.rows() == r_o_j.rows());

      if (true) { // add the gating test here
        r_o.segment(matrix_index, r_o_j.size()) = r_o_j;
        H_o.block(matrix_index, 0, H_o_j.rows(), H_o_j.cols()) = H_o_j;
        R_o.block(matrix_index, matrix_index, R_o_j.rows(), R_o_j.cols()) = R_o_j;

        matrix_index += H_o_j.rows();
      }
    }
    H_o.conservativeResize(matrix_index, H_o.cols());
    r_o.conservativeResize(matrix_index);
    R_o.conservativeResize(matrix_index, matrix_index);

    // Perform Measurement Update
    MeasurementUpdate(H_o, r_o, R_o);

  }

  VecXd ComputeTrackResidual(Landmark_To_Residualize& track) {

    VecXd residuals(track.landmark.projs.size() * 2);

    int iter = 0;
    for (int idx = 0; idx < track.landmark.projs.size(); idx++) {
      const auto& obs = track.landmark.projs[idx];
      auto cam_id = track.landmark.camera_ids[idx];

      const Vec3d& pt3D = track.landmark.pos_W;
      const auto& cam_state = track.connected_cams[cam_id];

      Mat44d T_wc = BuildTransform(cam_state.quat, cam_state.pos);
      Mat44d T_cw = InvertTransform(T_wc);
      Vec3d p_f_C = T_wc.block<3, 3>(0, 0) * pt3D + T_wc.block<3, 1>(0, 3);
      Vec2d projection = p_f_C.template head<2>() / p_f_C(2);

      Vec2d res = obs - projection;
      residuals[2 * idx] = res[0];
      residuals[2 * idx + 1] = res[1];
    }

    return residuals;
  }

  void ComputeJacobians(const Landmark_To_Residualize& track, MatXd& H_x_j, MatXd& H_f_j) {

    int n_obs = track.landmark.projs.size();

    H_f_j = MatXd::Zero(2 * n_obs, 3);
    H_x_j = MatXd::Zero(2 * n_obs, 15 + 6 * state.cam_states.size());

    Mat33d R_wi = state.GetQuat().toRotationMatrix(); //Rotation of IMU from World
    Mat33d R_ic = q_IMU_C.toRotationMatrix(); //Rotation of Camera from IMU

    Mat33d R_wc = R_wi * R_ic;

    for (int c_i = 0; c_i < n_obs; c_i++) {
      const int64_t cam_id = track.landmark.camera_ids[c_i];

      const Vec3d& pt3D = track.landmark.pos_W;
      const auto& camera_state = track.connected_cams.at(cam_id); // need at for const

      // Find index of camera in current list
      int cam_index = -1;
      for (int idx = 0; idx < state.cam_states.size(); ++idx) {
        if (state.cam_states[idx].cam_id == cam_id) {
          cam_index = idx;
          break;
        }
      }
      assert(cam_index != -1);

      Vec3d p_f_C = camera_state.quat.toRotationMatrix() *
          (pt3D - camera_state.pos);

      double X, Y, Z;

      X = p_f_C(0);
      Y = p_f_C(1);
      Z = p_f_C(2);

      // See Jacobian in MSCKF paper. Also is same in VINS-Mono Projection Factor
      Eigen::Matrix<double, 2, 3> J_i;
      J_i << 1, 0, -X / Z,
          0, 1, -Y / Z;
      J_i *= 1 / Z;

      Eigen::Matrix<double, 3, 6> dPcam_dstate = Eigen::Matrix<double, 3, 6>::Zero();

      // Jacobian w.r.t. Camera Orientation
      dPcam_dstate.leftCols(3) = -R_wc.transpose();

      // Jacobian w.r.t. Camera Position
      dPcam_dstate.rightCols(3) = CreateSkew(p_f_C);

      Eigen::Matrix<double, 2, 6> Jac = J_i * dPcam_dstate;

      // Jacobian w.r.t to the point in world
      Eigen::Matrix<double, 2, 3> dPc_dPw = J_i * R_wc.transpose();

      if (use_observability_augmentation) {
        return;
      } else {
        H_f_j.block<2, 3>(2 * c_i, 0) = dPc_dPw;
        H_x_j.block<2, 6>(2 * c_i, 15 + 6 * (cam_index)) = Jac;
      }
    }
  }

  bool TriangulateFeature(Landmark_To_Residualize& track);

  void MeasurementUpdate(const MatXd& H_o, const MatXd& r_o, const MatXd& R_o) {
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
      Eigen::HouseholderQR<MatXd> qr(H_o);
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

  void AugmentState() {
    // T_wc = T_wi*T_ic
    // w=world,c=camera,i=imu/body
    // T=Rt
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

    // Size is 6 since minimal Jacobian pose is 6(quat=3,pos=3)

    MatXd Jac = MatXd::Zero(6, 15 + 6 * state.cam_states.size());

    // Follows the format seen in Mourikis MSCKF Eq 16
    // Jacobians are with Body to Earth using Q_wc=Q_wi*Q_ic and t_wc= t_wi+ R_wi*t_ic

    Mat33d dQC_dQI = q_IMU_C.toRotationMatrix(); // Jac of cam_quat w.r.t imu_quat

    Mat33d s = CreateSkew(p_IMU_C);
    Mat33d dPC_dQI = -state.GetQuat().toRotationMatrix() * s; // Jac of cam_position with respect to imu_quat
    // derivative of this can be found in Bloesh primer

    Mat33d dPC_dPI = Mat33d::Identity(); // Jac of cam_position w.r.t imu position

    Jac.block<3, 3>(0, state.quat_error_idx) = dQC_dQI;
    Jac.block<3, 3>(3, state.quat_error_idx) = dPC_dQI;
    Jac.block<3, 3>(3, state.pos_error_idx) = dPC_dPI;

    int n_cams = state.cam_states.size();

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

}
