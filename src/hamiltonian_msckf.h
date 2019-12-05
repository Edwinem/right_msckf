

#pragma once

#include <Eigen/Geometry>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <list>

namespace msckf {

using Vec3d=Eigen::Vector3d;
using Vec2d=Eigen::Vector2d;
using Vec6d=Eigen::Matrix<double, 6, 1>;
using VecXd=Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Mat33d=Eigen::Matrix3d;
using Mat66d=Eigen::Matrix<double, 6, 6>;
using MatXd=Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Mat15x15d=Eigen::Matrix<double, 15, 15>;
using Mat15x12d=Eigen::Matrix<double, 15, 12>;

const double deg2rad = M_PI / 180;

Mat33d CreateSkew(const Vec3d &input);

Eigen::Quaterniond RotVec2Quat(const Vec3d &rot_vec);

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

struct State {

  int num_cam_clones;

  // order follows MSCKF paper
  // quat,b_g,vel,b_a,pos,clone1(quat,pos),clone2(quat,pos),...,cloneN(quat,pos)
  VecXd state_vec;

  MatXd imu_covar; //covariance of quat-imu_pos
  MatXd cams_covar;

  MatXd P;

  std::list<CamState> cam_states;

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

  // ---------------------------- IMU ---------------------------------------------------
  double sigma_gyr{1e-4};
  double sigma_acc{1e-2};
  double sigma_bg_drift{3.6733e-5};
  double sigma_ba_drift{7e-4};
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

    MatXd Q(12, 12);

    MatXd P = Phi * state.imu_covar * Phi.transpose() + G_k * Q * G_k.transpose();

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

  bool CheckLandMarkValidity(const Landmark &l) {
    return false;
  }

  void Marginilize() {
    if (feature_ids_to_remove.size() == 0) {
      return;
    }

    for (const auto feature_id:feature_ids_to_remove) {
      auto iter = map_id_landmark.find(feature_id);
      assert(iter != map_id_landmark.end());

      Landmark m(std::move(iter->second));

      map_id_landmark.erase(iter);

      bool valid = CheckLandMarkValidity(m);

      if (!valid) {
        continue;
      }

    }
  }

  /*
  void MeasurementUpdate(const MatXd &H_o, const MatXd &r_o, const MatXd &R_o) {
    if (r_o.size() != 0) {
      // Build MSCKF covariance matrix
      MatXd P = MatXd::Zero(15 + cam_covar_.rows(), 15 + cam_covar_.cols());
      P.template block<15, 15>(0, 0) = imu_covar_;
      if (cam_covar_.rows() != 0) {
        P.block(0, 15, 15, imu_cam_covar_.cols()) = imu_cam_covar_;
        P.block(15, 0, imu_cam_covar_.cols(), 15) = imu_cam_covar_.transpose();
        P.block(15, 15, cam_covar_.rows(), cam_covar_.cols()) = cam_covar_;
      }

      MatXd T_H, Q_1, R_n;
      VecXd r_n;

      // Put residuals in update-worthy form
      // Calculates T_H matrix according to Mourikis 2007
      HouseholderQR <MatXd> qr(H_o);
      MatXd Q = qr.householderQ();
      MatXd R = qr.matrixQR().template triangularView<Upper>();

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

      // Update IMU state (from updateState matlab function defined in MSCKF.m)
      Quaterniond q_IG_up = buildUpdateQuat(deltaX.template head<3>()) * imu_state_.q_IG;

      imu_state_.q_IG = q_IG_up;

      imu_state_.b_g += deltaX.template segment<3>(3);
      imu_state_.b_a += deltaX.template segment<3>(9);
      imu_state_.v_I_G += deltaX.template segment<3>(6);
      imu_state_.p_I_G += deltaX.template segment<3>(12);

      // Update Camera<_S> states
      for (size_t c_i = 0; c_i < cam_states_.size(); c_i++) {
        Quaterniond q_CG_up = buildUpdateQuat(deltaX.template segment<3>(15 + 6 * c_i)) *
            cam_states_[c_i].q_CG;
        cam_states_[c_i].q_CG = q_CG_up.normalized();
        cam_states_[c_i].p_C_G += deltaX.template segment<3>(18 + 6 * c_i);
      }

      // Covariance correction
      MatXd tempMat = MatXd::Identity(15 + 6 * cam_states_.size(),
                                      15 + 6 * cam_states_.size()) -
          K * T_H;

      MatXd P_corrected, P_corrected_transpose;
      P_corrected = tempMat * P * tempMat.transpose() + K * R_n * K.transpose();
      // Enforce symmetry
      P_corrected_transpose = P_corrected.transpose();
      P_corrected += P_corrected_transpose;
      P_corrected /= 2;

      if (P_corrected.rows() - 15 != cam_covar_.rows()) {
        std::cout << "[P:" << P_corrected.rows() << "," << P_corrected.cols() << "]";
        std::cout << "[cam_covar_:" << cam_covar_.rows() << "," << cam_covar_.cols() << "]";
        std::cout << std::endl;
      }

      // TODO : Verify need for eig check on P_corrected here (doesn't seem too
      // important for now)
      imu_covar_ = P_corrected.template block<15, 15>(0, 0);

      // TODO: Check here
      cam_covar_ = P_corrected.block(15, 15, P_corrected.rows() - 15,
                                     P_corrected.cols() - 15);
      imu_cam_covar_ = P_corrected.block(0, 15, 15, P_corrected.cols() - 15);

      return;
    } else
      return;
  }

   */
  void AugmentState() {

    int n_cams = state.cam_states.size();

    Eigen::Quaterniond cam_quat = state.GetQuat() * q_IMU_C; //
    cam_quat.normalize();

    Vec3d cam_pose = state.GetPos() + q_IMU_C * p_IMU_C;

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


    //Size is 6 since minimal Jacobian pose is 6(pos=3,quat=3)

    MatXd Jac = MatXd::Zero(6, 15 + 6 * state.cam_states.size());


    //Follows the format seen in Mourikis MSCKF Eq 16
    //Jacobians are with Body to Earth

    //Quaternion Jacs

    Jac.block<3, 3>(0, 0) = q_IMU_C.toRotationMatrix();



    //Jac of position w.r.t position
    Jac.block<3, 3>(3, state.pos_error_idx) = Mat33d::Identity();

    Mat33d s = CreateSkew(p_IMU_C);
    //Jac of position with respect to Body-Cam rotation eq cam_P=body_P+R_BC*IMU_CAM_P
    //derivative of this can be found in Bloesh primer
    Jac.block<3, 3>(3, state.quat_error_idx) = -state.GetQuat().toRotationMatrix() * s;

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
