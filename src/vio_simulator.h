#pragma once


#include <unordered_map>
#include <random>
#include <Eigen/Core>



using Vec3d=Eigen::Vector3d;
using Vec2d=Eigen::Vector2d;
using Vec6d=Eigen::Matrix<double,6,1>;
using VecXd=Eigen::Matrix<double,Eigen::Dynamic,1>;
using Mat33d=Eigen::Matrix3d;
using Mat66d=Eigen::Matrix<double,6,6>;
using MatXd=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>;
using Mat16x16d=Eigen::Matrix<double,16,16>;
using Mat16x12d=Eigen::Matrix<double,16,12>;

using unordered_map_vec3d= std::unordered_map<int64_t, Vec3d,
        std::hash<int64_t>, std::equal_to<int64_t>,
        Eigen::aligned_allocator<std::pair<const int64_t, Vec3d> > >;

class VIOSimulator {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


     unordered_map_vec3d map_id_3d_point;




    double freq_imu;
    double freq_cam;

    double imu_cam_offset;

    Vec3d gravity;

    std::mt19937 gen_meas_imu;

    /// Mersenne twister PRNG for measurements (CAMERAS)
    std::vector<std::mt19937> gen_meas_cams;

    /// Mersenne twister PRNG for state initialization
    std::mt19937 gen_state_init;

    /// Mersenne twister PRNG for state perturbations
    std::mt19937 gen_state_perturb;







};



