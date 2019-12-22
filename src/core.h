#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <unordered_map>

namespace msckf {

using Vec3d=Eigen::Vector3d;
using Vec2d=Eigen::Vector2d;
using Vec4d=Eigen::Vector4d;
using Vec6d=Eigen::Matrix<double, 6, 1>;
using VecXd=Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Mat33d=Eigen::Matrix3d;
using Mat44d=Eigen::Matrix4d;
using Mat66d=Eigen::Matrix<double, 6, 6>;
using MatXd=Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Mat15x15d=Eigen::Matrix<double, 15, 15>;
using Mat15x12d=Eigen::Matrix<double, 15, 12>;


using VectorVec3d=std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>>;
using VectorVec2d=std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>>;

template<typename KeyType, typename ValueType>
struct AlignedUnorderedMap {
  typedef std::unordered_map<KeyType, ValueType,
                             std::hash<KeyType>, std::equal_to<KeyType>,
                             Eigen::aligned_allocator<std::pair<const KeyType, ValueType> > > type;
};

const double deg2rad = M_PI / 180;

Mat33d CreateSkew(const Vec3d& input);

Eigen::Quaterniond RotVec2Quat(const Vec3d& rot_vec);

template<typename Scalar>
constexpr Scalar Square(Scalar val) {
  return val * val;
}

/**
 * Creates a 4x4 homogenous Transform
 * @param quat
 * @param trans
 * @return
 */
Mat44d BuildTransform(const Eigen::Quaterniond& quat, const Vec3d& trans);

/**
 * Inverts a homogenous transform efficiently. And is numerically safer
 */
Mat44d InvertTransform(const Mat44d& transform);

/**
 * Equation 198 from Sola
 * @param gyro_meas
 * @return
 */
Mat44d Omega(const Vec3d& gyro_meas);

/**
 * Creates Omega for Eigen::Quaternion which is x,y,z,w
 */
Mat44d OmegaEigenQuat(const Vec3d& gyro_meas);

}
