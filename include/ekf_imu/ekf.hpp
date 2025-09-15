#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>

class EKF {
 public:
  EKF();

  Eigen::Vector4d initial_state(const Eigen::Vector3d &acc,
                                const Eigen::Vector3d &mag);
  Eigen::Vector4d initial_state_6dof(const Eigen::Vector3d &acc);
  Eigen::Quaterniond update(const Eigen::Vector3d &gyr,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &mag, double dt);
  Eigen::Vector3d q2euler(const Eigen::Quaterniond &q) const;

 private:
  std::array<double, 3> noises_;
  Eigen::Vector4d x_check_;
  Eigen::Matrix4d P_check_;
  Eigen::Vector4d x_hat_;
  Eigen::Matrix4d P_hat_;

  Eigen::Matrix3d Q_;
  Eigen::Matrix3d R_6dof_;
  Eigen::Matrix<double, 6, 6> R_;

  Eigen::Vector3d g_;
  Eigen::Vector3d r_;
  Eigen::Quaterniond q_offset;

  Eigen::Vector4d f(const Eigen::Vector4d &x_hat, const Eigen::Vector3d &gyr,
                    double dt) const;
  Eigen::Matrix4d F(const Eigen::Vector3d &gyr, double dt) const;
  Eigen::Matrix<double, 4, 3> W(const Eigen::Vector4d &x_hat, double dt) const;
  Eigen::Matrix<double, 6, 1> h(const Eigen::Vector4d &x_check) const;
  Eigen::Matrix<double, 6, 4> H(const Eigen::Vector4d &x_check) const;
  Eigen::Matrix<double, 3, 1> h_6dof(const Eigen::Vector4d &x_check) const;
  Eigen::Matrix<double, 3, 4> H_6dof(const Eigen::Vector4d &x_check) const;

  template <int N>
  Eigen::Matrix<double, N, 1> normalize(
      const Eigen::Matrix<double, N, 1> &v) const;
  Eigen::Matrix3d skew(const Eigen::Vector3d &v) const;
  Eigen::Matrix4d Omega(const Eigen::Vector3d &w) const;
  Eigen::Matrix3d q2R(const Eigen::Vector4d &q) const;
};

#endif