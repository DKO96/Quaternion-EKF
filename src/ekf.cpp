#include "ekf_imu/ekf.hpp"

#include <math.h>

EKF::EKF(std::array<double, 3> noises) : noises_(noises) {
  x_check_.setZero();
  P_check_.setZero();
  x_hat_.setZero();
  P_hat_.setZero();

  const double sigma_g2 = noises_[0] * noises_[0];
  const double sigma_a2 = noises_[1] * noises_[1];
  const double sigma_m2 = noises_[2] * noises_[2];

  Q_.setIdentity();
  Q_ *= sigma_g2;

  R_.setZero();
  R_.block<3, 3>(0, 0).setIdentity();
  R_.block<3, 3>(0, 0) *= sigma_a2;
  R_.block<3, 3>(3, 3).setIdentity();
  R_.block<3, 3>(3, 3) *= sigma_m2;

  g_ << 0.0, 0.0, 1.0;

  const double theta = -0.17715091907742445;
  r_ << std::cos(theta), 0.0, std::sin(theta);
}

template <int N>
Eigen::Matrix<double, N, 1> EKF::normalize(
    const Eigen::Matrix<double, N, 1> &v) const {
  return v / v.norm();
}

Eigen::Matrix3d EKF::skew(const Eigen::Vector3d &v) const {
  Eigen::Matrix3d S;
  S(0, 0) = 0.0;
  S(0, 1) = -v(2);
  S(0, 2) = v(1);

  S(1, 0) = v(2);
  S(1, 1) = 0.0;
  S(1, 2) = -v(0);

  S(2, 0) = -v(1);
  S(2, 1) = v(0);
  S(2, 2) = 0.0;

  return S;
}

Eigen::Matrix4d EKF::Omega(const Eigen::Vector3d &w) const {
  Eigen::Matrix4d W;
  W(0, 0) = 0.0;
  W(0, 1) = -w(0);
  W(0, 2) = -w(1);
  W(0, 3) = -w(2);

  W(1, 0) = w(0);
  W(1, 1) = 0.0;
  W(1, 2) = w(2);
  W(1, 3) = -w(1);

  W(2, 0) = w(1);
  W(2, 1) = -w(2);
  W(2, 2) = 0.0;
  W(2, 3) = w(0);

  W(3, 0) = w(2);
  W(3, 1) = w(1);
  W(3, 2) = -w(0);
  W(3, 3) = 0.0;

  return W;
}

Eigen::Matrix3d EKF::q2R(const Eigen::Vector4d &q) const {
  Eigen::Matrix3d R;
  R(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
  R(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
  R(0, 2) = 2 * (q(0) * q(2) + q(1) * q(3));

  R(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
  R(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
  R(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));

  R(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
  R(2, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
  R(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

  return R;
}

Eigen::Vector4d EKF::f(const Eigen::Vector4d &x_hat, const Eigen::Vector3d &gyr,
                       double dt) const {
  Eigen::Vector4d q_check = x_hat + 0.5 * dt * EKF::Omega(gyr) * x_hat;

  return EKF::normalize(q_check);
}

Eigen::Matrix4d EKF::F(const Eigen::Vector3d &gyr, double dt) const {
  return Eigen::Matrix4d::Identity() + 0.5 * dt * EKF::Omega(gyr);
}

Eigen::Matrix<double, 4, 3> EKF::W(const Eigen::Vector4d &x_hat,
                                   double dt) const {
  const double qw = x_hat(0);
  const Eigen::Vector3d qv = x_hat.tail<3>();

  Eigen::Matrix<double, 4, 3> W;

  W.row(0) = -0.5 * dt * qv.transpose();

  Eigen::Matrix3d block = qw * Eigen::Matrix3d::Identity() - EKF::skew(qv);

  W.block<3, 3>(1, 0) = 0.5 * dt * block;

  return W;
}

Eigen::Matrix<double, 6, 1> EKF::h(const Eigen::Vector4d &x_check) const {
  Eigen::Matrix<double, 6, 1> est_measurement;
  const Eigen::Matrix3d R = EKF::q2R(x_check);
  // Eigen::Vector3d a_check = R.transpose() * this->g_;
  // Eigen::Vector3d m_check = R.transpose() * this->r_;
  Eigen::Vector3d a_check = R * this->g_;
  Eigen::Vector3d m_check = R * this->r_;

  Eigen::Matrix<double, 6, 1> z_check;
  z_check << a_check, m_check;

  return z_check;
}

static Eigen::Matrix<double, 3, 4> temp_block(const Eigen::Vector3d &n,
                                              const Eigen::Vector4d &q) {
  Eigen::Matrix<double, 3, 4> T;

  T(0, 0) = 2 * (n(0) * q(0) + n(1) * q(3) - n(2) * q(2));
  T(0, 1) = 2 * (n(0) * q(1) + n(1) * q(2) + n(2) * q(3));
  T(0, 2) = 2 * (-n(0) * q(2) + n(1) * q(1) - n(2) * q(0));
  T(0, 3) = 2 * (-n(0) * q(3) + n(1) * q(0) + n(2) * q(1));

  T(1, 0) = 2 * (-n(0) * q(3) + n(1) * q(0) + n(2) * q(1));
  T(1, 1) = 2 * (-n(0) * q(2) - n(1) * q(1) + n(2) * q(0));
  T(1, 2) = 2 * (n(0) * q(1) + n(1) * q(2) + n(2) * q(3));
  T(1, 3) = 2 * (-n(0) * q(0) - n(1) * q(3) + n(2) * q(2));

  T(2, 0) = 2 * (n(0) * q(2) - n(1) * q(1) + n(2) * q(0));
  T(2, 1) = 2 * (n(0) * q(3) - n(1) * q(0) - n(2) * q(1));
  T(2, 2) = 2 * (n(0) * q(0) + n(1) * q(3) - n(2) * q(2));
  T(2, 3) = 2 * (n(0) * q(1) + n(1) * q(2) + n(2) * q(3));

  return T;
}

Eigen::Matrix<double, 6, 4> EKF::H(const Eigen::Vector4d &x_check) const {
  Eigen::Matrix<double, 6, 4> H;

  H.topRows<3>() = temp_block(this->g_, x_check);
  H.bottomRows<3>() = temp_block(this->r_, x_check);

  return H;
}

Eigen::Vector4d EKF::initial_state(const Eigen::Vector3d &acc,
                                   const Eigen::Vector3d &mag) {
  // Eigen::Vector3d H_norm = EKF::normalize(mag.cross(acc));
  // Eigen::Vector3d a_norm = EKF::normalize(acc);

  // Eigen::Vector3d M = a_norm.cross(H_norm);

  Eigen::Vector3d a_norm = EKF::normalize(acc);
  Eigen::Vector3d H_norm = EKF::normalize(a_norm.cross(mag));
  Eigen::Vector3d M = EKF::normalize(H_norm.cross(a_norm));

  Eigen::Matrix3d R;
  R(0, 0) = H_norm(0);
  R(1, 0) = H_norm(1);
  R(2, 0) = H_norm(2);

  R(0, 1) = M(0);
  R(1, 1) = M(1);
  R(2, 1) = M(2);

  R(0, 2) = a_norm(0);
  R(1, 2) = a_norm(1);
  R(2, 2) = a_norm(2);

  Eigen::Vector4d q;
  q(0) = 0.5 * std::sqrt(1.0 + R.trace());
  q(1) = 0.25 * (R(1, 2) - R(2, 1)) / q(0);
  q(2) = 0.25 * (R(2, 0) - R(0, 2)) / q(0);
  q(3) = 0.25 * (R(0, 1) - R(1, 0)) / q(0);

  this->x_hat_ = EKF::normalize(q);
  this->P_hat_ = Eigen::Matrix4d::Identity();

  return this->x_hat_;
}

Eigen::Vector4d EKF::update(const Eigen::Vector3d &gyr,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &mag, double dt) {
  // Prediction
  this->x_check_ = EKF::f(this->x_hat_, gyr, dt);

  Eigen::Matrix4d Fk = EKF::F(gyr, dt);
  Eigen::Matrix<double, 4, 3> Wk = EKF::W(this->x_hat_, dt);

  this->P_check_ =
      Fk * this->P_hat_ * Fk.transpose() + Wk * this->Q_ * Wk.transpose();

  // Correction
  Eigen::Matrix<double, 6, 1> z;
  z << EKF::normalize(acc), EKF::normalize(mag);

  Eigen::Matrix<double, 6, 1> innovation;
  innovation = z - EKF::h(this->x_check_);

  Eigen::Matrix<double, 6, 4> Hk = EKF::H(this->x_check_);

  Eigen::Matrix<double, 6, 6> S =
      Hk * this->P_check_ * Hk.transpose() + this->R_;

  Eigen::Matrix<double, 4, 6> Kk =
      this->P_check_ * Hk.transpose() * S.inverse();

  Eigen::Vector4d x_hat = this->x_check_ + Kk * innovation;

  this->x_hat_ = EKF::normalize(x_hat);
  this->P_hat_ = this->P_check_ - Kk * Hk * this->P_check_;

  return this->x_hat_;
}

Eigen::Vector3d EKF::q2euler(const Eigen::Vector4d &q) const {
  Eigen::Vector3d E;
  E(0) = std::atan2(2 * (q(0) * q(1) + q(2) * q(3)),
                    1 - 2 * (q(1) * q(1) + q(2) * q(2)));
  E(1) =
      -M_PI_2 + 2 * std::atan2(std::sqrt(1 + 2 * (q(0) * q(2) - q(1) * q(3))),
                               std::sqrt(1 - 2 * (q(0) * q(2) - q(1) * q(3))));
  E(2) = std::atan2(2 * (q(0) * q(3) + q(1) * q(2)),
                    1 - 2 * (q(2) * q(2) + q(3) * q(3)));

  return E;
}