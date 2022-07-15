#include "ByteTrack/KalmanFilter.h"

#include "ByteTrack/Rect.h"

#include <cstddef>

namespace byte_track {
KalmanFilter::KalmanFilter(float std_weight_position, float std_weight_velocity)
    : std_weight_position_(std_weight_position),
      std_weight_velocity_(std_weight_velocity) {
  constexpr size_t ndim = 4;
  constexpr float dt = 1;

  motion_mat_ = Eigen::MatrixXf::Identity(8, 8);
  update_mat_ = Eigen::MatrixXf::Identity(4, 8);

  for (size_t i = 0; i < ndim; i++) {
    motion_mat_(i, ndim + i) = dt;
  }
}

void KalmanFilter::initiate(const DetectBox &measurement) {
  mean_.block<1, 4>(0, 0) = measurement.block<1, 4>(0, 0);
  mean_.block<1, 4>(0, 4) = Eigen::Vector4f::Zero();

  StateMean std;
  std(0) = 2 * std_weight_position_ * measurement[3];
  std(1) = 2 * std_weight_position_ * measurement[3];
  std(2) = 1e-2;
  std(3) = 2 * std_weight_position_ * measurement[3];
  std(4) = 10 * std_weight_velocity_ * measurement[3];
  std(5) = 10 * std_weight_velocity_ * measurement[3];
  std(6) = 1e-5;
  std(7) = 10 * std_weight_velocity_ * measurement[3];

  StateMean tmp = std.array().square();
  covariance_ = tmp.asDiagonal();
}

void KalmanFilter::predict(bool mean_eight_to_zero) {
  if (mean_eight_to_zero) mean_[7] = 0;
  StateMean std;
  std(0) = std_weight_position_ * mean_(3);
  std(1) = std_weight_position_ * mean_(3);
  std(2) = 1e-2;
  std(3) = std_weight_position_ * mean_(3);
  std(4) = std_weight_velocity_ * mean_(3);
  std(5) = std_weight_velocity_ * mean_(3);
  std(6) = 1e-5;
  std(7) = std_weight_velocity_ * mean_(3);

  StateMean tmp = std.array().square();
  StateCov motion_cov = tmp.asDiagonal();

  mean_ = motion_mat_ * mean_.transpose();
  covariance_ =
      motion_mat_ * covariance_ * (motion_mat_.transpose()) + motion_cov;
}

Xyah KalmanFilter::update(const DetectBox &measurement) {
  StateHMean projected_mean;
  StateHCov projected_cov;
  project(projected_mean, projected_cov);

  Eigen::Matrix<float, 4, 8> B =
      (covariance_ * (update_mat_.transpose())).transpose();
  Eigen::Matrix<float, 8, 4> kalman_gain =
      (projected_cov.llt().solve(B)).transpose();
  Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean;

  const auto tmp = innovation * (kalman_gain.transpose());
  mean_ = (mean_.array() + tmp.array()).matrix();
  covariance_ =
      covariance_ - kalman_gain * projected_cov * (kalman_gain.transpose());
  return Xyah(mean_.block<1, 4>(0, 0));
}

void KalmanFilter::project(StateHMean &projected_mean,
                           StateHCov &projected_covariance) {
  DetectBox std;
  std << std_weight_position_ * mean_(3), std_weight_position_ * mean_(3), 1e-1,
      std_weight_position_ * mean_(3);

  projected_mean = update_mat_ * mean_.transpose();
  projected_covariance = update_mat_ * covariance_ * (update_mat_.transpose());

  Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
  projected_covariance += diag.array().square().matrix();
}
}  // namespace byte_track
