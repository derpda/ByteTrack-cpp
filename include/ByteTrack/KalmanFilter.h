#pragma once

#include "ByteTrack/Rect.h"

#include "Eigen/Dense"

namespace byte_track {
class KalmanFilter {
 public:
  using DetectBox = Xyah;

  using StateMean = Eigen::Matrix<float, 1, 8, Eigen::RowMajor>;
  using StateCov = Eigen::Matrix<float, 8, 8, Eigen::RowMajor>;

  using StateHMean = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;
  using StateHCov = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

  KalmanFilter(float std_weight_position = 1. / 20,
               float std_weight_velocity = 1. / 160);

  void initiate(const DetectBox& measurement);

  void predict(bool mean_eight_to_zero);

  Xyah update(const DetectBox& measurement);

 private:
  float std_weight_position_;
  float std_weight_velocity_;

  Eigen::Matrix<float, 8, 8, Eigen::RowMajor> motion_mat_;
  Eigen::Matrix<float, 4, 8, Eigen::RowMajor> update_mat_;

  StateMean mean_;
  StateCov covariance_;

  void project(StateHMean& projected_mean, StateHCov& projected_covariance);
};
}  // namespace byte_track
