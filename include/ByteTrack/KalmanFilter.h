#pragma once

#include "ByteTrack/Rect.h"

#include "Eigen/Dense"

namespace byte_track {
class KalmanFilter {
 public:
  template <int rows, int cols>
  using Matrix = Eigen::Matrix<float, rows, cols, Eigen::RowMajor>;

  KalmanFilter(float std_weight_position = 1. / 20,
               float std_weight_velocity = 1. / 160);

  void initiate(const TlwhRect& measurement);

  TlwhRect predict(bool mean_eight_to_zero);

  TlwhRect update(const TlwhRect& measurement);

 private:
  float std_weight_position_;
  float std_weight_velocity_;

  Matrix<8, 8> motion_mat_;
  Matrix<4, 8> update_mat_;

  Matrix<1, 8> mean_;
  Matrix<8, 8> covariance_;

  void project(Matrix<1, 4>& projected_mean,
               Matrix<4, 4>& projected_covariance);

  Matrix<1, 4> rect_to_xyah(const TlwhRect& rect) const;

  TlwhRect xyah_to_tlwh(const Matrix<1, 4>& xyah) const;
};
}  // namespace byte_track
