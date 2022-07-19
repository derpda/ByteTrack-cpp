#pragma once

#include "Eigen/Dense"

#include <memory>

namespace byte_track {
using Xyah = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

class Rect {
 public:
  virtual ~Rect() = default;

  virtual const float& top() const = 0;
  virtual const float& left() const = 0;
  virtual const float& width() const = 0;
  virtual const float& height() const = 0;

  virtual float& top();
  virtual float& left();
  virtual float& width();
  virtual float& height();

  Xyah get_xyah() const;

  void set_from_xyah(const Xyah& xyah);
};

float calc_iou(const Rect& A, const Rect& B);
}  // namespace byte_track
