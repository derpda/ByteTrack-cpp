#pragma once

#include "Eigen/Dense"

namespace byte_track {
using Tlwh = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

using Tlbr = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

using Xyah = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

class Rect {
 public:
  Tlwh tlwh;

  Rect() = default;
  Rect(float x, float y, float width, float height);

  ~Rect();

  float x() const;
  float y() const;
  float width() const;
  float height() const;

  float &x();
  float &y();
  float &width();
  float &height();

  float left() const;
  float top() const;
  float right() const;
  float bottom() const;

  Tlbr getTlbr() const;
  Xyah getXyah() const;

  float calcIoU(const Rect &other) const;
};

Rect generate_rect_by_tlbr(const Tlbr &tlbr);

Rect generate_rect_by_xyah(const Xyah &xyah);

}  // namespace byte_track
