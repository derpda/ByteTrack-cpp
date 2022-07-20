#pragma once

#include "Eigen/Dense"

#include <memory>

namespace byte_track {
using Xyah = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

class RectBase {
 public:
  virtual ~RectBase() = default;

  virtual float top() const = 0;
  virtual float left() const = 0;
  virtual float width() const = 0;
  virtual float height() const = 0;

  virtual void set_top(float top) = 0;
  virtual void set_left(float left) = 0;
  virtual void set_width(float width) = 0;
  virtual void set_height(float height) = 0;

  Xyah xyah() const;

  void set_from_xyah(const Xyah& xyah);
};

float calc_iou(const RectBase& A, const RectBase& B);

class TlwhRect : public RectBase {
  float top_;
  float left_;
  float width_;
  float height_;

 public:
  TlwhRect(float top = 0, float left = 0, float width = 0, float height = 0);

  virtual float top() const override;
  virtual float left() const override;
  virtual float width() const override;
  virtual float height() const override;

  virtual void set_top(float top) override;
  virtual void set_left(float left) override;
  virtual void set_width(float width) override;
  virtual void set_height(float height) override;
};

}  // namespace byte_track
