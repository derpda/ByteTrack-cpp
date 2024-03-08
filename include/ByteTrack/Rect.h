#pragma once

namespace byte_track {

class RectBase {
 public:
  virtual ~RectBase() = default;

  virtual float top() const = 0;
  virtual float left() const = 0;
  virtual float width() const = 0;
  virtual float height() const = 0;
};

float calc_iou(const RectBase& A, const RectBase& B);

class TlwhRect : public RectBase {
  float top_;
  float left_;
  float width_;
  float height_;

 public:
  TlwhRect(float top = 0, float left = 0, float width = 0, float height = 0);
  TlwhRect(const RectBase& other);

  virtual float top() const override;
  virtual float left() const override;
  virtual float width() const override;
  virtual float height() const override;
};

}  // namespace byte_track
