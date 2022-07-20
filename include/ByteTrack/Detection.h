#pragma once
#include "ByteTrack/Rect.h"

#include <memory>

namespace byte_track {

class DetectionBase;
using DetectionPtr = std::shared_ptr<DetectionBase>;

class DetectionBase {
 public:
  virtual ~DetectionBase() = default;

  virtual const RectBase &rect() const = 0;
  virtual float score() const = 0;

  virtual void set_rect(const RectBase &rect) = 0;
  virtual void set_score(float score) = 0;
};

class Detection : public DetectionBase {
  TlwhRect rect_;
  float score_ = 0;

 public:
  Detection(const TlwhRect &rect, float score);

  const TlwhRect &rect() const override;
  float score() const override;

  void set_rect(const RectBase &rect) override;
  void set_score(float score) override;
};

}  // namespace byte_track
