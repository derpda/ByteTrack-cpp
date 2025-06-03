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

  virtual void update(const byte_track::DetectionBase &new_det) = 0;
};

class Detection : public DetectionBase {
  TlwhRect rect_;
  float score_ = 0;

 public:
  Detection(const TlwhRect &rect, float score);

  const TlwhRect &rect() const override;
  float score() const override;

  void update(const byte_track::DetectionBase &new_det) override;
};

}  // namespace byte_track
