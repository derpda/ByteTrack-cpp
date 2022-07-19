#pragma once
#include "ByteTrack/Rect.h"

#include <memory>

namespace byte_track {

class Detection;
using DetectionPtr = std::shared_ptr<Detection>;

class Detection {
 public:
  virtual ~Detection() = default;

  virtual const Rect& get_rect() const = 0;
  virtual const float& get_score() const = 0;

  virtual Rect& get_rect();
  virtual float& get_score();
};
}  // namespace byte_track
