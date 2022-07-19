#pragma once
#include "ByteTrack/Rect.h"

#include <memory>

namespace byte_track {

class Detection;
using DetectionPtr = std::shared_ptr<Detection>;

class Detection {
 public:
  virtual ~Detection() = default;

  virtual const Rect& getRect() const = 0;
  virtual const float& getScore() const = 0;

  virtual Rect& getRect();
  virtual float& getScore();
};
}  // namespace byte_track
