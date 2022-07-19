#pragma once
#include "ByteTrack/Rect.h"

#include <memory>

namespace byte_track {

class Detection;
using DetectionPtr = std::shared_ptr<Detection>;

class Detection {
 public:
  Detection(const Rect& rect, float score);

  const Rect& getRect() const;
  float getScore() const;

  void setRect(const Rect& rect);
  void setScore(float score);

 private:
  Rect rect_;
  float score_;
};
}  // namespace byte_track
