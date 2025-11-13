#pragma once

#include "ByteTrack/Rect.h"

#include <type_traits>

namespace byte_track {

template <typename T>
concept DetectionType = requires(T det) {
  { det.rect() } -> Rectangle;
  { det.score() } -> std::convertible_to<float>;
} && std::is_move_constructible_v<T>;

class Detection {
  TlwhRect rect_;
  float score_ = 0;

 public:
  Detection(const TlwhRect &rect, float score);

  const TlwhRect &rect() const;
  float score() const;
};

}  // namespace byte_track
