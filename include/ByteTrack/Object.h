#pragma once

#include "ByteTrack/Rect.h"

namespace byte_track {
struct Object {
  Rect<float> rect;
  int label;
  float prob;

  Object(const Rect<float> &_rect, int _label, float _prob);
};
}  // namespace byte_track
