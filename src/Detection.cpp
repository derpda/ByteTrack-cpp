#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Rect& Detection::get_rect() {
  return const_cast<Rect&>(const_cast<const Detection*>(this)->get_rect());
}

float& Detection::get_score() {
  return const_cast<float&>(const_cast<const Detection*>(this)->get_score());
}

}  // namespace byte_track
