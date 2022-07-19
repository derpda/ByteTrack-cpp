#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Rect& Detection::getRect() {
  return const_cast<Rect&>(const_cast<const Detection*>(this)->getRect());
}

float& Detection::getScore() {
  return const_cast<float&>(const_cast<const Detection*>(this)->getScore());
}

}  // namespace byte_track
