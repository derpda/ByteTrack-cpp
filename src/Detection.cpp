#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Detection::Detection(const TlwhRect &rect, float score)
    : rect_(rect), score_(score) {}

const TlwhRect &Detection::rect() const { return rect_; }
float Detection::score() const { return score_; }

}  // namespace byte_track
