#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Detection::Detection(const TlwhRect &rect, float score)
    : rect_(rect), score_(score) {}

const TlwhRect &Detection::rect() const { return rect_; }
float Detection::score() const { return score_; }

void Detection::update(const byte_track::DetectionBase &new_det) {
  rect_ = TlwhRect(new_det.rect());
  score_ = new_det.score();
}

}  // namespace byte_track
