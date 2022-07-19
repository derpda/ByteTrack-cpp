#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Detection::Detection(const Rect& rect, float score)
    : rect_(rect), score_(score) {}

const Rect& Detection::getRect() const { return rect_; }

float Detection::getScore() const { return score_; }

void Detection::setRect(const Rect& rect) { rect_ = rect; }

void Detection::setScore(float score) { score_ = score; }

}  // namespace byte_track
