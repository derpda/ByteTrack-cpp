#include "ByteTrack/Detection.h"

#include "ByteTrack/Rect.h"

namespace byte_track {

Detection::Detection(const TlwhRect &rect, float score)
    : rect_(rect), score_(score) {}

const TlwhRect &Detection::rect() const { return rect_; }
float Detection::score() const { return score_; }

void Detection::set_rect(const RectBase &rect) { rect_ = TlwhRect(rect); }
void Detection::set_score(float score) { score_ = score; }

}  // namespace byte_track
