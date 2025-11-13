#include "ByteTrack/Rect.h"

namespace byte_track {

TlwhRect::TlwhRect(float top, float left, float width, float height)
    : top_(top), left_(left), width_(width), height_(height) {}

float TlwhRect::top() const { return top_; }
float TlwhRect::left() const { return left_; }
float TlwhRect::width() const { return width_; }
float TlwhRect::height() const { return height_; }

}  // namespace byte_track
