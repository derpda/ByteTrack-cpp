#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {

float calc_iou(const RectBase& A, const RectBase& B) {
  const float iw = std::min(A.left() + A.width(), B.left() + B.width()) -
                   std::max(A.left(), B.left());
  float iou = 0;
  if (iw > 0) {
    const float ih = std::min(A.top() + A.height(), B.top() + B.height()) -
                     std::max(A.top(), B.top());
    if (ih > 0) {
      const float ua =
          A.width() * A.height() + B.width() * B.height() - iw * ih;
      iou = iw * ih / ua;
    }
  }
  return iou;
}

TlwhRect::TlwhRect(float top, float left, float width, float height)
    : top_(top), left_(left), width_(width), height_(height) {}

TlwhRect::TlwhRect(const RectBase& other)
    : top_(other.top()),
      left_(other.left()),
      width_(other.width()),
      height_(other.height()) {}

float TlwhRect::top() const { return top_; }
float TlwhRect::left() const { return left_; }
float TlwhRect::width() const { return width_; }
float TlwhRect::height() const { return height_; }

float& TlwhRect::top() { return top_; }
float& TlwhRect::left() { return left_; }
float& TlwhRect::width() { return width_; }
float& TlwhRect::height() { return height_; }

}  // namespace byte_track
