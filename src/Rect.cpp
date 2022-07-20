#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {

float calc_iou(const RectBase& A, const RectBase& B) {
  const float box_area = (B.width() + 1) * (B.height() + 1);
  const float iw = std::min(A.left() + A.width(), B.left() + B.width()) -
                   std::max(A.left(), B.left()) + 1;
  float iou = 0;
  if (iw > 0) {
    const float ih = std::min(A.top() + A.height(), B.top() + B.height()) -
                     std::max(A.top(), B.top()) + 1;
    if (ih > 0) {
      const float ua = (A.width() + 1) * (A.height() + 1) + box_area - iw * ih;
      iou = iw * ih / ua;
    }
  }
  return iou;
}

TlwhRect::TlwhRect(float top, float left, float width, float height)
    : top_(top), left_(left), width_(width), height_(height) {}

float TlwhRect::top() const { return top_; }
float TlwhRect::left() const { return left_; }
float TlwhRect::width() const { return width_; }
float TlwhRect::height() const { return height_; }

float& TlwhRect::top() { return top_; }
float& TlwhRect::left() { return left_; }
float& TlwhRect::width() { return width_; }
float& TlwhRect::height() { return height_; }

}  // namespace byte_track
