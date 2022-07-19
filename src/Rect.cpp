#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {

float& Rect::top() {
  return const_cast<float&>(const_cast<const Rect*>(this)->top());
}

float& Rect::left() {
  return const_cast<float&>(const_cast<const Rect*>(this)->left());
}

float& Rect::width() {
  return const_cast<float&>(const_cast<const Rect*>(this)->width());
}

float& Rect::height() {
  return const_cast<float&>(const_cast<const Rect*>(this)->height());
}

Xyah Rect::getXyah() const {
  return {
      left() + width() / 2,
      top() + height() / 2,
      width() / height(),
      height(),
  };
}

void Rect::set_from_xyah(const Xyah& xyah) {
  float xyah_width = xyah(2) * xyah(3);
  left() = xyah(0) - xyah_width / 2;
  top() = xyah(1) - xyah(3) / 2;
  width() = xyah_width;
  height() = xyah(3);
}

float calcIoU(const Rect& A, const Rect& B) {
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

}  // namespace byte_track
