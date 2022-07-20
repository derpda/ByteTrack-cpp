#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {

Xyah RectBase::xyah() const {
  return {
      left() + width() / 2,
      top() + height() / 2,
      width() / height(),
      height(),
  };
}

void RectBase::set_from_xyah(const Xyah& xyah) {
  float xyah_width = xyah(2) * xyah(3);
  set_left(xyah(0) - xyah_width / 2);
  set_top(xyah(1) - xyah(3) / 2);
  set_width(xyah_width);
  set_height(xyah(3));
}

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

void TlwhRect::set_top(float top) { top_ = top; }
void TlwhRect::set_left(float left) { left_ = left; }
void TlwhRect::set_width(float width) { width_ = width; }
void TlwhRect::set_height(float height) { height_ = height; }

}  // namespace byte_track
