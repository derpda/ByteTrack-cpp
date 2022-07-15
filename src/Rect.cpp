#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {

Rect::Rect(float x, float y, float width, float height)
    : tlwh({x, y, width, height}) {}

float Rect::x() const { return tlwh[0]; }

float Rect::y() const { return tlwh[1]; }

float Rect::width() const { return tlwh[2]; }

float Rect::height() const { return tlwh[3]; }

float& Rect::x() { return tlwh[0]; }

float& Rect::y() { return tlwh[1]; }

float& Rect::width() { return tlwh[2]; }

float& Rect::height() { return tlwh[3]; }

float Rect::left() const { return tlwh[0]; }

float Rect::top() const { return tlwh[1]; }

float Rect::right() const { return tlwh[0] + tlwh[2]; }

float Rect::bottom() const { return tlwh[1] + tlwh[3]; }

Tlbr Rect::getTlbr() const {
  return {
      tlwh[0],
      tlwh[1],
      tlwh[0] + tlwh[2],
      tlwh[1] + tlwh[3],
  };
}

Xyah Rect::getXyah() const {
  return {
      tlwh[0] + tlwh[2] / 2,
      tlwh[1] + tlwh[3] / 2,
      tlwh[2] / tlwh[3],
      tlwh[3],
  };
}

float Rect::calcIoU(const Rect& other) const {
  const float box_area = (other.tlwh[2] + 1) * (other.tlwh[3] + 1);
  const float iw = std::min(tlwh[0] + tlwh[2], other.tlwh[0] + other.tlwh[2]) -
                   std::max(tlwh[0], other.tlwh[0]) + 1;
  float iou = 0;
  if (iw > 0) {
    const float ih =
        std::min(tlwh[1] + tlwh[3], other.tlwh[1] + other.tlwh[3]) -
        std::max(tlwh[1], other.tlwh[1]) + 1;
    if (ih > 0) {
      const float ua = (tlwh[0] + tlwh[2] - tlwh[0] + 1) *
                           (tlwh[1] + tlwh[3] - tlwh[1] + 1) +
                       box_area - iw * ih;
      iou = iw * ih / ua;
    }
  }
  return iou;
}

Rect generate_rect_by_tlbr(const Tlbr& tlbr) {
  return Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
}

Rect generate_rect_by_xyah(const Xyah& xyah) {
  const auto width = xyah[2] * xyah[3];
  return Rect(xyah[0] - width / 2, xyah[1] - xyah[3] / 2, width, xyah[3]);
}
}  // namespace byte_track
