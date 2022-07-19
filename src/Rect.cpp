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

float calcIoU(const Rect& A, const Rect& B) {
  const float box_area = (B.tlwh[2] + 1) * (B.tlwh[3] + 1);
  const float iw = std::min(A.tlwh[0] + A.tlwh[2], B.tlwh[0] + B.tlwh[2]) -
                   std::max(A.tlwh[0], B.tlwh[0]) + 1;
  float iou = 0;
  if (iw > 0) {
    const float ih = std::min(A.tlwh[1] + A.tlwh[3], B.tlwh[1] + B.tlwh[3]) -
                     std::max(A.tlwh[1], B.tlwh[1]) + 1;
    if (ih > 0) {
      const float ua = (A.tlwh[0] + A.tlwh[2] - A.tlwh[0] + 1) *
                           (A.tlwh[1] + A.tlwh[3] - A.tlwh[1] + 1) +
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
