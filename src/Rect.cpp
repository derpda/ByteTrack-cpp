#include "ByteTrack/Rect.h"

#include <algorithm>

namespace byte_track {
template <typename T>
Rect<T>::Rect(T x, T y, T width, T height) : tlwh({x, y, width, height}) {}

template <typename T>
Rect<T>::~Rect() {}

template <typename T>
T Rect<T>::x() const {
  return tlwh[0];
}

template <typename T>
T Rect<T>::y() const {
  return tlwh[1];
}

template <typename T>
T Rect<T>::width() const {
  return tlwh[2];
}

template <typename T>
T Rect<T>::height() const {
  return tlwh[3];
}

template <typename T>
T& Rect<T>::x() {
  return tlwh[0];
}

template <typename T>
T& Rect<T>::y() {
  return tlwh[1];
}

template <typename T>
T& Rect<T>::width() {
  return tlwh[2];
}

template <typename T>
T& Rect<T>::height() {
  return tlwh[3];
}

template <typename T>
T Rect<T>::left() const {
  return tlwh[0];
}

template <typename T>
T Rect<T>::top() const {
  return tlwh[1];
}

template <typename T>
T Rect<T>::right() const {
  return tlwh[0] + tlwh[2];
}

template <typename T>
T Rect<T>::bottom() const {
  return tlwh[1] + tlwh[3];
}

template <typename T>
Tlbr<T> Rect<T>::getTlbr() const {
  return {
      tlwh[0],
      tlwh[1],
      tlwh[0] + tlwh[2],
      tlwh[1] + tlwh[3],
  };
}

template <typename T>
Xyah<T> Rect<T>::getXyah() const {
  return {
      tlwh[0] + tlwh[2] / 2,
      tlwh[1] + tlwh[3] / 2,
      tlwh[2] / tlwh[3],
      tlwh[3],
  };
}

template <typename T>
float Rect<T>::calcIoU(const Rect<T>& other) const {
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

template <typename T>
Rect<T> generate_rect_by_tlbr(const Tlbr<T>& tlbr) {
  return Rect<T>(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
}

template <typename T>
Rect<T> generate_rect_by_xyah(const Xyah<T>& xyah) {
  const auto width = xyah[2] * xyah[3];
  return Rect<T>(xyah[0] - width / 2, xyah[1] - xyah[3] / 2, width, xyah[3]);
}

// explicit instantiation
template class Rect<int>;
template class Rect<float>;

template Rect<int> generate_rect_by_tlbr<int>(const Tlbr<int>&);
template Rect<float> generate_rect_by_tlbr<float>(const Tlbr<float>&);

template Rect<int> generate_rect_by_xyah<int>(const Xyah<int>&);
template Rect<float> generate_rect_by_xyah<float>(const Xyah<float>&);
}  // namespace byte_track
