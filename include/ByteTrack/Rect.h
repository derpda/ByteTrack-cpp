#pragma once

#include <algorithm>
#include <concepts>

namespace byte_track {

template <typename T>
concept Rectangle = requires(T rect) {
  { rect.top() } -> std::convertible_to<float>;
  { rect.left() } -> std::convertible_to<float>;
  { rect.width() } -> std::convertible_to<float>;
  { rect.height() } -> std::convertible_to<float>;
};

template <Rectangle T1, Rectangle T2>
float calc_iou(const T1& A, const T2& B) {
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

class TlwhRect {
  float top_;
  float left_;
  float width_;
  float height_;

 public:
  TlwhRect(float top = 0, float left = 0, float width = 0, float height = 0);
  TlwhRect(const TlwhRect& other) = default;
  TlwhRect& operator=(const TlwhRect& other) = default;
  TlwhRect(TlwhRect&& other) = default;
  TlwhRect& operator=(TlwhRect&& other) = default;

  template <Rectangle T>
  TlwhRect(const T& other)
      : top_(other.top()),
        left_(other.left()),
        width_(other.width()),
        height_(other.height()) {}
  template <Rectangle T>
  TlwhRect& operator=(const T& other) {
    *this = TlwhRect(other);
    return *this;
  }

  float top() const;
  float left() const;
  float width() const;
  float height() const;
};

}  // namespace byte_track
