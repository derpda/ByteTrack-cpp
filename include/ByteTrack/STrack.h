#pragma once

#include "ByteTrack/KalmanFilter.h"
#include "ByteTrack/Rect.h"

#include <cstddef>
#include <memory>

namespace byte_track {

class STrack;

using STrackPtr = std::shared_ptr<STrack>;

enum class STrackState {
  New = 0,
  Tracked = 1,
  Lost = 2,
  Removed = 3,
};

class STrack {
 public:
  STrack(const Rect& rect, int label, float score);
  ~STrack();

  const Rect& getRect() const;
  int getLabel() const;
  float getScore() const;

  const STrackState& getSTrackState() const;
  bool isActivated() const;
  size_t getTrackId() const;
  size_t getFrameId() const;
  size_t getStartFrameId() const;
  size_t getTrackletLength() const;

  void activate(size_t frame_id, size_t track_id);
  void reActivate(const STrack& new_track, size_t frame_id,
                  int new_track_id = -1);

  void predict();
  void update(const STrack& new_track, size_t frame_id);

  void markAsLost();
  void markAsRemoved();

 private:
  KalmanFilter kalman_filter_;
  KalmanFilter::StateMean mean_;
  KalmanFilter::StateCov covariance_;

  Rect rect_;
  int label_;
  float score_;

  STrackState state_;
  bool is_activated_;
  size_t track_id_;
  size_t frame_id_;
  size_t start_frame_id_;
  size_t tracklet_len_;

  void updateRect();
};
}  // namespace byte_track
