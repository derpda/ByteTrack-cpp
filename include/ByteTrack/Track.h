#pragma once

#include "ByteTrack/Detection.h"
#include "ByteTrack/KalmanFilter.h"
#include "ByteTrack/Rect.h"

#include <cstddef>
#include <memory>

namespace byte_track {

class Track;

using TrackPtr = std::shared_ptr<Track>;

enum class TrackState {
  Tracked = 0,
  Lost = 1,
};

class Track {
 public:
  DetectionPtr detection;
  TlwhRect predictedRect;

  Track() = delete;
  Track(DetectionPtr detection, size_t start_frame_id, size_t track_id);

  const TrackState& get_track_state() const;
  bool is_confirmed() const;
  size_t get_track_id() const;
  size_t get_frame_id() const;
  size_t get_start_frame_id() const;
  size_t get_tracklet_length() const;

  void predict();
  void update(const DetectionPtr& new_track, size_t frame_id);

  void mark_as_lost();
  void mark_as_confirmed();

 private:
  KalmanFilter kalman_filter_;

  TrackState state_;
  bool is_confirmed_;
  size_t track_id_;
  size_t frame_id_;
  size_t start_frame_id_;
  size_t tracklet_len_;
};
}  // namespace byte_track
