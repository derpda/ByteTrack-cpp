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
  Removed = 2,
};

class Track {
 public:
  Track(DetectionPtr detection, size_t start_frame_id, size_t track_id);
  ~Track();

  const Detection& getDetection() const;

  const TrackState& getTrackState() const;
  bool isConfirmed() const;
  size_t getTrackId() const;
  size_t getFrameId() const;
  size_t getStartFrameId() const;
  size_t getTrackletLength() const;

  void predict();
  void update(const DetectionPtr& new_track, size_t frame_id);

  void markAsLost();
  void markAsRemoved();

 private:
  DetectionPtr detection_;

  KalmanFilter kalman_filter_;

  TrackState state_;
  bool is_confirmed_;
  size_t track_id_;
  size_t frame_id_;
  size_t start_frame_id_;
  size_t tracklet_len_;
};
}  // namespace byte_track
