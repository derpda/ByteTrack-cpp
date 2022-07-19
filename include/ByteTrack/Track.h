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
  New = 0,
  Tracked = 1,
  Lost = 2,
  Removed = 3,
};

class Track {
 public:
  Track(DetectionPtr detection);
  ~Track();

  const Detection& getDetection() const;

  const TrackState& getTrackState() const;
  bool isActivated() const;
  size_t getTrackId() const;
  size_t getFrameId() const;
  size_t getStartFrameId() const;
  size_t getTrackletLength() const;

  void activate(size_t frame_id, size_t track_id);
  void reActivate(const DetectionPtr& new_track, size_t frame_id,
                  int new_track_id = -1);

  void predict();
  void update(const DetectionPtr& new_track, size_t frame_id);

  void markAsLost();
  void markAsRemoved();

 private:
  DetectionPtr detection_;

  KalmanFilter kalman_filter_;

  TrackState state_;
  bool is_activated_;
  size_t track_id_;
  size_t frame_id_;
  size_t start_frame_id_;
  size_t tracklet_len_;
};
}  // namespace byte_track
