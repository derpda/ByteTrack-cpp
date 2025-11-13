#pragma once

#include "ByteTrack/Detection.h"
#include "ByteTrack/KalmanFilter.h"
#include "ByteTrack/Rect.h"

#include <cstddef>
#include <memory>

namespace byte_track {

enum class TrackState {
  Tracked = 0,
  Lost = 1,
};

template <typename T, typename DetType>
concept UserTrackType = requires(T track, const std::shared_ptr<DetType>& det) {
  { track.update(det) } -> std::same_as<void>;
  { T(det) } -> std::same_as<T>;
};

class DummyUserTrack {
 public:
  DummyUserTrack() = delete;
  DummyUserTrack(const std::shared_ptr<Detection>& /*det*/) {}

  void update(const std::shared_ptr<Detection>& /*det*/) {}
};

template <DetectionType DetType = Detection,
          UserTrackType<DetType> UserTrack = DummyUserTrack>
class Track {
  using DetectionPtr = std::shared_ptr<DetType>;

 public:
  std::shared_ptr<UserTrack> userTrack;
  TlwhRect rect;
  TlwhRect predictedRect;

  Track() = delete;
  ~Track() = default;
  Track(const Track&) = delete;
  Track& operator=(const Track&) = delete;
  Track(Track&&) = default;
  Track& operator=(Track&&) = default;

  Track(const DetectionPtr& detection, size_t start_frame_id, size_t track_id)
      : userTrack(std::make_shared<UserTrack>(detection)),
        rect(detection->rect()),
        predictedRect(rect),
        kalman_filter_(),
        state_(TrackState::Tracked),
        // Detections registered on first frame are considered as confirmed
        is_confirmed_(start_frame_id == 1 ? true : false),
        track_id_(track_id),
        frame_id_(start_frame_id),
        start_frame_id_(start_frame_id),
        tracklet_len_(0) {
    kalman_filter_.initiate(predictedRect);
  }

  const TrackState& get_track_state() const { return state_; }
  bool is_confirmed() const { return is_confirmed_; }
  size_t get_track_id() const { return track_id_; }
  size_t get_frame_id() const { return frame_id_; }
  size_t get_start_frame_id() const { return start_frame_id_; }
  size_t get_tracklet_length() const { return tracklet_len_; }

  void predict() {
    predictedRect = kalman_filter_.predict(state_ != TrackState::Tracked);
  }
  void update(const DetectionPtr& matched_detection, size_t frame_id) {
    rect = matched_detection->rect();
    predictedRect = kalman_filter_.update(matched_detection->rect());
    userTrack->update(matched_detection);

    // If the track was actively tracked, just increment the tracklet length
    // Otherwise, mark the track as tracked again and reset the tracklet length
    if (state_ == TrackState::Tracked) {
      tracklet_len_++;
    } else {
      state_ = TrackState::Tracked;
      tracklet_len_ = 0;
    }
    is_confirmed_ = true;
    frame_id_ = frame_id;
  }

  void mark_as_lost() { state_ = TrackState::Lost; }
  void mark_as_confirmed() { is_confirmed_ = true; }

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
