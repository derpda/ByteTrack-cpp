#include "ByteTrack/Track.h"

#include <cstddef>

namespace byte_track {
Track::Track(const Rect& rect, int label, float score)
    : kalman_filter_(),
      mean_(),
      covariance_(),
      rect_(rect),
      label_(label),
      score_(score),
      state_(TrackState::New),
      is_activated_(false),
      track_id_(0),
      frame_id_(0),
      start_frame_id_(0),
      tracklet_len_(0) {}

Track::~Track() {}

const Rect& Track::getRect() const { return rect_; }

int Track::getLabel() const { return label_; }

float Track::getScore() const { return score_; }

const TrackState& Track::getTrackState() const { return state_; }

bool Track::isActivated() const { return is_activated_; }

size_t Track::getTrackId() const { return track_id_; }

size_t Track::getFrameId() const { return frame_id_; }

size_t Track::getStartFrameId() const { return start_frame_id_; }

size_t Track::getTrackletLength() const { return tracklet_len_; }

void Track::activate(size_t frame_id, size_t track_id) {
  kalman_filter_.initiate(mean_, covariance_, rect_.getXyah());

  updateRect();

  state_ = TrackState::Tracked;
  if (frame_id == 1) {
    is_activated_ = true;
  }
  track_id_ = track_id;
  frame_id_ = frame_id;
  start_frame_id_ = frame_id;
  tracklet_len_ = 0;
}

void Track::reActivate(const Track& new_track, size_t frame_id,
                       int new_track_id) {
  kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

  updateRect();

  state_ = TrackState::Tracked;
  is_activated_ = true;
  score_ = new_track.getScore();
  if (0 <= new_track_id) {
    track_id_ = new_track_id;
  }
  frame_id_ = frame_id;
  tracklet_len_ = 0;
}

void Track::predict() {
  if (state_ != TrackState::Tracked) {
    mean_[7] = 0;
  }
  kalman_filter_.predict(mean_, covariance_);
}

void Track::update(const Track& new_track, size_t frame_id) {
  kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

  updateRect();

  state_ = TrackState::Tracked;
  is_activated_ = true;
  score_ = new_track.getScore();
  frame_id_ = frame_id;
  tracklet_len_++;
}

void Track::markAsLost() { state_ = TrackState::Lost; }

void Track::markAsRemoved() { state_ = TrackState::Removed; }

void Track::updateRect() {
  rect_.width() = mean_[2] * mean_[3];
  rect_.height() = mean_[3];
  rect_.x() = mean_[0] - rect_.width() / 2;
  rect_.y() = mean_[1] - rect_.height() / 2;
}
}  // namespace byte_track
