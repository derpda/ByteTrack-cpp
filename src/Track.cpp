#include "ByteTrack/Track.h"

#include <cstddef>

namespace byte_track {
Track::Track(const Rect& rect, int label, float score)
    : kalman_filter_(),
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
  kalman_filter_.initiate(rect_.getXyah());

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
  rect_ = generate_rect_by_xyah(
      kalman_filter_.update(new_track.getRect().getXyah()));

  state_ = TrackState::Tracked;
  is_activated_ = true;
  score_ = new_track.getScore();
  if (0 <= new_track_id) {
    track_id_ = new_track_id;
  }
  frame_id_ = frame_id;
  tracklet_len_ = 0;
}

void Track::predict() { kalman_filter_.predict(state_ != TrackState::Tracked); }

void Track::update(const Track& new_track, size_t frame_id) {
  rect_ = generate_rect_by_xyah(
      kalman_filter_.update(new_track.getRect().getXyah()));

  state_ = TrackState::Tracked;
  is_activated_ = true;
  score_ = new_track.getScore();
  frame_id_ = frame_id;
  tracklet_len_++;
}

void Track::markAsLost() { state_ = TrackState::Lost; }

void Track::markAsRemoved() { state_ = TrackState::Removed; }

}  // namespace byte_track
