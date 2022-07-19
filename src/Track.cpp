#include "ByteTrack/Track.h"

#include "ByteTrack/Detection.h"

#include <cstddef>

namespace byte_track {

Track::Track(DetectionPtr detection, size_t start_frame_id, size_t track_id)
    : detection_(detection),
      kalman_filter_(),
      state_(TrackState::Tracked),
      is_activated_(start_frame_id == 1 ? true : false),
      track_id_(track_id),
      frame_id_(start_frame_id),
      start_frame_id_(start_frame_id),
      tracklet_len_(0) {
  kalman_filter_.initiate(detection->getRect().getXyah());
}

Track::~Track() {}

const Detection& Track::getDetection() const { return *detection_.get(); }

const TrackState& Track::getTrackState() const { return state_; }

bool Track::isActivated() const { return is_activated_; }

size_t Track::getTrackId() const { return track_id_; }

size_t Track::getFrameId() const { return frame_id_; }

size_t Track::getStartFrameId() const { return start_frame_id_; }

size_t Track::getTrackletLength() const { return tracklet_len_; }

void Track::reActivate(const DetectionPtr& matched_detection, size_t frame_id,
                       int new_track_id) {
  detection_->setRect(generate_rect_by_xyah(
      kalman_filter_.update(matched_detection->getRect().getXyah())));
  detection_->setScore(matched_detection->getScore());

  state_ = TrackState::Tracked;
  is_activated_ = true;
  if (0 <= new_track_id) {
    track_id_ = new_track_id;
  }
  frame_id_ = frame_id;
  tracklet_len_ = 0;
}

void Track::predict() { kalman_filter_.predict(state_ != TrackState::Tracked); }

void Track::update(const DetectionPtr& matched_detection, size_t frame_id) {
  detection_->setRect(generate_rect_by_xyah(
      kalman_filter_.update(matched_detection->getRect().getXyah())));
  detection_->setScore(matched_detection->getScore());

  state_ = TrackState::Tracked;
  is_activated_ = true;
  frame_id_ = frame_id;
  tracklet_len_++;
}

void Track::markAsLost() { state_ = TrackState::Lost; }

void Track::markAsRemoved() { state_ = TrackState::Removed; }

}  // namespace byte_track
