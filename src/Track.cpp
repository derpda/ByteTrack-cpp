#include "ByteTrack/Track.h"

#include "ByteTrack/Detection.h"

#include <cstddef>

namespace byte_track {

Track::Track(DetectionPtr detection, size_t start_frame_id, size_t track_id)
    : detection(detection),
      kalman_filter_(),
      state_(TrackState::Tracked),
      // Detections registered on first frame are considered as confirmed
      is_confirmed_(start_frame_id == 1 ? true : false),
      track_id_(track_id),
      frame_id_(start_frame_id),
      start_frame_id_(start_frame_id),
      tracklet_len_(0) {
  kalman_filter_.initiate(detection->rect());
}

const TrackState& Track::get_track_state() const { return state_; }

bool Track::is_confirmed() const { return is_confirmed_; }

size_t Track::get_track_id() const { return track_id_; }

size_t Track::get_frame_id() const { return frame_id_; }

size_t Track::get_start_frame_id() const { return start_frame_id_; }

size_t Track::get_tracklet_length() const { return tracklet_len_; }

void Track::predict() { kalman_filter_.predict(state_ != TrackState::Tracked); }

void Track::update(const DetectionPtr& matched_detection, size_t frame_id) {
  detection->set_rect(kalman_filter_.update(matched_detection->rect()));
  detection->set_score(matched_detection->score());

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

void Track::mark_as_lost() { state_ = TrackState::Lost; }

void Track::mark_as_removed() { state_ = TrackState::Removed; }

}  // namespace byte_track
