#pragma once
#include "ByteTrack/KalmanFilter.h"
#include "ByteTrack/Rect.h"
#include "ByteTrack/lapjv.h"

#include <concepts>
#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace byte_track {

template <typename DetectionType>
concept UserDetectionType = requires(DetectionType det) {
  { det.rect() } -> Rectangle;
  { det.score() } -> std::convertible_to<float>;
} && std::is_move_constructible_v<DetectionType>;

template <typename TrackType, typename Detection>
concept UserTrackType =
    requires(TrackType track, const std::shared_ptr<Detection>& det) {
      { track.update(det) } -> std::same_as<void>;
      { TrackType(det) } -> std::same_as<TrackType>;
    };

template <UserDetectionType Detection, UserTrackType<Detection> Track>
class BYTETracker {
  using DetectionPtr = std::shared_ptr<Detection>;

  enum class TrackState {
    Tracked = 0,
    Lost = 1,
  };
  class InternalTrack {
   public:
    std::shared_ptr<Track> userTrack;
    TlwhRect rect;
    TlwhRect predictedRect;

    InternalTrack() = delete;
    ~InternalTrack() = default;
    InternalTrack(const InternalTrack&) = delete;
    InternalTrack& operator=(const InternalTrack&) = delete;
    InternalTrack(InternalTrack&&) = default;
    InternalTrack& operator=(InternalTrack&&) = default;

    InternalTrack(const DetectionPtr& detection, size_t start_frame_id,
                  size_t track_id)
        : userTrack(std::make_shared<Track>(detection)),
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
      // Otherwise, mark the track as tracked again and reset the tracklet
      // length
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
  using InternalTrackPtr = std::shared_ptr<InternalTrack>;

 public:
  BYTETracker(int frame_rate = 30, int track_buffer = 30,
              float track_thresh = 0.5, float high_thresh = 0.6,
              float match_thresh = 0.8)
      : track_thresh_(track_thresh),
        high_thresh_(high_thresh),
        match_thresh_(match_thresh),
        max_time_lost_(static_cast<size_t>(frame_rate / 30.0 * track_buffer)),
        frame_id_(0),
        track_id_count_(0) {}

  std::vector<std::shared_ptr<Track>> update(
      const std::vector<DetectionPtr>& input_detections) {
    frame_id_++;

    ////////// Step 1: Get detections //////////

    // Sort new tracks from detection by score
    std::vector<DetectionPtr> detections;
    std::vector<DetectionPtr> low_score_detections;
    for (const auto& detection : input_detections) {
      if (detection->score() >= track_thresh_)
        detections.push_back(detection);
      else
        low_score_detections.push_back(detection);
    }

    // Sort existing tracks by confirmed status
    std::vector<InternalTrackPtr> confirmed_tracks;
    std::vector<InternalTrackPtr> unconfirmed_tracks;

    for (const auto& track : tracked_tracks_) {
      if (!track->is_confirmed())
        unconfirmed_tracks.push_back(track);
      else
        confirmed_tracks.push_back(track);
    }

    std::vector<InternalTrackPtr> track_pool;
    track_pool = joint_tracks(confirmed_tracks, lost_tracks_);

    // Predict current pose by KF
    for (auto& track : track_pool) track->predict();

    ////////// Step 2: Find matches between tracks and detections //////////
    ////////// Step 2: First association, with IoU //////////
    auto [matched_tracks, unmatched_tracked_tracks, unmatched_detections] =
        iou_association(track_pool, detections);

    ////////// Step 3: Second association, using low score dets //////////
    auto new_lost_tracks = low_score_association(
        matched_tracks, low_score_detections, unmatched_tracked_tracks);

    ////////// Step 4: Init new tracks //////////
    auto removed_tracks = init_new_tracks(matched_tracks, unconfirmed_tracks,
                                          unmatched_detections);

    ////////// Step 5: Update state //////////
    for (auto& lost_track : lost_tracks_) {
      if (frame_id_ - lost_track->get_frame_id() > max_time_lost_) {
        removed_tracks.push_back(lost_track);
      }
    }

    lost_tracks_ = sub_tracks(
        joint_tracks(sub_tracks(lost_tracks_, matched_tracks), new_lost_tracks),
        removed_tracks);

    std::tie(tracked_tracks_, lost_tracks_) =
        remove_duplicate_tracks(matched_tracks, lost_tracks_);

    std::vector<std::shared_ptr<Track>> output_tracks;
    for (const auto& track : tracked_tracks_) {
      if (track->is_confirmed()) output_tracks.push_back(track->userTrack);
    }

    return output_tracks;
  }

  void clear() {
    tracked_tracks_.clear();
    lost_tracks_.clear();
    frame_id_ = 0;
    track_id_count_ = 0;
  }

 private:
  std::tuple<std::vector<InternalTrackPtr>, std::vector<InternalTrackPtr>,
             std::vector<DetectionPtr>>
  iou_association(const std::vector<InternalTrackPtr>& track_pool,
                  const std::vector<DetectionPtr>& detections) {
    auto [matches, unmatched_tracks, unmatched_detections] =
        linear_assignment(track_pool, detections, match_thresh_);

    std::vector<InternalTrackPtr> matched_tracks;
    for (auto& match : matches) {
      match.first->update(match.second, frame_id_);
      matched_tracks.push_back(match.first);
    }

    std::vector<InternalTrackPtr> unmatched_tracked_tracks;
    for (const auto& unmatch : unmatched_tracks) {
      if (unmatch->get_track_state() == TrackState::Tracked) {
        unmatched_tracked_tracks.push_back(unmatch);
      }
    }
    return {std::move(matched_tracks), std::move(unmatched_tracked_tracks),
            std::move(unmatched_detections)};
  }

  std::vector<InternalTrackPtr> low_score_association(
      std::vector<InternalTrackPtr>& matched_tracks,
      const std::vector<DetectionPtr>& low_score_detections,
      const std::vector<InternalTrackPtr>& unmatched_tracked_tracks) {
    auto [matches, unmatched_tracks, unmatch_detection] =
        linear_assignment(unmatched_tracked_tracks, low_score_detections, 0.5);

    for (auto& match : matches) {
      match.first->update(match.second, frame_id_);
      matched_tracks.push_back(match.first);
    }

    std::vector<InternalTrackPtr> new_lost_tracks;
    for (const auto& track : unmatched_tracks) {
      if (track->get_track_state() != TrackState::Lost) {
        track->mark_as_lost();
        new_lost_tracks.push_back(track);
      }
    }
    return new_lost_tracks;
  }

  std::vector<InternalTrackPtr> init_new_tracks(
      std::vector<InternalTrackPtr>& matched_tracks,
      const std::vector<InternalTrackPtr>& unconfirmed_tracks,
      const std::vector<DetectionPtr>& unmatched_detections) {
    // Deal with unconfirmed tracks, usually tracks with only one beginning
    // frame
    auto [matches, unmatched_unconfirmed_tracks, new_detections] =
        linear_assignment(unconfirmed_tracks, unmatched_detections, 0.7);

    for (const auto& match : matches) {
      match.first->update(match.second, frame_id_);
      matched_tracks.push_back(match.first);
    }

    std::vector<InternalTrackPtr> new_removed_tracks;
    for (const auto& track : unmatched_unconfirmed_tracks) {
      new_removed_tracks.push_back(track);
    }

    // Add new tracks
    for (const auto& detection : new_detections) {
      if (detection->score() < track_thresh_) continue;
      track_id_count_++;
      InternalTrackPtr new_track = std::make_shared<InternalTrack>(
          detection, frame_id_, track_id_count_);
      if (detection->score() >= high_thresh_) {
        new_track->mark_as_confirmed();
      }
      matched_tracks.push_back(new_track);
    }
    return new_removed_tracks;
  }

  std::vector<InternalTrackPtr> joint_tracks(
      const std::vector<InternalTrackPtr>& a_tlist,
      const std::vector<InternalTrackPtr>& b_tlist) const {
    std::set<int> exists;
    std::vector<InternalTrackPtr> res;
    for (auto& track : a_tlist) {
      exists.emplace(track->get_track_id());
      res.push_back(track);
    }
    for (auto& track : b_tlist) {
      if (exists.count(track->get_track_id()) == 0) res.push_back(track);
    }
    return res;
  }

  std::vector<InternalTrackPtr> sub_tracks(
      const std::vector<InternalTrackPtr>& a_tlist,
      const std::vector<InternalTrackPtr>& b_tlist) const {
    std::map<int, InternalTrackPtr> tracks;
    for (auto& track : a_tlist) tracks.emplace(track->get_track_id(), track);
    for (auto& track : b_tlist) tracks.erase(track->get_track_id());

    std::vector<InternalTrackPtr> res;
    for (auto& [_, track] : tracks) res.push_back(track);
    return res;
  }

  std::tuple<std::vector<InternalTrackPtr>, std::vector<InternalTrackPtr>>
  remove_duplicate_tracks(const std::vector<InternalTrackPtr>& a_tracks,
                          const std::vector<InternalTrackPtr>& b_tracks) const {
    if (a_tracks.empty() || b_tracks.empty()) return {a_tracks, b_tracks};

    std::vector<std::vector<float>> ious;
    ious.resize(a_tracks.size());
    for (size_t i = 0; i < ious.size(); i++) ious[i].resize(b_tracks.size());
    for (size_t ai = 0; ai < a_tracks.size(); ai++) {
      for (size_t bi = 0; bi < b_tracks.size(); bi++) {
        ious[ai][bi] = 1 - calc_iou(b_tracks[bi]->predictedRect,
                                    a_tracks[ai]->predictedRect);
      }
    }

    std::vector<bool> a_overlapping(a_tracks.size(), false),
        b_overlapping(b_tracks.size(), false);
    for (size_t ai = 0; ai < ious.size(); ai++) {
      for (size_t bi = 0; bi < ious[ai].size(); bi++) {
        if (ious[ai][bi] < 0.15) {
          const int timep =
              a_tracks[ai]->get_frame_id() - a_tracks[ai]->get_start_frame_id();
          const int timeq =
              b_tracks[bi]->get_frame_id() - b_tracks[bi]->get_start_frame_id();
          if (timep > timeq) {
            b_overlapping[bi] = true;
          } else {
            a_overlapping[ai] = true;
          }
        }
      }
    }

    std::vector<InternalTrackPtr> a_tracks_out;
    for (size_t ai = 0; ai < a_tracks.size(); ai++) {
      if (!a_overlapping[ai]) a_tracks_out.push_back(a_tracks[ai]);
    }

    std::vector<InternalTrackPtr> b_tracks_out;
    for (size_t bi = 0; bi < b_tracks.size(); bi++) {
      if (!b_overlapping[bi]) b_tracks_out.push_back(b_tracks[bi]);
    }
    return {std::move(a_tracks_out), std::move(b_tracks_out)};
  }

  std::tuple<std::vector<std::pair<InternalTrackPtr, DetectionPtr>>,
             std::vector<InternalTrackPtr>, std::vector<DetectionPtr>>
  linear_assignment(const std::vector<InternalTrackPtr>& tracks,
                    const std::vector<DetectionPtr>& detections,
                    float thresh) const {
    if (tracks.empty() || detections.empty()) return {{}, tracks, detections};

    size_t n_rows = tracks.size();
    size_t n_cols = detections.size();
    std::vector<float> cost_matrix(n_rows * n_cols);
    for (size_t i = 0; i < n_rows; i++) {
      for (size_t j = 0; j < n_cols; j++) {
        cost_matrix[i * n_cols + j] =
            1 - calc_iou(detections[j]->rect(), tracks[i]->predictedRect);
      }
    }

    std::vector<std::pair<InternalTrackPtr, DetectionPtr>> matches;
    std::vector<InternalTrackPtr> a_unmatched;
    std::vector<DetectionPtr> b_unmatched;

    auto [rowsol, colsol, _] =
        exec_lapjv(std::move(cost_matrix), n_rows, n_cols, true, thresh, true);
    for (size_t i = 0; i < rowsol.size(); i++) {
      if (rowsol[i] >= 0)
        matches.push_back({tracks[i], detections[rowsol[i]]});
      else
        a_unmatched.push_back(tracks[i]);
    }

    for (size_t i = 0; i < colsol.size(); i++) {
      if (colsol[i] < 0) b_unmatched.push_back(detections[i]);
    }
    return {std::move(matches), std::move(a_unmatched), std::move(b_unmatched)};
  }

 private:
  const float track_thresh_;
  const float high_thresh_;
  const float match_thresh_;
  const size_t max_time_lost_;

  size_t frame_id_;
  size_t track_id_count_;

  std::vector<InternalTrackPtr> tracked_tracks_;
  std::vector<InternalTrackPtr> lost_tracks_;
};
}  // namespace byte_track
