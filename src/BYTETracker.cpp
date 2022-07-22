#include "ByteTrack/BYTETracker.h"

#include "ByteTrack/Detection.h"
#include "ByteTrack/Rect.h"
#include "ByteTrack/Track.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace byte_track {

BYTETracker::BYTETracker(int frame_rate, int track_buffer, float track_thresh,
                         float high_thresh, float match_thresh)
    : track_thresh_(track_thresh),
      high_thresh_(high_thresh),
      match_thresh_(match_thresh),
      max_time_lost_(static_cast<size_t>(frame_rate / 30.0 * track_buffer)),
      frame_id_(0),
      track_id_count_(0) {}

std::vector<TrackPtr> BYTETracker::update(
    const std::vector<DetectionPtr> &input_detections) {
  frame_id_++;

  ////////// Step 1: Get detections                                   //////////

  // Sort new tracks from detection by score
  std::vector<DetectionPtr> detections;
  std::vector<DetectionPtr> low_score_detections;
  for (const auto &detection : input_detections) {
    if (detection->score() >= track_thresh_)
      detections.push_back(detection);
    else
      low_score_detections.push_back(detection);
  }

  // Sort existing tracks by confirmed status
  std::vector<TrackPtr> confirmed_tracks;
  std::vector<TrackPtr> unconfirmed_tracks;

  for (const auto &track : tracked_tracks_) {
    if (!track->is_confirmed())
      unconfirmed_tracks.push_back(track);
    else
      confirmed_tracks.push_back(track);
  }

  std::vector<TrackPtr> track_pool;
  track_pool = joint_tracks(confirmed_tracks, lost_tracks_);

  // Predict current pose by KF
  for (auto &track : track_pool) track->predict();

  ////////// Step 2: Find matches between tracks and detections       //////////
  ////////// Step 2: First association, with IoU                      //////////
  auto [matched_tracks, unmatched_tracked_tracks, unmatched_detections] =
      iou_association(track_pool, detections);

  ////////// Step 3: Second association, using low score dets         //////////
  auto new_lost_tracks = low_score_association(
      matched_tracks, low_score_detections, unmatched_tracked_tracks);

  ////////// Step 4: Init new tracks                                  //////////
  auto new_removed_tracks =
      init_new_tracks(matched_tracks, unconfirmed_tracks, unmatched_detections);

  ////////// Step 5: Update state                                     //////////
  for (auto &lost_track : lost_tracks_) {
    if (frame_id_ - lost_track->get_frame_id() > max_time_lost_) {
      lost_track->mark_as_removed();
      new_removed_tracks.push_back(lost_track);
    }
  }

  lost_tracks_ = sub_tracks(
      joint_tracks(sub_tracks(lost_tracks_, matched_tracks), new_lost_tracks),
      removed_tracks_);
  removed_tracks_ = joint_tracks(removed_tracks_, new_removed_tracks);

  std::tie(tracked_tracks_, lost_tracks_) =
      remove_duplicate_tracks(matched_tracks, lost_tracks_);

  std::vector<TrackPtr> output_tracks;
  for (const auto &track : tracked_tracks_) {
    if (track->is_confirmed()) output_tracks.push_back(track);
  }

  return output_tracks;
}

std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>,
           std::vector<DetectionPtr>>
BYTETracker::iou_association(const std::vector<TrackPtr> &track_pool,
                             const std::vector<DetectionPtr> &detections) {
  auto [matches, unmatched_tracks, unmatched_detections] =
      linear_assignment(track_pool, detections, match_thresh_);

  std::vector<TrackPtr> matched_tracks;
  for (const auto &match : matches) {
    const auto track = match.first;
    const auto detection = match.second;
    track->update(detection, frame_id_);
    matched_tracks.push_back(track);
  }

  std::vector<TrackPtr> unmatched_tracked_tracks;
  for (const auto &unmatch : unmatched_tracks) {
    if (unmatch->get_track_state() == TrackState::Tracked) {
      unmatched_tracked_tracks.push_back(unmatch);
    }
  }
  return {std::move(matched_tracks), std::move(unmatched_tracked_tracks),
          std::move(unmatched_detections)};
}

std::vector<TrackPtr> BYTETracker::low_score_association(
    std::vector<TrackPtr> &matched_tracks,
    const std::vector<DetectionPtr> &low_score_detections,
    const std::vector<TrackPtr> &unmatched_tracked_tracks) {
  auto [matches, unmatched_tracks, unmatch_detection] =
      linear_assignment(unmatched_tracked_tracks, low_score_detections, 0.5);

  for (const auto &match : matches) {
    const auto track = match.first;
    const auto detection = match.second;
    track->update(detection, frame_id_);
    matched_tracks.push_back(track);
  }

  std::vector<TrackPtr> new_lost_tracks;
  for (const auto &track : unmatched_tracks) {
    if (track->get_track_state() != TrackState::Lost) {
      track->mark_as_lost();
      new_lost_tracks.push_back(track);
    }
  }
  return new_lost_tracks;
}

std::vector<TrackPtr> BYTETracker::init_new_tracks(
    std::vector<TrackPtr> &matched_tracks,
    const std::vector<TrackPtr> &unconfirmed_tracks,
    const std::vector<DetectionPtr> &unmatched_detections) {
  // Deal with unconfirmed tracks, usually tracks with only one beginning frame
  auto [matches, unmatched_unconfirmed_tracks, new_detections] =
      linear_assignment(unconfirmed_tracks, unmatched_detections, 0.7);

  for (const auto &match : matches) {
    match.first->update(match.second, frame_id_);
    matched_tracks.push_back(match.first);
  }

  std::vector<TrackPtr> new_removed_tracks;
  for (const auto &track : unmatched_unconfirmed_tracks) {
    track->mark_as_removed();
    new_removed_tracks.push_back(track);
  }

  // Add new tracks
  for (const auto &detection : new_detections) {
    if (detection->score() < high_thresh_) continue;
    track_id_count_++;
    TrackPtr new_track =
        std::make_shared<Track>(detection, frame_id_, track_id_count_);
    matched_tracks.push_back(new_track);
  }
  return new_removed_tracks;
}

std::vector<TrackPtr> BYTETracker::joint_tracks(
    const std::vector<TrackPtr> &a_tlist,
    const std::vector<TrackPtr> &b_tlist) const {
  std::set<int> exists;
  std::vector<TrackPtr> res;
  for (auto &track : a_tlist) {
    exists.emplace(track->get_track_id());
    res.push_back(track);
  }
  for (auto &track : b_tlist) {
    if (exists.count(track->get_track_id()) == 0) res.push_back(track);
  }
  return res;
}

std::vector<TrackPtr> BYTETracker::sub_tracks(
    const std::vector<TrackPtr> &a_tlist,
    const std::vector<TrackPtr> &b_tlist) const {
  std::map<int, TrackPtr> tracks;
  for (auto &track : a_tlist) tracks.emplace(track->get_track_id(), track);
  for (auto &track : b_tlist) tracks.erase(track->get_track_id());

  std::vector<TrackPtr> res;
  for (auto &[_, track] : tracks) res.push_back(track);
  return res;
}

std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>>
BYTETracker::remove_duplicate_tracks(
    const std::vector<TrackPtr> &a_tracks,
    const std::vector<TrackPtr> &b_tracks) const {
  if (a_tracks.empty() || b_tracks.empty()) return {a_tracks, b_tracks};

  std::vector<std::vector<float>> ious;
  ious.resize(a_tracks.size());
  for (size_t i = 0; i < ious.size(); i++) ious[i].resize(b_tracks.size());
  for (size_t ai = 0; ai < a_tracks.size(); ai++) {
    for (size_t bi = 0; bi < b_tracks.size(); bi++) {
      ious[ai][bi] = 1 - calc_iou(b_tracks[bi]->detection->rect(),
                                  a_tracks[ai]->detection->rect());
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

  std::vector<TrackPtr> a_tracks_out;
  for (size_t ai = 0; ai < a_tracks.size(); ai++) {
    if (!a_overlapping[ai]) a_tracks_out.push_back(a_tracks[ai]);
  }

  std::vector<TrackPtr> b_tracks_out;
  for (size_t bi = 0; bi < b_tracks.size(); bi++) {
    if (!b_overlapping[bi]) b_tracks_out.push_back(b_tracks[bi]);
  }
  return {std::move(a_tracks_out), std::move(b_tracks_out)};
}

std::tuple<std::vector<std::pair<TrackPtr, DetectionPtr>>,
           std::vector<TrackPtr>, std::vector<DetectionPtr>>
BYTETracker::linear_assignment(const std::vector<TrackPtr> &tracks,
                               const std::vector<DetectionPtr> &detections,
                               float thresh) const {
  if (tracks.empty() || detections.empty()) return {{}, tracks, detections};

  std::vector<std::vector<float>> cost_matrix;
  cost_matrix.resize(tracks.size());
  for (size_t i = 0; i < cost_matrix.size(); i++)
    cost_matrix[i].resize(detections.size());
  for (size_t bi = 0; bi < detections.size(); bi++) {
    for (size_t ai = 0; ai < tracks.size(); ai++) {
      cost_matrix[ai][bi] =
          1 - calc_iou(detections[bi]->rect(), tracks[ai]->detection->rect());
    }
  }

  std::vector<std::pair<TrackPtr, DetectionPtr>> matches;
  std::vector<TrackPtr> a_unmatched;
  std::vector<DetectionPtr> b_unmatched;

  auto [rowsol, colsol, _] = exec_lapjv(cost_matrix, true, thresh);
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

std::tuple<std::vector<int>, std::vector<int>, double> BYTETracker::exec_lapjv(
    const std::vector<std::vector<float>> &cost, bool extend_cost,
    float cost_limit, bool return_cost) const {
  int n_rows = cost.size();
  int n_cols = cost[0].size();
  std::vector<int> rowsol(n_rows);
  std::vector<int> colsol(n_cols);

  if (n_rows != n_cols && !extend_cost) {
    throw std::runtime_error("The `extend_cost` variable should set True");
  }

  int n = 0;
  std::vector<float> cost_c;
  if (extend_cost || cost_limit < std::numeric_limits<float>::max()) {
    n = n_rows + n_cols;
    cost_c.resize(n * n);

    if (cost_limit < std::numeric_limits<float>::max()) {
      cost_c.assign(cost_c.size(), cost_limit / 2.0);
    } else {
      float cost_max = -1;
      for (size_t i = 0; i < cost.size(); i++) {
        for (size_t j = 0; j < cost[i].size(); j++) {
          if (cost[i][j] > cost_max) cost_max = cost[i][j];
        }
      }
      cost_c.assign(cost_c.size(), cost_max + 1);
    }

    for (int i = n_rows; i < n; i++) {
      for (int j = n_cols; j < n; j++) {
        cost_c[i * n + j] = 0;
      }
    }
    for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
        cost_c[i * n + j] = cost[i][j];
      }
    }
  } else {
    n = n_rows;
    cost_c.resize(n_rows * n_cols);
    for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
        cost_c[i * n_cols + j] = cost[i][j];
      }
    }
  }

  std::vector<int> x_c(n), y_c(n);

  int ret = lapjv_internal(n, cost_c, x_c, y_c);
  if (ret != 0) {
    throw std::runtime_error("The result of lapjv_internal() is invalid.");
  }

  double opt = 0.0;

  if (n != n_rows) {
    for (int i = 0; i < n; i++) {
      if (x_c[i] >= n_cols) x_c[i] = -1;
      if (y_c[i] >= n_rows) y_c[i] = -1;
    }
    for (int i = 0; i < n_rows; i++) {
      rowsol[i] = x_c[i];
    }
    for (int i = 0; i < n_cols; i++) {
      colsol[i] = y_c[i];
    }

    if (return_cost) {
      for (size_t i = 0; i < rowsol.size(); i++) {
        if (rowsol[i] != -1) {
          opt += cost_c[i * n_cols + rowsol[i]];
        }
      }
    }
  } else if (return_cost) {
    for (size_t i = 0; i < rowsol.size(); i++) {
      opt += cost_c[i * n_cols + rowsol[i]];
    }
  }

  return {std::move(rowsol), std::move(colsol), opt};
}
}  // namespace byte_track
