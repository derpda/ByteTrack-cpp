#include "ByteTrack/BYTETracker.h"

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

BYTETracker::~BYTETracker() {}

std::array<std::vector<TrackPtr>, 4> BYTETracker::iouAssociation(
    const std::vector<TrackPtr> &track_pool,
    const std::vector<TrackPtr> &detections) {
  auto [matches, unmatched_tracks, unmatched_detections] =
      linearAssignment(track_pool, detections, match_thresh_);

  std::vector<TrackPtr> matched_tracks;
  std::vector<TrackPtr> refound_tracks;
  for (const auto &match : matches) {
    const auto track = match.first;
    const auto detection = match.second;
    if (track->getTrackState() == TrackState::Tracked) {
      track->update(*detection, frame_id_);
      matched_tracks.push_back(track);
    } else {
      track->reActivate(*detection, frame_id_);
      refound_tracks.push_back(track);
    }
  }

  std::vector<TrackPtr> unmatched_tracked_tracks;
  for (const auto &unmatch : unmatched_tracks) {
    if (unmatch->getTrackState() == TrackState::Tracked) {
      unmatched_tracked_tracks.push_back(unmatch);
    }
  }
  return {std::move(matched_tracks), std::move(unmatched_tracked_tracks),
          std::move(unmatched_detections), std::move(refound_tracks)};
}

std::vector<TrackPtr> BYTETracker::lowScoreAssociation(
    std::vector<TrackPtr> &matched_tracks,
    std::vector<TrackPtr> &refound_tracks,
    const std::vector<TrackPtr> &low_score_detections,
    const std::vector<TrackPtr> &unmatched_tracked_tracks) {
  auto [matches, unmatched_tracks, unmatch_detection] =
      linearAssignment(unmatched_tracked_tracks, low_score_detections, 0.5);

  for (const auto &match : matches) {
    const auto track = match.first;
    const auto detection = match.second;
    if (track->getTrackState() == TrackState::Tracked) {
      track->update(*detection, frame_id_);
      matched_tracks.push_back(track);
    } else {
      track->reActivate(*detection, frame_id_);
      refound_tracks.push_back(track);
    }
  }

  std::vector<TrackPtr> new_lost_tracks;
  for (const auto &track : unmatched_tracks) {
    if (track->getTrackState() != TrackState::Lost) {
      track->markAsLost();
      new_lost_tracks.push_back(track);
    }
  }
  return new_lost_tracks;
}

std::vector<TrackPtr> BYTETracker::initNewTracks(
    std::vector<TrackPtr> &matched_tracks,
    const std::vector<TrackPtr> &inactive_tracks,
    const std::vector<TrackPtr> &unmatched_detections) {
  // Deal with unconfirmed tracks, usually tracks with only one beginning frame
  auto [matches, unconfirmed_tracks, new_tracks] =
      linearAssignment(inactive_tracks, unmatched_detections, 0.7);

  for (const auto &match : matches) {
    match.first->update(*match.second, frame_id_);
    matched_tracks.push_back(match.first);
  }

  std::vector<TrackPtr> new_removed_tracks;
  for (const auto &track : unconfirmed_tracks) {
    track->markAsRemoved();
    new_removed_tracks.push_back(track);
  }

  // Add new tracks
  for (const auto &track : new_tracks) {
    if (track->getScore() < high_thresh_) continue;
    track_id_count_++;
    track->activate(frame_id_, track_id_count_);
    matched_tracks.push_back(track);
  }
  return new_removed_tracks;
}

// std::array<std::vector<TrackPtr>, 3> BYTETracker::matchDetections(
//     const std::vector<TrackPtr> &input_detections) {
//   // Sort new tracks from detection by score
//   std::vector<TrackPtr> detections;
//   std::vector<TrackPtr> low_score_detections;
//   for (const auto &track : input_detections) {
//     if (track->getScore() >= track_thresh_)
//       detections.push_back(track);
//     else
//       low_score_detections.push_back(track);
//   }

//   // Sort existing tracks by activity
//   std::vector<TrackPtr> active_tracks;
//   std::vector<TrackPtr> inactive_tracks;
//   for (const auto &track : tracked_tracks_) {
//     if (!track->isActivated())
//       inactive_tracks.push_back(track);
//     else
//       active_tracks.push_back(track);
//   }

//   std::vector<TrackPtr> track_pool;
//   track_pool = jointTracks(active_tracks, lost_tracks_);

//   // First match with detections that meet the score threshold
//   auto [matched_tracks, unmatched_tracked_tracks, unmatched_detections,
//         refound_tracks] = iouAssociation(track_pool, detections);

//   // Then match with detections that do not
//   auto new_lost_tracks =
//       lowScoreAssociation(matched_tracks, refound_tracks,
//       low_score_detections,
//                           unmatched_tracked_tracks);

//   // Deal with inactive tracks, usually tracks with only one beginning frame
//   auto [matches, unconfirmed_tracks, new_tracks] =
//       linearAssignment(inactive_tracks, unmatched_detections, 0.7);

//   for (const auto &match : matches) {
//     match.first->update(*match.second, frame_id_);
//     matched_tracks.push_back(match.first);
//   }

//   std::vector<TrackPtr> new_removed_tracks;
//   for (const auto &track : unconfirmed_tracks) {
//     track->markAsRemoved();
//     new_removed_tracks.push_back(track);
//   }

//   return {std::move(matched_tracks), std::move(new_lost_tracks),
//           std::move(new_removed_tracks)};
// }

std::vector<TrackPtr> BYTETracker::update(
    const std::vector<TrackPtr> &input_detections) {
  frame_id_++;

  ////////// Step 1: Get detections                                   //////////

  // Sort new tracks from detection by score
  std::vector<TrackPtr> detections;
  std::vector<TrackPtr> low_score_detections;
  for (const auto &track : input_detections) {
    if (track->getScore() >= track_thresh_)
      detections.push_back(track);
    else
      low_score_detections.push_back(track);
  }

  // Sort existing tracks by activity
  std::vector<TrackPtr> active_tracks;
  std::vector<TrackPtr> inactive_tracks;

  for (const auto &track : tracked_tracks_) {
    if (!track->isActivated())
      inactive_tracks.push_back(track);
    else
      active_tracks.push_back(track);
  }

  std::vector<TrackPtr> track_pool;
  track_pool = jointTracks(active_tracks, lost_tracks_);

  // Predict current pose by KF
  for (auto &track : track_pool) track->predict();

  ////////// Step 2: Find matches between tracks and detections       //////////
  ////////// Step 2: First association, with IoU                      //////////
  auto [matched_tracks, unmatched_tracked_tracks, unmatched_detections,
        refound_tracks] = iouAssociation(track_pool, detections);

  ////////// Step 3: Second association, using low score dets         //////////
  auto new_lost_tracks =
      lowScoreAssociation(matched_tracks, refound_tracks, low_score_detections,
                          unmatched_tracked_tracks);

  ////////// Step 4: Init new tracks                                  //////////
  auto new_removed_tracks =
      initNewTracks(matched_tracks, inactive_tracks, unmatched_detections);

  ////////// Step 5: Update state                                     //////////
  for (auto &lost_track : lost_tracks_) {
    if (frame_id_ - lost_track->getFrameId() > max_time_lost_) {
      lost_track->markAsRemoved();
      new_removed_tracks.push_back(lost_track);
    }
  }

  tracked_tracks_ = jointTracks(matched_tracks, refound_tracks);
  lost_tracks_ = subTracks(
      jointTracks(subTracks(lost_tracks_, tracked_tracks_), new_lost_tracks),
      removed_tracks_);
  removed_tracks_ = jointTracks(removed_tracks_, new_removed_tracks);

  std::tie(tracked_tracks_, lost_tracks_) =
      removeDuplicateTracks(tracked_tracks_, lost_tracks_);

  std::vector<TrackPtr> output_tracks;
  for (const auto &track : tracked_tracks_) {
    if (track->isActivated()) {
      output_tracks.push_back(track);
    }
  }

  return output_tracks;
}

std::vector<TrackPtr> BYTETracker::jointTracks(
    const std::vector<TrackPtr> &a_tlist,
    const std::vector<TrackPtr> &b_tlist) const {
  std::set<int> exists;
  std::vector<TrackPtr> res;
  for (auto &track : a_tlist) {
    exists.emplace(track->getTrackId());
    res.push_back(track);
  }
  for (auto &track : b_tlist) {
    if (exists.count(track->getTrackId()) == 0) res.push_back(track);
  }
  return res;
}

std::vector<TrackPtr> BYTETracker::subTracks(
    const std::vector<TrackPtr> &a_tlist,
    const std::vector<TrackPtr> &b_tlist) const {
  std::map<int, TrackPtr> tracks;
  for (auto &track : a_tlist) tracks.emplace(track->getTrackId(), track);
  for (auto &track : b_tlist) tracks.erase(track->getTrackId());

  std::vector<TrackPtr> res;
  for (auto &[_, track] : tracks) res.push_back(track);
  return res;
}

std::vector<std::vector<float>> BYTETracker::calcIouDistance(
    const std::vector<TrackPtr> &a_tracks,
    const std::vector<TrackPtr> &b_tracks) const {
  if (a_tracks.empty() || b_tracks.empty()) return {};

  std::vector<std::vector<float>> ious;
  ious.resize(a_tracks.size());
  for (size_t i = 0; i < ious.size(); i++) {
    ious[i].resize(b_tracks.size());
  }

  for (size_t bi = 0; bi < b_tracks.size(); bi++) {
    for (size_t ai = 0; ai < a_tracks.size(); ai++) {
      ious[ai][bi] = b_tracks[bi]->getRect().calcIoU(a_tracks[ai]->getRect());
    }
  }

  std::vector<std::vector<float>> cost_matrix;
  for (size_t i = 0; i < ious.size(); i++) {
    std::vector<float> iou;
    for (size_t j = 0; j < ious[i].size(); j++) {
      iou.push_back(1 - ious[i][j]);
    }
    cost_matrix.push_back(iou);
  }

  return cost_matrix;
}

std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>>
BYTETracker::removeDuplicateTracks(
    const std::vector<TrackPtr> &a_tracks,
    const std::vector<TrackPtr> &b_tracks) const {
  const auto ious = calcIouDistance(a_tracks, b_tracks);

  std::vector<bool> a_overlapping(a_tracks.size(), false),
      b_overlapping(b_tracks.size(), false);
  for (size_t ai = 0; ai < ious.size(); ai++) {
    for (size_t bi = 0; bi < ious[ai].size(); bi++) {
      if (ious[ai][bi] < 0.15) {
        const int timep =
            a_tracks[ai]->getFrameId() - a_tracks[ai]->getStartFrameId();
        const int timeq =
            b_tracks[bi]->getFrameId() - b_tracks[bi]->getStartFrameId();
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

std::tuple<std::vector<std::pair<TrackPtr, TrackPtr>>, std::vector<TrackPtr>,
           std::vector<TrackPtr>>
BYTETracker::linearAssignment(const std::vector<TrackPtr> &a_tracks,
                              const std::vector<TrackPtr> &b_tracks,
                              float thresh) const {
  const auto cost_matrix = calcIouDistance(a_tracks, b_tracks);
  if (cost_matrix.size() == 0) return {{}, a_tracks, b_tracks};

  std::vector<std::pair<TrackPtr, TrackPtr>> matches;
  std::vector<TrackPtr> a_unmatched, b_unmatched;

  std::vector<int> rowsol;
  std::vector<int> colsol;
  execLapjv(cost_matrix, rowsol, colsol, true, thresh);
  for (size_t i = 0; i < rowsol.size(); i++) {
    if (rowsol[i] >= 0)
      matches.push_back({a_tracks[i], b_tracks[rowsol[i]]});
    else
      a_unmatched.push_back(a_tracks[i]);
  }

  for (size_t i = 0; i < colsol.size(); i++) {
    if (colsol[i] < 0) b_unmatched.push_back(b_tracks[i]);
  }
  return {std::move(matches), std::move(a_unmatched), std::move(b_unmatched)};
}

double BYTETracker::execLapjv(const std::vector<std::vector<float>> &cost,
                              std::vector<int> &rowsol,
                              std::vector<int> &colsol, bool extend_cost,
                              float cost_limit, bool return_cost) const {
  std::vector<std::vector<float>> cost_c;
  cost_c.assign(cost.begin(), cost.end());

  std::vector<std::vector<float>> cost_c_extended;

  int n_rows = cost.size();
  int n_cols = cost[0].size();
  rowsol.resize(n_rows);
  colsol.resize(n_cols);

  int n = 0;
  if (n_rows == n_cols) {
    n = n_rows;
  } else {
    if (!extend_cost) {
      throw std::runtime_error("The `extend_cost` variable should set True");
    }
  }

  if (extend_cost || cost_limit < std::numeric_limits<float>::max()) {
    n = n_rows + n_cols;
    cost_c_extended.resize(n);
    for (size_t i = 0; i < cost_c_extended.size(); i++)
      cost_c_extended[i].resize(n);

    if (cost_limit < std::numeric_limits<float>::max()) {
      for (size_t i = 0; i < cost_c_extended.size(); i++) {
        for (size_t j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_limit / 2.0;
        }
      }
    } else {
      float cost_max = -1;
      for (size_t i = 0; i < cost_c.size(); i++) {
        for (size_t j = 0; j < cost_c[i].size(); j++) {
          if (cost_c[i][j] > cost_max) cost_max = cost_c[i][j];
        }
      }
      for (size_t i = 0; i < cost_c_extended.size(); i++) {
        for (size_t j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_max + 1;
        }
      }
    }

    for (size_t i = n_rows; i < cost_c_extended.size(); i++) {
      for (size_t j = n_cols; j < cost_c_extended[i].size(); j++) {
        cost_c_extended[i][j] = 0;
      }
    }
    for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
        cost_c_extended[i][j] = cost_c[i][j];
      }
    }

    cost_c.clear();
    cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
  }

  double **cost_ptr;
  cost_ptr = new double *[sizeof(double *) * n];
  for (int i = 0; i < n; i++) cost_ptr[i] = new double[sizeof(double) * n];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      cost_ptr[i][j] = cost_c[i][j];
    }
  }

  int *x_c = new int[sizeof(int) * n];
  int *y_c = new int[sizeof(int) * n];

  int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
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
          opt += cost_ptr[i][rowsol[i]];
        }
      }
    }
  } else if (return_cost) {
    for (size_t i = 0; i < rowsol.size(); i++) {
      opt += cost_ptr[i][rowsol[i]];
    }
  }

  for (int i = 0; i < n; i++) {
    delete[] cost_ptr[i];
  }
  delete[] cost_ptr;
  delete[] x_c;
  delete[] y_c;

  return opt;
}
}  // namespace byte_track
