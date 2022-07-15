#pragma once

#include "ByteTrack/STrack.h"
#include "ByteTrack/lapjv.h"

#include <array>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace byte_track {
class BYTETracker {
 public:
  BYTETracker(int frame_rate = 30, int track_buffer = 30,
              float track_thresh = 0.5, float high_thresh = 0.6,
              float match_thresh = 0.8);
  ~BYTETracker();

  std::vector<STrackPtr> update(const std::vector<STrackPtr> &objects);

 private:
  std::array<std::vector<STrackPtr>, 4> iouAssociation(
      const std::vector<STrackPtr> &track_pool,
      const std::vector<STrackPtr> &detections);

  std::vector<STrackPtr> lowScoreAssociation(
      std::vector<STrackPtr> &matched_tracks,
      std::vector<STrackPtr> &refound_tracks,
      const std::vector<STrackPtr> &low_score_detections,
      const std::vector<STrackPtr> &unmatched_tracked_tracks);

  std::vector<STrackPtr> initNewTracks(
      std::vector<STrackPtr> &matched_tracks,
      const std::vector<STrackPtr> &inactive_tracks,
      const std::vector<STrackPtr> &unmatched_detections);

  std::vector<STrackPtr> jointTracks(
      const std::vector<STrackPtr> &a_tlist,
      const std::vector<STrackPtr> &b_tlist) const;

  std::vector<STrackPtr> subTracks(const std::vector<STrackPtr> &a_tlist,
                                   const std::vector<STrackPtr> &b_tlist) const;

  std::vector<std::vector<float>> calcIouDistance(
      const std::vector<STrackPtr> &a_tracks,
      const std::vector<STrackPtr> &b_tracks) const;

  std::vector<std::vector<float>> calcIous(
      const std::vector<Rect> &a_rect, const std::vector<Rect> &b_rect) const;

  std::tuple<std::vector<STrackPtr>, std::vector<STrackPtr>>
  removeDuplicateTracks(const std::vector<STrackPtr> &a_tracks,
                        const std::vector<STrackPtr> &b_tracks) const;

  std::tuple<std::vector<std::pair<STrackPtr, STrackPtr>>,
             std::vector<STrackPtr>, std::vector<STrackPtr>>
  linearAssignment(const std::vector<STrackPtr> &a_tracks,
                   const std::vector<STrackPtr> &b_tracks, float thresh) const;

  double execLapjv(const std::vector<std::vector<float>> &cost,
                   std::vector<int> &rowsol, std::vector<int> &colsol,
                   bool extend_cost = false,
                   float cost_limit = std::numeric_limits<float>::max(),
                   bool return_cost = true) const;

 private:
  float track_thresh_;
  float high_thresh_;
  float match_thresh_;
  size_t max_time_lost_;

  size_t frame_id_;
  size_t track_id_count_;

  std::vector<STrackPtr> tracked_tracks_;
  std::vector<STrackPtr> lost_tracks_;
  std::vector<STrackPtr> removed_tracks_;
};
}  // namespace byte_track
