#pragma once

#include "ByteTrack/Track.h"
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

  std::vector<TrackPtr> update(const std::vector<TrackPtr> &objects);

 private:
  std::array<std::vector<TrackPtr>, 4> iouAssociation(
      const std::vector<TrackPtr> &track_pool,
      const std::vector<TrackPtr> &detections);

  std::vector<TrackPtr> lowScoreAssociation(
      std::vector<TrackPtr> &matched_tracks,
      std::vector<TrackPtr> &refound_tracks,
      const std::vector<TrackPtr> &low_score_detections,
      const std::vector<TrackPtr> &unmatched_tracked_tracks);

  std::vector<TrackPtr> initNewTracks(
      std::vector<TrackPtr> &matched_tracks,
      const std::vector<TrackPtr> &inactive_tracks,
      const std::vector<TrackPtr> &unmatched_detections);

  std::vector<TrackPtr> jointTracks(const std::vector<TrackPtr> &a_tlist,
                                    const std::vector<TrackPtr> &b_tlist) const;

  std::vector<TrackPtr> subTracks(const std::vector<TrackPtr> &a_tlist,
                                  const std::vector<TrackPtr> &b_tlist) const;

  std::vector<std::vector<float>> calcIouDistance(
      const std::vector<TrackPtr> &a_tracks,
      const std::vector<TrackPtr> &b_tracks) const;

  std::vector<std::vector<float>> calcIous(
      const std::vector<Rect> &a_rect, const std::vector<Rect> &b_rect) const;

  std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>>
  removeDuplicateTracks(const std::vector<TrackPtr> &a_tracks,
                        const std::vector<TrackPtr> &b_tracks) const;

  std::tuple<std::vector<std::pair<TrackPtr, TrackPtr>>, std::vector<TrackPtr>,
             std::vector<TrackPtr>>
  linearAssignment(const std::vector<TrackPtr> &a_tracks,
                   const std::vector<TrackPtr> &b_tracks, float thresh) const;

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

  std::vector<TrackPtr> tracked_tracks_;
  std::vector<TrackPtr> lost_tracks_;
  std::vector<TrackPtr> removed_tracks_;
};
}  // namespace byte_track
