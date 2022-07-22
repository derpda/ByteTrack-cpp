#pragma once

#include "ByteTrack/Detection.h"
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

  std::vector<TrackPtr> update(const std::vector<DetectionPtr> &objects);

  void clear();

 private:
  std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>,
             std::vector<DetectionPtr>>
  iou_association(const std::vector<TrackPtr> &track_pool,
                  const std::vector<DetectionPtr> &detections);

  std::vector<TrackPtr> low_score_association(
      std::vector<TrackPtr> &matched_tracks,
      const std::vector<DetectionPtr> &low_score_detections,
      const std::vector<TrackPtr> &unmatched_tracked_tracks);

  std::vector<TrackPtr> init_new_tracks(
      std::vector<TrackPtr> &matched_tracks,
      const std::vector<TrackPtr> &inactive_tracks,
      const std::vector<DetectionPtr> &unmatched_detections);

  std::vector<TrackPtr> joint_tracks(
      const std::vector<TrackPtr> &a_tlist,
      const std::vector<TrackPtr> &b_tlist) const;

  std::vector<TrackPtr> sub_tracks(const std::vector<TrackPtr> &a_tlist,
                                   const std::vector<TrackPtr> &b_tlist) const;

  std::tuple<std::vector<TrackPtr>, std::vector<TrackPtr>>
  remove_duplicate_tracks(const std::vector<TrackPtr> &a_tracks,
                          const std::vector<TrackPtr> &b_tracks) const;

  std::tuple<std::vector<std::pair<TrackPtr, DetectionPtr>>,
             std::vector<TrackPtr>, std::vector<DetectionPtr>>
  linear_assignment(const std::vector<TrackPtr> &tracks,
                    const std::vector<DetectionPtr> &detections,
                    float thresh) const;

  std::tuple<std::vector<int>, std::vector<int>, double> exec_lapjv(
      const std::vector<std::vector<float>> &cost, bool extend_cost = false,
      float cost_limit = std::numeric_limits<float>::max(),
      bool return_cost = true) const;

 private:
  const float track_thresh_;
  const float high_thresh_;
  const float match_thresh_;
  const size_t max_time_lost_;

  size_t frame_id_;
  size_t track_id_count_;

  std::vector<TrackPtr> tracked_tracks_;
  std::vector<TrackPtr> lost_tracks_;
  std::vector<TrackPtr> removed_tracks_;
};
}  // namespace byte_track
