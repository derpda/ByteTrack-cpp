#pragma once

#include "ByteTrack/Object.h"
#include "ByteTrack/STrack.h"
#include "ByteTrack/lapjv.h"

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <vector>

namespace byte_track {
class BYTETracker {
 public:
  using STrackPtr = std::shared_ptr<STrack>;

  BYTETracker(int frame_rate = 30, int track_buffer = 30,
              float track_thresh = 0.5, float high_thresh = 0.6,
              float match_thresh = 0.8);
  ~BYTETracker();

  std::vector<STrackPtr> update(const std::vector<Object> &objects);

 private:
  std::array<std::vector<STrackPtr>, 4> iou_association(
      const std::vector<STrackPtr> &strack_pool,
      const std::vector<STrackPtr> &det_stracks);

  std::vector<STrackPtr> low_score_association(
      std::vector<STrackPtr> &current_tracked_stracks,
      std::vector<STrackPtr> &refind_stracks,
      const std::vector<STrackPtr> det_low_stracks,
      const std::vector<STrackPtr> &remain_tracked_stracks);

  std::vector<STrackPtr> init_new_stracks(
      std::vector<STrackPtr> &current_tracked_stracks,
      const std::vector<STrackPtr> &non_active_stracks,
      const std::vector<STrackPtr> &remain_det_stracks);

  std::vector<STrackPtr> jointStracks(
      const std::vector<STrackPtr> &a_tlist,
      const std::vector<STrackPtr> &b_tlist) const;

  std::vector<STrackPtr> subStracks(
      const std::vector<STrackPtr> &a_tlist,
      const std::vector<STrackPtr> &b_tlist) const;

  void removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                              const std::vector<STrackPtr> &b_stracks,
                              std::vector<STrackPtr> &a_res,
                              std::vector<STrackPtr> &b_res) const;

  void linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                        int cost_matrix_size, int cost_matrix_size_size,
                        float thresh, std::vector<std::vector<int>> &matches,
                        std::vector<int> &b_unmatched,
                        std::vector<int> &a_unmatched) const;

  std::vector<std::vector<float>> calcIouDistance(
      const std::vector<STrackPtr> &a_tracks,
      const std::vector<STrackPtr> &b_tracks) const;

  std::vector<std::vector<float>> calcIous(
      const std::vector<Rect<float>> &a_rect,
      const std::vector<Rect<float>> &b_rect) const;

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

  std::vector<STrackPtr> tracked_stracks_;
  std::vector<STrackPtr> lost_stracks_;
  std::vector<STrackPtr> removed_stracks_;
};
}  // namespace byte_track
