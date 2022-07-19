#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/Detection.h"
#include "ByteTrack/Rect.h"
#include "ByteTrack/Track.h"

#include "boost/foreach.hpp"
#include "boost/optional.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "boost/property_tree/ptree.hpp"
#include "gtest/gtest.h"

#include <cstddef>
#include <memory>

namespace {

class RectImpl : public byte_track::Rect {
  float x_;
  float y_;
  float width_;
  float height_;

 public:
  RectImpl(float x = 0, float y = 0, float width = 0, float height = 0)
      : x_(x), y_(y), width_(width), height_(height) {}

  const float &left() const override { return x_; }
  const float &top() const override { return y_; }
  const float &width() const override { return width_; }
  const float &height() const override { return height_; }
};

class DetectionImpl : public byte_track::Detection {
  RectImpl rect_;
  float score_ = 0;

 public:
  DetectionImpl(const RectImpl &rect, float score)
      : rect_(rect), score_(score) {}

  const RectImpl &get_rect() const override { return rect_; }

  const float &get_score() const override { return score_; }
};

constexpr double EPS = 1e-2;

const std::string D_RESULTS_FILE = "detection_results.json";
const std::string T_RESULTS_FILE = "tracking_results.json";

// key: track_id, value: rect of tracking object
using BYTETrackerOut = std::map<size_t, RectImpl>;

template <typename T>
T get_data(const boost::property_tree::ptree &pt, const std::string &key) {
  T ret;
  if (boost::optional<T> data = pt.get_optional<T>(key)) {
    ret = data.get();
  } else {
    throw std::runtime_error(
        "Could not read the data from ptree: [key: " + key + "]");
  }
  return ret;
}

std::map<size_t, std::vector<byte_track::DetectionPtr>> get_inputs_ref(
    const boost::property_tree::ptree &pt) {
  std::map<size_t, std::vector<byte_track::DetectionPtr>> inputs_ref;
  BOOST_FOREACH (const boost::property_tree::ptree::value_type &child,
                 pt.get_child("results")) {
    const boost::property_tree::ptree &result = child.second;
    const auto frame_id = get_data<int>(result, "frame_id");
    const auto prob = get_data<float>(result, "prob");
    const auto x = get_data<float>(result, "x");
    const auto y = get_data<float>(result, "y");
    const auto width = get_data<float>(result, "width");
    const auto height = get_data<float>(result, "height");

    decltype(inputs_ref)::iterator itr = inputs_ref.find(frame_id);
    if (itr != inputs_ref.end()) {
      itr->second.emplace_back(
          std::make_shared<DetectionImpl>(RectImpl(x, y, width, height), prob));
    } else {
      std::vector<byte_track::DetectionPtr> v{
          std::make_shared<DetectionImpl>(RectImpl(x, y, width, height), prob)};
      inputs_ref.emplace_hint(inputs_ref.end(), frame_id, v);
    }
  }
  return inputs_ref;
}

std::map<size_t, BYTETrackerOut> get_outputs_ref(
    const boost::property_tree::ptree &pt) {
  std::map<size_t, BYTETrackerOut> outputs_ref;
  BOOST_FOREACH (const boost::property_tree::ptree::value_type &child,
                 pt.get_child("results")) {
    const boost::property_tree::ptree &result = child.second;
    const auto frame_id = get_data<int>(result, "frame_id");
    const auto track_id = get_data<int>(result, "track_id");
    const auto x = get_data<float>(result, "x");
    const auto y = get_data<float>(result, "y");
    const auto width = get_data<float>(result, "width");
    const auto height = get_data<float>(result, "height");

    decltype(outputs_ref)::iterator itr = outputs_ref.find(frame_id);
    if (itr != outputs_ref.end()) {
      itr->second.emplace(track_id, RectImpl(x, y, width, height));
    } else {
      BYTETrackerOut v{
          {track_id, RectImpl(x, y, width, height)},
      };
      outputs_ref.emplace_hint(outputs_ref.end(), frame_id, v);
    }
  }
  return outputs_ref;
}
}  // namespace

TEST(ByteTrack, BYTETracker) {
  boost::property_tree::ptree pt_d_results;
  boost::property_tree::read_json(D_RESULTS_FILE, pt_d_results);

  boost::property_tree::ptree pt_t_results;
  boost::property_tree::read_json(T_RESULTS_FILE, pt_t_results);

  try {
    // Get infomation of reference data
    const auto detection_results_name =
        get_data<std::string>(pt_d_results, "name");
    const auto tracking_results_name =
        get_data<std::string>(pt_t_results, "name");
    const auto fps = get_data<int>(pt_d_results, "fps");
    const auto track_buffer = get_data<int>(pt_d_results, "track_buffer");

    if (detection_results_name != tracking_results_name) {
      throw std::runtime_error(
          "The name of the tests are different: [detection_results_name: " +
          detection_results_name +
          ", tracking_results_name: " + tracking_results_name + "]");
    }

    // Get input reference data from D_RESULTS_FILE
    const auto inputs_ref = get_inputs_ref(pt_d_results);

    // Get output reference data from T_RESULTS_FILE
    auto outputs_ref = get_outputs_ref(pt_t_results);

    // Test BYTETracker::update()
    byte_track::BYTETracker tracker(fps, track_buffer);
    for (const auto &[frame_id, objects] : inputs_ref) {
      const auto outputs = tracker.update(objects);

      // Verify between the reference data and the output of the BYTETracker
      // impl
      EXPECT_EQ(outputs.size(), outputs_ref[frame_id].size());
      for (const auto &outputs_per_frame : outputs) {
        const auto &rect = outputs_per_frame->get_detection().get_rect();
        const auto &track_id = outputs_per_frame->get_track_id();
        const auto &ref = outputs_ref[frame_id][track_id];
        EXPECT_NEAR(ref.top(), rect.top(), EPS);
        EXPECT_NEAR(ref.left(), rect.left(), EPS);
        EXPECT_NEAR(ref.width(), rect.width(), EPS);
        EXPECT_NEAR(ref.height(), rect.height(), EPS);
      }
    }
  } catch (const std::exception &e) {
    FAIL() << e.what();
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
