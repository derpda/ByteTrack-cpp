#pragma once

#include "ByteTrack/Detection.h"
#include "ByteTrack/Track.h"

namespace byte_track {

template <DetectionType DetType, UserTrackType<DetType> UserTrack>
class BYTETracker;

}  // namespace byte_track
