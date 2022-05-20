#include "keyframe.h"

namespace artslam::core::types {
    // Constructor with parameters:
    // - timestamp: timestamp, in nanoseconds, of the keyframe
    // - accumulated_distance: accumulated distance, in meters, from the first keyframe or a given milestone
    // - reliability: value associated to the distance metric between two keyframes
    Keyframe::Keyframe(uint64_t timestamp, double accumulated_distance, double reliability) : timestamp_(timestamp),
                                                                                              accumulated_distance_(
                                                                                                      accumulated_distance),
                                                                                              reliability_(
                                                                                                      reliability) {

    }

    // Simple destructor
    Keyframe::~Keyframe() = default;
}
