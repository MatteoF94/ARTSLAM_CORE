#ifndef ARTSLAM_CORE_KEYFRAME_H
#define ARTSLAM_CORE_KEYFRAME_H

#include <string>

namespace artslam::core::types {
    // Semi-abstract class representing a keyframe, which is a milestone in the trajectory of the robot
    class Keyframe {
    public:
        // Constructor, with default parameters
        // - timestamp: timestamp, in nanoseconds, of the keyframe
        // - accumulated_distance: accumulated distance, in meters, from the first keyframe or a given milestone
        // - reliability: value associated to the distance metric between two keyframes
        explicit Keyframe(uint64_t timestamp, double accumulated_distance = 0, double reliability = 0);

        // Simple destructor
        virtual ~Keyframe();

    public:
        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        uint64_t timestamp_ = 0;            // timestamp, in nanoseconds, of the keyframe
        double accumulated_distance_ = 0;   // accumulated distance, in meters, from the first keyframe or a given milestone
        double reliability_ = 0;            // value associated to the distance metric between two keyframes
    };
}


#endif //ARTSLAM_CORE_KEYFRAME_H
