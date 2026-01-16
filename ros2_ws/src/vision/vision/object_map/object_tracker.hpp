#include <iostream>
#include <memory.h>
#include <format>

#include <unordered_map>

#include <Eigen/Dense>

using double_ms = std::chrono::duration<double, std::milli>;

class ObjectTracker {
public:

    explicit ObjectTracker(const float min_new_track_distance = 0.5);

    ~ObjectTracker() = default;

    void create_kf();  // TODO: Add Eigen 3D vector type to constructor

    void update(); // TODO: Parameterize properly

private:

    float min_new_track_distance = 0.5;

    int track_id_counter = 1;
        
    // Tuning Parameters
    float gating_threshold = 3.5;   // Mahalanobis gate (~3 sigma)
    int min_hits = 20;              // CONSECUTIVE frames to confirm
    int max_age = 8;                // Frames to keep lost track
    float max_position_jump = 2.0;  // Max jump (meters) - higher to handle VIO rotation errors

    float min_new_track_distance;   // set within constructor

    // Known object limits (prevents creating too many tracks per class)
    std::unordered_map<std::string, int> MAX_PER_CLASS = {
        { "gate", 3 },
        { "lane_marker", 2 }, 
        { "red_pipe", 2 }, 
        { "white_pipe", 2 }, 
        { "octagon", 2 },
        { "table", 1 }, 
        { "bin", 1 }, 
        { "board", 1 }, 
        { "shark", 1 },
        { "sawfish", 2 }
    };

}