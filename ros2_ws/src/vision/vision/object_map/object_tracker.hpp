#pragma once

#include <iostream>
#include <memory.h>
#include <format>
#include <vector>
#include <string>

#include <unordered_map>

#include <Eigen/Dense>

#include "kalman.hpp"

using double_ms = std::chrono::duration<double, std::milli>;

enum class TrackState {
    TENTATIVE,
    CONFIRMED
};

struct Track {
    int id;
    std::string label;
    KalmanFilter kf;
    int consecutive_hits = 0;
    int hits = 0;
    int age = 0;
    int total_updates = 0;
    TrackState state = TrackState::TENTATIVE;
};


class ObjectTracker {
public: 

    explicit ObjectTracker(const float min_new_track_distance = 0.5);

    ~ObjectTracker() = default;

    KalmanFilter create_kf(const Eigen::Vector3d& initial_pos);

    // Accepts current frame data and returns the list of CONFIRMED tracks
    std::vector<Track> update(
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<Eigen::Matrix3d>& measurement_covariances,
        const std::vector<std::string>& classes
    ); 

private:
    std::vector<Track> tracks;

    int track_id_counter = 1;
    float min_new_track_distance;   // set within constructor
        
    // Tuning Parameters
    float gating_threshold = 3.5;   // Mahalanobis gate (~3 sigma)
    int min_hits = 20;              // CONSECUTIVE frames to confirm
    int max_age = 8;                // Frames to keep lost track
    float max_position_jump = 2.0;  // Max jump (meters) - higher to handle VIO rotation errors


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
