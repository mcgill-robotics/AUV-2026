#pragma once

#include <algorithm>
#include <iostream>
#include <memory.h>
#include <vector>
#include <string>
#include <set>
#include <unordered_map>
#include <chrono>

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
    TrackState state;

    int hits = 0;               // total number of matched measurements
    int consecutive_hits = 0;   // CONSECUTIVE matched frames (resets on miss)
    int age = 0;                // total frames since creation
    int time_since_updates = 0; // if this is > max_age, delete the track
    
    KalmanFilter kf;

    double theta_z = 0.0;
    double confidence = 0.0;

    Eigen::Vector3d get_position() const {
        return kf.state();
    }
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
        const std::vector<std::string>& classes,
        const std::vector<double>& orientations,
        const std::vector<double>& confidences 
    ); 

private:
    // Breaking down the update() logic into specific steps
    // Step 1: Compute the cost matrix (MAHALANOBIS)

    // TODO: Potentially update into a matrix return type instead?
    std::vector<std::vector<double>> compute_cost_matrix(
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<Eigen::Matrix3d>& measurement_covariances,
        const std::vector<std::string>& classes
    );  

    // Step 2: Match tracks to detections (HUNGARIAN)
    // returns: matches, and modifies the unmatched sets by reference
    std::vector<std::pair<size_t, size_t>> match_tracks(
        const std::vector<std::vector<double>>& cost_matrix,
        size_t num_meas,
        std::vector<size_t>& unmatched_tracks,
        std::vector<size_t>& unmatched_detections
    );

    // Step 3: Update existing tracks with matched measurements
    void update_matched_tracks(
        const std::vector<std::pair<size_t, size_t>>& matches,
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<Eigen::Matrix3d>& measurement_covariances,
        const std::vector<double>& orientations,
        const std::vector<double>& confidences
    );

    // Step 4: Handle tracks that weren't seen this frame
    void handle_unmatched_tracks(const std::vector<size_t>& unmatched_tracks);

    // Step 5: Prune dead tracks
    void delete_dead_tracks();
    
    // Step 6: Create new tracks from unmatched detections
    void create_new_tracks(
        const std::vector<size_t>& unmatched_detections,
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<std::string>& classes,
        const std::vector<double>& orientations,
        const std::vector<double>& confidences
    );


    std::vector<Track> tracks;

    int track_id_counter = 1;
    float min_new_track_distance;   // set within constructor
        
    // Tuning Parameters
    float gating_threshold = 3.5;   // Mahalanobis gate (~3 sigma)
    int min_hits = 20;              // CONSECUTIVE frames to confirm
    int max_age = 8;                // Frames to keep lost track
    float max_position_jump = 2.0;  // Max jump (meters) - higher to handle VIO rotation errors
    
    // Track State Transition Parameters
    int conf_to_tent_threshold = 10; // Misses before downgrading CONFIRMED -> TENTATIVE
    int tent_init_buffer = 5;       // Extra frames allowed for initialization before zombie cull

    std::vector<std::pair<size_t,size_t>> matches;

    // Known object limits (prevents creating too many tracks per class)
    std::unordered_map<std::string, int> MAX_PER_CLASS = {
        { "gate", 1 },
        { "lane_marker", 2 }, 
        { "red_pipe", 3 }, 
        { "white_pipe", 6 }, 
        { "octagon", 1 },
        { "table", 1 }, 
        { "bin", 1 }, 
        { "board", 1 }, 
        { "shark", 2 },
        { "sawfish", 2 }
    };

};
