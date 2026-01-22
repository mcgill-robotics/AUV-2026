#pragma once

#include <iostream>
#include <memory.h>
#include <format>
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
    CONFIRMED,
    DELETED
};

struct Track {
    int id;
    std::string label;
    TrackState state;

    int hits = 0;               // total number of matched measurements
    int age = 0;                // total frames since creation
    int time_since_updates = 0; // if this is > max_age, delete the track
    
    KalmanFilter kf;

    double theta_z = 0.0;
    double confidence = 0.0;
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
    Eigen::MatrixXd compute_cost_matrix(
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<std::string>& classes
    );  

    // Step 2: Match tracks to detections (HUNGARIAN)
    // returns: matches, and modifies the unmatched sets by reference
    std::vector<std::pair<int, int>> match_tracks(
        const Eigen::Ref<const Eigen::MatrixXd>& cost_matrix,
        size_t num_meas,
        std::vector<int>& unmatched_tracks,
        std::vector<int>& unmatched_detections
    );

    // Step 3: Update existing tracks with matched measurements
    void update_matched_tracks(
        const std::vector<std::pair<int, int>>& matches,
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<double>& orientations,
        const std::vector<double>& confidences
    );

    // Step 4: Handle tracks that weren't seen this frame
    void handle_unmatched_tracks(const std::vector<int>& unmatched_tracks);

    // Step 5: Prune dead tracks
    void delete_dead_tracks();
    
    // Step 6: Create new tracks from unmatched detections
    void create_new_tracks(
        const std::vector<int>& unmatched_detections,
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

    std::vector<int> matches;
    std::vector<int> untracked_tracks;
    std::vector<int> untracked_detections;

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
