#pragma once

#include <algorithm>
#include <iostream>
#include <memory.h>
#include <vector>
#include <string>
#include <set>
#include <unordered_map>
#include <unordered_set>
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
    int age = 0;                // frames since last hit (lost frames)
    int total_updates = 0;      // total frames since creation
    
    KalmanFilter kf;

    double theta_z = 0.0;
    bool has_orientation = false;
    double confidence = 0.0;

    Eigen::Vector3d get_position() const {
        return kf.state();
    }
};

class ObjectTracker {
public: 

    explicit ObjectTracker(
        const std::unordered_map<std::string, int>& max_per_class = {},
        const std::vector<std::string>& large_structure_labels = {},
        const std::vector<std::string>& pipe_labels = {},
        float min_new_track_distance = 0.5,
        float min_large_structure_separation = 2.0,
        float min_large_structure_pipe_separation = 1.0,
        float gating_threshold = 3.5,
        int min_hits = 20,
        int max_age = 8,
        float max_position_jump = 2.0,
        int conf_to_tent_threshold = 5,
        int tent_init_buffer = 5,
        const std::vector<std::string>& semi_persistent_labels = {},
        int semi_persistent_conf_to_tent_threshold = 300,
        bool enable_gate_midpoint_refinement = true,
        bool enable_board_icon_refinement = true,
        float refinement_plausibility_radius = 5.0
    );

    ~ObjectTracker() = default;

    KalmanFilter create_kf(const Eigen::Vector3d& initial_pos);

    // Accepts current frame data and returns the list of CONFIRMED tracks
    // persistent_positions: positions of objects that are no longer tracked but still 
    // exist in the persistent map (used for large-structure proximity checks)
    std::vector<Track> update(
        const std::vector<Eigen::Vector3d>& measurements,
        const std::vector<Eigen::Matrix3d>& measurement_covariances,
        const std::vector<std::string>& classes,
        const std::vector<double>& orientations,
        const std::vector<double>& confidences,
        const Eigen::Vector3d& observer_position,
        bool has_observer_position,
        const std::vector<std::pair<std::string, Eigen::Vector3d>>& persistent_positions = {}
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
        const std::vector<double>& confidences,
        const std::vector<std::pair<std::string, Eigen::Vector3d>>& persistent_positions
    );

    // Step 7: Post-processing constraints applied to tracking states
    void apply_physical_constraints();
    void apply_gate_physical_constraints();
    void apply_board_physical_constraints();

    std::vector<Track> tracks;

    int track_id_counter = 1;
    float min_new_track_distance;   // set within constructor
    float min_large_structure_separation;
    float min_large_structure_pipe_separation;
        
    // Tuning Parameters
    float gating_threshold;         // Mahalanobis gate (~3 sigma)
    int min_hits;                   // CONSECUTIVE frames to confirm
    int max_age;                    // Frames to keep lost track
    float max_position_jump;        // Max jump (meters) - higher to handle VIO rotation errors

    // Track State Transition Parameters
    int conf_to_tent_threshold;     // Misses before downgrading CONFIRMED -> TENTATIVE
    int tent_init_buffer;           // Extra frames allowed for initialization before zombie cull

    bool enable_gate_midpoint_refinement;
    bool enable_board_icon_refinement;
    float refinement_plausibility_radius;
    Eigen::Vector3d observer_position = Eigen::Vector3d::Zero();
    bool has_observer_position = false;
    
    std::vector<std::pair<size_t,size_t>> matches;

    // Known object limits (prevents creating too many tracks per class, -1 = unlimited)
    std::unordered_map<std::string, int> max_per_class;
    std::unordered_set<std::string> large_structure_labels;
    std::unordered_set<std::string> pipe_labels;

    std::unordered_set<std::string> semi_persistent_labels;
    int semi_persistent_conf_to_tent_threshold;

};
