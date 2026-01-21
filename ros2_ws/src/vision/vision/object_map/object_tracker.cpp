#include <iostream>
#include <memory.h>
#include <format>
#include <chrono>

#include <object_tracker.hpp>


ObjectTracker::ObjectTracker(const float min_new_track_distance = 0.5) {
    this->tracks = std::vector<Track>();
    this->min_new_track_distance = min_new_track_distance;
}

// destructor implicitly defined

KalmanFilter ObjectTracker::create_kf(const Eigen::Vector3d& initial_pos) {
    int n = 3; // Number of states
    int m = 1; // Number of measurements

    double dt = 1.0/30; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    A << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    C << 1, 0, 0;

    // Reasonable covariance matrices
    // TODO: Reset with actual measurement covariance from YOLO
    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    R << 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;


    KalmanFilter kf(dt,A, C, Q, R, P);
    
    // initialize with initial states at zero
    kf.init()

    return kf
}   

std::vector<Track> ObjectTracker::update(
    const std::const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<Eigen::Matrix3d>& measurement_covariances,
    const std::vector<std::string>& classes,
    const std::vector<double>& orientations, 
    const std::vector<double>& confidences  
) {
    // 1. Compute cost matrix
    auto cost_matrix = compute_cost_matrix(measurements, classes);

    // 2. Match tracks
    std::set<int> unmatched_tracks;
    std::set<int> unmatched_dets;
    auto matches = match_tracks(cost_matrix, measurements.size(), unmatched_tracks, unmatched_dets);

    // 3. Update matched tracks
    update_matched_tracks(matches, measurements, orientations, confidences);

    // 4. Handle unmatched tracks
    handle_unmatched_tracks(unmatched_tracks);

    // 5. Delete dead tracks
    delete_dead_tracks();

    // 6. Create new tracks
    create_new_tracks(unmatched_dets, measurements, classes, orientations, confidences);

    return tracks;
}  

std::vector<std::vector<double>> ObjectTracker::compute_cost_matrix(
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<std::string>& classes
) {
    // TODO: Implement cost matrix computation (Mahalanobis distance + Gating)
    return {};
}

std::vector<std::pair<int, int>> ObjectTracker::match_tracks(
    const std::vector<std::vector<double>>& cost_matrix,
    size_t num_meas,
    std::set<int>& unmatched_tracks,
    std::set<int>& unmatched_dets
) {
    // TODO: Implement matching logic (Hungarian Algorithm or Greedy Match)
    return {};
}

void ObjectTracker::update_matched_tracks(
    const std::vector<std::pair<int, int>>& matches,
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<std::string>& classes,
    const std::vector<double>& orientations,
    const std::vector<double>& confidences
) {
    // TODO: Update KF state, reset age, update orientation/confidence, and promote tentative tracks
}

void ObjectTracker::handle_unmatched_tracks(const std::set<int>& unmatched_tracks) {
    // TODO: Increment age of tracks not seen in this frame
}

void ObjectTracker::delete_dead_tracks() {
    // TODO: Remove tracks that have exceeded max_age or time_since_update thresholds
}

void ObjectTracker::create_new_tracks(
    const std::set<int>& unmatched_dets,
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<std::string>& classes,
    const std::vector<double>& orientations,
    const std::vector<double>& confidences
) {
    // TODO: Initialize new tracks for unmatched detections (check MAX_PER_CLASS limits)
}
