#include <iostream>
#include <memory.h>
#include <format>

#include <object_tracker.hpp>


ObjectTracker::ObjectTracker(const float min_new_track_distance = 0.5) {
    this->min_new_track_distance = min_new_track_distance;
}

// destructor implicitly defined

void create_kf() {

}   // TODO: Add Eigen 3D vector type to constructor

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
}   // TODO: Parameterize properly

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
    // TODO: Implement matching logic (e.g., Hungarian Algorithm or Greedy Match)
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
    // TODO: Remove tracks that have exceeded max_age
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
