#include <object_tracker.hpp>
#include <hungarian.hpp>


ObjectTracker::ObjectTracker(const float min_new_track_distance) {
    this->tracks = std::vector<Track>();
    this->min_new_track_distance = min_new_track_distance;

    this->matches = std::vector<std::pair<size_t, size_t>>();
}

// destructor implicitly defined

KalmanFilter ObjectTracker::create_kf(const Eigen::Vector3d& initial_pos) {
    int n = 3; // Number of states (x, y, z)
    int m = 3; // Number of measurements (x, y, z) - FIXED from 1

    double dt = 1.0/30; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // State transition: x' = x (static object model)
    A << 1, 0, 0, 
         0, 1, 0, 
         0, 0, 1;
         
    // Measurement matrix: y = x (direct measurement)
    C << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Reasonable covariance matrices
    // Q (Process Noise): Small for static objects
    Q << 0.05, 0, 0, 
         0, 0.05, 0, 
         0, 0, 0.05;
         
    // R (Measurement Noise): Initial placeholder, will be overwritten by ZED cov
    R << 0.1, 0, 0, 
         0, 0.1, 0, 
         0, 0, 0.1;
         
    // P (Initial Error): High uncertainty initially
    P << 1.0, 0, 0, 
         0, 1.0, 0, 
         0, 0, 1.0;


    KalmanFilter kf(dt,A, C, Q, R, P);
    
    // initialize with initial states at zero
    kf.init(0.0, initial_pos);

    return kf;
}   

std::vector<Track> ObjectTracker::update(
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<Eigen::Matrix3d>& measurement_covariances,
    const std::vector<std::string>& classes,
    const std::vector<double>& orientations, 
    const std::vector<double>& confidences  
) {
    // 1. Compute cost matrix
    auto cost_matrix = compute_cost_matrix(measurements, measurement_covariances, classes);

    std::vector<size_t> unmatched_tracks;
    std::vector<size_t> unmatched_dets;
    
    // Initialize sets
    unmatched_tracks.reserve(tracks.size());
    unmatched_dets.reserve(measurements.size());
    for(size_t i = 0; i < tracks.size(); ++i) unmatched_tracks.push_back(i);
    for(size_t i = 0; i < measurements.size(); ++i) unmatched_dets.push_back(i);

    // 2. Match tracks
    auto matches = match_tracks(
        cost_matrix, 
        measurements.size(), 
        unmatched_tracks, 
        unmatched_dets
    );

    // 3. Update matched tracks
    update_matched_tracks(matches, measurements, measurement_covariances, orientations, confidences);

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
    const std::vector<Eigen::Matrix3d>& measurement_covariances,
    const std::vector<std::string>& classes
) {
    size_t num_tracks = this->tracks.size();
    size_t num_meas = measurements.size();
    size_t num_classes = classes.size();

    if (num_meas != num_classes) {
        throw std::invalid_argument("Input vector sizes unequal. Fix argument.");
    }

    // Initialize vector<vector<double>> with dimensions [num_tracks][num_meas], initialized to 0.0
    std::vector<std::vector<double>> cost_matrix(num_tracks, std::vector<double>(num_meas, 0.0));

    // Compute the cost matrix
    if (num_meas > 0) {
        for (size_t track_idx = 0; track_idx < num_tracks; ++track_idx) {
            const auto& curr_track = this->tracks[track_idx];

            for (size_t meas_idx = 0; meas_idx < num_meas; ++meas_idx) { 
                const std::string& meas_label = classes[meas_idx];

                // 1. Class mismatch check
                if (curr_track.label != meas_label) {
                    cost_matrix[track_idx][meas_idx] = 1e6;
                    continue;
                }

                Eigen::Vector3d meas = measurements[meas_idx];
                Eigen::Vector3d diff = meas - curr_track.kf.state(); // Assuming state() returns Vector3d compatible
                double euclidean_distance = diff.norm();

                // 2. Max position jump check
                if (euclidean_distance > max_position_jump) {
                    cost_matrix[track_idx][meas_idx] = 1e6;
                    continue;
                }

                // 3. Mahalanobis distance
                // S = C*P*C^T + R
                // Since C is Identity, S = P + R
                Eigen::Matrix3d P = curr_track.kf.covariance();
                Eigen::Matrix3d R_meas = measurement_covariances[meas_idx];
                Eigen::Matrix3d S = P + R_meas;
                
                Eigen::Matrix3d inv_S = S.inverse();
                
                double dist = diff.transpose() * inv_S * diff;

                // 4. Gating and Assignment
                if (dist > this->gating_threshold) {
                    cost_matrix[track_idx][meas_idx] = 1e6;
                } else {
                    cost_matrix[track_idx][meas_idx] = dist;
                }
            }
        }
    }
    return cost_matrix;
}


// Note: The Hungarian Algorithm library we are using only takes std::vector<std::vector>
// FUTURE TASK POTENTIAL:
// Implement a Hungarian Algorithm that takes MatrixXd cost matrix
// and leverage the temporal CPU cache benefits that the MatrixXd exploits
//
// This will significantly improve computational speed.
std::vector<std::pair<size_t, size_t>> ObjectTracker::match_tracks(
    const std::vector<std::vector<double>>& cost_matrix,
    size_t num_meas,
    std::vector<size_t>& unmatched_tracks,
    std::vector<size_t>& unmatched_detections
) {
    HungarianAlgorithm solver;
	
	// FIX: Clear matches from previous frame!
	matches.clear(); 
	
	// Suppress unused warning
	(void)num_meas;

    std::vector<int> assignment;
    // assignment is grown dynamically within the solver, no need to init with fixed size
    // res = overall cost of found optimal assignment
    solver.Solve(cost_matrix, assignment);

    // NOTE: at this point the unmatched detections and tracks are ordered lists
    // Simply remove item as specific index

    for (size_t assign_idx = 0; assign_idx < assignment.size(); ++assign_idx) {
        size_t track_idx = assign_idx;
        int det_idx = assignment[assign_idx];

        // (track_idx, det_idx) forms the pairing
        // TODO: Iterate through unmatched tracks and detections and remove from those lists, tentative implementation below
        // assignment is set to -1 if no assignment was made for that track
        // otherwise we check if we cost threshold
        if (det_idx != -1 && cost_matrix[track_idx][det_idx] <= gating_threshold) {
            // Valid match, at this point det_idx must be positive i.e. valid size_t
            matches.push_back(std::make_pair(track_idx, det_idx));

            // TODO: Refactor unmatching function to client-side to prevent
            // monolithic function
            // Remove from unmatched lists
            unmatched_tracks.erase(std::remove(unmatched_tracks.begin(), unmatched_tracks.end(), track_idx), unmatched_tracks.end());
            unmatched_detections.erase(std::remove(unmatched_detections.begin(), unmatched_detections.end(), det_idx), unmatched_detections.end());
        }
    }

    return matches;
}

void ObjectTracker::update_matched_tracks(
    const std::vector<std::pair<size_t, size_t>>& matches,
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<Eigen::Matrix3d>& measurement_covariances,
    const std::vector<double>& orientations,
    const std::vector<double>& confidences
) {
    for (const auto& match : matches) {
        int track_idx = match.first;
        int meas_idx = match.second;

        Track& track = this->tracks[track_idx];
        
        // Prepare measurement for kf
        Eigen::Vector3d meas = measurements[meas_idx];
        Eigen::Matrix3d meas_cov = measurement_covariances[meas_idx];

        // Update Kalman Filter with dynamic covariance
        track.kf.update(meas, meas_cov);

        // Update Track Metadata
        // ref: track metadata in .hpp file
        track.hits++;
        track.consecutive_hits++;

        // TODO: Refactor age increasing of tracks into the main loop
        track.age++;

        // TODO: Update to the time data type instead of using ints
        track.time_since_updates = 0;
        track.confidence = confidences[meas_idx];
        track.theta_z = orientations[meas_idx]; 

        
        if (track.state == TrackState::TENTATIVE && track.consecutive_hits >= min_hits) {
            track.state = TrackState::CONFIRMED;
        }
    }
}

void ObjectTracker::handle_unmatched_tracks(const std::vector<size_t>& unmatched_tracks) {
    for (size_t track_idx : unmatched_tracks) {
        Track& track = this->tracks[track_idx];

        track.age++;
        track.time_since_updates++;
        // STRICT MODE: Reset consecutive hits if track is missed
        track.consecutive_hits = 0; 
        
        // Downgrade confirmed tracks that lost detection
        if (track.state == TrackState::CONFIRMED && track.time_since_updates > conf_to_tent_threshold) {
             track.state = TrackState::TENTATIVE;
        }
    }
}

void ObjectTracker::delete_dead_tracks() {
    auto it = tracks.begin();
    while (it != tracks.end()) {
        bool delete_track = false;

        // time exceeded
        if (it->time_since_updates > max_age) {
            delete_track = true;
        }
        
        // tentative for too long 
        if (it->state == TrackState::TENTATIVE && it->age > min_hits + tent_init_buffer) {
            delete_track = true;
        }

        if (delete_track) {
            it = tracks.erase(it);
        } else {
            ++it;
        }
    }
}

void ObjectTracker::create_new_tracks(
    const std::vector<size_t>& unmatched_detections,
    const std::vector<Eigen::Vector3d>& measurements,
    const std::vector<std::string>& classes,
    const std::vector<double>& orientations,
    const std::vector<double>& confidences
) {
    for (int det_idx : unmatched_detections) {
        std::string label = classes[det_idx];

        int current_count = 0;
        for (const auto& t : tracks) {
            if (t.label == label) current_count++;
        }

        // If we already have number of tracks for that class, skip creation
        if (MAX_PER_CLASS.find(label) != MAX_PER_CLASS.end()) {
            if (current_count >= MAX_PER_CLASS[label]) {
                continue;
            }
        }

        Track new_track;
        new_track.id = track_id_counter++;
        new_track.label = label;
        new_track.state = TrackState::TENTATIVE;
        new_track.hits = 1;
        new_track.consecutive_hits = 1;
        new_track.age = 1;
        new_track.time_since_updates = 0;
        new_track.theta_z = orientations[det_idx];
        new_track.confidence = confidences[det_idx];

        new_track.kf = create_kf(measurements[det_idx]);

        tracks.push_back(new_track);
    }
}
