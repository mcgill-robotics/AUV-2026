#include <object_tracker.hpp>
#include <hungarian.hpp>

#include <cmath>

namespace {

constexpr double kRefinementPlausibilityRadiusMeters = 5.0;

Eigen::Vector2d xy(const Eigen::Vector3d& position) {
    return position.head<2>();
}

double xy_distance(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
    return (xy(lhs) - xy(rhs)).norm();
}

bool compute_facing_normal(
    const Eigen::Vector2d& axis_xy,
    const Eigen::Vector2d& object_xy,
    const Eigen::Vector2d& observer_xy,
    Eigen::Vector2d& chosen_normal)
{
    if (axis_xy.norm() < 1e-6) {
        return false;
    }

    Eigen::Vector2d to_observer = observer_xy - object_xy;
    if (to_observer.norm() < 1e-6) {
        return false;
    }

    Eigen::Vector2d axis_unit = axis_xy.normalized();
    Eigen::Vector2d candidate_a(-axis_unit.y(), axis_unit.x());
    Eigen::Vector2d candidate_b = -candidate_a;

    chosen_normal =
        candidate_a.dot(to_observer) >= candidate_b.dot(to_observer)
            ? candidate_a
            : candidate_b;
    return true;
}

bool compute_facing_yaw(
    const Eigen::Vector2d& axis_xy,
    const Eigen::Vector2d& object_xy,
    const Eigen::Vector2d& observer_xy,
    double& yaw_out,
    Eigen::Vector2d* normal_out = nullptr)
{
    Eigen::Vector2d chosen_normal;
    if (!compute_facing_normal(axis_xy, object_xy, observer_xy, chosen_normal)) {
        return false;
    }

    if (normal_out != nullptr) {
        *normal_out = chosen_normal;
    }

    yaw_out = std::atan2(chosen_normal.y(), chosen_normal.x());
    return true;
}

}  // namespace

ObjectTracker::ObjectTracker(
    const std::unordered_map<std::string, int>& max_per_class,
    const std::vector<std::string>& large_structure_labels,
    const std::vector<std::string>& pipe_labels,
    float min_new_track_distance,
    float min_large_structure_separation,
    float min_large_structure_pipe_separation,
    float gating_threshold,
    int min_hits,
    int max_age,
    float max_position_jump,
    int conf_to_tent_threshold,
    int tent_init_buffer,
    bool enable_gate_midpoint_refinement,
    bool enable_board_icon_refinement
) {
    this->tracks = std::vector<Track>();
    this->matches = std::vector<std::pair<size_t, size_t>>();
    this->max_per_class = max_per_class;
    this->large_structure_labels =
        std::unordered_set<std::string>(large_structure_labels.begin(), large_structure_labels.end());
    this->pipe_labels =
        std::unordered_set<std::string>(pipe_labels.begin(), pipe_labels.end());

    this->min_new_track_distance = min_new_track_distance;
    this->min_large_structure_separation = min_large_structure_separation;
    this->min_large_structure_pipe_separation = min_large_structure_pipe_separation;
    this->gating_threshold = gating_threshold;
    this->min_hits = min_hits;
    this->max_age = max_age;
    this->max_position_jump = max_position_jump;
    this->conf_to_tent_threshold = conf_to_tent_threshold;
    this->tent_init_buffer = tent_init_buffer;
    this->enable_gate_midpoint_refinement = enable_gate_midpoint_refinement;
    this->enable_board_icon_refinement = enable_board_icon_refinement;
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
    // Q (Process Noise): Extremely small for static objects (enforces heavy inertia)
    Q << 0.001, 0, 0, 
         0, 0.001, 0, 
         0, 0, 0.001;
         
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
    const std::vector<double>& confidences,
    const Eigen::Vector3d& observer_position,
    bool has_observer_position
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

    this->observer_position = observer_position;
    this->has_observer_position = has_observer_position;
    
    // 7. Post-processing: Apply physical domain constraints to tracking state
    apply_physical_constraints();

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
                
                double dist_sq = diff.transpose() * inv_S * diff;
                double dist = std::sqrt(dist_sq);

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
    if (cost_matrix.empty()) {
        // All detections are unmatched
        // unmatched_tracks is already full of indices (but size 0)
        // unmatched_detections is already full of indices
        return {};
    }

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
        } else {
             // Debug print for rejected match?
             // std::cerr << "[TRACKER] Rejected match: track " << track_idx << " det " << det_idx << " cost " << cost_matrix[track_idx][det_idx] << std::endl;
        }
    }

    // Identify unmatched tracks and detections
    // Efficiently remove matched indices
    std::set<size_t> matched_track_indices;
    std::set<size_t> matched_det_indices;
    for(const auto& match : matches) {
        matched_track_indices.insert(match.first);
        matched_det_indices.insert(match.second);
    }

    // Filter unmatched_tracks
    auto track_it = unmatched_tracks.begin();
    while(track_it != unmatched_tracks.end()) {
        if(matched_track_indices.count(*track_it)) {
            track_it = unmatched_tracks.erase(track_it);
        } else {
            ++track_it;
        }
    }

    // Filter unmatched_detections
    auto det_it = unmatched_detections.begin();
    while(det_it != unmatched_detections.end()) {
        if(matched_det_indices.count(*det_it)) {
            det_it = unmatched_detections.erase(det_it);
        } else {
            ++det_it;
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
        
        if (track_idx < 0 || track_idx >= (int)tracks.size()) {
            continue;
        }
        if (meas_idx < 0 || meas_idx >= (int)measurements.size()) {
             continue;
        }

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
        track.total_updates++;

        // TODO: Update to the time data type instead of using ints
        track.age = 0;
        track.confidence = confidences[meas_idx];
        
        // Don't overwrite the calculated gate orientation with the default measurement orientation
        if (!track.has_orientation) {
            track.theta_z = orientations[meas_idx]; 
        }

        
        if (track.state == TrackState::TENTATIVE && track.consecutive_hits >= min_hits) {
            track.state = TrackState::CONFIRMED;
        }
    }
}

void ObjectTracker::handle_unmatched_tracks(const std::vector<size_t>& unmatched_tracks) {
    for (size_t track_idx : unmatched_tracks) {
        Track& track = this->tracks[track_idx];

        track.total_updates++;
        track.age++;
        // STRICT MODE: Reset consecutive hits if track is missed
        track.consecutive_hits = 0; 
        
        // Downgrade confirmed tracks that lost detection
        if (track.state == TrackState::CONFIRMED && track.age > conf_to_tent_threshold) {
             track.state = TrackState::TENTATIVE;
        }
    }
}

void ObjectTracker::delete_dead_tracks() {
    auto it = tracks.begin();
    while (it != tracks.end()) {
        bool delete_track = false;

        // time exceeded
        if (it->age > max_age) {
            delete_track = true;
        }
        
        // tentative for too long 
        if (it->state == TrackState::TENTATIVE && it->total_updates > min_hits + tent_init_buffer) {
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
        const bool is_large_structure = large_structure_labels.count(label) > 0;

        int current_count = 0;
        for (const auto& t : tracks) {
            if (t.label == label) {
                current_count++;
            }
        }

        // If we already have number of tracks for that class, skip creation
        if (max_per_class.find(label) != max_per_class.end()) {
            if (max_per_class[label] != -1 && current_count >= max_per_class[label]) {
                continue;
            }
        }

        // Check 2: Too close to ANY existing track? (Prevent duplicate objects of different classes)
        bool too_close = false;
        Eigen::Vector3d new_pos = measurements[det_idx];
        for (const auto& existing_track : tracks) {
            Eigen::Vector3d existing_pos = existing_track.kf.state();
            double dist = (new_pos - existing_pos).norm();
            if (existing_track.label == label && dist < min_new_track_distance) {
                too_close = true;
                break;
            }
        }

        if (!too_close && is_large_structure) {
            for (const auto& existing_track : tracks) {
                if (existing_track.state != TrackState::CONFIRMED) {
                    continue;
                }

                const Eigen::Vector3d existing_pos = existing_track.kf.state();
                const double dist_xy = xy_distance(new_pos, existing_pos);

                if (pipe_labels.count(existing_track.label) > 0 &&
                    dist_xy < min_large_structure_pipe_separation) {
                    too_close = true;
                    break;
                }

                if (large_structure_labels.count(existing_track.label) > 0 &&
                    existing_track.label != label &&
                    dist_xy < min_large_structure_separation) {
                    too_close = true;
                    break;
                }
            }
        }
        
        // Skip this detection, its space is already occupied
        if (too_close) {
            continue;
        }

        Track new_track;
        new_track.id = track_id_counter++;
        new_track.label = label;
        new_track.state = TrackState::TENTATIVE;
        new_track.hits = 1;
        new_track.consecutive_hits = 1;
        new_track.total_updates = 1;
        new_track.age = 0;
        new_track.theta_z = orientations[det_idx];
        new_track.confidence = confidences[det_idx];

        new_track.kf = create_kf(measurements[det_idx]);

        tracks.push_back(new_track);
    }
}

void ObjectTracker::apply_physical_constraints() {
    // Apply physical realm constraints that affect the tracked position 
    apply_gate_physical_constraints();
    apply_board_physical_constraints();
}

void ObjectTracker::apply_gate_physical_constraints() {
    if (!enable_gate_midpoint_refinement) {
        return;
    }

    Track* gate_track = nullptr;
    Track* search_rescue_track = nullptr;
    Track* survey_repair_track = nullptr;

    // Single loop to find all necessary tracks
    for (auto& t : tracks) {
        // Only use confirmed tracks for refinement and orientation
        if (t.state != TrackState::CONFIRMED) continue;

        if (t.label == "gate") gate_track = &t;
        else if (t.label == "search_rescue") search_rescue_track = &t;
        else if (t.label == "survey_repair") survey_repair_track = &t;
    }

    if (gate_track && search_rescue_track && survey_repair_track) {
        Eigen::Vector3d p_search_rescue = search_rescue_track->kf.state();
        Eigen::Vector3d p_survey_repair = survey_repair_track->kf.state();
        Eigen::Vector3d p_gate = gate_track->kf.state();
        
        // 1. Position Refinement
        Eigen::Vector3d midpoint = (p_search_rescue + p_survey_repair) / 2.0;

        if ((p_gate - midpoint).norm() < kRefinementPlausibilityRadiusMeters) {
            Eigen::Matrix3d tiny_R = Eigen::Matrix3d::Identity() * 0.001;
            gate_track->kf.update(midpoint, tiny_R);
            p_gate = gate_track->kf.state();
        }

        if (has_observer_position) {
            Eigen::Vector2d gate_span = xy(p_survey_repair) - xy(p_search_rescue);
            double yaw = 0.0;
            if (compute_facing_yaw(gate_span, xy(p_gate), xy(observer_position), yaw)) {
                gate_track->theta_z = yaw;
                gate_track->has_orientation = true;
            }
        }
    }
}

void ObjectTracker::apply_board_physical_constraints() {
    if (!enable_board_icon_refinement) {
        return;
    }

    Track* board_track = nullptr;
    Track* ambulance_track = nullptr;
    Track* firetruck_track = nullptr;
    Track* blood_track = nullptr;
    Track* fire_track = nullptr;

    for (auto& t : tracks) {
        if (t.state != TrackState::CONFIRMED) continue;

        if (t.label == "board") board_track = &t;
        else if (t.label == "ambulance") ambulance_track = &t;
        else if (t.label == "firetruck") firetruck_track = &t;
        else if (t.label == "blood") blood_track = &t;
        else if (t.label == "fire") fire_track = &t;
    }

    if (!board_track) {
        return;
    }

    const bool has_vehicle_pair = ambulance_track && firetruck_track;
    const bool has_hazard_pair = fire_track && blood_track;
    if (!has_vehicle_pair && !has_hazard_pair) {
        return;
    }

    Eigen::Vector3d refined_board_position = board_track->kf.state();
    if (has_vehicle_pair && has_hazard_pair) {
        Eigen::Vector3d centroid =
            (ambulance_track->kf.state() + firetruck_track->kf.state() +
             fire_track->kf.state() + blood_track->kf.state()) /
            4.0;
        if ((refined_board_position - centroid).norm() < kRefinementPlausibilityRadiusMeters) {
            board_track->kf.update(centroid, Eigen::Matrix3d::Identity() * 0.001);
            refined_board_position = board_track->kf.state();
        }
    } else if (has_vehicle_pair) {
        Eigen::Vector3d midpoint =
            (ambulance_track->kf.state() + firetruck_track->kf.state()) / 2.0;
        if ((refined_board_position - midpoint).norm() < kRefinementPlausibilityRadiusMeters) {
            board_track->kf.update(midpoint, Eigen::Matrix3d::Identity() * 0.001);
            refined_board_position = board_track->kf.state();
        }
    } else {
        Eigen::Vector3d midpoint =
            (fire_track->kf.state() + blood_track->kf.state()) / 2.0;
        if ((refined_board_position - midpoint).norm() < kRefinementPlausibilityRadiusMeters) {
            board_track->kf.update(midpoint, Eigen::Matrix3d::Identity() * 0.001);
            refined_board_position = board_track->kf.state();
        }
    }

    if (!has_observer_position) {
        return;
    }

    std::vector<Eigen::Vector2d> chosen_normals;

    if (has_vehicle_pair) {
        Eigen::Vector2d normal;
        if (compute_facing_normal(
                xy(firetruck_track->kf.state()) - xy(ambulance_track->kf.state()),
                xy(refined_board_position),
                xy(observer_position),
                normal)) {
            chosen_normals.push_back(normal);
        }
    }

    if (has_hazard_pair) {
        Eigen::Vector2d normal;
        if (compute_facing_normal(
                xy(blood_track->kf.state()) - xy(fire_track->kf.state()),
                xy(refined_board_position),
                xy(observer_position),
                normal)) {
            chosen_normals.push_back(normal);
        }
    }

    if (chosen_normals.empty()) {
        return;
    }

    Eigen::Vector2d fused_normal = Eigen::Vector2d::Zero();
    for (const auto& normal : chosen_normals) {
        fused_normal += normal;
    }

    if (fused_normal.norm() < 1e-6) {
        return;
    }

    fused_normal.normalize();
    board_track->theta_z = std::atan2(fused_normal.y(), fused_normal.x());
    board_track->has_orientation = true;
}
