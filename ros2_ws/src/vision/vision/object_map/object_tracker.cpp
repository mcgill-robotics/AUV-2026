#include <object_tracker.hpp>
#include <hungarian.hpp>

#include <cmath>

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kTinyCovariance = 0.01;

Eigen::Vector2d xy(const Eigen::Vector3d& position) {
    return position.head<2>();
}

double xy_distance(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
    return (xy(lhs) - xy(rhs)).norm();
}
/// @brief Computes the vector that is normal to the given axis and faces towards the observer.
/// @param axis_xy The axis vector in the XY plane that we want to compute the normal from.
/// @param object_xy Reference point for direction in which observer is facing
/// @param observer_xy The position of the observer in the XY plane.
/// @param chosen_normal The computed normal vector. filled in only if the function returns true.
/// @return True if the normal vector was computed successfully, false otherwise.
bool compute_facing_normal(
    const Eigen::Vector2d& axis_xy,
    const Eigen::Vector2d& object_xy,
    const Eigen::Vector2d& observer_xy,
    Eigen::Vector2d& chosen_normal)
{
    // if axis is too small, we can't determine direction
    if (axis_xy.norm() < kEpsilon) {
        return false;
    }

    // if observer is too close to object, we can't reliably determine facing direction (could be on either side of axis)
    Eigen::Vector2d to_observer = observer_xy - object_xy;
    if (to_observer.norm() < kEpsilon) {
        return false;
    }
    
    // normals can be either (-y,x) or (y,-x). we want to choose the one that faces towards the observer.
    Eigen::Vector2d axis_unit = axis_xy.normalized();
    Eigen::Vector2d positive_unit_normal(-axis_unit.y(), axis_unit.x());
    Eigen::Vector2d positive_negative_normal = -positive_unit_normal;

    // large positive dot product implies vectors in the same direction
    // so pick the larger dot product to get the normal that faces towards the observer
    chosen_normal =
        positive_unit_normal.dot(to_observer) >= positive_negative_normal.dot(to_observer)
            ? positive_unit_normal
            : positive_negative_normal;
    return true;
}
/// @brief Computes the yaw angle that makes the object face towards the observer, first computing the normal vector to the given axis and then computing the angle from that normal.
/// @param axis_xy Axis we want to face head on (such that we point toward the axis' normal).
/// @param object_xy Reference point for direction in which observer is facing
/// @param observer_xy The position of the observer in the XY plane.
/// @param yaw_out The computed yaw angle.
/// @param normal_out The computed normal vector.
/// @return True if the yaw angle was computed successfully, false otherwise.
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
    // compute angle of normal to get the absolute orientation that faces axis_xy
    yaw_out = std::atan2(chosen_normal.y(), chosen_normal.x());
    return true;
}

}  // namespace

ObjectTracker::ObjectTracker(
    const std::unordered_map<std::string, int>& max_per_class,
    const std::vector<std::string>& large_structure_labels,
    const std::vector<std::string>& pipe_labels,
    float min_new_track_distance,
    bool large_structure_separation_enabled,
    float min_large_structure_separation,
    bool large_structure_pipe_separation_enabled,
    float min_large_structure_pipe_separation,
    bool same_label_pipe_separation_enabled,
    float min_same_label_pipe_separation,
    float gating_threshold,
    int min_hits,
    int max_age,
    float max_position_jump,
    int conf_to_tent_threshold,
    int tent_init_buffer,
    const std::vector<std::string>& semi_persistent_labels,
    int semi_persistent_conf_to_tent_threshold,
    bool enable_gate_midpoint_refinement,
    bool enable_board_icon_refinement,
    float refinement_plausibility_radius
) {
    this->tracks = std::vector<Track>();
    this->matches = std::vector<std::pair<size_t, size_t>>();
    this->max_per_class = max_per_class;
    this->large_structure_labels =
        std::unordered_set<std::string>(large_structure_labels.begin(), large_structure_labels.end());
    this->pipe_labels =
        std::unordered_set<std::string>(pipe_labels.begin(), pipe_labels.end());
 
    this->min_new_track_distance = min_new_track_distance;
    this->large_structure_separation_enabled = large_structure_separation_enabled;
    this->min_large_structure_separation = min_large_structure_separation;
    this->large_structure_pipe_separation_enabled = large_structure_pipe_separation_enabled;
    this->min_large_structure_pipe_separation = min_large_structure_pipe_separation;
    this->same_label_pipe_separation_enabled = same_label_pipe_separation_enabled;
    this->min_same_label_pipe_separation = min_same_label_pipe_separation;
    this->gating_threshold = gating_threshold;
    this->min_hits = min_hits;
    this->max_age = max_age;
    this->max_position_jump = max_position_jump;
    this->conf_to_tent_threshold = conf_to_tent_threshold;
    this->tent_init_buffer = tent_init_buffer;
    this->enable_gate_midpoint_refinement = enable_gate_midpoint_refinement;
    this->enable_board_icon_refinement = enable_board_icon_refinement;
    this->refinement_plausibility_radius = refinement_plausibility_radius;
    this->semi_persistent_labels = std::unordered_set<std::string>(semi_persistent_labels.begin(), semi_persistent_labels.end());
    this->semi_persistent_conf_to_tent_threshold = semi_persistent_conf_to_tent_threshold;
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
    Q << kTinyCovariance, 0, 0, 
         0, kTinyCovariance, 0, 
         0, 0, kTinyCovariance;
         
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
    bool has_observer_position,
    const std::vector<std::pair<std::string, Eigen::Vector3d>>& persistent_positions
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
    create_new_tracks(unmatched_dets, measurements, classes, orientations, confidences, persistent_positions);

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

                // 3. Euclidean distance cost
                // We previously used Mahalanobis distance, but poor covariance estimates
                // caused it to blow up and spawn duplicate tracks.
                double dist = euclidean_distance;

                // 4. Gating and Assignment
                // Since we are using Euclidean distance, we gate using max_position_jump.
                // We still use gating_threshold here if we want to retain the YAML parameter, 
                // but for Euclidean distance, max_position_jump is more intuitive.
                // Let's use max_position_jump as the true gate, and set cost to dist.
                cost_matrix[track_idx][meas_idx] = dist;

                // PREVIOUS cost
                // // 3. Mahalanobis distance
                // // S = C*P*C^T + R
                // // Since C is Identity, S = P + R
                // Eigen::Matrix3d P = curr_track.kf.covariance();
                // Eigen::Matrix3d R_meas = measurement_covariances[meas_idx];
                // Eigen::Matrix3d S = P + R_meas;
                
                // Eigen::Matrix3d inv_S = S.inverse();
                
                // double dist_sq = diff.transpose() * inv_S * diff;
                // double dist = std::sqrt(dist_sq);

                // // 4. Gating and Assignment
                // if (dist > this->gating_threshold) {
                //     cost_matrix[track_idx][meas_idx] = 1e6;
                // } else {
                //     cost_matrix[track_idx][meas_idx] = dist;
                // }
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
        // Iterate through unmatched tracks and detections and remove from those lists, tentative implementation below
        // assignment is set to -1 if no assignment was made for that track
        // otherwise we check if we cost threshold
        if (det_idx != -1 && cost_matrix[track_idx][det_idx] <= max_position_jump) {
            // Valid match, at this point det_idx must be positive i.e. valid size_t
            matches.push_back(std::make_pair(track_idx, det_idx));
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

        track.age = 0;
        track.confidence = confidences[meas_idx];
        
        // Don't overwrite the calculated gate orientation with the default measurement orientation
        if (!track.has_orientation && !std::isnan(orientations[meas_idx])) {
            track.theta_z = orientations[meas_idx]; 
        }

        
        if (track.state == TrackState::TENTATIVE && track.hits >= min_hits) {
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
        int effective_threshold = (semi_persistent_labels.count(track.label) > 0)
            ? semi_persistent_conf_to_tent_threshold
            : conf_to_tent_threshold;

        if (track.state == TrackState::CONFIRMED && track.age > effective_threshold) {
             track.state = TrackState::TENTATIVE;
        }
    }
}

void ObjectTracker::delete_dead_tracks() {
    auto it = tracks.begin();
    while (it != tracks.end()) {
        bool delete_track = false;

        // tentative for too long 
        if (it->state == TrackState::TENTATIVE && it->total_updates - it->hits > tent_init_buffer) {
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
    const std::vector<double>& confidences,
    const std::vector<std::pair<std::string, Eigen::Vector3d>>& persistent_positions
) {
    for (int det_idx : unmatched_detections) {
        std::string label = classes[det_idx];
        const bool is_large_structure = large_structure_labels.count(label) > 0;
        const bool is_pipe = pipe_labels.count(label) > 0;

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
            if (existing_track.label != label) {
                continue;
            }
            Eigen::Vector3d existing_pos = existing_track.kf.state();
            double dist = (new_pos - existing_pos).norm();
            if (dist < min_new_track_distance) {
                too_close = true;
                break;
            }
        }
        // special consideration for large structures
        if (!too_close && is_large_structure) {
            for (const auto& existing_track : tracks) {
                if (existing_track.state != TrackState::CONFIRMED) {
                    continue;
                }

                const Eigen::Vector3d existing_pos = existing_track.kf.state();
                const double dist_xy = xy_distance(new_pos, existing_pos);
                // check if new large structure is too close to existing pipe
                if (large_structure_pipe_separation_enabled && pipe_labels.count(existing_track.label) > 0 &&
                    dist_xy < min_large_structure_pipe_separation) {
                    too_close = true;
                    break;
                }
                // check if new large structure is too close to existing large structure of different class
                if (large_structure_separation_enabled && large_structure_labels.count(existing_track.label) > 0 &&
                    existing_track.label != label &&
                    dist_xy < min_large_structure_separation) {
                    too_close = true;
                    break;
                }
            }
        }
        // Also check persistent objects that have been pruned from the tracker
        // but still exist in the object map (prevents placing a new large structure
        // where a previously confirmed one was)
        if (!too_close && is_large_structure) {
            for (const auto& [persistent_label, persistent_pos] : persistent_positions) {
                const double dist_xy = xy_distance(new_pos, persistent_pos);
                if (large_structure_pipe_separation_enabled && pipe_labels.count(persistent_label) > 0 &&
                    dist_xy < min_large_structure_pipe_separation) {
                    too_close = true;
                    break;
                }
                if (large_structure_separation_enabled && large_structure_labels.count(persistent_label) > 0 &&
                    persistent_label != label &&
                    dist_xy < min_large_structure_separation) {
                    too_close = true;
                    break;
                }
            }
        }

        // Prevent pipes from spawning too close to existing large structures (such as the gate)
        if (!too_close && is_pipe && large_structure_pipe_separation_enabled) {
            for (const auto& existing_track : tracks) {
                if (existing_track.state != TrackState::CONFIRMED) {
                    continue;
                }
                if (large_structure_labels.count(existing_track.label) > 0) {
                    const double dist_xy = xy_distance(new_pos, existing_track.kf.state());
                    if (dist_xy < min_large_structure_pipe_separation) {
                        too_close = true;
                        break;
                    }
                }
            }
            if (!too_close) {
                for (const auto& [persistent_label, persistent_pos] : persistent_positions) {
                    if (large_structure_labels.count(persistent_label) > 0) {
                        const double dist_xy = xy_distance(new_pos, persistent_pos);
                        if (dist_xy < min_large_structure_pipe_separation) {
                            too_close = true;
                            break;
                        }
                    }
                }
            }
        }

        // Prevent slalom pipes of the same label (e.g. red_pipe and red_pipe) from spawning too close to each other
        if (!too_close && is_pipe && same_label_pipe_separation_enabled) {
            for (const auto& existing_track : tracks) {
                if (existing_track.label == label) {
                    const double dist_xy = xy_distance(new_pos, existing_track.kf.state());
                    if (dist_xy < min_same_label_pipe_separation) {
                        too_close = true;
                        break;
                    }
                }
            }
            if (!too_close) {
                for (const auto& [persistent_label, persistent_pos] : persistent_positions) {
                    if (persistent_label == label) {
                        const double dist_xy = xy_distance(new_pos, persistent_pos);
                        if (dist_xy < min_same_label_pipe_separation) {
                            too_close = true;
                            break;
                        }
                    }
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
        new_track.theta_z = std::isnan(orientations[det_idx]) ? 0.0 : orientations[det_idx];
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
        if (t.label == "gate") {
            gate_track = &t;
        }
        // Only use confirmed tracks for panel constraints
        else if (t.state == TrackState::CONFIRMED) {
            if (t.label == "search_rescue") search_rescue_track = &t;
            else if (t.label == "survey_repair") survey_repair_track = &t;
        }
    }

    if (search_rescue_track && survey_repair_track) {
        Eigen::Vector3d p_search_rescue = search_rescue_track->kf.state();
        Eigen::Vector3d p_survey_repair = survey_repair_track->kf.state();
        
        double panel_distance = (p_search_rescue - p_survey_repair).norm();
        // If panels are too far apart, they might be false positives, do not create/refine gate
        if (panel_distance > refinement_plausibility_radius) {
            return;
        }

        Eigen::Vector3d midpoint = (p_search_rescue + p_survey_repair) / 2.0;

        if (gate_track) {
            Eigen::Vector3d p_gate = gate_track->kf.state();
            // only update if refined position is within plausibility radius
            if ((p_gate - midpoint).norm() < refinement_plausibility_radius) {
                // Use very small measurement covariance to strongly pull gate position towards midpoint
                Eigen::Matrix3d tiny_R = Eigen::Matrix3d::Identity() * kTinyCovariance;
                gate_track->kf.update(midpoint, tiny_R);
                p_gate = gate_track->kf.state();
                
                // Keep the gate alive since we clearly see its panels
                gate_track->age = 0;
                gate_track->state = TrackState::CONFIRMED;
                
                if (has_observer_position) {
                    Eigen::Vector2d gate_span = xy(p_survey_repair) - xy(p_search_rescue);
                    double yaw = 0.0;
                    if (compute_facing_yaw(gate_span, xy(p_gate), xy(observer_position), yaw)) {
                        gate_track->theta_z = yaw;
                        gate_track->has_orientation = true;
                    }
                }
            }
        } else {
            // No gate track exists, create a synthetic one!
            Track new_gate;
            new_gate.id = track_id_counter++;
            new_gate.label = "gate";
            new_gate.state = TrackState::CONFIRMED;
            new_gate.hits = min_hits; // ensure it doesn't get instantly deleted as unconfirmed
            new_gate.consecutive_hits = min_hits;
            new_gate.total_updates = min_hits;
            new_gate.age = 0;
            new_gate.confidence = std::min(search_rescue_track->confidence, survey_repair_track->confidence);
            new_gate.kf = create_kf(midpoint);
            
            if (has_observer_position) {
                Eigen::Vector2d gate_span = xy(p_survey_repair) - xy(p_search_rescue);
                double yaw = 0.0;
                if (compute_facing_yaw(gate_span, xy(midpoint), xy(observer_position), yaw)) {
                    new_gate.theta_z = yaw;
                    new_gate.has_orientation = true;
                }
            }
            tracks.push_back(new_gate);
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

    Eigen::Vector3d board_pos = board_track->kf.state();

    // Filter out noisy icons that are physically too far from the board's current estimate
    if (ambulance_track && (ambulance_track->kf.state() - board_pos).norm() > refinement_plausibility_radius) ambulance_track = nullptr;
    if (firetruck_track && (firetruck_track->kf.state() - board_pos).norm() > refinement_plausibility_radius) firetruck_track = nullptr;
    if (blood_track && (blood_track->kf.state() - board_pos).norm() > refinement_plausibility_radius) blood_track = nullptr;
    if (fire_track && (fire_track->kf.state() - board_pos).norm() > refinement_plausibility_radius) fire_track = nullptr;

    const bool has_vehicle_pair = ambulance_track && firetruck_track;
    const bool has_hazard_pair = fire_track && blood_track;
    // if neither pair exists, we have no basis for icon refinement, so skip entire process
    if (!has_vehicle_pair && !has_hazard_pair) {
        return;
    }

    Eigen::Vector3d refined_board_position = board_track->kf.state();
    if (has_vehicle_pair && has_hazard_pair) {
        // take average of all 4 points for icon position refinement
        Eigen::Vector3d centroid =
            (ambulance_track->kf.state() + firetruck_track->kf.state() +
             fire_track->kf.state() + blood_track->kf.state()) /
            4.0;
        // only update if refined position is within plausibility radius of original board position (prevents large jumps from erroneous measurements)
        if ((refined_board_position - centroid).norm() < refinement_plausibility_radius) {
            // Use very small measurement covariance to strongly pull board position towards centroid
            board_track->kf.update(centroid, Eigen::Matrix3d::Identity() * kTinyCovariance);
            refined_board_position = board_track->kf.state();
        }
    } else if (has_vehicle_pair) {
        // take average of vehicle positions for icon position refinement
        Eigen::Vector3d midpoint =
            (ambulance_track->kf.state() + firetruck_track->kf.state()) / 2.0;
        // only update if refined position is within plausibility radius of original board position (prevents large jumps from erroneous measurements)
        if ((refined_board_position - midpoint).norm() < refinement_plausibility_radius) {
            // Use very small measurement covariance to strongly pull board position towards centroid
            board_track->kf.update(midpoint, Eigen::Matrix3d::Identity() * kTinyCovariance);
            refined_board_position = board_track->kf.state();
        }
    } else {
        // take average of hazard positions for icon position refinement
        Eigen::Vector3d midpoint =
            (fire_track->kf.state() + blood_track->kf.state()) / 2.0;
        // only update if refined position is within plausibility radius of original board position (prevents large jumps from erroneous measurements)
        if ((refined_board_position - midpoint).norm() < refinement_plausibility_radius) {
            // Use very small measurement covariance to strongly pull board position towards centroid
            board_track->kf.update(midpoint, Eigen::Matrix3d::Identity() * kTinyCovariance);
            refined_board_position = board_track->kf.state();
        }
    }

    if (!has_observer_position) {
        return;
    }

    std::vector<Eigen::Vector2d> chosen_normals;

    if (has_vehicle_pair) {
        Eigen::Vector2d normal;
        // vector between vehicles is the direction (in global frame) of the board
        // compute the normal to the board that faces towards the observer, which gives us the board orientation
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
        // vector between hazards is the direction (in global frame) of the board
        // compute the normal to the board that faces towards the observer, which gives us the board orientation
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
    // vehicles and hazard both give us a normal, we can fuse them by adding them (we only case about orientation)
    Eigen::Vector2d fused_normal = Eigen::Vector2d::Zero();
    for (const auto& normal : chosen_normals) {
        fused_normal += normal;
    }

    if (fused_normal.norm() < kEpsilon) {
        return;
    }
    // extract yaw from orientation of fused normal
    fused_normal.normalize();
    board_track->theta_z = std::atan2(fused_normal.y(), fused_normal.x());
    board_track->has_orientation = true;
}
