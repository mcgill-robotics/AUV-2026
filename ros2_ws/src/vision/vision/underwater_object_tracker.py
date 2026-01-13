import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

class UnderwaterObjectTracker:
    """
    Global Nearest Neighbor (GNN) tracker with Mahalanobis gating.
    Uses CONSTANT POSITION model (objects don't move).
    
    STRICT MODE: Requires consecutive detections at stable positions.
    """
    
    def __init__(self, min_new_track_distance=0.5):
        self.tracks = [] 
        self.track_id_counter = 1
        
        # Tuning Parameters
        self.gating_threshold = 3.5   # Mahalanobis gate (~3 sigma)
        self.min_hits = 20            # CONSECUTIVE frames to confirm
        self.max_age = 8              # Frames to keep lost track
        self.max_position_jump = 2.0  # Max jump (meters) - higher to handle VIO rotation errors
        
        # New parameter
        self.min_new_track_distance = min_new_track_distance

        # Known object limits (prevents creating too many tracks per class)
        self.max_per_class = {
            "gate": 1,
            "lane_marker": 1,
            "red_pipe": 3,
            "white_pipe": 6,
            "octagon": 1,
            "table": 1,
            "bin": 1,
            "board": 1,
            "shark": 2,
            "sawfish": 2,
        }
        
    def create_kf(self, initial_pos):
        """Create 3D Constant Position Kalman Filter (no velocity states)."""
        # Simplified: dim_x=3, dim_z=3 (position only, no velocity)
        kf = KalmanFilter(dim_x=3, dim_z=3)
        
        # State: [x, y, z] - position only
        kf.x = initial_pos.reshape(3, 1)
        
        # Transition Matrix: x_k+1 = x_k (object stays where it is)
        kf.F = np.eye(3)
        
        # Measurement Function: we observe position directly
        kf.H = np.eye(3)
        
        # Initial Uncertainty (moderate - we trust the first measurement somewhat)
        kf.P = np.eye(3) * 1.0
        
        # Process Noise (low for static underwater objects)
        kf.Q = np.eye(3) * 0.01
        
        # Default Measurement Noise (will be overridden by ZED covariance)
        kf.R = np.eye(3) * 0.1
        
        return kf

    def update(self, measurements, measurement_covariances, classes):
        """
        Main tracker update step (no dt needed for Constant Position model).
        
        Args:
            measurements: List of np.array([x,y,z]) world positions
            measurement_covariances: List of 3x3 covariance matrices
            classes: List of class labels (strings)
            
        Returns:
            List of confirmed track dicts
        """
        # --- 1. PREDICT STEP (trivial for constant position: x stays same) ---
        for track in self.tracks:
            track['kf'].predict()

        # --- 2. DATA ASSOCIATION ---
        if len(measurements) == 0:
            matches = []
            unmatched_dets = []
            unmatched_tracks = list(range(len(self.tracks)))
        elif len(self.tracks) == 0:
            matches = []
            unmatched_dets = list(range(len(measurements)))
            unmatched_tracks = []
        else:
            # Build cost matrix using Mahalanobis distance
            cost_matrix = np.zeros((len(self.tracks), len(measurements)))
            
            for t, track in enumerate(self.tracks):
                for m, meas in enumerate(measurements):
                    # Hard Gate 1: Class mismatch = infinite cost
                    if track['class'] != classes[m]:
                        cost_matrix[t, m] = 1e6
                        continue
                    
                    # Hard Gate 2: Position jump too large (noisy detection)
                    euclidean_dist = np.linalg.norm(meas - track['kf'].x[:3].flatten())
                    if euclidean_dist > self.max_position_jump:
                        cost_matrix[t, m] = 1e6
                        continue
                    
                    # Compute Mahalanobis distance
                    S = track['kf'].H @ track['kf'].P @ track['kf'].H.T + measurement_covariances[m]
                    try:
                        inv_S = np.linalg.inv(S)
                    except np.linalg.LinAlgError:
                        inv_S = np.eye(3)
                    
                    diff = meas - track['kf'].x[:3].flatten()
                    dist = np.sqrt(diff.T @ inv_S @ diff)
                    
                    # Soft Gate: Statistical outlier
                    if dist > self.gating_threshold:
                        cost_matrix[t, m] = 1e6
                    else:
                        cost_matrix[t, m] = dist

            # Hungarian Algorithm for optimal assignment
            row_inds, col_inds = linear_sum_assignment(cost_matrix)
            
            matches = []
            unmatched_tracks = set(range(len(self.tracks)))
            unmatched_dets = set(range(len(measurements)))
            
            for r, c in zip(row_inds, col_inds):
                if cost_matrix[r, c] < 1e5:
                    matches.append((r, c))
                    if r in unmatched_tracks: unmatched_tracks.remove(r)
                    if c in unmatched_dets: unmatched_dets.remove(c)

        # --- 3. UPDATE MATCHED TRACKS ---
        for t_idx, m_idx in matches:
            track = self.tracks[t_idx]
            track['kf'].update(measurements[m_idx], R=measurement_covariances[m_idx])
            track['consecutive_hits'] += 1  # Increment consecutive counter
            track['total_updates'] = track.get('total_updates', 0) + 1
            track['age'] = 0
            
            # Only confirm if seen CONSECUTIVELY
            if track['consecutive_hits'] >= self.min_hits:
                track['state'] = 'CONFIRMED'

        # --- 4. HANDLE UNMATCHED ---
        # Age unmatched tracks AND RESET their consecutive hit counter
        for t_idx in unmatched_tracks:
            self.tracks[t_idx]['age'] += 1
            self.tracks[t_idx]['total_updates'] = self.tracks[t_idx].get('total_updates', 0) + 1
            self.tracks[t_idx]['consecutive_hits'] = 0  # RESET - must be consecutive
            
            # Downgrade confirmed tracks that lost detection
            if self.tracks[t_idx]['state'] == 'CONFIRMED' and self.tracks[t_idx]['age'] > 2:
                self.tracks[t_idx]['state'] = 'TENTATIVE'
        
        # Remove dead tracks:
        # 1. Unmatched for too long (age > max_age)
        # 2. Stuck in TENTATIVE for too long (total_updates > 20) without confirming
        def is_track_alive(t):
            if t['age'] >= self.max_age: return False
            if t['state'] == 'TENTATIVE' and t.get('total_updates', 0) > self.min_hits+5: return False
            return True

        self.tracks = [t for t in self.tracks if is_track_alive(t)]

        # Create new tracks for unmatched detections
        # BUT: Skip if too close OR if we've hit the class limit
        
        for m_idx in unmatched_dets:
            new_pos = measurements[m_idx]
            new_class = classes[m_idx]
            
            # Check 1: Class limit reached?
            current_count = sum(1 for t in self.tracks if t['class'] == new_class)
            max_allowed = self.max_per_class.get(new_class, 10)  # Default to 10 if unknown
            if current_count >= max_allowed:
                continue  # Skip - already tracking max objects of this class
            
            # Check 2: Too close to ANY existing track? (Prevent duplicate objects of different classes)
            # e.g., don't create a Red Pipe track if a White Pipe track is already there
            too_close = False
            for existing_track in self.tracks:
                existing_pos = existing_track['kf'].x[:3].flatten()
                dist = np.linalg.norm(new_pos - existing_pos)
                if dist < self.min_new_track_distance:
                    too_close = True
                    break
            
            if too_close:
                # Log why we are skipping this
                # print(f"SKIPPED {new_class} at {new_pos[:3]} - SPACE_OCCUPIED")
                continue  # Skip this detection, it's space is already occupied
            
            self.tracks.append({
                'id': self.track_id_counter,
                'class': new_class,
                'kf': self.create_kf(new_pos),
                'consecutive_hits': 1,
                'hits': 1,
                'age': 0,
                'total_updates': 1,  # Track lifespan
                'state': 'TENTATIVE'
            })
            self.track_id_counter += 1
            
        # Return only confirmed tracks
        return [t for t in self.tracks if t['state'] == 'CONFIRMED']
