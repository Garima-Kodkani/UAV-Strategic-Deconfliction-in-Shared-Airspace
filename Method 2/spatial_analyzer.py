import numpy as np
import math
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple, Any

class SpatialAnalyzer:
    """Handles spatial conflict detection and position calculations."""
    
    def __init__(self, drone_radius: float = 10.0, safety_time: float = 3.0, interpolation_step: int = 1):
        self.drone_radius = drone_radius  # meters (configurable safety distance)
        self.safety_time = safety_time  # seconds (configurable temporal threshold)
        self.interpolation_step = interpolation_step  # seconds
        self.safety_threshold = drone_radius*3
    
    def closest_points_on_segments_in_3D(self, A1, A2, B1, B2, eps=1e-12):
        """
        Compute closest points between segments P1–P2 and Q1–Q2,
        and return (closest_P, closest_Q, distance, (t1, t2)).
        
        Each segment point contains [x, y, z, time_seconds]
        """
        P1 = A1[:3]
        P2 = A2[:3]
        Q1 = B1[:3]
        Q2 = B2[:3]
        tp1 = A1[3]
        tp2 = A2[3]
        tq1 = B1[3]
        tq2 = B2[3]
        
        # Vector helpers
        def sub(a, b):        return [ai - bi for ai, bi in zip(a, b)]
        def dot(a, b):        return sum(ai*bi for ai, bi in zip(a, b))
        def add(a, b):        return [ai + bi for ai, bi in zip(a, b)]
        def scale(a, s):      return [ai * s for ai in a]
        def norm(a):          return math.sqrt(dot(a, a))
        def clamp(x, lo=0.0, hi=1.0): return max(lo, min(x, hi))

        def time_calculations(ta1, ta2, s, tb1, tb2, t):
            """Calculate the time difference between two points on flight paths"""
            t1 = (ta2 - ta1) * s + ta1
            t2 = (tb2 - tb1) * t + tb1
            return t1, t2

        u = sub(P2, P1)
        v = sub(Q2, Q1)
        w = sub(Q1, P1)

        a = dot(u, u)       # squared length of u
        b = dot(u, v)
        c = dot(v, v)       # squared length of v
        d = dot(u, w)
        e = dot(v, w)

        D = - a*c + b*b       # denominator

        # Parameters on infinite lines
        if abs(D) > eps:
            s = (b*e - c*d) / D 
            t = (a*e - b*d) / D
        else:
            # Lines almost parallel: pick arbitrary s, then solve for t
            s = 0.0
            t = (e / c) if c > eps else 0.0

        # Clamp to [0,1]
        s_clamped = clamp(s)
        t_clamped = clamp(t)

        # Compute the two candidate points
        Pc = add(P1, scale(u, s_clamped))
        Qc = add(Q1, scale(v, t_clamped))

        # If both inside, that's the answer
        if 0.0 <= s <= 1.0 and 0.0 <= t <= 1.0:
            return Pc, Qc, norm(sub(Pc, Qc)), time_calculations(tp1, tp2, s_clamped, tq1, tq2, t_clamped)

        # Otherwise, must be on an endpoint of one segment against the other segment
        def point_to_seg_closest(A, B, X):
            """Project X onto segment A–B and clamp."""
            AB = sub(B, A)
            t = dot(sub(X, A), AB) / dot(AB, AB) if dot(AB, AB) > eps else 0.0
            t = clamp(t)
            return add(A, scale(AB, t)), t

        best_dist = float('inf')
        best_pair = (None, None, None, None)

        # Check P1 & P2 against segment Q
        temp_var = 0
        for P in (P1, P2):
            Qp, t_qp = point_to_seg_closest(Q1, Q2, P)
            dPQ = norm(sub(P, Qp))
            if dPQ < best_dist:
                best_dist = dPQ
                best_pair = (P, temp_var, Qp, t_qp)
            temp_var += 1

        # Check Q1 & Q2 against segment P
        temp_var = 0
        for Q in (Q1, Q2):
            Pp, t_pp = point_to_seg_closest(P1, P2, Q)
            dPQ = norm(sub(Pp, Q))
            if dPQ < best_dist:
                best_dist = dPQ
                best_pair = (Pp, t_pp, Q, temp_var)
            temp_var += 1

        return best_pair[0], best_pair[2], best_dist, time_calculations(tp1, tp2, best_pair[1], tq1, tq2, best_pair[3])

    def check_for_conflicts(self, results):
        """
        Check for conflicts based on distance and time thresholds.
        Returns: conflict_level, closest_P, closest_Q, t1, t2
        - conflict_level = 0: no conflict
        - conflict_level = 1: spatial conflict only
        - conflict_level = 2: spatiotemporal conflict
        """
        closest_P, closest_Q, min_dist_val, (t1, t2) = results
        count = 0
        
        if min_dist_val < self.safety_threshold:  # Uses configurable drone_radius
            count += 1  # Spatial conflict
            if abs(t2 - t1) < self.safety_time:  # Uses configurable safety_time
                count += 1  # Also temporal conflict
        
        if count > 0:
            return count, closest_P, closest_Q, t1, t2
        else:
            return count, None, None, None, None

    def convert_timestamp_to_seconds(self, timestamp: str, reference_time: datetime) -> float:
        """Convert ISO timestamp to seconds from reference time."""
        dt = datetime.fromisoformat(timestamp)
        return (dt - reference_time).total_seconds()

    def interpolate_position(self, wp1: Dict, wp2: Dict, t: datetime) -> Optional[np.ndarray]:
        """Interpolate position between two waypoints at given time."""
        t1 = datetime.fromisoformat(wp1['timestamp'])
        t2 = datetime.fromisoformat(wp2['timestamp'])
        if t < t1 or t > t2:
            return None
        total = (t2 - t1).total_seconds()
        alpha = (t - t1).total_seconds() / total if total > 0 else 0
        x = wp1['x'] + alpha * (wp2['x'] - wp1['x'])
        y = wp1['y'] + alpha * (wp2['y'] - wp1['y'])
        z = wp1['z'] + alpha * (wp2['z'] - wp1['z'])
        return np.array([x, y, z])
    
    def get_position_at_time(self, waypoints: List[Dict], t: datetime) -> Optional[np.ndarray]:
        """Get drone position at specific time."""
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            t1 = datetime.fromisoformat(wp1['timestamp'])
            t2 = datetime.fromisoformat(wp2['timestamp'])
            if t1 <= t <= t2:
                return self.interpolate_position(wp1, wp2, t)
        return None
    
    def get_segments_with_time(self, waypoints: List[Dict]) -> List[Tuple]:
        """Get path segments with time information."""
        return [
            (
                np.array([w1['x'], w1['y'], w1['z']]),
                np.array([w2['x'], w2['y'], w2['z']]),
                datetime.fromisoformat(w1['timestamp']),
                datetime.fromisoformat(w2['timestamp'])
            )
            for w1, w2 in zip(waypoints[:-1], waypoints[1:])
        ]
    
    def sample_path_positions(self, waypoints: List[Dict]) -> List[Tuple[np.ndarray, datetime]]:
        """Sample positions along the entire flight path."""
        samples = []
        for i in range(len(waypoints) - 1):
            wp1, wp2 = waypoints[i], waypoints[i + 1]
            t1 = datetime.fromisoformat(wp1['timestamp'])
            t2 = datetime.fromisoformat(wp2['timestamp'])
            
            segment_duration = (t2 - t1).total_seconds()
            for s in range(0, int(segment_duration) + 1, self.interpolation_step):
                t = t1 + timedelta(seconds=s)
                pos = self.interpolate_position(wp1, wp2, t)
                if pos is not None:
                    samples.append((pos, t))
        return samples
    
    def check_spatial_conflicts(self, primary_waypoints: List[Dict], 
                              other_waypoints: List[Dict], 
                              other_drone_id: str) -> List[Dict]:
        """Check for all spatial conflicts between two drone paths using 3D segment analysis."""
        
        # Get reference time (earliest timestamp)
        all_timestamps = []
        for wp in primary_waypoints + other_waypoints:
            all_timestamps.append(datetime.fromisoformat(wp['timestamp']))
        reference_time = min(all_timestamps)
        
        # Convert waypoints to segment format [x, y, z, time_seconds]
        def convert_waypoints_to_segments(waypoints):
            segments = []
            for i in range(len(waypoints) - 1):
                wp1, wp2 = waypoints[i], waypoints[i + 1]
                seg1 = [wp1['x'], wp1['y'], wp1['z'], 
                       self.convert_timestamp_to_seconds(wp1['timestamp'], reference_time)]
                seg2 = [wp2['x'], wp2['y'], wp2['z'], 
                       self.convert_timestamp_to_seconds(wp2['timestamp'], reference_time)]
                segments.append((seg1, seg2))
            return segments
        
        primary_segments = convert_waypoints_to_segments(primary_waypoints)
        other_segments = convert_waypoints_to_segments(other_waypoints)
        
        conflicts = []
        
        # Check each primary segment against each other segment
        for i, (p1, p2) in enumerate(primary_segments):
            for j, (o1, o2) in enumerate(other_segments):
                # Calculate closest points between segments
                results = self.closest_points_on_segments_in_3D(p1, p2, o1, o2)
                conflict_level, closest_P, closest_Q, t1, t2 = self.check_for_conflicts(results)
                
                if conflict_level > 0:
                    # Convert time back to timestamp
                    timestamp_primary = (reference_time + timedelta(seconds=t1)).isoformat()
                    timestamp_other = (reference_time + timedelta(seconds=t2)).isoformat()
                    
                    # Determine conflict type
                    if conflict_level == 1:
                        conflict_type = 'SPATIAL'
                        spatial_conflict = True
                        temporal_conflict = False
                    else:  # conflict_level == 2
                        conflict_type = 'SPATIOTEMPORAL'
                        spatial_conflict = True
                        temporal_conflict = True
                    
                    conflict_data = {
                        'with_drone': other_drone_id,
                        'min_distance': results[2],  # Distance from closest_points_on_segments_in_3D
                        'location_primary': closest_P,
                        'location_other': closest_Q,
                        'timestamp_primary': timestamp_primary,
                        'timestamp_other': timestamp_other,
                        'time_difference': abs(t2 - t1),
                        'spatial_conflict': spatial_conflict,
                        'temporal_conflict': temporal_conflict,
                        'conflict_type': conflict_type,
                        'primary_segment': i,
                        'other_segment': j
                    }
                    
                    conflicts.append(conflict_data)
        
        return conflicts