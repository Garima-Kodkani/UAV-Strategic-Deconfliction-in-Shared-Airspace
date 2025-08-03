import numpy as np
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple, Any

class SpatialAnalyzer:
    """Handles spatial conflict detection and position calculations."""
    
    def __init__(self, drone_radius: float = 10.0, interpolation_step: int = 1):
        self.drone_radius = drone_radius  # meters
        self.interpolation_step = interpolation_step  # seconds
    
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
        """Check for all spatial conflicts between two drone paths."""
        # Get flight time ranges
        primary_start = datetime.fromisoformat(primary_waypoints[0]['timestamp'])
        primary_end = datetime.fromisoformat(primary_waypoints[-1]['timestamp'])
        other_start = datetime.fromisoformat(other_waypoints[0]['timestamp'])
        other_end = datetime.fromisoformat(other_waypoints[-1]['timestamp'])
        
        # Check temporal relevance
        max_time_gap = 5  # seconds
        if primary_end < other_start:
            time_gap = (other_start - primary_end).total_seconds()
            if time_gap > max_time_gap:
                return []
        elif other_end < primary_start:
            time_gap = (primary_start - other_end).total_seconds()
            if time_gap > max_time_gap:
                return []
        
        # Sample positions
        primary_samples = self.sample_path_positions(primary_waypoints)
        other_samples = self.sample_path_positions(other_waypoints)
        
        conflicts = []
        conflict_interval = 2  # Report conflicts every 2 seconds to avoid spam
        last_conflict_time = None
        
        # Check all combinations for conflicts
        for primary_pos, primary_time in primary_samples:
            closest_distance = float('inf')
            closest_other_data = None
            
            # Find closest other drone position at this time
            for other_pos, other_time in other_samples:
                # Only consider positions that are temporally close (within 1 second)
                time_diff = abs((primary_time - other_time).total_seconds())
                if time_diff <= 1.0:  # Within 1 second
                    dist = np.linalg.norm(primary_pos - other_pos)
                    if dist < closest_distance:
                        closest_distance = dist
                        closest_other_data = {
                            'other_pos': other_pos,
                            'other_time': other_time,
                            'time_diff': time_diff
                        }
            
            # Check if this is a spatial conflict
            if closest_other_data and closest_distance < self.drone_radius:
                # Avoid reporting conflicts too frequently
                if (last_conflict_time is None or 
                    (primary_time - last_conflict_time).total_seconds() >= conflict_interval):
                    
                    conflicts.append({
                        'with_drone': other_drone_id,
                        'min_distance': closest_distance,
                        'location_primary': primary_pos.tolist(),
                        'location_other': closest_other_data['other_pos'].tolist(),
                        'timestamp_primary': primary_time.isoformat(),
                        'timestamp_other': closest_other_data['other_time'].isoformat(),
                        'time_difference': closest_other_data['time_diff'],
                        'spatial_conflict': True,
                        'temporal_conflict': False,
                        'conflict_type': 'SPATIAL'
                    })
                    last_conflict_time = primary_time
        
        return conflicts