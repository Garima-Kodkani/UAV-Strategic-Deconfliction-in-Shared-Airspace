from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import numpy as np

class TemporalAnalyzer:
    """Handles temporal conflict detection."""
    
    def __init__(self, time_threshold: float = 2.0, position_threshold: float = 1.0):
        self.time_threshold = time_threshold  # seconds
        self.position_threshold = position_threshold  # meters
    
    def check_temporal_conflict(self, primary_samples: List[Tuple], 
                              other_samples: List[Tuple], 
                              other_drone_id: str) -> List[Dict]:
        """Check for temporal conflicts (same position within time threshold)."""
        conflicts = []
        for primary_pos, primary_time in primary_samples:
            for other_pos, other_time in other_samples:
                spatial_distance = np.linalg.norm(primary_pos - other_pos)
                time_diff = abs((primary_time - other_time).total_seconds())
                
                # Temporal conflict: same/close position within time threshold
                if spatial_distance < self.position_threshold and time_diff < self.time_threshold:
                    conflicts.append({
                        'with_drone': other_drone_id,
                        'min_distance': spatial_distance,
                        'location_primary': primary_pos.tolist(),
                        'location_other': other_pos.tolist(),
                        'timestamp_primary': primary_time.isoformat(),
                        'timestamp_other': other_time.isoformat(),
                        'time_difference': time_diff,
                        'spatial_conflict': False,
                        'temporal_conflict': True,
                        'conflict_type': 'TEMPORAL'
                    })
        return conflicts