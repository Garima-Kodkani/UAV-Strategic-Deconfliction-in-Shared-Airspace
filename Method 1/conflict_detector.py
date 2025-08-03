from typing import List, Dict, Any
from spatial_analyzer import SpatialAnalyzer
from temporal_analyzer import TemporalAnalyzer

class ConflictDetector:
    """Main conflict detection coordinator."""
    
    def __init__(self, drone_radius: float = 10.0, time_threshold: float = 2.0):
        self.spatial_analyzer = SpatialAnalyzer(drone_radius)
        self.temporal_analyzer = TemporalAnalyzer(time_threshold)
    
    def detect_all_conflicts(self, primary_path: Dict, flight_data: Dict) -> List[Dict]:
        """Detect all types of conflicts between primary drone and others."""
        primary_waypoints = primary_path['waypoints']
        conflicts = []
        
        for drone in flight_data['flights']:
            other_waypoints = drone['waypoints']
            drone_id = drone['drone_id']
            
            # Check spatial conflicts (returns list now)
            spatial_conflicts = self.spatial_analyzer.check_spatial_conflicts(
                primary_waypoints, other_waypoints, drone_id
            )
            conflicts.extend(spatial_conflicts)
            
            # Check temporal conflicts (returns list now)
            primary_samples = self.spatial_analyzer.sample_path_positions(primary_waypoints)
            other_samples = self.spatial_analyzer.sample_path_positions(other_waypoints)
            
            temporal_conflicts = self.temporal_analyzer.check_temporal_conflict(
                primary_samples, other_samples, drone_id
            )
            conflicts.extend(temporal_conflicts)
        
        return conflicts
    
    def print_conflict_summary(self, conflicts: List[Dict]):
        """Print formatted conflict summary."""
        if not conflicts:
            print("\n No conflicts detected.")
            return
        
        print("\n⚠️ Conflicts Detected:")
        spatial_count = sum(1 for c in conflicts if c.get('spatial_conflict', False))
        temporal_count = sum(1 for c in conflicts if c.get('temporal_conflict', False))
        
        print(f"CONFLICT SUMMARY:")
        print(f"  • SPATIAL: {spatial_count} conflicts")
        print(f"  • TEMPORAL: {temporal_count} conflicts")
        print(f"DETAILED CONFLICT LIST:")
        
        for i, c in enumerate(conflicts):
            conflict_type = c.get('conflict_type', 'UNKNOWN')
            print(f"Conflict {i+1}: {conflict_type} with {c['with_drone']}")
            print(f"  → Primary Location: {c['location_primary']} at {c['timestamp_primary']}")
            print(f"  → Other Location:   {c['location_other']} at {c['timestamp_other']}")
            print(f"  → Min Distance: {c['min_distance']:.2f} m")
            print(f"  → Time Difference: {c['time_difference']} s")
            
            if c['spatial_conflict']:
                print("SPATIAL CONFLICT: Drones within safety radius")
            if c['temporal_conflict']:
                print("TEMPORAL CONFLICT: Same position within time threshold")
            print()