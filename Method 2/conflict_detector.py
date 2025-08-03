from typing import List, Dict, Any
from spatial_analyzer import SpatialAnalyzer
from temporal_analyzer import TemporalAnalyzer

class ConflictDetector:
    """Main conflict detection coordinator."""
    
    def __init__(self, drone_radius: float = 10.0, time_threshold: float = 2.0):
        self.spatial_analyzer = SpatialAnalyzer(drone_radius, time_threshold)
        self.temporal_analyzer = TemporalAnalyzer(time_threshold)
    
    def detect_all_conflicts(self, primary_path: Dict, flight_data: Dict) -> List[Dict]:
        """Detect all types of conflicts between primary drone and others."""
        primary_waypoints = primary_path['waypoints']
        conflicts = []
        
        for drone in flight_data['flights']:
            other_waypoints = drone['waypoints']
            drone_id = drone['drone_id']
            
            # Check spatial conflicts using new 3D segment-based approach
            # This now handles both spatial and spatiotemporal conflicts
            spatial_conflicts = self.spatial_analyzer.check_spatial_conflicts(
                primary_waypoints, other_waypoints, drone_id
            )
            conflicts.extend(spatial_conflicts)
            
            # Note: Temporal analysis is now integrated into spatial analysis
            # The old separate temporal check is disabled to avoid duplicates
            # If you need the old approach, you can uncomment the lines below:
            
            # primary_samples = self.spatial_analyzer.sample_path_positions(primary_waypoints)
            # other_samples = self.spatial_analyzer.sample_path_positions(other_waypoints)
            # 
            # temporal_conflicts = self.temporal_analyzer.check_temporal_conflict(
            #     primary_samples, other_samples, drone_id
            # )
            # conflicts.extend(temporal_conflicts)
        
        return conflicts
    
    def print_conflict_summary(self, conflicts: List[Dict]):
        """Print formatted conflict summary."""
        if not conflicts:
            print("\n✅ No conflicts detected.")
            return
        
        print("\n⚠️ Conflicts Detected:")
        spatial_count = sum(1 for c in conflicts if c.get('conflict_type') == 'SPATIAL')
        spatiotemporal_count = sum(1 for c in conflicts if c.get('conflict_type') == 'SPATIOTEMPORAL')
        temporal_count = sum(1 for c in conflicts if c.get('conflict_type') == 'TEMPORAL')
        
        print(f"CONFLICT SUMMARY:")
        print(f"  • SPATIAL: {spatial_count} conflicts")
        print(f"  • SPATIOTEMPORAL: {spatiotemporal_count} conflicts")
        print(f"  • TEMPORAL: {temporal_count} conflicts")
        print(f"  • TOTAL: {len(conflicts)} conflicts")
        print(f"\nDETAILED CONFLICT LIST:")
        
        for i, c in enumerate(conflicts):
            conflict_type = c.get('conflict_type', 'UNKNOWN')
            print(f"\nConflict {i+1}: {conflict_type} with {c['with_drone']}")
            print(f"  → Primary Location: {c['location_primary']} at {c['timestamp_primary']}")
            print(f"  → Other Location:   {c['location_other']} at {c['timestamp_other']}")
            print(f"  → Min Distance: {c['min_distance']:.2f} m")
            print(f"  → Time Difference: {c['time_difference']:.2f} s")
            
            if c.get('primary_segment') is not None:
                print(f"  → Primary Segment: {c['primary_segment']}")
            if c.get('other_segment') is not None:
                print(f"  → Other Segment: {c['other_segment']}")
                
            if conflict_type == 'SPATIAL':
                print("  → Type: Drones within safety radius but not simultaneous")
            elif conflict_type == 'SPATIOTEMPORAL':
                print("  → Type: Drones within safety radius AND within time threshold")
            elif conflict_type == 'TEMPORAL':
                print("  → Type: Same position within time threshold")