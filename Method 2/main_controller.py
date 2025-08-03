from data_loader import DroneDataLoader
from conflict_detector import ConflictDetector
from visualization_engine import VisualizationEngine

class DroneFlightAnalyzer:
    """Main controller class that coordinates all modules."""
    
    def __init__(self, drone_radius: float = 10.0, time_threshold: float = 2.0):
        self.drone_radius = drone_radius
        self.time_threshold = time_threshold
        
        self.data_loader = DroneDataLoader()
        self.conflict_detector = ConflictDetector(drone_radius, time_threshold)
        self.visualizer = VisualizationEngine(drone_radius=drone_radius)
    
    def analyze_flight_data(self, flight_data_path: str, primary_drone_path: str):
        """Main analysis workflow."""
        print("Starting Drone Flight Analysis...")
        print(f"Configuration: drone_radius={self.drone_radius}m, time_threshold={self.time_threshold}s")
        
        # Load data
        print("Loading flight data...")
        flight_data = self.data_loader.load_flight_data(flight_data_path)
        primary_path = self.data_loader.load_primary_drone(primary_drone_path)
        
        # Validate data
        if not self.data_loader.validate_waypoint_data(primary_path['waypoints']):
            print("Invalid primary drone waypoint data")
            return
        
        # Detect conflicts
        print("Detecting conflicts...")
        conflicts = self.conflict_detector.detect_all_conflicts(primary_path, flight_data)
        
        # Print results
        self.conflict_detector.print_conflict_summary(conflicts)
        
        # Visualize results
        print("Generating visualizations...")
        self.visualizer.visualize_conflict_3d(primary_path, flight_data, conflicts)
        self.visualizer.visualize_position_over_time(primary_path, flight_data)
        self.visualizer.animate_3d_trajectories(primary_path, flight_data, conflicts)
        
        return conflicts