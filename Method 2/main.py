from main_controller import DroneFlightAnalyzer

def main():
    """Entry point for the drone flight analysis system."""
    
    # Configurable safety parameters - change these values as needed
    DRONE_RADIUS = 5.0      # Safety radius/distance threshold in meters
    SAFETY_TIME = 2.0        # Time threshold for temporal conflicts in seconds
    
    print("=== Drone Flight Conflict Analysis System ===")
    print(f"Safety Configuration:")
    print(f"  - Drone Safety Radius: {DRONE_RADIUS} meters")
    print(f"  - Safety Time Threshold: {SAFETY_TIME} seconds")
    print("=" * 50)
    
    analyzer = DroneFlightAnalyzer(
        drone_radius=DRONE_RADIUS,    # This controls spatial safety distance
        time_threshold=SAFETY_TIME    # This controls temporal safety threshold
    )
    
    # File paths - update these to your actual file locations
    flight_data_file = '/home/garima/Documents/Test Cases/tc17_st_full/traffic.json'
    primary_drone_file = '/home/garima/Documents/Test Cases/tc17_st_full/primary.json'
    
    conflicts = analyzer.analyze_flight_data(flight_data_file, primary_drone_file)
    
    if conflicts:
        print(f"\n🚨 Analysis complete. Found {len(conflicts)} conflicts.")
        print(f"Safety thresholds used: {DRONE_RADIUS}m radius, {SAFETY_TIME}s time")
    else:
        print(f"\n✅ Analysis complete. No conflicts detected.")
        print(f"Safety thresholds used: {DRONE_RADIUS}m radius, {SAFETY_TIME}s time")


if __name__ == "__main__":
    main()