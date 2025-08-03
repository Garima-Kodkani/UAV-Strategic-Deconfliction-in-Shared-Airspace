from main_controller import DroneFlightAnalyzer

def main():
    """Entry point for the drone flight analysis system."""
    analyzer = DroneFlightAnalyzer(
        drone_radius = 10.0,    # Safety radius in meters
        time_threshold = 2.0    # Time threshold for temporal conflicts in seconds
    )
    
    conflicts = analyzer.analyze_flight_data('/home/garima/Documents/Test Cases/tc3_parallel_eq_threshold/traffic.json', '/home/garima/Documents/Test Cases/tc3_parallel_eq_threshold/primary.json')
    
    if conflicts:
        print(f"\nAnalysis complete. Found {len(conflicts)} conflicts.")
    else:
        print("\nAnalysis complete. No conflicts detected.")


if __name__ == "__main__":
    main()
