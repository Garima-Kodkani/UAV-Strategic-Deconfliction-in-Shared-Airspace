import json
from datetime import datetime
from typing import Dict, List, Any

class DroneDataLoader:
    """Handles loading and parsing of drone flight data from JSON files."""
    
    def __init__(self):
        pass
    
    def load_flight_data(self, json_file_path: str) -> Dict[str, Any]:
        """Load flight data from JSON file."""
        with open(json_file_path, 'r') as file:
            data = json.load(file)
            print(f"Loaded {len(data['flights'])} drones")
            return data
    
    def load_primary_drone(self, path: str) -> Dict[str, Any]:
        """Load primary drone data from JSON file."""
        with open(path, 'r') as file:
            data = json.load(file)
            return data['Primary flight'][0]
    
    def validate_waypoint_data(self, waypoints: List[Dict]) -> bool:
        """Validate that waypoint data has required fields."""
        required_fields = ['x', 'y', 'z', 'timestamp']
        for wp in waypoints:
            if not all(field in wp for field in required_fields):
                return False
        return True

