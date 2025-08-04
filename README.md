# UAV-Strategic-Deconfliction-in-Shared-Airspace

Two methods have been implemented to calculate the occurence of spatial and temporal conflicts in a shared airspace.
A detailed explaination and comparison of the two is given in the documention provided.

---

## Method 1

### System Architecture

The system is built with a modular architecture consisting of several key components:

```
├── main.py                    → Entry point
├── main_controller.py         → Main orchestration logic
├── data_loader.py             → JSON data loading and validation
├── conflict_detector.py       → Main conflict detection coordinator
├── spatial_analyzer.py        → Spatial conflict detection algorithms
├── temporal_analyzer.py       → Temporal conflict detection algorithms
├── visualization_engine.py    → All visualization functionality
├── flight_data.json           → Sample flight data
└── primary_drone.json         → Sample primary drone data
```

## Method 2

### System Architecture

The system is built with a modular architecture consisting of several key components:
  main.py                    # Entry point
  main_controller.py         # Main orchestration logic
  data_loader.py            # JSON data loading and validation
  conflict_detector.py      # Main conflict detection coordinator
  spatial_analyzer.py       # Conflict detection algorithm
  visualization_engine.py   # All visualization functionality
  flight_data.json         # Sample flight data
  primary_drone.json       # Sample primary drone data


---

## Prerequisites

- Python 3.7+
- Required Python packages:

```bash
pip install numpy matplotlib plotly datetime typing
```

---

## Basic Usage

1. **Prepare Your Data Files**

   Create JSON files for your drone flight data following the provided format

2. **Update File Paths**

   Modify the file paths in main.py to point to your data files

3. **Run the analysis**
    ```bash
      python3 main.py
    ```
---

## Configuration

You can customize the system parameters in the file main.py

```bash
  drone_radius=10.0,    # Safety radius in meters
  time_threshold=2.0    # Time threshold for temporal conflicts in seconds
```
---

## Data Format

### Flight Data Format (flight_data.json)
```bash
  {
    "flights": [
      {
        "drone_id": "DRONE_001",
        "mission_name": "Package Delivery Route A",
        "start_time": "2025-08-01T10:00:00",
        "end_time": "2025-08-01T10:00:10",
        "waypoints": [
          { "x": 0, "y": 0, "z": 10, "timestamp": "2025-08-01T10:00:00" },
          { "x": 10, "y": 0, "z": 10, "timestamp": "2025-08-01T10:00:10" }
        ]
      }
    ]
  }
```

### Primary Drone Format (primary_drone.json)
```bash
  {
    "Primary flight": [
      {
        "drone_id": "PRIMARY",
        "mission_name": "Emergency Operation",
        "start_time": "2025-08-01T10:00:20",
        "end_time": "2025-08-01T10:00:30",
        "waypoints": [
          { "x": 0, "y": 0, "z": 15, "timestamp": "2025-08-01T10:00:00" },
          { "x": 10, "y": 0, "z": 15, "timestamp": "2025-08-01T10:00:10" }
        ]
      }
    ]
  }
```
