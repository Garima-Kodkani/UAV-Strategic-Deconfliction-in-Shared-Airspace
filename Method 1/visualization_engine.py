import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import plotly.graph_objects as go
from datetime import datetime, timedelta
from typing import List, Dict, Any
from spatial_analyzer import SpatialAnalyzer

class VisualizationEngine:
    """Handles all visualization tasks."""
    
    def __init__(self, colors: List[str] = None, drone_radius: float = 10.0):
        self.colors = colors or ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        self.drone_radius = drone_radius
        self.spatial_analyzer = SpatialAnalyzer(drone_radius)
    
    def generate_offset_tube(self, ax, A: np.ndarray, B: np.ndarray, color: str):
        """Generate cylindrical tube between two points for safety visualization."""
        v = B - A
        length = np.linalg.norm(v)
        if length == 0:
            return
        v = v / length
        not_v = np.array([1, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1, 0])
        n1 = np.cross(v, not_v)
        n1 /= np.linalg.norm(n1)
        n2 = np.cross(v, n1)

        t = np.linspace(0, length, 10)
        theta = np.linspace(0, 2 * np.pi, 20)
        t, theta = np.meshgrid(t, theta)

        X, Y, Z = [A[i] + v[i]*t + self.drone_radius*np.sin(theta)*n1[i] + 
                   self.drone_radius*np.cos(theta)*n2[i] for i in range(3)]
        ax.plot_surface(X, Y, Z, color=color, alpha=0.1, linewidth=0)
    
    def generate_comprehensive_time_samples(self, all_drones_data: List[tuple]) -> List[datetime]:
        """Generate time samples covering all drones' flight times."""
        all_times = []
        
        for drone_id, waypoints, color in all_drones_data:
            for wp in waypoints:
                all_times.append(datetime.fromisoformat(wp['timestamp']))
        
        if not all_times:
            return []
            
        start_time = min(all_times)
        end_time = max(all_times)
        
        print(f"Animation time range: {start_time.strftime('%H:%M:%S')} to {end_time.strftime('%H:%M:%S')}")
        
        return [start_time + timedelta(seconds=i) 
                for i in range(0, int((end_time - start_time).total_seconds()) + 1, 1)]
    
    def visualize_conflict_3d(self, primary_path: Dict, flight_data: Dict, conflicts: List[Dict]):
        """Create 3D visualization with conflict points."""
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Plot other drones
        for i, flight in enumerate(flight_data['flights']):
            color = self.colors[i % len(self.colors)]
            waypoints = flight['waypoints']
            x = [wp['x'] for wp in waypoints]
            y = [wp['y'] for wp in waypoints]
            z = [wp['z'] for wp in waypoints]
            ax.plot(x, y, z, color=color, label=flight['drone_id'])
            ax.scatter(x, y, z, color=color, s=10)
            
            segments = self.spatial_analyzer.get_segments_with_time(waypoints)
            for a, b, _, _ in segments:
                self.generate_offset_tube(ax, a, b, color)

        # Plot primary drone
        waypoints = primary_path['waypoints']
        px = [wp['x'] for wp in waypoints]
        py = [wp['y'] for wp in waypoints]
        pz = [wp['z'] for wp in waypoints]
        ax.plot(px, py, pz, color='black', linewidth=3, label=primary_path['drone_id'])
        ax.scatter(px, py, pz, color='black', s=10)
        
        segments = self.spatial_analyzer.get_segments_with_time(waypoints)
        for a, b, _, _ in segments:
            self.generate_offset_tube(ax, a, b, 'black')

        # Plot conflicts
        spatial_conflicts = []
        temporal_conflicts = []
        
        for idx, c in enumerate(conflicts):
            x, y, z = c['location_primary']
            
            if c.get('conflict_type') == 'SPATIAL':
                ax.scatter(x, y, z, color='red', s=100, marker='o')
                spatial_conflicts.append((x, y, z, idx))
            elif c.get('conflict_type') == 'TEMPORAL':
                ax.scatter(x, y, z, color='orange', s=100, marker='s')
                temporal_conflicts.append((x, y, z, idx))
            else:
                ax.scatter(x, y, z, color='red', s=100, marker='o')
                spatial_conflicts.append((x, y, z, idx))
                
            ax.text(x, y, z + 2, f"C{idx}", color='black')

        # Add legend entries
        if spatial_conflicts:
            ax.scatter([], [], [], color='red', s=100, marker='o', label='Spatial Conflict')
        if temporal_conflicts:
            ax.scatter([], [], [], color='orange', s=100, marker='s', label='Temporal Conflict')

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("3D Visualization with Conflict Points")
        ax.legend()
        plt.show()
    
    def visualize_position_over_time(self, primary_path: Dict, flight_data: Dict):
        """Create matplotlib 3D space-time visualization."""
        print("\n  Generating 3D space-time visualization...")

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        def plot_path(ax, waypoints, label, color):
            t0 = datetime.fromisoformat(waypoints[0]['timestamp'])
            ts = [(datetime.fromisoformat(wp['timestamp']) - t0).total_seconds() for wp in waypoints]
            xs = [wp['x'] for wp in waypoints]
            ys = [wp['y'] for wp in waypoints]
            ax.plot(xs, ys, ts, label=label, color=color)

        # Plot other drones
        for i, drone in enumerate(flight_data['flights']):
            plot_path(ax, drone['waypoints'], drone['drone_id'], self.colors[i % len(self.colors)])

        # Plot primary drone
        plot_path(ax, primary_path['waypoints'], primary_path['drone_id'], 'black')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Time (s from start)')
        ax.set_title('Drone Trajectories in Space-Time')
        ax.legend()
        plt.show()

        # Plotly Interactive 3D Plot
        self.create_plotly_visualization(primary_path, flight_data)
    
    def create_plotly_visualization(self, primary_path: Dict, flight_data: Dict):
        """Create Plotly interactive visualization."""
        print("  Launching Plotly interactive visualization...")
        fig = go.Figure()
        t0 = datetime.fromisoformat(primary_path['waypoints'][0]['timestamp'])

        def add_trace(waypoints, label, color):
            ts = [(datetime.fromisoformat(wp['timestamp']) - t0).total_seconds() for wp in waypoints]
            xs = [wp['x'] for wp in waypoints]
            ys = [wp['y'] for wp in waypoints]
            fig.add_trace(go.Scatter3d(x=xs, y=ys, z=ts, mode='lines+markers', 
                                     name=label, line=dict(color=color)))

        # Add traces for all drones
        for i, drone in enumerate(flight_data['flights']):
            add_trace(drone['waypoints'], drone['drone_id'], self.colors[i % len(self.colors)])
        add_trace(primary_path['waypoints'], primary_path['drone_id'], 'black')

        fig.update_layout(scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Time (s)'
        ), title='Drone Flight Paths Over Time', showlegend=True)

        fig.show()
    
    def animate_3d_trajectories(self, primary_path: Dict, flight_data: Dict, conflicts: List[Dict] = None):
        """Create animated 3D trajectory visualization."""
        print("🎞 Animating drone flight trajectories over time...")
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')

        all_drones = [(primary_path['drone_id'], primary_path['waypoints'], 'black')] + [
            (d['drone_id'], d['waypoints'], self.colors[i % len(self.colors)])
            for i, d in enumerate(flight_data['flights'])
        ]

        time_samples = self.generate_comprehensive_time_samples(all_drones)
        
        # Calculate axis limits
        all_x, all_y, all_z = [], [], []
        for _, waypoints, _ in all_drones:
            all_x.extend([wp['x'] for wp in waypoints])
            all_y.extend([wp['y'] for wp in waypoints])
            all_z.extend([wp['z'] for wp in waypoints])
        
        margin = 10

        def update(frame):
            if frame >= len(time_samples):
                return []
                
            t = time_samples[frame]
            ax.clear()
            
            # Draw trajectory lines and tubes
            for drone_id, waypoints, color in all_drones:
                x_path = [wp['x'] for wp in waypoints]
                y_path = [wp['y'] for wp in waypoints]
                z_path = [wp['z'] for wp in waypoints]
                ax.plot(x_path, y_path, z_path, color=color, alpha=0.2, linewidth=1)
                
                segments = self.spatial_analyzer.get_segments_with_time(waypoints)
                for a, b, _, _ in segments:
                    self.generate_offset_tube(ax, a, b, color)
            
            # Draw current positions
            active_drones = []
            for i, (drone_id, waypoints, color) in enumerate(all_drones):
                pos = self.spatial_analyzer.get_position_at_time(waypoints, t)
                if pos is not None:
                    ax.scatter(pos[0], pos[1], pos[2], c=color, s=150, 
                             label=drone_id, alpha=1.0, edgecolors='black', linewidth=2)
                    active_drones.append(drone_id)
                    ax.text(pos[0], pos[1], pos[2] + 2, drone_id, fontsize=8, 
                           color=color, weight='bold')
            
            # Mark conflicts
            if conflicts:
                for idx, c in enumerate(conflicts):
                    conflict_time = c['timestamp_primary']
                    if t.strftime('%Y-%m-%dT%H:%M:%S') >= conflict_time[:19]:
                        x, y, z = c['location_primary']
                        
                        if c.get('conflict_type') == 'SPATIAL':
                            color = 'red'
                            marker = 'o'
                        elif c.get('conflict_type') == 'TEMPORAL':
                            color = 'orange'
                            marker = 's'
                        else:
                            color = 'red'
                            marker = 'o'
                            
                        ax.scatter(x, y, z, color=color, s=100, marker=marker, 
                                 label=f'{c.get("conflict_type", "Conflict")} Point' if idx == 0 else None)
                        ax.text(x, y, z + 2, f"C{idx}", color='black')
            
            # Set labels and limits
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f"Drone Animation - Time: {t.strftime('%H:%M:%S')} | Active: {len(active_drones)}/{len(all_drones)} drones")
            
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
            ax.set_zlim(min(all_z) - margin, max(all_z) + margin)
            
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            
            return []

        print(f"Starting animation with {len(time_samples)} frames...")
        anim = FuncAnimation(fig, update, frames=len(time_samples), interval=100, blit=False, repeat=True)
        
        plt.tight_layout()
        plt.show()
        
        return anim