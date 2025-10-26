import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from PIL import Image
import random
import math
from collections import defaultdict
import openpyxl
import re

# ============================================================================
# Target Categories and Color Mapping
# ============================================================================

def load_color_categories():
    """Load category colors from xlsx file"""
    categories = {}
    try:
        wb = openpyxl.load_workbook('color_coding_semantic_segmentation_classes.xlsx')
        ws = wb.active
        
        for row in ws.iter_rows(min_row=2, values_only=True):
            if row[0] is None or row[4] is None:
                break
            try:
                category_id = int(row[0])
                category_name = str(row[4]).strip().lower()
                color_str = row[1]
                
                match = re.search(r'\((\d+),\s*(\d+),\s*(\d+)\)', color_str)
                if match:
                    r, g, b = int(match.group(1)), int(match.group(2)), int(match.group(3))
                    categories[category_name] = {
                        'id': category_id,
                        'color': np.array([r, g, b], dtype=np.uint8),
                    }
            except (ValueError, TypeError, AttributeError):
                pass
    except Exception as e:
        print(f"Error loading categories: {e}")
    
    return categories

# ============================================================================
# RRT Algorithm Implementation
# ============================================================================

class RRTPlanner:
    def __init__(self, occupancy_grid, start, goal, max_iterations=10000, step_size=100):
        """
        Initialize RRT planner
        
        Args:
            occupancy_grid: 2D binary grid (True = obstacle, False = free)
            start: (x, y) starting position
            goal: (x, y) goal position
            max_iterations: maximum iterations
            step_size: maximum step size for extending tree
        """
        self.grid = occupancy_grid
        self.start = start
        self.goal = goal
        self.max_iterations = max_iterations
        self.step_size = step_size
        
        self.nodes = [start]
        self.parent = {0: None}
        
    def distance(self, p1, p2):
        """Euclidean distance"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def nearest_node_idx(self, point):
        """Find nearest node in tree"""
        min_dist = float('inf')
        nearest_idx = 0
        for i, node in enumerate(self.nodes):
            dist = self.distance(point, node)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx
    
    def steer(self, from_point, to_point):
        """Steer from one point toward another"""
        dist = self.distance(from_point, to_point)
        if dist < 0.01:
            return to_point
        
        ratio = min(self.step_size / dist, 1.0)
        new_point = (
            from_point[0] + (to_point[0] - from_point[0]) * ratio,
            from_point[1] + (to_point[1] - from_point[1]) * ratio
        )
        return new_point
    
    def is_collision_free(self, p1, p2, num_checks=30):
        """Check if line segment is collision free"""
        for i in range(num_checks):
            t = i / num_checks
            check_point = (
                int(p1[0] + (p2[0] - p1[0]) * t),
                int(p1[1] + (p2[1] - p1[1]) * t)
            )
            
            # Check bounds
            if check_point[0] < 0 or check_point[0] >= self.grid.shape[1]:
                return False
            if check_point[1] < 0 or check_point[1] >= self.grid.shape[0]:
                return False
            
            # Check collision (True = obstacle)
            if self.grid[check_point[1], check_point[0]]:
                return False
        
        return True
    
    def plan(self):
        """Run RRT algorithm"""
        for iteration in range(self.max_iterations):
            # Random sample
            if random.random() < 0.1:  # 10% goal bias
                rand_point = self.goal
            else:
                rand_point = (
                    random.uniform(0, self.grid.shape[1]),
                    random.uniform(0, self.grid.shape[0])
                )
            
            # Find nearest node
            nearest_idx = self.nearest_node_idx(rand_point)
            nearest_node = self.nodes[nearest_idx]
            
            # Steer
            new_node = self.steer(nearest_node, rand_point)
            
            # Check collision
            if not self.is_collision_free(nearest_node, new_node):
                continue
            
            # Add new node
            self.nodes.append(new_node)
            new_idx = len(self.nodes) - 1
            self.parent[new_idx] = nearest_idx
            
            # Check if goal is reached
            if self.distance(new_node, self.goal) < self.step_size * 2:
                if self.is_collision_free(new_node, self.goal):
                    self.nodes.append(self.goal)
                    goal_idx = len(self.nodes) - 1
                    self.parent[goal_idx] = new_idx
                    print(f"Goal reached in {iteration} iterations with {len(self.nodes)} nodes")
                    return self.extract_path()
            
            if (iteration + 1) % 1000 == 0:
                print(f"  Iteration {iteration + 1}/{self.max_iterations}, nodes: {len(self.nodes)}")
        
        print(f"Failed to find path after {self.max_iterations} iterations. Nodes: {len(self.nodes)}")
        return None
    
    def extract_path(self):
        """Extract path from start to goal"""
        path = []
        current_idx = len(self.nodes) - 1  # Goal node
        
        while current_idx is not None:
            path.append(self.nodes[current_idx])
            current_idx = self.parent.get(current_idx)
        
        path.reverse()
        return path


# ============================================================================
# Interactive Map Interface
# ============================================================================

class InteractiveMapUI:
    def __init__(self):
        self.categories = load_color_categories()
        self.target_categories = ['rack', 'cushion', 'sofa', 'stair', 'cooktop']
        
        # Load map image using PIL
        try:
            pil_image = Image.open('map.png')
            # Convert to RGB if needed (remove alpha channel if present)
            if pil_image.mode == 'RGBA':
                pil_image = pil_image.convert('RGB')
            self.map_image_rgb = np.array(pil_image)
        except Exception as e:
            print(f"Error: Could not load map.png - {e}")
            return
        
        self.height, self.width = self.map_image_rgb.shape[:2]
        
        # Load calibration info
        self.calibration = np.load('calibration_info.npy', allow_pickle=True).item()
        
        # Create masks for target categories
        self.target_masks = self._create_target_masks()
        
        self.start_point = None
        self.target_category = None
        self.goal_point = None
        self.path = None
        
        self.fig = None
        self.ax = None
        
    def _create_target_masks(self):
        """Create binary masks for target categories"""
        masks = {}
        
        for cat in self.target_categories:
            if cat not in self.categories:
                print(f"Warning: {cat} not found in categories")
                continue
            
            target_color = self.categories[cat]['color']
            
            # Find pixels matching the target color
            mask = np.all(self.map_image_rgb == target_color, axis=2)
            masks[cat] = mask
        
        return masks
    
    def _pixel_to_habitat(self, pixel_x, pixel_y):
        """Convert pixel coordinates to Habitat coordinates"""
        # This is a simple linear mapping
        # Map pixel coordinates to the range of habitat coordinates
        
        calib = self.calibration
        x_ratio = pixel_x / self.width
        z_ratio = pixel_y / self.height
        
        habitat_x = calib['x_min'] + x_ratio * (calib['x_max'] - calib['x_min'])
        habitat_z = calib['z_min'] + z_ratio * (calib['z_max'] - calib['z_min'])
        
        return (habitat_x, habitat_z)
    
    def _habitat_to_pixel(self, habitat_x, habitat_z):
        """Convert Habitat coordinates to pixel coordinates"""
        calib = self.calibration
        
        x_ratio = (habitat_x - calib['x_min']) / (calib['x_max'] - calib['x_min'])
        z_ratio = (habitat_z - calib['z_min']) / (calib['z_max'] - calib['z_min'])
        
        pixel_x = int(x_ratio * self.width)
        pixel_y = int(z_ratio * self.height)
        
        return (pixel_x, pixel_y)
    
    def _create_occupancy_grid(self):
        """Create occupancy grid from map (white = free, other = obstacle)"""
        # Convert to grayscale by taking maximum RGB value
        gray = np.max(self.map_image_rgb, axis=2)
        
        # Threshold: high values (>200) are free space (white background)
        # low values are obstacles (colored points)
        occupancy_grid = gray < 200
        
        # Dilate obstacles slightly for safety
        occupancy_grid = np.pad(occupancy_grid, 2, mode='constant', constant_values=False)
        
        return occupancy_grid
    
    def _find_target_point(self):
        """Find a point in front of the target category"""
        if self.target_category not in self.target_masks:
            print(f"No pixels found for {self.target_category}")
            return None
        
        mask = self.target_masks[self.target_category]
        
        if not np.any(mask):
            print(f"No pixels found for {self.target_category}")
            return None
        
        # Find center of target region
        y_indices, x_indices = np.where(mask)
        target_pixel_x = int(np.mean(x_indices))
        target_pixel_y = int(np.mean(y_indices))
        
        print(f"Target {self.target_category} center pixel: ({target_pixel_x}, {target_pixel_y})")
        
        return (target_pixel_x, target_pixel_y)
    
    def _on_click(self, event):
        """Handle mouse click on the map"""
        if event.inaxes != self.ax:
            return
        
        if self.start_point is None:
            # First click: set start point
            self.start_point = (int(event.xdata), int(event.ydata))
            print(f"Start point set at pixel: {self.start_point}")
            print(f"Habitat coordinates: {self._pixel_to_habitat(self.start_point[0], self.start_point[1])}")
            
            # Update display
            self.ax.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label='Start')
            self.fig.canvas.draw_idle()
        else:
            # Second click: set goal point
            self.goal_point = (int(event.xdata), int(event.ydata))
            print(f"Goal point set at pixel: {self.goal_point}")
            print(f"Habitat coordinates: {self._pixel_to_habitat(self.goal_point[0], self.goal_point[1])}")
            
            # Run RRT
            print("Running RRT algorithm...")
            self.run_rrt()
    
    def run_rrt(self):
        """Run RRT and plot result"""
        if self.start_point is None or self.goal_point is None:
            print("Please set both start and goal points")
            return
        
        # Create occupancy grid
        occupancy_grid = self._create_occupancy_grid()
        
        # Run RRT
        planner = RRTPlanner(
            occupancy_grid,
            self.start_point,
            self.goal_point,
            max_iterations=5000,
            step_size=50
        )
        
        self.path = planner.plan()
        
        if self.path:
            print(f"Path found with {len(self.path)} waypoints")
            self.plot_path()
        else:
            print("Failed to find path")
    
    def plot_path(self):
        """Plot the found path on the map"""
        if self.path is None:
            return
        
        # Create new figure if needed
        if self.fig is None:
            self.fig, self.ax = plt.subplots(figsize=(12, 12))
        
        self.ax.clear()
        self.ax.imshow(self.map_image_rgb)
        
        # Plot path
        path_x = [p[0] for p in self.path]
        path_y = [p[1] for p in self.path]
        
        self.ax.plot(path_x, path_y, 'r-', linewidth=2, label='Path')
        self.ax.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label='Start')
        self.ax.plot(self.goal_point[0], self.goal_point[1], 'r*', markersize=15, label='Goal')
        
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(self.height, 0)
        self.ax.legend()
        self.ax.set_title('RRT Path')
        
        self.fig.canvas.draw_idle()
        
        # Save path visualization
        plt.savefig('rrt_path.png', dpi=150, bbox_inches='tight')
        print("Path visualization saved as 'rrt_path.png'")
        
        # Print path in both pixel and habitat coordinates
        print("\nPath waypoints:")
        print("Pixel Coordinates -> Habitat Coordinates")
        for i, p in enumerate(self.path):
            habitat_coords = self._pixel_to_habitat(p[0], p[1])
            print(f"  {i}: ({p[0]:4.0f}, {p[1]:4.0f}) -> ({habitat_coords[0]:7.2f}, {habitat_coords[1]:7.2f})")
    
    def interactive_search(self, target_category):
        """Interactive search for target category"""
        if target_category not in self.target_categories:
            print(f"Invalid target category. Choose from: {self.target_categories}")
            return
        
        self.target_category = target_category
        
        # Reset
        self.start_point = None
        self.goal_point = None
        self.path = None
        
        # Create figure
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.ax.imshow(self.map_image_rgb)
        self.ax.set_title(f'Click to set start point. Searching for: {target_category}')
        
        # Highlight target regions
        if target_category in self.target_masks:
            mask = self.target_masks[target_category]
            highlighted = self.map_image_rgb.copy()
            highlighted[mask] = [0, 255, 0]  # Green highlight
            self.ax.imshow(highlighted, alpha=0.3)
        
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(self.height, 0)
        
        # Find target point automatically
        self.goal_point = self._find_target_point()
        if self.goal_point:
            self.ax.plot(self.goal_point[0], self.goal_point[1], 'r*', markersize=15, label='Target')
        
        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        
        print(f"\nSearching for: {target_category}")
        print(f"Target goal point (pixel): {self.goal_point}")
        if self.goal_point:
            habitat_coords = self._pixel_to_habitat(self.goal_point[0], self.goal_point[1])
            print(f"Target goal point (habitat): {habitat_coords}")
        print("Click on the map to set the starting point")
        
        plt.show()


# ============================================================================
# Main
# ============================================================================

def main():
    print("Loading interactive map UI...")
    ui = InteractiveMapUI()
    
    print("\nTarget categories available:")
    for cat in ui.target_categories:
        if cat in ui.categories:
            print(f"  - {cat} (ID: {ui.categories[cat]['id']}, RGB: {ui.categories[cat]['color']})")
    
    # Interactive search
    print("\n" + "="*60)
    print("RRT Pathfinding Interface")
    print("="*60)
    
    while True:
        print("\nAvailable commands:")
        print("  1. Search for target (enter target name)")
        print("  2. Exit")
        
        target = input("Enter target category or command: ").strip().lower()
        
        if target == 'exit' or target == '2':
            break
        
        if target in ui.target_categories:
            ui.interactive_search(target)
        else:
            print(f"Invalid input. Choose from: {ui.target_categories}")


if __name__ == '__main__':
    main()
