#!/usr/bin/env python3
"""
RRT Pathfinding Implementation

This module implements the Rapidly-exploring Random Tree (RRT) algorithm
for pathfinding on a 2D semantic map.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Circle
from typing import Tuple, List, Optional, Dict
import math
import random
import os


class Node:
    """Represents a node in the RRT tree."""
    
    def __init__(self, x: float, y: float, parent: Optional['Node'] = None):
        """
        Initialize a node.
        
        Args:
            x: X coordinate
            y: Y coordinate
            parent: Parent node (None for root)
        """
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []
        
        if parent is not None:
            parent.children.append(self)
    
    def distance_to(self, other: 'Node') -> float:
        """Calculate Euclidean distance to another node."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def distance_to_point(self, x: float, y: float) -> float:
        """Calculate Euclidean distance to a point."""
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)


class RRTPathfinder:
    """
    Rapidly-exploring Random Tree (RRT) pathfinder for 2D semantic maps.
    """
    
    def __init__(self, map_path: str, excel_path: str, transform_path: str):
        """
        Initialize the RRT pathfinder.
        
        Args:
            map_path: Path to the map image file
            excel_path: Path to the color coding Excel file
            transform_path: Path to coordinate transformation file
        """
        self.map_path = map_path
        self.excel_path = excel_path
        self.transform_path = transform_path
        
        # Load data
        self.map_image = None
        self.color_mapping = {}
        self.transform_params = {}
        self.load_data()
        
        # RRT parameters
        self.step_size = 50.0  # pixels - larger steps for more exploration
        self.max_iterations = 3000  # fewer iterations since steps are bigger
        self.goal_tolerance = 25.0  # pixels
        self.goal_bias_probability = 0.1  # 10% chance to sample toward goal
        
        # Robot parameters
        self.robot_radius_pixels = 20.0  # Robot radius in pixels (smaller for more exploration)
        self.safety_margin_pixels = 10.0  # Additional safety margin beyond robot radius
        
        # Pathfinding state
        self.tree_nodes = []
        self.start_node = None
        self.goal_point = None
        self.path = []
        
        # Define target category colors
        self.target_categories = {
            'rack': (0, 255, 133),
            'cushion': (255, 9, 92),
            'sofa': (10, 0, 255),
            'stair': (173, 255, 0),
            'cooktop': (7, 255, 224)
        }
        
        # Define obstacle colors (non-navigable areas)
        # Including walls, furniture, appliances, etc.
        self.obstacle_colors = self._get_obstacle_colors()
        
        # Precompute obstacle map for efficient collision checking
        self.obstacle_map = None
        self._create_obstacle_map()
    
    def load_data(self):
        """Load map image, color mapping, and transformation parameters."""
        print("Loading map and transformation data...")
        
        # Load map image
        self.map_image = mpimg.imread(self.map_path)
        print(f"Loaded map image: {self.map_image.shape}")
        
        # Load color mapping from Excel file
        df = pd.read_excel(self.excel_path)
        for _, row in df.iterrows():
            name = row['Name']
            rgb_str = row['Color_Code (R,G,B)']
            # Parse RGB string like "(120, 120, 120)" or "(255 ,82, 0)"
            rgb_str = rgb_str.strip('()')
            # Handle different spacing patterns
            rgb_str = rgb_str.replace(' ,', ',').replace(', ', ',')
            r, g, b = map(int, rgb_str.split(','))
            self.color_mapping[name] = (r, g, b)
        
        print(f"Loaded {len(self.color_mapping)} color mappings")
        
        # Load transformation parameters
        self._load_transform_params()
    
    def _load_transform_params(self):
        """Load coordinate transformation parameters from file."""
        with open(self.transform_path, 'r') as f:
            content = f.read()
        
        # Parse transformation parameters
        lines = content.split('\n')
        for line in lines:
            if 'X:' in line and 'habitat_x_range' not in self.transform_params:
                # Parse X range
                parts = line.split('[')[1].split(']')[0].split(', ')
                x_min, x_max = float(parts[0]), float(parts[1])
                self.transform_params['habitat_x_range'] = (x_min, x_max)
            elif 'Z:' in line and 'habitat_z_range' not in self.transform_params:
                # Parse Z range
                parts = line.split('[')[1].split(']')[0].split(', ')
                z_min, z_max = float(parts[0]), float(parts[1])
                self.transform_params['habitat_z_range'] = (z_min, z_max)
            elif 'Width:' in line:
                width = float(line.split()[-2])
                self.transform_params['image_width_px'] = width
            elif 'Height:' in line:
                height = float(line.split()[-2])
                self.transform_params['image_height_px'] = height
            elif 'X scale:' in line:
                scale = float(line.split()[-2])
                self.transform_params['x_scale'] = scale
            elif 'Z scale:' in line:
                scale = float(line.split()[-2])
                self.transform_params['z_scale'] = scale
            elif 'px = (x -' in line:
                offset = float(line.split('(x - ')[1].split(')')[0])
                self.transform_params['x_offset'] = offset
            elif 'py = (z -' in line:
                offset = float(line.split('(z - ')[1].split(')')[0])
                self.transform_params['z_offset'] = offset
        
        print("Loaded coordinate transformation parameters")
    
    def _get_obstacle_colors(self) -> List[Tuple[int, int, int]]:
        """Get list of colors that represent obstacles (non-navigable areas)."""
        # Colors to avoid during pathfinding
        obstacle_names = [
            'wall', 'door', 'cabinet', 'base-cabinet', 'wall-cabinet',
            'refrigerator', 'major-appliance', 'desk', 'table', 'chair',
            'bed', 'bathtub', 'toilet', 'sink', 'shower-stall',
            'countertop', 'ceiling', 'pillar', 'beam'
        ]
        
        obstacle_colors = []
        for name in obstacle_names:
            if name in self.color_mapping:
                obstacle_colors.append(self.color_mapping[name])
        
        return obstacle_colors
    
    def _create_obstacle_map(self):
        """Create a binary obstacle map with robot radius consideration."""
        print("Creating obstacle map with robot radius consideration...")
        
        # Convert map to RGB
        map_rgb = (self.map_image * 255).astype(np.uint8)
        if map_rgb.shape[2] == 4:
            map_rgb = map_rgb[:, :, :3]
        
        height, width = map_rgb.shape[:2]
        
        # Create binary obstacle map (True = obstacle, False = free)
        obstacle_map = np.zeros((height, width), dtype=bool)
        
        # Mark obstacle pixels
        for y in range(height):
            for x in range(width):
                pixel_color = tuple(map_rgb[y, x])
                if not self._is_navigable(pixel_color):
                    obstacle_map[y, x] = True
        
        # Dilate obstacles by robot radius + safety margin
        total_radius = int(self.robot_radius_pixels + self.safety_margin_pixels)
        dilated_obstacles = self._dilate_obstacles(obstacle_map, total_radius)
        
        self.obstacle_map = dilated_obstacles
        
        obstacle_count = np.sum(obstacle_map)
        dilated_count = np.sum(dilated_obstacles)
        print(f"Original obstacles: {obstacle_count} pixels")
        print(f"Dilated obstacles: {dilated_count} pixels (radius: {total_radius})")
    
    def _dilate_obstacles(self, obstacle_map: np.ndarray, radius: int) -> np.ndarray:
        """
        Dilate obstacle map by specified radius to account for robot size.
        
        Args:
            obstacle_map: Binary obstacle map
            radius: Dilation radius in pixels
            
        Returns:
            Dilated obstacle map
        """
        if radius <= 0:
            return obstacle_map.copy()
        
        height, width = obstacle_map.shape
        dilated = obstacle_map.copy()
        
        # Create circular structuring element
        y_coords, x_coords = np.ogrid[-radius:radius+1, -radius:radius+1]
        circle_mask = x_coords*x_coords + y_coords*y_coords <= radius*radius
        
        # Apply dilation
        for y in range(height):
            for x in range(width):
                if obstacle_map[y, x]:
                    # Dilate around this obstacle pixel
                    y_min = max(0, y - radius)
                    y_max = min(height, y + radius + 1)
                    x_min = max(0, x - radius)
                    x_max = min(width, x + radius + 1)
                    
                    # Apply circular mask
                    mask_y_min = max(0, radius - y)
                    mask_y_max = mask_y_min + (y_max - y_min)
                    mask_x_min = max(0, radius - x)
                    mask_x_max = mask_x_min + (x_max - x_min)
                    
                    region_mask = circle_mask[mask_y_min:mask_y_max, mask_x_min:mask_x_max]
                    dilated[y_min:y_max, x_min:x_max] |= region_mask
        
        return dilated
    
    def display_map_for_selection(self, target_category: str) -> Tuple[int, int]:
        """
        Display the map and capture user's starting point selection.
        
        Args:
            target_category: Target object category to find
            
        Returns:
            Tuple of (x, y) pixel coordinates of selected starting point
        """
        # Set interactive backend
        import matplotlib
        matplotlib.use('TkAgg')
        
        print(f"\nTarget category: {target_category}")
        print("🖱️  LEFT CLICK on the map to select your starting point...")
        print("🖱️  RIGHT CLICK to reset selection")
        print("❌ CLOSE WINDOW when done")
        
        # Find goal point for display
        try:
            goal_point = self.find_goal_point(target_category)
        except:
            goal_point = (100, 100)  # fallback
        
        # Create figure and display map
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.imshow(self.map_image)
        
        # Mark goal point
        ax.plot(goal_point[0], goal_point[1], 's', color='red', markersize=12,
               markeredgecolor='darkred', markeredgewidth=2, label=f'Target: {target_category}')
        
        ax.set_title(f"Interactive Starting Point Selection\nTarget: {target_category.upper()}", 
                    fontsize=14, fontweight='bold')
        
        # Add instructions
        ax.text(0.02, 0.98, 
               "🖱️  LEFT CLICK: Select starting point\n"
               "🖱️  RIGHT CLICK: Reset selection\n" 
               "❌ CLOSE WINDOW: Confirm selection",
               transform=ax.transAxes, verticalalignment='top', fontsize=11,
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        
        ax.legend()
        ax.axis('off')
        
        # Store click coordinates
        clicked_point = []
        selected_marker = None
        
        def on_click(event):
            nonlocal selected_marker
            if event.inaxes != ax:
                return
            
            x, y = int(event.xdata), int(event.ydata)
            
            if event.button == 1:  # Left click
                # Check if point is collision-free
                if self._is_point_collision_free(x, y):
                    # Clear previous selection
                    if selected_marker:
                        selected_marker.remove()
                    
                    clicked_point.clear()
                    clicked_point.append((x, y))
                    
                    # Mark the selected point
                    selected_marker = ax.plot(x, y, 'o', color='lime', markersize=15, 
                                            markeredgecolor='darkgreen', markeredgewidth=3, 
                                            label=f'Start: ({x},{y})')[0]
                    ax.legend()
                    plt.draw()
                    
                    print(f"✓ Valid starting point selected: ({x}, {y})")
                else:
                    print(f"✗ Invalid point ({x}, {y}) - too close to obstacles!")
                    # Show warning marker
                    warning = ax.plot(x, y, 'X', color='red', markersize=12, markeredgewidth=3)[0]
                    plt.draw()
                    # Remove warning after 1 second
                    fig.canvas.start_event_loop(timeout=1)
                    try:
                        warning.remove()
                        plt.draw()
                    except:
                        pass
                        
            elif event.button == 3:  # Right click - reset
                if selected_marker:
                    selected_marker.remove()
                    selected_marker = None
                clicked_point.clear()
                ax.legend()
                plt.draw()
                print("🔄 Selection reset")
        
        # Connect click event
        fig.canvas.mpl_connect('button_press_event', on_click)
        
        # Show the plot and wait for user interaction
        plt.show(block=True)
        
        if not clicked_point:
            raise ValueError("No starting point was selected!")
        
        return clicked_point[0]
    
    def find_goal_point(self, target_category: str) -> Tuple[int, int]:
        """
        Find the goal point near the target object category.
        
        Args:
            target_category: Category to find (e.g., 'rack', 'sofa')
            
        Returns:
            Tuple of (x, y) pixel coordinates of the goal point
        """
        if target_category not in self.target_categories:
            raise ValueError(f"Unknown target category: {target_category}")
        
        target_color = self.target_categories[target_category]
        print(f"Looking for {target_category} with color {target_color}...")
        
        # Convert map to RGB values (0-255)
        map_rgb = (self.map_image * 255).astype(np.uint8)
        # Handle RGBA images by taking only RGB channels
        if map_rgb.shape[2] == 4:
            map_rgb = map_rgb[:, :, :3]
        
        # Find pixels matching the target color
        matches = np.all(map_rgb == target_color, axis=2)
        target_pixels = np.where(matches)
        
        if len(target_pixels[0]) == 0:
            raise ValueError(f"No pixels found for category {target_category}")
        
        print(f"Found {len(target_pixels[0])} pixels for {target_category}")
        
        # Calculate centroid of target object
        centroid_y = int(np.mean(target_pixels[0]))
        centroid_x = int(np.mean(target_pixels[1]))
        
        print(f"Target object centroid: ({centroid_x}, {centroid_y})")
        
        # Find nearest navigable pixel to centroid
        goal_point = self._find_nearest_navigable_pixel(centroid_x, centroid_y)
        
        print(f"Goal point: {goal_point}")
        return goal_point
    
    def _find_nearest_navigable_pixel(self, x: int, y: int) -> Tuple[int, int]:
        """
        Find the nearest navigable pixel to the given coordinates.
        Uses the obstacle map that accounts for robot radius.
        
        Args:
            x, y: Target coordinates
            
        Returns:
            Nearest navigable pixel coordinates
        """
        # Search in expanding circles around the target point
        for radius in range(1, 100):  # Increased search radius
            for angle in np.linspace(0, 2*np.pi, max(8, 4*radius)):
                check_x = int(x + radius * np.cos(angle))
                check_y = int(y + radius * np.sin(angle))
                
                # Check if this pixel is navigable using obstacle map
                if self._is_point_collision_free(check_x, check_y):
                    return (check_x, check_y)
        
        # If no navigable pixel found nearby, return original point
        print(f"Warning: Could not find navigable pixel near ({x}, {y})")
        return (x, y)
    
    def _is_navigable(self, pixel_color: Tuple[int, int, int]) -> bool:
        """
        Check if a pixel color represents a navigable area.
        
        Args:
            pixel_color: RGB color tuple
            
        Returns:
            True if navigable, False if obstacle
        """
        # Check if it's an obstacle color
        for obstacle_color in self.obstacle_colors:
            if pixel_color == obstacle_color:
                return False
        
        # Floor is navigable
        floor_color = self.color_mapping.get('floor', (255, 194, 7))
        if pixel_color == floor_color:
            return True
        
        # Also consider some other safe colors as navigable
        safe_colors = ['rug', 'mat']
        for safe_name in safe_colors:
            if safe_name in self.color_mapping:
                if pixel_color == self.color_mapping[safe_name]:
                    return True
        
        # Default: assume navigable if not explicitly an obstacle
        return True
    
    def _is_point_collision_free(self, x: float, y: float) -> bool:
        """
        Check if a point is collision-free using precomputed obstacle map.
        
        Args:
            x, y: Point coordinates in pixels
            
        Returns:
            True if collision-free, False if in collision
        """
        if self.obstacle_map is None:
            return True
        
        height, width = self.obstacle_map.shape
        
        # Check bounds
        if not (0 <= x < width and 0 <= y < height):
            return False
        
        # Check if point is in obstacle (dilated map already includes robot radius)
        return not self.obstacle_map[int(y), int(x)]
    
    def _is_path_clear(self, from_node: Node, to_x: float, to_y: float) -> bool:
        """
        Check if the path between two points is clear of obstacles.
        Uses the precomputed obstacle map for efficient collision checking.
        
        Args:
            from_node: Starting node
            to_x, to_y: End coordinates
            
        Returns:
            True if path is clear, False if blocked
        """
        # Check if endpoints are collision-free
        if not self._is_point_collision_free(from_node.x, from_node.y):
            return False
        if not self._is_point_collision_free(to_x, to_y):
            return False
        
        # Sample points along the line
        dx = to_x - from_node.x
        dy = to_y - from_node.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1:
            return True
        
        # Sample every 2 pixels along the path for efficiency
        num_samples = max(int(distance / 2), 1)
        
        for i in range(num_samples + 1):
            t = i / max(1, num_samples)
            check_x = from_node.x + t * dx
            check_y = from_node.y + t * dy
            
            if not self._is_point_collision_free(check_x, check_y):
                return False
        
        return True
    
    def find_nearest_node(self, x: float, y: float) -> Node:
        """Find the nearest node in the tree to given coordinates."""
        min_distance = float('inf')
        nearest_node = None
        
        for node in self.tree_nodes:
            distance = node.distance_to_point(x, y)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
    
    def extend_tree(self, from_node: Node, toward_x: float, toward_y: float) -> Optional[Node]:
        """
        Extend the tree from a node toward a target point.
        
        Args:
            from_node: Node to extend from
            toward_x, toward_y: Target coordinates
            
        Returns:
            New node if successful, None if blocked
        """
        # Calculate direction vector
        dx = toward_x - from_node.x
        dy = toward_y - from_node.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return None
        
        # Normalize and scale by step size
        unit_x = dx / distance
        unit_y = dy / distance
        
        new_x = from_node.x + unit_x * min(self.step_size, distance)
        new_y = from_node.y + unit_y * min(self.step_size, distance)
        
        # Check if path is clear
        if self._is_path_clear(from_node, new_x, new_y):
            new_node = Node(new_x, new_y, from_node)
            self.tree_nodes.append(new_node)
            return new_node
        
        return None
    
    def run_rrt(self, start_point: Tuple[int, int], goal_point: Tuple[int, int]) -> List[Node]:
        """
        Run the RRT algorithm to find a path.
        
        Args:
            start_point: Starting (x, y) coordinates
            goal_point: Goal (x, y) coordinates
            
        Returns:
            List of nodes representing the path (empty if no path found)
        """
        print(f"Running RRT from {start_point} to {goal_point}...")
        
        # Validate start point is collision-free
        if not self._is_point_collision_free(start_point[0], start_point[1]):
            print("Warning: Start point is not collision-free, finding nearest safe point...")
            safe_start = self._find_nearest_navigable_pixel(start_point[0], start_point[1])
            print(f"Using safe start point: {safe_start}")
            start_point = safe_start
        
        # Validate goal point is collision-free
        if not self._is_point_collision_free(goal_point[0], goal_point[1]):
            print("Warning: Goal point is not collision-free, finding nearest safe point...")
            safe_goal = self._find_nearest_navigable_pixel(goal_point[0], goal_point[1])
            print(f"Using safe goal point: {safe_goal}")
            goal_point = safe_goal
        
        # Initialize tree with start node
        self.start_node = Node(start_point[0], start_point[1])
        self.tree_nodes = [self.start_node]
        self.goal_point = goal_point
        
        height, width = self.map_image.shape[:2]
        
        for iteration in range(self.max_iterations):
            # Sample random point with goal bias
            if random.random() < self.goal_bias_probability:
                # Sample toward goal
                rand_x = goal_point[0]
                rand_y = goal_point[1]
            else:
                # Random sample
                rand_x = random.uniform(0, width)
                rand_y = random.uniform(0, height)
            
            # Find nearest node in tree
            nearest_node = self.find_nearest_node(rand_x, rand_y)
            
            # Extend tree toward random point
            new_node = self.extend_tree(nearest_node, rand_x, rand_y)
            
            if new_node is None:
                continue
            
            # Check if we've reached the goal
            distance_to_goal = new_node.distance_to_point(goal_point[0], goal_point[1])
            if distance_to_goal <= self.goal_tolerance:
                print(f"Goal reached in {iteration + 1} iterations!")
                
                # Try to connect directly to goal
                final_node = self.extend_tree(new_node, goal_point[0], goal_point[1])
                if final_node is not None:
                    new_node = final_node
                
                # Reconstruct path
                self.path = self._reconstruct_path(new_node)
                return self.path
            
            # Progress update
            if (iteration + 1) % 500 == 0:
                print(f"Iteration {iteration + 1}, tree size: {len(self.tree_nodes)}")
        
        print("Max iterations reached without finding path!")
        return []
    
    def _reconstruct_path(self, goal_node: Node) -> List[Node]:
        """Reconstruct path from goal node back to start."""
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current)
            current = current.parent
        
        path.reverse()
        return path
    
    def visualize_path(self, output_path: str = "path_result.png"):
        """
        Visualize the found path on the map and save the result.
        
        Args:
            output_path: Path to save the visualization
        """
        if not self.path:
            print("No path to visualize!")
            return
        
        fig, ax = plt.subplots(figsize=(15, 10))
        ax.imshow(self.map_image)
        
        # Draw tree structure like in the reference image
        for node in self.tree_nodes:
            if node.parent is not None:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                       'c-', alpha=0.4, linewidth=0.8)
        
        # Draw path with enhanced visibility
        if len(self.path) > 1:
            path_x = [node.x for node in self.path]
            path_y = [node.y for node in self.path]
            # Draw path with thick red line and white outline
            ax.plot(path_x, path_y, 'w-', linewidth=6, alpha=0.8)  # White outline
            ax.plot(path_x, path_y, 'r-', linewidth=4, label='RRT Path')  # Red path
        
        # Mark start and goal
        start = self.path[0]
        goal = self.path[-1]
        ax.plot(start.x, start.y, 'go', markersize=12, markeredgecolor='white',
               markeredgewidth=2, label='Start')
        ax.plot(goal.x, goal.y, 'ro', markersize=12, markeredgecolor='white',
               markeredgewidth=2, label='Goal')
        
        # Add goal tolerance circle
        circle = Circle((self.goal_point[0], self.goal_point[1]), 
                       self.goal_tolerance, fill=False, color='red', 
                       linestyle='--', alpha=0.7, label='Goal Region')
        ax.add_patch(circle)
        
        ax.set_title('RRT Pathfinding Result')
        ax.legend()
        ax.axis('off')
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"Path visualization saved to: {output_path}")
    
    def visualize_obstacle_map(self, output_path: str = "obstacle_map.png"):
        """
        Visualize the obstacle map for debugging purposes.
        
        Args:
            output_path: Path to save the obstacle map visualization
        """
        if self.obstacle_map is None:
            print("No obstacle map to visualize!")
            return
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # Original map
        ax1.imshow(self.map_image)
        ax1.set_title('Original Map')
        ax1.axis('off')
        
        # Obstacle map
        ax2.imshow(self.obstacle_map, cmap='gray')
        ax2.set_title(f'Obstacle Map (Robot Radius: {self.robot_radius_pixels:.1f}px + Safety: {self.safety_margin_pixels:.1f}px)')
        ax2.axis('off')
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.show()
        
        print(f"Obstacle map visualization saved to: {output_path}")
    
    def pixel_to_habitat_coordinates(self, pixel_path: List[Node]) -> List[Tuple[float, float]]:
        """
        Transform pixel coordinates to habitat coordinate system.
        
        Args:
            pixel_path: List of nodes in pixel coordinates
            
        Returns:
            List of (x, z) coordinates in habitat coordinate system
        """
        if not pixel_path:
            return []
        
        habitat_coords = []
        
        for node in pixel_path:
            # Convert from pixel to habitat coordinates using transformation parameters
            habitat_x = node.x / self.transform_params['x_scale'] + self.transform_params['x_offset']
            habitat_z = node.y / self.transform_params['z_scale'] + self.transform_params['z_offset']
            
            habitat_coords.append((habitat_x, habitat_z))
        
        return habitat_coords
    
    def run_pathfinding(self, target_category: str) -> Tuple[List[Node], List[Tuple[float, float]]]:
        """
        Complete pathfinding pipeline.
        
        Args:
            target_category: Target object category
            
        Returns:
            Tuple of (pixel_path, habitat_coordinates)
        """
        # Step 1: Get user's starting point
        start_point = self.display_map_for_selection(target_category)
        
        # Step 2: Find goal point
        goal_point = self.find_goal_point(target_category)
        
        # Step 3: Run RRT algorithm
        path = self.run_rrt(start_point, goal_point)
        
        if not path:
            print("No path found!")
            return [], []
        
        # Step 4: Visualize result
        self.visualize_path()
        
        # Step 5: Transform to habitat coordinates
        habitat_coords = self.pixel_to_habitat_coordinates(path)
        
        print(f"\nPath found with {len(path)} points:")
        print("Habitat coordinates:")
        for i, (x, z) in enumerate(habitat_coords):
            print(f"  {i}: ({x:.3f}, {z:.3f})")
        
        return path, habitat_coords


def main():
    """Main function to run the RRT pathfinding."""
    # File paths
    map_path = "map.png"
    excel_path = "../color_coding_semantic_segmentation_classes.xlsx"
    transform_path = "coordinate_transformation.txt"
    
    # Initialize pathfinder
    pathfinder = RRTPathfinder(map_path, excel_path, transform_path)
    
    # Get target category from user
    print("Available target categories:")
    for category in pathfinder.target_categories.keys():
        print(f"  - {category}")
    
    target_category = input("\nEnter target category: ").strip().lower()
    
    if target_category not in pathfinder.target_categories:
        print(f"Invalid category! Choose from: {list(pathfinder.target_categories.keys())}")
        return
    
    # Run pathfinding
    try:
        pixel_path, habitat_coords = pathfinder.run_pathfinding(target_category)
        
        if habitat_coords:
            print(f"\nPathfinding successful! Found path to {target_category}.")
        else:
            print(f"\nPathfinding failed. Could not find path to {target_category}.")
            
    except Exception as e:
        print(f"Error during pathfinding: {e}")


if __name__ == "__main__":
    main()