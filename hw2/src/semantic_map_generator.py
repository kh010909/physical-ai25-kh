import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from typing import Tuple, Dict, Any


class SemanticMapGenerator:
    """
    A class to generate 2D semantic maps from 3D point cloud data.
    """
    
    def __init__(self, data_dir: str = "../", output_dir: str = "./"):
        """
        Initialize the semantic map generator.
        
        Args:
            data_dir: Directory containing the data files
            output_dir: Directory to save output files
        """
        self.data_dir = data_dir
        self.output_dir = output_dir
        
        # File paths
        self.point_file = os.path.join(data_dir, "semantic_3d_pointcloud", "point.npy")
        self.color01_file = os.path.join(data_dir, "semantic_3d_pointcloud", "color01.npy")
        self.color0255_file = os.path.join(data_dir, "semantic_3d_pointcloud", "color0255.npy")
        self.excel_file = os.path.join(data_dir, "color_coding_semantic_segmentation_classes.xlsx")
        
        # Data containers
        self.points = None
        self.colors01 = None
        self.colors0255 = None
        self.color_mapping = None
        
        # Ceiling and floor colors (RGB 0-255 format)
        self.ceiling_color = (8, 255, 214)  # From Excel file
        self.floor_color = (255, 194, 7)    # From Excel file
        
    def load_data(self) -> None:
        """Load all required data files."""
        print("Loading data files...")
        
        # Load point cloud data
        self.points = np.load(self.point_file)
        self.colors01 = np.load(self.color01_file)
        self.colors0255 = np.load(self.color0255_file)
        
        # Apply coordinate scaling as mentioned in the notes
        # apartment_0_coordinates = point_array * 10000.0 / 255.0
        self.points = self.points * 10000.0 / 255.0
        
        print(f"Loaded {len(self.points)} points")
        print(f"Point cloud bounds: X[{self.points[:, 0].min():.2f}, {self.points[:, 0].max():.2f}], "
              f"Y[{self.points[:, 1].min():.2f}, {self.points[:, 1].max():.2f}], "
              f"Z[{self.points[:, 2].min():.2f}, {self.points[:, 2].max():.2f}]")
        
        # Load color mapping
        self.color_mapping = pd.read_excel(self.excel_file)
        print(f"Loaded color mapping with {len(self.color_mapping)} semantic classes")
        
    def filter_ceiling_floor_height(self, height_min: float = None, height_max: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Filter out ceiling and floor points from the point cloud, and apply height filtering.
        
        Args:
            height_min: Minimum Y-coordinate to keep (if None, calculated automatically)
            height_max: Maximum Y-coordinate to keep (if None, calculated automatically)
        
        Returns:
            Tuple of (filtered_points, filtered_colors) as numpy arrays
        """
        print("Filtering out ceiling and floor points...")
        
        # Convert ceiling and floor colors to the same format as our data (0-255)
        ceiling_rgb = np.array(self.ceiling_color, dtype=np.uint8)
        floor_rgb = np.array(self.floor_color, dtype=np.uint8)
        
        # Convert colors0255 to uint8 for exact matching
        colors_uint8 = np.round(self.colors0255).astype(np.uint8)
        
        # Create masks for ceiling and floor points
        ceiling_mask = np.all(colors_uint8 == ceiling_rgb, axis=1)
        floor_mask = np.all(colors_uint8 == floor_rgb, axis=1)
        
        # Combine masks (points to remove)
        semantic_remove_mask = ceiling_mask | floor_mask
        
        print(f"Found {np.sum(ceiling_mask)} ceiling points")
        print(f"Found {np.sum(floor_mask)} floor points")
        print(f"Removing {np.sum(semantic_remove_mask)} semantic ceiling/floor points")
        
        # Apply height filtering
        print("\nApplying height filtering...")
        y_coords = self.points[:, 1]
        
        if height_min is None or height_max is None:
            # Calculate reasonable height range (remove extreme top/bottom percentiles)
            y_sorted = np.sort(y_coords)
            if height_min is None:
                height_min = np.percentile(y_sorted, 10)  # Remove bottom 10%
            if height_max is None:
                height_max = np.percentile(y_sorted, 90)  # Remove top 10%
        
        print(f"Height range: Y ∈ [{height_min:.3f}, {height_max:.3f}]")
        print(f"Original Y range: [{y_coords.min():.3f}, {y_coords.max():.3f}]")
        
        # Create height filter mask
        height_mask = (y_coords >= height_min) & (y_coords <= height_max)
        
        # Combine all filters: keep points that are NOT (ceiling OR floor) AND within height range
        final_keep_mask = (~semantic_remove_mask) & height_mask
        
        filtered_points = self.points[final_keep_mask]
        filtered_colors = self.colors01[final_keep_mask]  # Use 0-1 range for matplotlib
        
        print(f"Points removed by height filter: {np.sum(~height_mask)}")
        print(f"Total points removed: {np.sum(~final_keep_mask)}")
        print(f"Remaining points after filtering: {len(filtered_points)}")
        
        return filtered_points, filtered_colors
    
    def generate_2d_map(self, points: np.ndarray, colors: np.ndarray, 
                       figsize: Tuple[int, int] = (12, 8), dpi: int = 300) -> Dict[str, Any]:
        """
        Generate a 2D top-down map using X and Z coordinates.
        
        Args:
            points: Filtered 3D points array
            colors: Corresponding RGB colors (0-1 range)
            figsize: Figure size in inches
            dpi: Dots per inch for the saved image
            
        Returns:
            Dictionary containing transformation parameters
        """
        print("Generating 2D map...")
        
        # Extract X and Z coordinates for top-down view
        x_coords = points[:, 0]  # X-axis in plot
        z_coords = points[:, 2]  # Y-axis in plot (Z becomes Y in top-down view)
        
        # Create the plot with no frame
        fig, ax = plt.subplots(figsize=figsize, dpi=dpi)
        
        # Create scatter plot with original RGB colors
        scatter = ax.scatter(x_coords, z_coords, c=colors, s=0.1, alpha=0.8)
        
        # Set equal aspect ratio to maintain spatial relationships
        ax.set_aspect('equal')
        
        # Remove all axes, labels, ticks, and frame
        ax.set_xticks([])
        ax.set_yticks([])
        ax.axis('off')
        
        # Get plot boundaries for coordinate transformation
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        # Save the map with no padding or borders
        output_path = os.path.join(self.output_dir, "map.png")
        plt.savefig(output_path, dpi=dpi, bbox_inches='tight', pad_inches=0, 
                   facecolor='white', edgecolor='none')
        print(f"Map saved as: {output_path}")
        
        plt.close()  # Close the figure to free memory
        
        return True