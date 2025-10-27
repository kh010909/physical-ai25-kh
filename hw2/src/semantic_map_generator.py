#!/usr/bin/env python3
"""
2D Semantic Map Generator

This script constructs a 2D semantic map by projecting a 3D semantic point cloud 
onto a top-down (X-Z plane) view, filtering out ceiling and floor points.

Requirements:
- numpy
- matplotlib 
- pandas
- openpyxl (for reading Excel files)

Usage:
    python semantic_map_generator.py
"""

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
        
        # Calculate transformation parameters
        fig_width_inch, fig_height_inch = fig.get_size_inches()
        fig_width_px = fig_width_inch * dpi
        fig_height_px = fig_height_inch * dpi
        
        transformation = {
            'habitat_x_range': xlim,
            'habitat_z_range': ylim,
            'image_width_px': fig_width_px,
            'image_height_px': fig_height_px,
            'x_scale': fig_width_px / (xlim[1] - xlim[0]),
            'z_scale': fig_height_px / (ylim[1] - ylim[0]),
            'x_offset': xlim[0],
            'z_offset': ylim[0],
            'dpi': dpi
        }
        
        plt.close()  # Close the figure to free memory
        
        return transformation
    
    def save_transformation(self, transformation: Dict[str, Any]) -> None:
        """
        Save transformation parameters for future use.
        
        Args:
            transformation: Dictionary containing transformation parameters
        """
        transform_file = os.path.join(self.output_dir, "coordinate_transformation.txt")
        
        with open(transform_file, 'w') as f:
            f.write("=== COORDINATE TRANSFORMATION PARAMETERS ===\n\n")
            f.write("This file contains the transformation between 3D Habitat coordinates (X, Z)\n")
            f.write("and pixel coordinates in the saved map.png file.\n\n")
            
            f.write("Habitat Coordinate Ranges:\n")
            f.write(f"  X: [{transformation['habitat_x_range'][0]:.6f}, {transformation['habitat_x_range'][1]:.6f}]\n")
            f.write(f"  Z: [{transformation['habitat_z_range'][0]:.6f}, {transformation['habitat_z_range'][1]:.6f}]\n\n")
            
            f.write("Image Properties:\n")
            f.write(f"  Width:  {transformation['image_width_px']:.0f} pixels\n")
            f.write(f"  Height: {transformation['image_height_px']:.0f} pixels\n")
            f.write(f"  DPI:    {transformation['dpi']}\n\n")
            
            f.write("Scaling Factors:\n")
            f.write(f"  X scale: {transformation['x_scale']:.6f} pixels/habitat_unit\n")
            f.write(f"  Z scale: {transformation['z_scale']:.6f} pixels/habitat_unit\n\n")
            
            f.write("Conversion Formulas:\n")
            f.write("  From Habitat (x, z) to Pixel (px, py):\n")
            f.write(f"    px = (x - {transformation['x_offset']:.6f}) * {transformation['x_scale']:.6f}\n")
            f.write(f"    py = (z - {transformation['z_offset']:.6f}) * {transformation['z_scale']:.6f}\n\n")
            
            f.write("  From Pixel (px, py) to Habitat (x, z):\n")
            f.write(f"    x = px / {transformation['x_scale']:.6f} + {transformation['x_offset']:.6f}\n")
            f.write(f"    z = py / {transformation['z_scale']:.6f} + {transformation['z_offset']:.6f}\n\n")
            
            f.write("Note: The Y-axis in the image corresponds to the Z-axis in Habitat coordinates.\n")
            f.write("The origin (0,0) in image coordinates is at the top-left corner.\n")
        
        print(f"Transformation parameters saved to: {transform_file}")
    
    def run(self, height_min: float = None, height_max: float = None) -> None:
        """
        Execute the complete pipeline.
        
        Args:
            height_min: Minimum Y-coordinate to keep (if None, uses 10th percentile)
            height_max: Maximum Y-coordinate to keep (if None, uses 90th percentile)
        """
        print("=== Starting 2D Semantic Map Generation ===\n")
        
        # Step 1: Load data
        self.load_data()
        print()
        
        # Step 2 & 3: Filter point cloud and extract data
        filtered_points, filtered_colors = self.filter_ceiling_floor_height(height_min, height_max)
        print()
        
        # Step 4 & 5: Generate and save 2D map
        transformation = self.generate_2d_map(filtered_points, filtered_colors)
        print()
        
        # Step 6: Save transformation parameters
        self.save_transformation(transformation)
        print()
        
        print("=== 2D Semantic Map Generation Complete ===")