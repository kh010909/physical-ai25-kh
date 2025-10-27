#!/usr/bin/env python3
"""
Main script for 2D Semantic Map Generation

This script allows manual adjustment of height filtering parameters
using absolute Y-coordinate values.
"""

from semantic_map_generator import SemanticMapGenerator
import os


def main():
    """Main function with manual height adjustment."""
    # Create output directory if it doesn't exist
    os.makedirs("./", exist_ok=True)
    
    # Initialize the semantic map generator
    generator = SemanticMapGenerator(data_dir="../")
    
    # Load data first to see the height range
    generator.load_data()
    
    y_coords = generator.points[:, 1]
    print(f"\nHeight statistics:")
    print(f"  Min Y: {y_coords.min():.3f}")
    print(f"  Max Y: {y_coords.max():.3f}")
    print(f"  Mean Y: {y_coords.mean():.3f}")
    print(f"  Median Y: {y_coords[len(y_coords)//2]:.3f}")
    
    # Manual height adjustment - EDIT THESE VALUES AS NEEDED
    # Set to None to use automatic percentile-based filtering
    
    HEIGHT_MIN = -1.2  # Absolute minimum Y coordinate to keep
    HEIGHT_MAX = -0.2   # Absolute maximum Y coordinate to keep
    
    # Uncomment one of these configurations:
    
    # Option 1: Use manual absolute values
    print(f"\nUsing manual height range: Y ∈ [{HEIGHT_MIN}, {HEIGHT_MAX}]")
    filtered_points, filtered_colors = generator.filter_ceiling_floor_height(
        height_min=HEIGHT_MIN, 
        height_max=HEIGHT_MAX
    )
    
    # Option 2: Use automatic percentile-based filtering (uncomment to use)
    # print("\nUsing automatic height range (10th to 90th percentile)...")
    # filtered_points, filtered_colors = generator.filter_ceiling_floor_height()
    
    # Option 3: Conservative filtering (uncomment to use)
    # print("\nUsing conservative height range...")
    # filtered_points, filtered_colors = generator.filter_ceiling_floor_height(
    #     height_min=-0.8, 
    #     height_max=0.8
    # )
    
    # Generate and save the map
    print()
    transformation = generator.generate_2d_map(filtered_points, filtered_colors)
    print()
    generator.save_transformation(transformation)
    print()
    print("=== 2D Semantic Map Generation Complete ===")


if __name__ == "__main__":
    main()