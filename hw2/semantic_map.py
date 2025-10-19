"""
Part 1: 2D Semantic Map Construction
This script generates a 2D semantic map from a 3D semantic point cloud.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import argparse


def load_data(data_dir='semantic_3d_pointcloud'):
    """
    Load point cloud data and color information.
    
    Returns:
        points: (N, 3) array of 3D coordinates
        colors_01: (N, 3) array of RGB colors in [0, 1]
        colors_0255: (N, 3) array of RGB colors in [0, 255]
    """
    points = np.load(f'{data_dir}/point.npy')
    colors_01 = np.load(f'{data_dir}/color01.npy')
    colors_0255 = np.load(f'{data_dir}/color0255.npy')
    
    print(f"Loaded point cloud with {len(points)} points")
    print(f"Point cloud shape: {points.shape}")
    print(f"Colors (0-1) shape: {colors_01.shape}")
    print(f"Colors (0-255) shape: {colors_0255.shape}")
    
    return points, colors_01, colors_0255


def remove_ceiling_and_floor(points, colors, floor_threshold=0.1, ceiling_threshold=0.9):
    """
    Remove ceiling and floor points from the point cloud.
    
    Args:
        points: (N, 3) array of 3D coordinates
        colors: (N, 3) array of RGB colors
        floor_threshold: Y-coordinate threshold for floor (normalized)
        ceiling_threshold: Y-coordinate threshold for ceiling (normalized)
    
    Returns:
        filtered_points: Point cloud without ceiling and floor
        filtered_colors: Corresponding colors
    """
    # Get Y coordinates (vertical axis)
    y_coords = points[:, 1]
    
    # Find min and max Y to understand the range
    y_min, y_max = y_coords.min(), y_coords.max()
    print(f"\nY-coordinate range: [{y_min:.4f}, {y_max:.4f}]")
    
    # Create mask to filter out floor and ceiling
    # Keep points that are not too low (floor) and not too high (ceiling)
    mask = (y_coords > floor_threshold) & (y_coords < ceiling_threshold)
    
    filtered_points = points[mask]
    filtered_colors = colors[mask]
    
    print(f"Points after filtering: {len(filtered_points)} (removed {len(points) - len(filtered_points)})")
    
    return filtered_points, filtered_colors


def convert_to_habitat_coordinates(points):
    """
    Convert point cloud coordinates to Habitat coordinates.
    
    Scale relationship: apartment_0 = points array * 10000.0 / 255.0
    
    Args:
        points: (N, 3) array in point cloud coordinates
    
    Returns:
        habitat_coords: (N, 3) array in Habitat coordinates
    """
    habitat_coords = points * 10000.0 / 255.0
    print(f"\nHabitat coordinate range:")
    print(f"  X: [{habitat_coords[:, 0].min():.2f}, {habitat_coords[:, 0].max():.2f}]")
    print(f"  Y: [{habitat_coords[:, 1].min():.2f}, {habitat_coords[:, 1].max():.2f}]")
    print(f"  Z: [{habitat_coords[:, 2].min():.2f}, {habitat_coords[:, 2].max():.2f}]")
    
    return habitat_coords


def create_2d_semantic_map(points, colors, output_file='map.png', dpi=150, figsize=(12, 10)):
    """
    Create a 2D semantic map by projecting 3D points to 2D (X-Z plane).
    
    Args:
        points: (N, 3) array of 3D coordinates in Habitat space
        colors: (N, 3) array of RGB colors (0-1 or 0-255)
        output_file: Output filename for the map
        dpi: Resolution of the output image
        figsize: Figure size in inches
    """
    # Extract X and Z coordinates for 2D projection
    x_coords = points[:, 0]
    z_coords = points[:, 2]
    
    # Normalize colors if needed
    if colors.max() > 1.0:
        colors_normalized = colors / 255.0
    else:
        colors_normalized = colors
    
    # Create the scatter plot
    plt.figure(figsize=figsize, dpi=dpi)
    
    # Plot points with their semantic colors
    plt.scatter(x_coords, z_coords, c=colors_normalized, s=1, marker='.')
    
    plt.xlabel('X Coordinate (Habitat)', fontsize=12)
    plt.ylabel('Z Coordinate (Habitat)', fontsize=12)
    plt.title('2D Semantic Map (Top-Down View)', fontsize=14, fontweight='bold')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    
    # Add coordinate information
    x_range = x_coords.max() - x_coords.min()
    z_range = z_coords.max() - z_coords.min()
    plt.text(0.02, 0.98, 
             f'X range: [{x_coords.min():.2f}, {x_coords.max():.2f}] ({x_range:.2f}m)\n'
             f'Z range: [{z_coords.min():.2f}, {z_coords.max():.2f}] ({z_range:.2f}m)\n'
             f'Points: {len(points):,}',
             transform=plt.gca().transAxes,
             fontsize=10,
             verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=dpi, bbox_inches='tight')
    print(f"\n2D semantic map saved to: {output_file}")
    
    plt.show()
    
    return x_coords, z_coords


def save_filtered_data(points, colors, output_dir='semantic_3d_pointcloud'):
    """
    Save the filtered points and colors for later use.
    
    Args:
        points: Filtered 3D points
        colors: Filtered colors
        output_dir: Directory to save the data
    """
    np.save(f'{output_dir}/filtered_points.npy', points)
    np.save(f'{output_dir}/filtered_colors.npy', colors)
    print(f"\nFiltered data saved to {output_dir}/")


def calculate_coordinate_mapping(points):
    """
    Calculate the relationship between map (pixel) coordinates and Habitat coordinates.
    This is necessary for Part 3.
    
    Args:
        points: (N, 3) array in Habitat coordinates
    
    Returns:
        mapping_info: Dictionary containing mapping parameters
    """
    x_coords = points[:, 0]
    z_coords = points[:, 2]
    
    mapping_info = {
        'x_min': x_coords.min(),
        'x_max': x_coords.max(),
        'z_min': z_coords.min(),
        'z_max': z_coords.max(),
        'x_range': x_coords.max() - x_coords.min(),
        'z_range': z_coords.max() - z_coords.min(),
    }
    
    print("\n" + "="*60)
    print("COORDINATE MAPPING INFORMATION (for Part 3)")
    print("="*60)
    print(f"Habitat X range: [{mapping_info['x_min']:.3f}, {mapping_info['x_max']:.3f}] m")
    print(f"Habitat Z range: [{mapping_info['z_min']:.3f}, {mapping_info['z_max']:.3f}] m")
    print(f"Map dimensions: {mapping_info['x_range']:.3f} x {mapping_info['z_range']:.3f} m")
    print("\nTo convert Habitat coordinates (x_h, z_h) to map pixel coordinates (x_p, z_p):")
    print("  x_p = (x_h - x_min) * scale")
    print("  z_p = (z_h - z_min) * scale")
    print("where scale depends on your desired map resolution")
    print("="*60)
    
    return mapping_info


def main():
    parser = argparse.ArgumentParser(description='Generate 2D semantic map from 3D point cloud')
    parser.add_argument('--data_dir', type=str, default='semantic_3d_pointcloud',
                        help='Directory containing point cloud data')
    parser.add_argument('--output', type=str, default='map.png',
                        help='Output filename for the 2D map')
    parser.add_argument('--floor_threshold', type=float, default=0.1,
                        help='Y-coordinate threshold for floor removal (normalized)')
    parser.add_argument('--ceiling_threshold', type=float, default=0.9,
                        help='Y-coordinate threshold for ceiling removal (normalized)')
    parser.add_argument('--dpi', type=int, default=150,
                        help='Output image resolution (DPI)')
    parser.add_argument('--use_color_0255', action='store_true',
                        help='Use color_0255.npy instead of color01.npy')
    args = parser.parse_args()
    
    print("="*60)
    print("2D SEMANTIC MAP CONSTRUCTION - PART 1")
    print("="*60)
    
    # Step 1: Load data
    print("\nStep 1: Loading point cloud data...")
    points, colors_01, colors_0255 = load_data(args.data_dir)
    
    # Choose which color array to use
    colors = colors_0255 if args.use_color_0255 else colors_01
    print(f"Using color range: [0, {'255' if args.use_color_0255 else '1'}]")
    
    # Step 2: Remove ceiling and floor
    print("\nStep 2: Removing ceiling and floor points...")
    filtered_points, filtered_colors = remove_ceiling_and_floor(
        points, colors, 
        floor_threshold=args.floor_threshold,
        ceiling_threshold=args.ceiling_threshold
    )
    
    # Step 3: Convert to Habitat coordinates
    print("\nStep 3: Converting to Habitat coordinates...")
    habitat_points = convert_to_habitat_coordinates(filtered_points)
    
    # Step 4: Calculate coordinate mapping (for Part 3)
    print("\nStep 4: Calculating coordinate mapping...")
    mapping_info = calculate_coordinate_mapping(habitat_points)
    
    # Step 5: Create and save 2D semantic map
    print("\nStep 5: Creating 2D semantic map...")
    x_coords, z_coords = create_2d_semantic_map(
        habitat_points, filtered_colors, 
        output_file=args.output,
        dpi=args.dpi
    )
    
    # Step 6: Save filtered data
    print("\nStep 6: Saving filtered data...")
    save_filtered_data(habitat_points, filtered_colors, args.data_dir)
    
    # Save mapping info
    np.save(f'{args.data_dir}/mapping_info.npy', mapping_info)
    
    print("\n" + "="*60)
    print("COMPLETED SUCCESSFULLY!")
    print("="*60)
    print(f"Output files:")
    print(f"  - 2D map: {args.output}")
    print(f"  - Filtered points: {args.data_dir}/filtered_points.npy")
    print(f"  - Filtered colors: {args.data_dir}/filtered_colors.npy")
    print(f"  - Mapping info: {args.data_dir}/mapping_info.npy")
    print("="*60)


if __name__ == '__main__':
    main()
