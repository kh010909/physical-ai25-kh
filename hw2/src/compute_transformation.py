import numpy as np
import cv2
import matplotlib.pyplot as plt


def load_pointcloud_data(pointcloud_path='semantic_3d_pointcloud'):
    """Load and process point cloud data."""
    print("Loading point cloud data...")
    
    # Load the 3D semantic map from npy files
    points = np.load(f'../{pointcloud_path}/point.npy')
    colors = np.load(f'../{pointcloud_path}/color0255.npy')
    
    # Apply coordinate scaling as per apartment_0 documentation
    points = points * 10000.0 / 255.0
    
    print(f"Loaded {len(points)} points")
    print(f"Point cloud bounds:")
    print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
    print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
    print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
    
    return points, colors


def filter_ceiling_floor(points, colors):
    """Filter out ceiling and floor points."""
    print("\nFiltering ceiling and floor...")
    
    # Ceiling color: (8, 255, 214) in RGB
    # Floor color: (255, 194, 7) in RGB
    ceiling_color = np.array([8, 255, 214], dtype=np.uint8)
    floor_color = np.array([255, 194, 7], dtype=np.uint8)
    
    # Convert colors to uint8 for exact matching
    colors_uint8 = np.round(colors).astype(np.uint8)
    
    # Create masks
    ceiling_mask = np.all(colors_uint8 == ceiling_color, axis=1)
    floor_mask = np.all(colors_uint8 == floor_color, axis=1)
    combined_mask = ~(ceiling_mask | floor_mask)
    
    filtered_points = points[combined_mask]
    filtered_colors = colors[combined_mask]
    
    print(f"Removed {np.sum(~combined_mask)} points (ceiling/floor)")
    print(f"Remaining: {len(filtered_points)} points")
    
    return filtered_points, filtered_colors


def filter_by_height(points, colors, floor=1):
    """Filter points by height for the specified floor."""
    print(f"\nFiltering for floor {floor}...")
    
    # Height ranges (Y coordinate)
    # Floor 1: approximately -2.0 to -0.5
    # Floor 2: approximately 0.5 to 2.5
    if floor == 1:
        y_min, y_max = -2.5, -0.3
    else:
        y_min, y_max = 0.3, 2.8
    
    mask = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    
    filtered_points = points[mask]
    filtered_colors = colors[mask]
    
    print(f"Height range: [{y_min}, {y_max}]")
    print(f"Remaining: {len(filtered_points)} points")
    
    return filtered_points, filtered_colors


def compute_transformation(points, map_image, floor=1):
    """
    Compute transformation parameters between map and Habitat coordinates.
    
    The map needs to be flipped horizontally and rotated 90° CW to match Habitat orientation.
    This function computes parameters considering this transformation.
    
    Args:
        points: Filtered point cloud (N x 3) - already in Habitat coordinates
        map_image: Original map image (before transformation)
        floor: Floor number
    
    Returns:
        Dictionary with transformation parameters
    """
    print("\n" + "="*70)
    print("COMPUTING TRANSFORMATION PARAMETERS")
    print("="*70)
    
    # Get X and Z coordinates (horizontal plane)
    x_coords = points[:, 0]
    z_coords = points[:, 2]
    
    # Calculate Habitat coordinate bounds from actual point cloud
    x_min, x_max = x_coords.min(), x_coords.max()
    z_min, z_max = z_coords.min(), z_coords.max()
    
    print(f"\nHabitat coordinate ranges (from point cloud):")
    print(f"  X: [{x_min:.6f}, {x_max:.6f}]  span: {x_max - x_min:.6f}m")
    print(f"  Z: [{z_min:.6f}, {z_max:.6f}]  span: {z_max - z_min:.6f}m")
    
    # Original map dimensions
    orig_height, orig_width = map_image.shape[:2]
    print(f"\nOriginal map dimensions: {orig_width} x {orig_height} (W x H)")
    
    # We apply: 1) Flip horizontal, 2) Rotate 90° CW
    # After these transformations:
    # - Width becomes Height: orig_height
    # - Height becomes Width: orig_width
    transformed_width = orig_height
    transformed_height = orig_width
    print(f"Transformed map dimensions: {transformed_width} x {transformed_height} (W x H)")
    
    print(f"\n" + "-"*70)
    print("COORDINATE SYSTEM ALIGNMENT:")
    print("-"*70)
    print("\nAfter flip + rotate transformation:")
    print("  - Map X-axis (horizontal, left to right) -> Habitat Z-axis (forward/back)")
    print("  - Map Y-axis (vertical, top to bottom) -> Habitat X-axis (left/right)")
    print("\nThe point cloud in Habitat coordinates should span:")
    print(f"  - Along map X (→ Habitat Z): {z_max - z_min:.3f}m")
    print(f"  - Along map Y (→ Habitat X): {x_max - x_min:.3f}m")
    
    # Calculate what the actual map dimensions represent
    print(f"\nMap spans in transformed coordinates:")
    print(f"  - Transformed width {transformed_width} pixels represents {z_max - z_min:.3f}m (Z span)")
    print(f"  - Transformed height {transformed_height} pixels represents {x_max - x_min:.3f}m (X span)")
    
    # Check if there's a rotation/skew in the original map plotting
    # The ratio of pixels to meters should be similar in both dimensions if properly aligned
    pixels_per_meter_z = transformed_width / (z_max - z_min)
    pixels_per_meter_x = transformed_height / (x_max - x_min)
    
    print(f"\nPixels per meter:")
    print(f"  - Z direction: {pixels_per_meter_z:.2f} pixels/meter")
    print(f"  - X direction: {pixels_per_meter_x:.2f} pixels/meter")
    
    if abs(pixels_per_meter_z - pixels_per_meter_x) / pixels_per_meter_z > 0.1:
        print(f"\n  WARNING: Aspect ratio mismatch!")
        print(f"   The map may have been plotted with non-uniform scaling.")
        print(f"   Difference: {abs(pixels_per_meter_z - pixels_per_meter_x):.2f} pixels/meter")
    
    print(f"\n" + "-"*70)
    print("TRANSFORMATION FORMULAS (for transformed map):")
    print("-"*70)
    
    print(f"\nFrom Transformed Pixel (px_t, py_t) to Habitat (hx, hz):")
    print(f"  norm_x = px_t / {transformed_width}")
    print(f"  norm_y = py_t / {transformed_height}")
    print(f"  habitat_z = {z_min:.6f} + norm_x * {z_max - z_min:.6f}")
    print(f"  habitat_x = {x_max:.6f} - norm_y * {x_max - x_min:.6f}")
    print(f"\nNote: habitat_x uses (x_max - ...) because image Y increases downward")
    
    print(f"\n" + "-"*70)
    print("REVERSE TRANSFORMATION:")
    print("-"*70)
    print(f"\nFrom Habitat (hx, hz) to Transformed Pixel (px_t, py_t):")
    print(f"  norm_x = (habitat_z - {z_min:.6f}) / {z_max - z_min:.6f}")
    print(f"  norm_y = ({x_max:.6f} - habitat_x) / {x_max - x_min:.6f}")
    print(f"  px_t = norm_x * {transformed_width}")
    print(f"  py_t = norm_y * {transformed_height}")
    
    # Return transformation parameters
    return {
        'habitat_x_min': x_min,
        'habitat_x_max': x_max,
        'habitat_z_min': z_min,
        'habitat_z_max': z_max,
        'original_map_width': orig_width,
        'original_map_height': orig_height,
        'transformed_map_width': transformed_width,
        'transformed_map_height': transformed_height,
        'pixels_per_meter_z': pixels_per_meter_z,
        'pixels_per_meter_x': pixels_per_meter_x,
    }


def visualize_transformation(points, colors, params):
    """Visualize the point cloud and transformation."""
    print("\n" + "="*70)
    print("VISUALIZATION")
    print("="*70)
    
    # Create top-down view
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot 1: Raw point cloud (X-Z plane)
    ax1.scatter(points[:, 2], points[:, 0], c=colors/255.0, s=1, alpha=0.5)
    ax1.set_xlabel('Z (forward/back)')
    ax1.set_ylabel('X (left/right)')
    ax1.set_title('Point Cloud (Top-Down View)\nBefore Map Transformation')
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # Add bounds
    ax1.axhline(params['habitat_x_min'], color='r', linestyle='--', alpha=0.5, label='X bounds')
    ax1.axhline(params['habitat_x_max'], color='r', linestyle='--', alpha=0.5)
    ax1.axvline(params['habitat_z_min'], color='b', linestyle='--', alpha=0.5, label='Z bounds')
    ax1.axvline(params['habitat_z_max'], color='b', linestyle='--', alpha=0.5)
    ax1.legend()
    
    # Plot 2: After map transformation (flip + rotate)
    # Simulate what the map will look like after transformation
    # The map coordinates get rotated, so Z maps to horizontal axis
    ax2.scatter(points[:, 2], points[:, 0], c=colors/255.0, s=1, alpha=0.5)
    ax2.set_xlabel('Map X (after transform) -> Habitat Z')
    ax2.set_ylabel('Map Y (after transform) -> Habitat X')
    ax2.set_title('After Map Transformation\n(Flip Left-Right + Rotate 90° CW)')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    ax2.invert_yaxis()  # Image Y is inverted
    
    plt.tight_layout()
    output_file = 'transformation_visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nVisualization saved to: {output_file}")
    plt.close()


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Compute coordinate transformation parameters')
    parser.add_argument('--floor', type=int, default=1, help='Floor number (1 or 2)')
    parser.add_argument('--map', type=str, default='map.png', help='Path to map image')
    parser.add_argument('--pointcloud', type=str, default='semantic_3d_pointcloud', 
                       help='Path to point cloud directory')
    args = parser.parse_args()
    
    print("="*70)
    print("COORDINATE TRANSFORMATION PARAMETER COMPUTATION")
    print("="*70)
    print(f"Floor: {args.floor}")
    print(f"Map: {args.map}")
    print(f"Point cloud: {args.pointcloud}")
    
    # Load point cloud
    points, colors = load_pointcloud_data(args.pointcloud)
    
    # Filter ceiling and floor
    points, colors = filter_ceiling_floor(points, colors)
    
    # Filter by height for the floor
    points, colors = filter_by_height(points, colors, args.floor)
    
    # Load map image
    print(f"\nLoading map image: {args.map}")
    map_image = cv2.imread(args.map)
    if map_image is None:
        print(f"ERROR: Could not load {args.map}")
        return
    print(f"Map loaded: {map_image.shape[1]} x {map_image.shape[0]}")
    
    # Compute transformation
    params = compute_transformation(points, map_image, args.floor)
    
    # Visualize
    visualize_transformation(points, colors, params)
    
    # Save parameters to file
    output_file = 'computed_transformation.txt'
    with open(output_file, 'w') as f:
        f.write("="*70 + "\n")
        f.write("COMPUTED COORDINATE TRANSFORMATION PARAMETERS\n")
        f.write("="*70 + "\n\n")
        f.write(f"Floor: {args.floor}\n\n")
        
        f.write("Habitat Coordinate Ranges:\n")
        f.write(f"  X: [{params['habitat_x_min']:.6f}, {params['habitat_x_max']:.6f}]\n")
        f.write(f"  Z: [{params['habitat_z_min']:.6f}, {params['habitat_z_max']:.6f}]\n\n")
        
        f.write("Map Dimensions:\n")
        f.write(f"  Original: {params['original_map_width']} x {params['original_map_height']}\n")
        f.write(f"  After flip+rotate: {params['transformed_map_width']} x {params['transformed_map_height']}\n\n")
        
        f.write("Transformation Formulas (for transformed map):\n\n")
        f.write("From Transformed Pixel to Habitat:\n")
        f.write(f"  habitat_z = {params['habitat_z_min']:.6f} + (px_t / {params['transformed_map_width']}) * {params['habitat_z_max'] - params['habitat_z_min']:.6f}\n")
        f.write(f"  habitat_x = {params['habitat_x_max']:.6f} - (py_t / {params['transformed_map_height']}) * {params['habitat_x_max'] - params['habitat_x_min']:.6f}\n\n")
        
        f.write("From Habitat to Transformed Pixel:\n")
        f.write(f"  px_t = ((habitat_z - {params['habitat_z_min']:.6f}) / {params['habitat_z_max'] - params['habitat_z_min']:.6f}) * {params['transformed_map_width']}\n")
        f.write(f"  py_t = (({params['habitat_x_max']:.6f} - habitat_x) / {params['habitat_x_max'] - params['habitat_x_min']:.6f}) * {params['transformed_map_height']}\n")
    
    print(f"\nParameters saved to: {output_file}")
    print("\n" + "="*70)
    print("DONE!")
    print("="*70)
    print(f"\nUse these parameters in your map_utils.py")


if __name__ == '__main__':
    main()
