"""
3D Point Cloud Visualizer

This script helps visualize the 3D point cloud before and after ceiling/floor removal.
Useful for understanding the data and adjusting thresholds.
"""

import numpy as np
import argparse
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("⚠️  Open3D not available. Install it with: pip install open3d")
    print("   This script requires Open3D for 3D visualization.")


def load_and_visualize_3d(data_dir='semantic_3d_pointcloud', 
                          floor_threshold=0.1, 
                          ceiling_threshold=0.9,
                          use_color_0255=False):
    """
    Load and visualize the 3D semantic point cloud with Open3D.
    """
    
    if not OPEN3D_AVAILABLE:
        print("❌ Cannot visualize without Open3D.")
        return
    
    print("="*60)
    print("3D POINT CLOUD VISUALIZATION")
    print("="*60)
    
    # Load data
    print("\nLoading point cloud data...")
    points = np.load(f'{data_dir}/point.npy')
    colors_01 = np.load(f'{data_dir}/color01.npy')
    colors_0255 = np.load(f'{data_dir}/color0255.npy')
    
    colors = colors_0255 if use_color_0255 else colors_01
    print(f"Loaded {len(points):,} points")
    
    # Convert to Habitat coordinates
    habitat_points = points * 10000.0 / 255.0
    
    # Normalize colors if needed
    if colors.max() > 1.0:
        colors_normalized = colors / 255.0
    else:
        colors_normalized = colors
    
    # Create full point cloud
    print("\nCreating full point cloud...")
    pcd_full = o3d.geometry.PointCloud()
    pcd_full.points = o3d.utility.Vector3dVector(habitat_points)
    pcd_full.colors = o3d.utility.Vector3dVector(colors_normalized)
    
    # Create filtered point cloud
    print(f"Creating filtered point cloud (floor: {floor_threshold}, ceiling: {ceiling_threshold})...")
    y_coords = points[:, 1]
    mask = (y_coords > floor_threshold) & (y_coords < ceiling_threshold)
    
    filtered_points = habitat_points[mask]
    filtered_colors = colors_normalized[mask]
    
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points)
    pcd_filtered.colors = o3d.utility.Vector3dVector(filtered_colors)
    
    print(f"Filtered: {len(filtered_points):,} points (removed {len(points) - len(filtered_points):,})")
    
    # Visualize
    print("\n" + "="*60)
    print("VISUALIZATION CONTROLS")
    print("="*60)
    print("Mouse:")
    print("  - Left button: Rotate")
    print("  - Right button: Zoom")
    print("  - Middle button: Pan")
    print("Keyboard:")
    print("  - Q: Quit")
    print("  - H: Show/hide controls")
    print("="*60)
    
    print("\nShowing FULL point cloud (including ceiling and floor)...")
    print("Close the window to see the filtered version.")
    o3d.visualization.draw_geometries(
        [pcd_full],
        window_name="Full Point Cloud (with ceiling and floor)",
        width=1280,
        height=720
    )
    
    print("\nShowing FILTERED point cloud (ceiling and floor removed)...")
    o3d.visualization.draw_geometries(
        [pcd_filtered],
        window_name=f"Filtered Point Cloud (threshold: {floor_threshold}-{ceiling_threshold})",
        width=1280,
        height=720
    )
    
    print("\n✓ Visualization complete!")


def visualize_y_distribution(data_dir='semantic_3d_pointcloud'):
    """
    Show the distribution of Y-coordinates to help choose thresholds.
    """
    import matplotlib.pyplot as plt
    
    print("\n" + "="*60)
    print("Y-COORDINATE DISTRIBUTION ANALYSIS")
    print("="*60)
    
    points = np.load(f'{data_dir}/point.npy')
    y_coords = points[:, 1]
    
    # Calculate statistics
    print(f"\nY-coordinate statistics (normalized [0-255]):")
    print(f"  Min:    {y_coords.min():.4f}")
    print(f"  Max:    {y_coords.max():.4f}")
    print(f"  Mean:   {y_coords.mean():.4f}")
    print(f"  Median: {np.median(y_coords):.4f}")
    print(f"  Std:    {y_coords.std():.4f}")
    
    # Percentiles
    percentiles = [1, 5, 10, 25, 50, 75, 90, 95, 99]
    print(f"\nPercentiles:")
    for p in percentiles:
        val = np.percentile(y_coords, p)
        print(f"  {p:2d}%: {val:.4f}")
    
    # Create histogram
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.hist(y_coords, bins=100, color='steelblue', alpha=0.7, edgecolor='black')
    plt.xlabel('Y Coordinate (normalized)', fontsize=12)
    plt.ylabel('Count', fontsize=12)
    plt.title('Distribution of Y Coordinates', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    
    # Add threshold lines
    plt.axvline(0.1, color='red', linestyle='--', linewidth=2, label='Default floor (0.1)')
    plt.axvline(0.9, color='orange', linestyle='--', linewidth=2, label='Default ceiling (0.9)')
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.hist(y_coords, bins=100, cumulative=True, density=True, 
             color='green', alpha=0.7, edgecolor='black')
    plt.xlabel('Y Coordinate (normalized)', fontsize=12)
    plt.ylabel('Cumulative Probability', fontsize=12)
    plt.title('Cumulative Distribution', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.axhline(0.1, color='red', linestyle='--', linewidth=1, alpha=0.5)
    plt.axhline(0.9, color='orange', linestyle='--', linewidth=1, alpha=0.5)
    
    plt.tight_layout()
    plt.savefig('y_distribution.png', dpi=150, bbox_inches='tight')
    print(f"\n✓ Y-coordinate distribution plot saved to: y_distribution.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize 3D semantic point cloud')
    parser.add_argument('--data_dir', type=str, default='semantic_3d_pointcloud',
                        help='Directory containing point cloud data')
    parser.add_argument('--floor_threshold', type=float, default=0.1,
                        help='Y-coordinate threshold for floor removal')
    parser.add_argument('--ceiling_threshold', type=float, default=0.9,
                        help='Y-coordinate threshold for ceiling removal')
    parser.add_argument('--use_color_0255', action='store_true',
                        help='Use color_0255.npy instead of color01.npy')
    parser.add_argument('--distribution', action='store_true',
                        help='Show Y-coordinate distribution (helps choose thresholds)')
    args = parser.parse_args()
    
    if args.distribution:
        visualize_y_distribution(args.data_dir)
    else:
        load_and_visualize_3d(
            args.data_dir,
            args.floor_threshold,
            args.ceiling_threshold,
            args.use_color_0255
        )


if __name__ == '__main__':
    main()
