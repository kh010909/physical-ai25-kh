"""
Quick test script to verify data loading and basic statistics.
Run this first to check if everything is set up correctly.
"""

import numpy as np
import os


def test_data_loading():
    """Test if all required data files exist and can be loaded."""
    
    print("="*60)
    print("DATA VERIFICATION TEST")
    print("="*60)
    
    data_dir = 'semantic_3d_pointcloud'
    
    # Check if directory exists
    if not os.path.exists(data_dir):
        print(f"❌ ERROR: Directory '{data_dir}' not found!")
        print(f"   Please create the directory and add the data files.")
        return False
    
    print(f"✓ Directory '{data_dir}' exists")
    
    # Required files
    required_files = {
        'point.npy': 'Point coordinates',
        'color01.npy': 'RGB colors [0, 1]',
        'color0255.npy': 'RGB colors [0, 255]'
    }
    
    all_exist = True
    loaded_data = {}
    
    for filename, description in required_files.items():
        filepath = os.path.join(data_dir, filename)
        if os.path.exists(filepath):
            print(f"✓ Found: {filename} ({description})")
            try:
                data = np.load(filepath)
                loaded_data[filename] = data
                print(f"  Shape: {data.shape}, dtype: {data.dtype}")
            except Exception as e:
                print(f"  ❌ Error loading: {e}")
                all_exist = False
        else:
            print(f"❌ Missing: {filename} ({description})")
            all_exist = False
    
    if not all_exist:
        print("\n❌ Some files are missing or cannot be loaded!")
        print("   Please download the data from the link provided in the spec.")
        return False
    
    print("\n" + "="*60)
    print("DATA STATISTICS")
    print("="*60)
    
    # Analyze point cloud
    points = loaded_data['point.npy']
    colors_01 = loaded_data['color01.npy']
    colors_0255 = loaded_data['color0255.npy']
    
    print(f"\nPoint Cloud:")
    print(f"  Total points: {len(points):,}")
    print(f"  Shape: {points.shape}")
    print(f"\nCoordinate ranges (normalized):")
    print(f"  X: [{points[:, 0].min():.4f}, {points[:, 0].max():.4f}]")
    print(f"  Y: [{points[:, 1].min():.4f}, {points[:, 1].max():.4f}]")
    print(f"  Z: [{points[:, 2].min():.4f}, {points[:, 2].max():.4f}]")
    
    # Convert to Habitat coordinates
    habitat_points = points * 10000.0 / 255.0
    print(f"\nHabitat coordinates (meters):")
    print(f"  X: [{habitat_points[:, 0].min():.2f}, {habitat_points[:, 0].max():.2f}]")
    print(f"  Y: [{habitat_points[:, 1].min():.2f}, {habitat_points[:, 1].max():.2f}]")
    print(f"  Z: [{habitat_points[:, 2].min():.2f}, {habitat_points[:, 2].max():.2f}]")
    
    print(f"\nColor ranges:")
    print(f"  color01.npy:   [{colors_01.min():.4f}, {colors_01.max():.4f}]")
    print(f"  color0255.npy: [{colors_0255.min():.0f}, {colors_0255.max():.0f}]")
    
    # Estimate floor/ceiling
    y_coords = points[:, 1]
    percentiles = np.percentile(y_coords, [5, 10, 90, 95])
    print(f"\nY-coordinate percentiles (for threshold estimation):")
    print(f"  5th percentile:  {percentiles[0]:.4f}")
    print(f"  10th percentile: {percentiles[1]:.4f}")
    print(f"  90th percentile: {percentiles[2]:.4f}")
    print(f"  95th percentile: {percentiles[3]:.4f}")
    print(f"\nSuggested thresholds:")
    print(f"  Floor:   --floor_threshold {percentiles[1]:.2f}")
    print(f"  Ceiling: --ceiling_threshold {percentiles[2]:.2f}")
    
    # Check unique colors (semantic categories)
    unique_colors_0255 = np.unique(colors_0255.reshape(-1, 3), axis=0)
    print(f"\nSemantic categories:")
    print(f"  Unique colors found: {len(unique_colors_0255)}")
    
    print("\n" + "="*60)
    print("✓ ALL TESTS PASSED!")
    print("="*60)
    print("\nYou can now run: python semantic_map.py")
    print("="*60)
    
    return True


if __name__ == '__main__':
    success = test_data_loading()
    
    if not success:
        print("\n⚠️  Please fix the issues above before proceeding.")
        exit(1)
