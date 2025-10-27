#!/usr/bin/env python3
"""
Agent Navigation Demo Script

This script demonstrates the agent navigation functionality described in AGENTS.md
without requiring a working Habitat environment. It simulates the complete pipeline:

1. Load RRT path data
2. Transform coordinates 
3. Simulate agent navigation
4. Highlight target objects
5. Generate video output

Usage:
    python demo_agent_navigation.py --target sofa
    python demo_agent_navigation.py --target rack --frames 100
"""

import numpy as np
import cv2
import argparse
import os
import sys
from typing import List, Tuple
import math
import matplotlib.pyplot as plt

# Import our navigation modules
try:
    from agent_navigation import AgentNavigator
    from rrt_pathfinder import RRTPathfinder
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("💡 Make sure you're running from the src/ directory")
    sys.exit(1)


def create_sample_rrt_path(target_category: str) -> List:
    """
    Create a sample RRT path for demonstration when actual pathfinding isn't available.
    
    Args:
        target_category: Target object category
        
    Returns:
        List of Node objects representing the path
    """
    class SampleNode:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    
    # Define sample paths for different targets
    sample_paths = {
        'sofa': [
            (300, 500), (320, 480), (350, 460), (380, 440),
            (410, 420), (440, 400), (470, 380), (500, 360),
            (530, 340), (560, 320), (590, 300)
        ],
        'rack': [
            (600, 400), (580, 380), (560, 360), (540, 340),
            (520, 320), (500, 300), (480, 280), (460, 260),
            (440, 240), (420, 220)
        ],
        'cushion': [
            (700, 600), (680, 580), (660, 560), (640, 540),
            (620, 520), (600, 500), (580, 480), (560, 460),
            (540, 440), (520, 420), (500, 400)
        ],
        'stair': [
            (400, 800), (420, 780), (440, 760), (460, 740),
            (480, 720), (500, 700), (520, 680), (540, 660),
            (560, 640), (580, 620)
        ],
        'cooktop': [
            (200, 300), (220, 320), (240, 340), (260, 360),
            (280, 380), (300, 400), (320, 420), (340, 440),
            (360, 460), (380, 480)
        ]
    }
    
    # Get path for target category, default to sofa
    path_coords = sample_paths.get(target_category, sample_paths['sofa'])
    
    # Convert to Node objects
    path_nodes = [SampleNode(x, y) for x, y in path_coords]
    
    return path_nodes


def create_mock_pathfinder(target_category: str):
    """
    Create a mock pathfinder object for demonstration.
    
    Args:
        target_category: Target object category
        
    Returns:
        Mock pathfinder object
    """
    class MockPathfinder:
        def __init__(self):
            self.target_categories = {
                'rack': (0, 255, 133),
                'cushion': (255, 9, 92),
                'sofa': (10, 0, 255),
                'stair': (173, 255, 0),
                'cooktop': (7, 255, 224)
            }
            
            # Mock transformation parameters
            self.transform_params = {
                'x_scale': 351.463912,
                'z_scale': 147.021786,
                'x_offset': -3.554479,
                'z_offset': -5.671297
            }
    
    return MockPathfinder()


def demonstrate_coordinate_transformation():
    """Demonstrate the coordinate transformation process."""
    print("\n📐 COORDINATE TRANSFORMATION DEMO")
    print("=" * 50)
    
    # Sample pixel coordinates
    pixel_coords = [(300, 500), (400, 600), (500, 400)]
    
    # Transformation parameters (from coordinate_transformation.txt)
    x_scale = 351.463912
    z_scale = 147.021786
    x_offset = -3.554479
    z_offset = -5.671297
    
    print("Pixel → Habitat Coordinate Transformation:")
    print("Formula: habitat_coord = pixel / scale + offset")
    print(f"X scale: {x_scale:.3f}, X offset: {x_offset:.3f}")
    print(f"Z scale: {z_scale:.3f}, Z offset: {z_offset:.3f}")
    print()
    
    print("Sample transformations:")
    print("Pixel (x, y)      →  Habitat (x, z)")
    print("-" * 40)
    
    for px, py in pixel_coords:
        hx = px / x_scale + x_offset
        hz = py / z_scale + z_offset
        print(f"({px:3d}, {py:3d})        →  ({hx:6.3f}, {hz:6.3f})")
    
    print()


def visualize_path(path_nodes: List, target_category: str, output_file: str = None):
    """
    Visualize the RRT path on a simple map.
    
    Args:
        path_nodes: List of path nodes
        target_category: Target object category
        output_file: Optional output file path
    """
    print(f"\n🗺️ Visualizing RRT path to {target_category.upper()}")
    
    # Create a simple map visualization
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    
    # Draw a simple room layout
    room_x = [0, 1000, 1000, 0, 0]
    room_y = [0, 0, 800, 800, 0]
    ax.plot(room_x, room_y, 'k-', linewidth=2, label='Room boundaries')
    
    # Draw some obstacles
    obstacles = [
        ([200, 300, 300, 200, 200], [100, 100, 200, 200, 100]),  # Table
        ([600, 800, 800, 600, 600], [500, 500, 700, 700, 500]),  # Sofa area
        ([100, 150, 150, 100, 100], [600, 600, 750, 750, 600]),  # Chair
    ]
    
    for obs_x, obs_y in obstacles:
        ax.plot(obs_x, obs_y, 'gray', linewidth=1)
        ax.fill(obs_x, obs_y, color='lightgray', alpha=0.5)
    
    # Plot the path
    if path_nodes:
        path_x = [node.x for node in path_nodes]
        path_y = [node.y for node in path_nodes]
        
        # Plot path line
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='RRT Path')
        
        # Plot waypoints
        ax.scatter(path_x, path_y, c='blue', s=30, alpha=0.7, label='Waypoints')
        
        # Highlight start and end
        ax.scatter(path_x[0], path_y[0], c='green', s=100, marker='o', label='Start')
        ax.scatter(path_x[-1], path_y[-1], c='red', s=100, marker='*', label='Goal')
        
        print(f"Path details:")
        print(f"  • Start: ({path_x[0]:.0f}, {path_y[0]:.0f})")
        print(f"  • Goal: ({path_x[-1]:.0f}, {path_y[-1]:.0f})")
        print(f"  • Waypoints: {len(path_nodes)}")
    
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 800)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title(f'RRT Path Planning to {target_category.upper()}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"📁 Path visualization saved: {output_file}")
    else:
        plt.show()
    
    plt.close()


def run_navigation_demo(target_category: str, num_frames: int = 50):
    """
    Run the complete navigation demonstration.
    
    Args:
        target_category: Target object category
        num_frames: Number of frames to generate
    """
    print(f"\n🤖 AGENT NAVIGATION DEMO - {target_category.upper()}")
    print("=" * 60)
    
    try:
        # Step 1: Create sample data
        print("📝 Creating sample RRT path...")
        path_nodes = create_sample_rrt_path(target_category)
        mock_pathfinder = create_mock_pathfinder(target_category)
        
        # Step 2: Visualize path
        visualize_path(path_nodes, target_category, f"demo_path_{target_category}.png")
        
        # Step 3: Initialize navigator
        print("🔧 Initializing agent navigator...")
        navigator = AgentNavigator(mock_pathfinder, target_category)
        
        # Step 4: Transform coordinates
        print("📍 Transforming pixel coordinates to 3D world coordinates...")
        waypoints_3d = navigator.transform_rrt_path_to_3d(path_nodes)
        
        print(f"Generated {len(waypoints_3d)} 3D waypoints:")
        for i, wp in enumerate(waypoints_3d[:5]):  # Show first 5
            print(f"  Waypoint {i+1}: ({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})")
        if len(waypoints_3d) > 5:
            print(f"  ... and {len(waypoints_3d) - 5} more waypoints")
        
        # Step 5: Simulate navigation and generate frames
        print(f"🎬 Generating {num_frames} navigation frames...")
        
        frames = []
        frames_per_waypoint = max(1, num_frames // len(waypoints_3d))
        
        for i, waypoint in enumerate(waypoints_3d):
            waypoint_frames = navigator._simulate_waypoint_navigation(waypoint, i)
            frames.extend(waypoint_frames[:frames_per_waypoint])  # Limit frames per waypoint
            
            if len(frames) >= num_frames:
                frames = frames[:num_frames]  # Trim to exact number
                break
        
        print(f"✅ Generated {len(frames)} frames")
        
        # Step 6: Generate video
        print("📹 Creating navigation video...")
        video_path = navigator.generate_video(frames)
        
        # Step 7: Generate summary
        print(f"\n📊 DEMO SUMMARY")
        print(f"  • Target: {target_category.upper()}")
        print(f"  • Path waypoints: {len(waypoints_3d)}")
        print(f"  • Video frames: {len(frames)}")
        print(f"  • Video output: {video_path}")
        print(f"  • Path visualization: demo_path_{target_category}.png")
        
        return True
        
    except Exception as e:
        print(f"❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function for the navigation demo."""
    parser = argparse.ArgumentParser(description="Agent Navigation Demo")
    parser.add_argument('--target', '-t', type=str, default='sofa',
                       choices=['sofa', 'rack', 'cushion', 'stair', 'cooktop'],
                       help='Target object category')
    parser.add_argument('--frames', '-f', type=int, default=50,
                       help='Number of frames to generate for video')
    parser.add_argument('--coords-only', action='store_true',
                       help='Only demonstrate coordinate transformation')
    
    args = parser.parse_args()
    
    print("🤖 AGENT NAVIGATION DEMONSTRATION")
    print("=" * 60)
    print("This demo simulates the agent navigation system described in AGENTS.md")
    print("without requiring a working Habitat environment.")
    print()
    
    # Always show coordinate transformation
    demonstrate_coordinate_transformation()
    
    if args.coords_only:
        print("✅ Coordinate transformation demo completed!")
        return
    
    # Run full navigation demo
    success = run_navigation_demo(args.target, args.frames)
    
    if success:
        print("\n🎉 Navigation demo completed successfully!")
        print("\n💡 To run with different targets:")
        print("   python demo_agent_navigation.py --target rack")
        print("   python demo_agent_navigation.py --target cushion --frames 100")
    else:
        print("\n❌ Navigation demo failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()