"""
RRT Pathfinding Demo Script
This script demonstrates how to use the RRT algorithm to find paths from a starting point to target categories.
"""

import numpy as np
import matplotlib.pyplot as plt
from rrt_pathfinding import InteractiveMapUI

def demo_rrt_path():
    """Demonstrate RRT pathfinding"""
    print("RRT Pathfinding Demo")
    print("=" * 60)
    
    # Initialize UI
    ui = InteractiveMapUI()
    
    # Demo: Find path to sofa
    target_cat = 'sofa'
    ui.target_category = target_cat
    
    print(f"\nSearching for: {target_cat}")
    
    # Set start and goal points
    # Using example coordinates
    ui.start_point = (200, 800)  # Example start point (pixel coords)
    
    # Find target point
    ui.goal_point = ui._find_target_point()
    
    if ui.goal_point is None:
        print(f"Could not find target: {target_cat}")
        return
    
    print(f"Start point (pixel): {ui.start_point}")
    print(f"  Habitat coordinates: {ui._pixel_to_habitat(ui.start_point[0], ui.start_point[1])}")
    
    print(f"Goal point (pixel): {ui.goal_point}")
    print(f"  Habitat coordinates: {ui._pixel_to_habitat(ui.goal_point[0], ui.goal_point[1])}")
    
    # Run RRT
    print("\nRunning RRT algorithm...")
    ui.run_rrt()
    
    if ui.path:
        print(f"\nPath found! Waypoints:")
        for i, (px, py) in enumerate(ui.path):
            hx, hz = ui._pixel_to_habitat(px, py)
            print(f"  {i:2d}: Pixel ({px:4.0f}, {py:4.0f}) -> Habitat ({hx:7.2f}, {hz:7.2f})")
    else:
        print("Failed to find path")


def demo_interactive():
    """Run interactive demo"""
    print("Interactive RRT Pathfinding Demo")
    print("=" * 60)
    print("\nThis will launch an interactive interface where you can:")
    print("  1. Select a target category (rack, cushion, sofa, stair, cooktop)")
    print("  2. Click on the map to set the starting point")
    print("  3. The algorithm will find a path to the target")
    print("\nLaunching interactive interface...")
    
    ui = InteractiveMapUI()
    ui.interactive_search('sofa')


if __name__ == '__main__':
    # Run demo
    demo_rrt_path()
    
    # Uncomment to run interactive demo
    # demo_interactive()
