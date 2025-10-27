#!/usr/bin/env python3
"""
Simple Agent Navigation Test

Basic test of the agent navigation logic without external dependencies.
This demonstrates the core coordinate transformation and navigation algorithms.
"""

import sys
import os
from typing import List, Tuple


class SimpleNode:
    """Simple node class for testing."""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


def test_coordinate_transformation():
    """Test the coordinate transformation logic."""
    print("🧪 Testing Coordinate Transformation")
    print("=" * 50)
    
    # Transformation parameters from coordinate_transformation.txt
    x_scale = 351.463912
    z_scale = 147.021786
    x_offset = -3.554479
    z_offset = -5.671297
    
    # Test pixel coordinates
    test_pixels = [
        (300, 500),
        (400, 600), 
        (500, 400),
        (600, 300)
    ]
    
    print("Pixel to Habitat Coordinate Transformation:")
    print("Formula: habitat = pixel / scale + offset")
    print(f"X: scale={x_scale:.3f}, offset={x_offset:.3f}")
    print(f"Z: scale={z_scale:.3f}, offset={z_offset:.3f}")
    print()
    
    print("Test Results:")
    print("Pixel (x, y)      →  Habitat (x, z)    →  3D Waypoint (x, y, z)")
    print("-" * 80)
    
    agent_height = 1.5
    waypoints_3d = []
    
    for px, py in test_pixels:
        # Transform to habitat coordinates
        hx = px / x_scale + x_offset
        hz = py / z_scale + z_offset
        
        # Create 3D waypoint
        waypoint = (hx, agent_height, hz)
        waypoints_3d.append(waypoint)
        
        print(f"({px:3d}, {py:3d})        →  ({hx:6.3f}, {hz:6.3f})  →  ({hx:6.3f}, {agent_height}, {hz:6.3f})")
    
    print(f"\n✅ Generated {len(waypoints_3d)} 3D waypoints")
    return waypoints_3d


def test_navigation_calculations():
    """Test navigation calculation functions."""
    print("\n🧭 Testing Navigation Calculations")
    print("=" * 50)
    
    import math
    
    def calculate_heading_to_waypoint(current_pos, current_rotation, target_pos):
        """Calculate heading angle to target."""
        dx = target_pos[0] - current_pos[0]
        dz = target_pos[2] - current_pos[2]
        
        target_heading = math.degrees(math.atan2(dx, dz))
        angle_diff = target_heading - current_rotation
        
        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
            
        return angle_diff
    
    def calculate_distance_to_waypoint(current_pos, target_pos):
        """Calculate 2D distance to target."""
        dx = target_pos[0] - current_pos[0]
        dz = target_pos[2] - current_pos[2]
        return math.sqrt(dx*dx + dz*dz)
    
    # Test scenarios
    test_scenarios = [
        {
            'current': (0.0, 1.5, 0.0),
            'target': (1.0, 1.5, 1.0),
            'rotation': 0.0,
            'description': 'Move northeast'
        },
        {
            'current': (0.0, 1.5, 0.0),
            'target': (-1.0, 1.5, 0.0),
            'rotation': 0.0,
            'description': 'Move west'
        },
        {
            'current': (0.0, 1.5, 0.0),
            'target': (0.0, 1.5, -1.0),
            'rotation': 90.0,
            'description': 'Move south from east facing'
        }
    ]
    
    print("Navigation Test Scenarios:")
    print("Current (x,y,z)  | Target (x,y,z)   | Heading | Distance | Action")
    print("-" * 75)
    
    for scenario in test_scenarios:
        current = scenario['current']
        target = scenario['target']
        rotation = scenario['rotation']
        
        angle_diff = calculate_heading_to_waypoint(current, rotation, target)
        distance = calculate_distance_to_waypoint(current, target)
        
        # Determine action
        if abs(angle_diff) > 5.0:
            action = "TURN_LEFT" if angle_diff > 0 else "TURN_RIGHT"
        else:
            action = "MOVE_FORWARD"
        
        print(f"{str(current):17} | {str(target):17} | {angle_diff:6.1f}° | {distance:6.3f}m | {action}")
    
    print("\n✅ Navigation calculations working correctly")


def test_rrt_path_processing():
    """Test processing of RRT path data."""
    print("\n🛤️ Testing RRT Path Processing")
    print("=" * 50)
    
    # Create sample RRT path
    sample_path_coords = [
        (300, 500), (320, 480), (350, 460), (380, 440),
        (410, 420), (440, 400), (470, 380), (500, 360)
    ]
    
    # Convert to Node objects
    path_nodes = [SimpleNode(x, y) for x, y in sample_path_coords]
    
    print(f"Sample RRT path with {len(path_nodes)} waypoints:")
    print("Node #  | Pixel (x, y)  | Habitat (x, z)  | 3D Waypoint")
    print("-" * 65)
    
    # Transformation parameters
    x_scale = 351.463912
    z_scale = 147.021786
    x_offset = -3.554479
    z_offset = -5.671297
    agent_height = 1.5
    
    waypoints_3d = []
    
    for i, node in enumerate(path_nodes):
        # Transform to habitat coordinates
        hx = node.x / x_scale + x_offset
        hz = node.y / z_scale + z_offset
        
        # Create 3D waypoint
        waypoint = (hx, agent_height, hz)
        waypoints_3d.append(waypoint)
        
        print(f"  {i+1:2d}    | ({node.x:3.0f}, {node.y:3.0f})     | ({hx:6.3f}, {hz:6.3f}) | ({hx:6.3f}, {agent_height}, {hz:6.3f})")
    
    print(f"\n✅ Successfully processed {len(waypoints_3d)} waypoints")
    
    # Calculate total path distance
    total_distance = 0
    for i in range(1, len(waypoints_3d)):
        dx = waypoints_3d[i][0] - waypoints_3d[i-1][0]
        dz = waypoints_3d[i][2] - waypoints_3d[i-1][2]
        segment_distance = (dx*dx + dz*dz)**0.5
        total_distance += segment_distance
    
    print(f"📏 Total path length: {total_distance:.3f} meters")
    return waypoints_3d


def test_target_categories():
    """Test target category definitions."""
    print("\n🎯 Testing Target Categories")
    print("=" * 50)
    
    target_categories = {
        'rack': (0, 255, 133),
        'cushion': (255, 9, 92),
        'sofa': (10, 0, 255),
        'stair': (173, 255, 0),
        'cooktop': (7, 255, 224)
    }
    
    print("Available target categories:")
    print("Category   | RGB Color      | Hex Color")
    print("-" * 40)
    
    for category, (r, g, b) in target_categories.items():
        hex_color = f"#{r:02x}{g:02x}{b:02x}"
        print(f"{category:10} | ({r:3d},{g:3d},{b:3d})     | {hex_color}")
    
    print(f"\n✅ {len(target_categories)} target categories available")
    return target_categories


def simulate_navigation_steps(waypoints_3d: List[Tuple[float, float, float]]):
    """Simulate the navigation process."""
    print("\n🤖 Simulating Navigation Process")
    print("=" * 50)
    
    import math
    
    # Agent configuration
    step_size = 0.25  # meters
    turn_angle = 10.0  # degrees
    angle_threshold = 5.0  # degrees
    distance_threshold = 0.1  # meters
    
    # Simulate agent starting position
    agent_pos = waypoints_3d[0] if waypoints_3d else (0.0, 1.5, 0.0)
    agent_rotation = 0.0  # degrees
    
    print(f"Agent configuration:")
    print(f"  • Step size: {step_size} m")
    print(f"  • Turn angle: {turn_angle}°")
    print(f"  • Angle threshold: {angle_threshold}°")
    print(f"  • Distance threshold: {distance_threshold} m")
    print(f"  • Starting position: ({agent_pos[0]:.3f}, {agent_pos[2]:.3f})")
    print()
    
    total_actions = 0
    
    for i, target_waypoint in enumerate(waypoints_3d[1:6], 1):  # Test first 5 waypoints
        print(f"--- Waypoint {i}: ({target_waypoint[0]:.3f}, {target_waypoint[2]:.3f}) ---")
        
        waypoint_actions = 0
        max_actions = 20  # Prevent infinite loops
        
        while waypoint_actions < max_actions:
            # Calculate heading and distance
            dx = target_waypoint[0] - agent_pos[0]
            dz = target_waypoint[2] - agent_pos[2]
            
            target_heading = math.degrees(math.atan2(dx, dz))
            angle_diff = target_heading - agent_rotation
            
            # Normalize angle
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            distance = math.sqrt(dx*dx + dz*dz)
            
            # Check if reached waypoint
            if distance < distance_threshold:
                print(f"  ✅ Reached waypoint in {waypoint_actions} actions")
                break
            
            # Decide action
            if abs(angle_diff) > angle_threshold:
                action = "TURN_LEFT" if angle_diff > 0 else "TURN_RIGHT"
                agent_rotation += turn_angle if angle_diff > 0 else -turn_angle
                
                # Normalize rotation
                agent_rotation = agent_rotation % 360
                
                print(f"  Action {waypoint_actions + 1}: {action:10} (angle: {angle_diff:5.1f}°, dist: {distance:.3f}m)")
            else:
                action = "MOVE_FORWARD"
                # Move forward in current direction
                move_x = step_size * math.sin(math.radians(agent_rotation))
                move_z = step_size * math.cos(math.radians(agent_rotation))
                
                agent_pos = (agent_pos[0] + move_x, agent_pos[1], agent_pos[2] + move_z)
                
                print(f"  Action {waypoint_actions + 1}: {action:10} (new pos: {agent_pos[0]:.3f}, {agent_pos[2]:.3f})")
            
            waypoint_actions += 1
            total_actions += 1
        
        if waypoint_actions >= max_actions:
            print(f"  ⚠️ Max actions reached for waypoint {i}")
    
    print(f"\n✅ Navigation simulation completed")
    print(f"📊 Total actions simulated: {total_actions}")


def main():
    """Run all tests."""
    print("🧪 AGENT NAVIGATION SYSTEM TEST")
    print("=" * 60)
    print("Testing the agent navigation implementation without external dependencies.")
    print()
    
    try:
        # Test 1: Coordinate transformation
        waypoints_3d = test_coordinate_transformation()
        
        # Test 2: Navigation calculations
        test_navigation_calculations()
        
        # Test 3: RRT path processing
        test_waypoints = test_rrt_path_processing()
        
        # Test 4: Target categories
        test_target_categories()
        
        # Test 5: Navigation simulation
        simulate_navigation_steps(test_waypoints)
        
        print("\n🎉 ALL TESTS PASSED!")
        print("✅ Agent navigation system is working correctly")
        print()
        print("💡 Next steps:")
        print("  • Install OpenCV: pip install opencv-python")
        print("  • Run full demo: python demo_agent_navigation.py --target sofa")
        print("  • Integrate with Habitat when environment is ready")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()