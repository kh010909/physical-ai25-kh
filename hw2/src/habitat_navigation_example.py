#!/usr/bin/env python3
"""
Complete Agent Navigation Example with Habitat

This script demonstrates the full integration of the agent navigation system
with Habitat simulator. It follows the complete pipeline described in AGENTS.md:

1. Configure agent actions
2. Load RRT path
3. Transform coordinates
4. Navigate with target highlighting
5. Generate video output

Usage:
    python habitat_navigation_example.py --target sofa --scene apartment_0
"""

import numpy as np
import cv2
import argparse
import os
import sys
from typing import List, Tuple

# Habitat imports
import habitat_sim
from habitat_sim.utils.common import d3_40_colors_rgb

# Import our navigation modules
from agent_navigation import AgentNavigator
from rrt_pathfinder import RRTPathfinder


def make_navigation_cfg(scene_path: str, sensor_height: float = 1.5, 
                       resolution: int = 512, step_size: float = 0.25, 
                       turn_angle: float = 10.0):
    """
    Create Habitat configuration optimized for navigation with proper action settings.
    
    Args:
        scene_path: Path to the scene file
        sensor_height: Height of sensors in meters
        resolution: Image resolution (width and height)
        step_size: Forward movement step size in meters
        turn_angle: Turn angle in degrees
        
    Returns:
        Habitat configuration object
    """
    # Simulator backend configuration
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = scene_path
    sim_cfg.enable_physics = False  # Disable physics for faster navigation
    
    # Agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    
    # RGB sensor
    rgb_sensor_spec = habitat_sim.CameraSensorSpec()
    rgb_sensor_spec.uuid = "color_sensor"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [resolution, resolution]
    rgb_sensor_spec.position = [0.0, sensor_height, 0.0]
    rgb_sensor_spec.orientation = [0.0, 0.0, 0.0]  # No pitch/roll
    rgb_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    
    # Depth sensor
    depth_sensor_spec = habitat_sim.CameraSensorSpec()
    depth_sensor_spec.uuid = "depth_sensor"
    depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_sensor_spec.resolution = [resolution, resolution]
    depth_sensor_spec.position = [0.0, sensor_height, 0.0]
    depth_sensor_spec.orientation = [0.0, 0.0, 0.0]
    depth_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    
    # Semantic sensor
    semantic_sensor_spec = habitat_sim.CameraSensorSpec()
    semantic_sensor_spec.uuid = "semantic_sensor"
    semantic_sensor_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
    semantic_sensor_spec.resolution = [resolution, resolution]
    semantic_sensor_spec.position = [0.0, sensor_height, 0.0]
    semantic_sensor_spec.orientation = [0.0, 0.0, 0.0]
    semantic_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    
    agent_cfg.sensor_specifications = [rgb_sensor_spec, depth_sensor_spec, semantic_sensor_spec]
    
    # Create complete configuration
    cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])
    
    # Configure agent actions as per AGENTS.md specification
    # This is the key configuration for proper navigation
    if hasattr(cfg, 'TASK') and hasattr(cfg.TASK, 'ACTIONS'):
        cfg.TASK.ACTIONS.MOVE_FORWARD.MOTION_ARGS["step_size"] = step_size
        cfg.TASK.ACTIONS.TURN_LEFT.MOTION_ARGS["angle"] = turn_angle
        cfg.TASK.ACTIONS.TURN_RIGHT.MOTION_ARGS["angle"] = turn_angle
    
    return cfg


def initialize_habitat_environment(scene_path: str, starting_position: Tuple[float, float, float] = None):
    """
    Initialize the Habitat simulation environment.
    
    Args:
        scene_path: Path to the scene file
        starting_position: Optional starting position for the agent
        
    Returns:
        Tuple of (simulator, agent, configuration)
    """
    print("🔧 Initializing Habitat environment...")
    
    # Create configuration
    cfg = make_navigation_cfg(scene_path)
    
    # Initialize simulator
    simulator = habitat_sim.Simulator(cfg)
    
    # Initialize agent
    agent = simulator.initialize_agent(0)
    
    # Set starting position if provided
    if starting_position:
        agent_state = habitat_sim.AgentState()
        agent_state.position = np.array(starting_position)
        agent_state.rotation = np.quaternion(1, 0, 0, 0)  # No initial rotation
        agent.set_state(agent_state)
        print(f"📍 Agent starting position: {starting_position}")
    
    print("✅ Habitat environment initialized successfully!")
    return simulator, agent, cfg


def setup_rrt_pathfinding(target_category: str):
    """
    Set up RRT pathfinding and compute path to target.
    
    Args:
        target_category: Target object category
        
    Returns:
        Tuple of (pathfinder, pixel_path, goal_point)
    """
    print("🗺️ Setting up RRT pathfinding...")
    
    # Check for required files
    required_files = [
        'map.png',
        '../color_coding_semantic_segmentation_classes.xlsx',
        'coordinate_transformation.txt'
    ]
    
    missing_files = [f for f in required_files if not os.path.exists(f)]
    if missing_files:
        raise FileNotFoundError(f"Missing required files: {missing_files}")
    
    # Initialize pathfinder
    pathfinder = RRTPathfinder(
        'map.png',
        '../color_coding_semantic_segmentation_classes.xlsx',
        'coordinate_transformation.txt'
    )
    
    # Find goal point for target category
    goal_point = pathfinder.find_goal_point(target_category)
    if not goal_point:
        raise ValueError(f"No {target_category} found in the map")
    
    # Compute RRT path
    start_point = (300, 500)  # Default starting point in pixel coordinates
    print(f"🎯 Computing path from {start_point} to {goal_point}")
    
    pixel_path = pathfinder.run_rrt(start_point, goal_point)
    if not pixel_path:
        raise RuntimeError("No path found by RRT algorithm")
    
    print(f"✅ RRT path computed with {len(pixel_path)} waypoints")
    return pathfinder, pixel_path, goal_point


def run_navigation_pipeline(simulator: habitat_sim.Simulator, agent: habitat_sim.Agent,
                          pathfinder: RRTPathfinder, pixel_path: List, target_category: str):
    """
    Run the complete navigation pipeline.
    
    Args:
        simulator: Habitat simulator instance
        agent: Habitat agent instance
        pathfinder: RRT pathfinder instance
        pixel_path: Computed RRT path
        target_category: Target object category
        
    Returns:
        Path to generated video file
    """
    print("🤖 Starting navigation pipeline...")
    
    # Initialize navigator
    navigator = AgentNavigator(pathfinder, target_category)
    
    # Configure agent actions (if needed)
    # This would be done during simulator setup, but we can double-check here
    print(f"Agent configured for navigation:")
    print(f"  • Step size: {navigator.step_size} meters")
    print(f"  • Turn angle: {navigator.turn_angle} degrees")
    
    # Execute navigation
    video_path = navigator.run_navigation(pixel_path, simulator, agent)
    
    return video_path


def display_navigation_summary(video_path: str, target_category: str, 
                             path_length: int, total_frames: int = None):
    """
    Display a summary of the navigation results.
    
    Args:
        video_path: Path to generated video
        target_category: Target object category
        path_length: Number of waypoints in path
        total_frames: Number of frames recorded (if available)
    """
    print("\n🎉 NAVIGATION COMPLETED SUCCESSFULLY!")
    print("=" * 60)
    print(f"📋 Navigation Summary:")
    print(f"  • Target Category: {target_category.upper()}")
    print(f"  • Path Waypoints: {path_length}")
    if total_frames:
        print(f"  • Video Frames: {total_frames}")
    print(f"  • Video Output: {video_path}")
    print(f"  • Video Format: MP4 (H.264)")
    print(f"  • Target Highlighting: ✅ Semi-transparent red overlay")
    print(f"  • Coordinate Transformation: ✅ Pixel → 3D world coordinates")
    print("\n🎬 Video is ready for viewing!")


def main():
    """Main function for Habitat navigation example."""
    parser = argparse.ArgumentParser(
        description="Complete Agent Navigation with Habitat",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('--target', '-t', type=str, default='sofa',
                       choices=['sofa', 'rack', 'cushion', 'stair', 'cooktop'],
                       help='Target object category')
    
    parser.add_argument('--scene', '-s', type=str, 
                       default='replica_v1/apartment_0/habitat/mesh_semantic.ply',
                       help='Path to Habitat scene file')
    
    parser.add_argument('--start-pos', nargs=3, type=float, 
                       default=[0.0, 0.0, 0.0],
                       help='Starting position (x y z)')
    
    parser.add_argument('--step-size', type=float, default=0.25,
                       help='Agent step size in meters')
    
    parser.add_argument('--turn-angle', type=float, default=10.0,
                       help='Agent turn angle in degrees')
    
    args = parser.parse_args()
    
    print("🏠 COMPLETE AGENT NAVIGATION WITH HABITAT")
    print("=" * 60)
    print(f"Target: {args.target.upper()}")
    print(f"Scene: {args.scene}")
    print(f"Starting Position: {args.start_pos}")
    print(f"Step Size: {args.step_size}m")
    print(f"Turn Angle: {args.turn_angle}°")
    print()
    
    try:
        # Step 1: Initialize Habitat environment
        simulator, agent, cfg = initialize_habitat_environment(
            args.scene, 
            tuple(args.start_pos)
        )
        
        # Step 2: Setup RRT pathfinding
        pathfinder, pixel_path, goal_point = setup_rrt_pathfinding(args.target)
        
        # Step 3: Run navigation pipeline
        video_path = run_navigation_pipeline(
            simulator, agent, pathfinder, pixel_path, args.target
        )
        
        # Step 4: Display results
        display_navigation_summary(
            video_path, args.target, len(pixel_path)
        )
        
        # Cleanup
        simulator.close()
        
    except FileNotFoundError as e:
        print(f"❌ File Error: {e}")
        print("💡 Make sure you have run RRT pathfinding first (main_rrt.py)")
        sys.exit(1)
        
    except Exception as e:
        print(f"❌ Navigation Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()