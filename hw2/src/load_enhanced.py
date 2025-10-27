#!/usr/bin/env python3
"""
Enhanced Load Script with Agent Navigation

This is an enhanced version of load.py that incorporates the agent navigation
functionality described in AGENTS.md. It can run in two modes:
1. Manual control mode (original functionality)
2. Automatic navigation mode (following RRT path)

Note: Habitat imports are commented out due to environment issues.
"""

import numpy as np
from PIL import Image
import cv2
import os
import sys
import argparse
import shutil
import math
from typing import List, Tuple, Optional

# Import our agent navigation module
from agent_navigation import AgentNavigator
from rrt_pathfinder import RRTPathfinder

# Habitat imports
import habitat_sim
from habitat_sim.utils.common import d3_40_colors_rgb


# Scene configuration
test_scene = "replica_v1/apartment_0/habitat/mesh_semantic.ply"

sim_settings = {
    "scene": test_scene,
    "default_agent": 0,
    "sensor_height": 1.5,
    "width": 512,
    "height": 512,
    "sensor_pitch": 0,
}

# Global variables
cam_extr = []
count = 0
navigator = None


def transform_rgb_bgr(image):
    """Transform RGB to BGR for OpenCV display."""
    return image[:, :, [2, 1, 0]]


def transform_depth(image):
    """Transform depth image for visualization."""
    depth_img = (image / 10 * 255).astype(np.uint8)
    return depth_img


def transform_semantic(semantic_obs):
    """Transform semantic segmentation for visualization."""
    semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
    semantic_img.putpalette(d3_40_colors_rgb.flatten())
    semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
    semantic_img = semantic_img.convert("RGB")
    semantic_img = cv2.cvtColor(np.asarray(semantic_img), cv2.COLOR_RGB2BGR)
    return semantic_img


def make_simple_cfg(settings):
    """
    Create a simple configuration for the simulator.
    Enhanced with agent action configuration for navigation.
    """
    # Simulator backend
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = settings["scene"]
    
    # Agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()

    # RGB visual sensor
    rgb_sensor_spec = habitat_sim.CameraSensorSpec()
    rgb_sensor_spec.uuid = "color_sensor"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
    rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    rgb_sensor_spec.orientation = [settings["sensor_pitch"], 0.0, 0.0]
    rgb_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    # Depth sensor
    depth_sensor_spec = habitat_sim.CameraSensorSpec()
    depth_sensor_spec.uuid = "depth_sensor"
    depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_sensor_spec.resolution = [settings["height"], settings["width"]]
    depth_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    depth_sensor_spec.orientation = [settings["sensor_pitch"], 0.0, 0.0]
    depth_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    # Semantic sensor
    semantic_sensor_spec = habitat_sim.CameraSensorSpec()
    semantic_sensor_spec.uuid = "semantic_sensor"
    semantic_sensor_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
    semantic_sensor_spec.resolution = [settings["height"], settings["width"]]
    semantic_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    semantic_sensor_spec.orientation = [settings["sensor_pitch"], 0.0, 0.0]
    semantic_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    agent_cfg.sensor_specifications = [rgb_sensor_spec, depth_sensor_spec, semantic_sensor_spec]

    # Create configuration
    cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])
    
    # Configure agent actions as per AGENTS.md specification
    cfg.TASK.ACTIONS.MOVE_FORWARD.MOTION_ARGS["step_size"] = 0.25  # meters
    cfg.TASK.ACTIONS.TURN_LEFT.MOTION_ARGS["angle"] = 10.0      # degrees
    cfg.TASK.ACTIONS.TURN_RIGHT.MOTION_ARGS["angle"] = 10.0     # degrees
    
    return cfg


def navigate_and_see(action="", data_root='data_collection/second_floor/', auto_mode=False, target_category=None):
    """
    Enhanced navigation function that can work in manual or automatic mode.
    
    Args:
        action: Action to perform (manual mode)
        data_root: Directory to save data
        auto_mode: Whether running in automatic navigation mode
        target_category: Target object category for highlighting
    """
    global count, navigator, sim, agent
    
    # Get observations from Habitat simulator
    observations = sim.step(action)
    
    # If in auto mode and navigator exists, highlight target object
    if auto_mode and navigator and target_category:
        semantic_scene = sim.semantic_scene
        highlighted_rgb = navigator.highlight_target_object(
            observations["color_sensor"],
            observations["semantic_sensor"],
            semantic_scene
        )
        display_image = transform_rgb_bgr(highlighted_rgb)
    else:
        display_image = transform_rgb_bgr(observations["color_sensor"])
    
    # Display images
    cv2.imshow("RGB", display_image)
    cv2.imshow("depth", transform_depth(observations["depth_sensor"]))
    cv2.imshow("semantic", transform_semantic(observations["semantic_sensor"]))
    
    # Get actual agent state from Habitat
    agent_state = agent.get_state()
    sensor_state = agent_state.sensor_states['color_sensor']
    
    print("Frame:", count)
    print("camera pose: x y z rw rx ry rz")
    print(sensor_state.position[0], sensor_state.position[1], sensor_state.position[2], 
          sensor_state.rotation.w, sensor_state.rotation.x, sensor_state.rotation.y, sensor_state.rotation.z)
    
    count += 1
    
    # Save images
    cv2.imwrite(data_root + f"rgb/{count}.png", display_image)
    cv2.imwrite(data_root + f"depth/{count}.png", transform_depth(observations["depth_sensor"]))
    cv2.imwrite(data_root + f"semantic/{count}.png", transform_semantic(observations["semantic_sensor"]))
    
    # Save camera extrinsics
    cam_extr.append([sensor_state.position[0], sensor_state.position[1], sensor_state.position[2], 
                    sensor_state.rotation.w, sensor_state.rotation.x, sensor_state.rotation.y, sensor_state.rotation.z])





def run_automatic_navigation(target_category: str, data_root: str):
    """
    Run automatic navigation mode following RRT path with Habitat.
    
    Args:
        target_category: Target object category
        data_root: Directory to save data
    """
    global navigator, count, sim, agent
    
    print(f"\n🤖 Starting automatic navigation to: {target_category.upper()}")
    print("=" * 60)
    
    try:
        # Check for required files
        required_files = [
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
            'coordinate_transformation.txt'
        ]
        
        missing_files = [f for f in required_files if not os.path.exists(f)]
        if missing_files:
            print(f"❌ Missing required files: {missing_files}")
            print("💡 Please run RRT pathfinding first (main_rrt.py)")
            return False
        
        # Initialize RRT pathfinder
        print("🔧 Initializing RRT pathfinder...")
        pathfinder = RRTPathfinder(
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
            'coordinate_transformation.txt'
        )
        
        # Initialize navigator
        navigator = AgentNavigator(pathfinder, target_category)
        
        # Get RRT path
        print("🗺️ Computing RRT path...")
        start_point = (300, 500)  # Default start point
        goal_point = pathfinder.find_goal_point(target_category)
        
        if not goal_point:
            print(f"❌ No {target_category} found in the map")
            return False
        
        pixel_path = pathfinder.run_rrt(start_point, goal_point)
        
        if not pixel_path:
            print("❌ No path found by RRT algorithm")
            return False
        
        print(f"✅ RRT path found with {len(pixel_path)} waypoints")
        
        # Run navigation using the AgentNavigator with Habitat
        print("🚀 Beginning automatic navigation with Habitat...")
        video_path = navigator.run_navigation(pixel_path, sim, agent)
        
        print(f"\n✅ Automatic navigation completed!")
        print(f"🎬 Video saved as: {video_path}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error during automatic navigation: {e}")
        import traceback
        traceback.print_exc()
        return False


def generate_video_from_saved_frames(data_root: str, target_category: str):
    """
    Generate video from saved RGB frames.
    
    Args:
        data_root: Directory containing saved frames
        target_category: Target object category for video naming
    """
    try:
        rgb_dir = os.path.join(data_root, "rgb")
        if not os.path.exists(rgb_dir):
            print("❌ No RGB frames found")
            return
        
        # Get list of frame files
        frame_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith('.png')])
        
        if not frame_files:
            print("❌ No frame files found")
            return
        
        # Read first frame to get dimensions
        first_frame_path = os.path.join(rgb_dir, frame_files[0])
        first_frame = cv2.imread(first_frame_path)
        
        if first_frame is None:
            print("❌ Could not read first frame")
            return
        
        height, width, layers = first_frame.shape
        
        # Set up video writer
        video_name = f"{target_category}_navigation.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_name, fourcc, 10, (width, height))
        
        print(f"📹 Writing {len(frame_files)} frames to {video_name}...")
        
        # Write frames to video
        for i, frame_file in enumerate(frame_files):
            frame_path = os.path.join(rgb_dir, frame_file)
            frame = cv2.imread(frame_path)
            
            if frame is not None:
                video_writer.write(frame)
                
                if (i + 1) % 10 == 0:
                    print(f"   Written {i + 1}/{len(frame_files)} frames...")
        
        video_writer.release()
        print(f"✅ Video saved successfully: {video_name}")
        
    except Exception as e:
        print(f"❌ Error generating video: {e}")


def main():
    """
    Main function with enhanced argument parsing for navigation modes.
    """
    global count, sim, agent
    
    parser = argparse.ArgumentParser(description="Enhanced Habitat Agent with Navigation")
    parser.add_argument('-f', '--floor', type=int, default=1, help='Floor number (1 or 2)')
    parser.add_argument('-m', '--mode', type=str, default='manual', 
                       choices=['manual', 'auto'], help='Control mode (manual or auto)')
    parser.add_argument('-t', '--target', type=str, default='sofa',
                       choices=['sofa', 'rack', 'cushion', 'stair', 'cooktop'],
                       help='Target object category for automatic mode')
    
    args = parser.parse_args()
    
    print("🏠 Enhanced Habitat Agent with Navigation")
    print("=" * 50)
    print(f"Mode: {args.mode.upper()}")
    print(f"Floor: {args.floor}")
    if args.mode == 'auto':
        print(f"Target: {args.target.upper()}")
    
    # Set up data collection directory
    if args.floor == 1:
        data_root = "data_collection/first_floor/"
    elif args.floor == 2:
        data_root = "data_collection/second_floor/"
    else:
        data_root = "data_collection/default/"
    
    # Clean and create directories
    if os.path.isdir(data_root):
        shutil.rmtree(data_root)
    
    for sub_dir in ['rgb/', 'depth/', 'semantic/']:
        os.makedirs(data_root + sub_dir)
    
    print(f"📁 Data will be saved to: {data_root}")
    
    # Initialize Habitat simulator
    cfg = make_simple_cfg(sim_settings)
    sim = habitat_sim.Simulator(cfg)
    agent = sim.initialize_agent(sim_settings["default_agent"])
    
    print("✅ Habitat simulator initialized")
    
    # Set initial agent state
    agent_state = habitat_sim.AgentState()
    if args.floor == 1:
        agent_state.position = np.array([0.0, 0.0, 0.0])
    elif args.floor == 2:
        agent_state.position = np.array([0.0, 1.0, -1.0])
    agent.set_state(agent_state)
    
    # Get available actions
    action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())
    print("Discrete action space: ", action_names)
    
    if args.mode == 'auto':
        # Automatic navigation mode
        print(f"\n🤖 Starting automatic navigation to {args.target}...")
        success = run_automatic_navigation(args.target, data_root)
        
        if success:
            print("\n🎉 Automatic navigation completed successfully!")
        else:
            print("\n❌ Automatic navigation failed")
            
    else:
        # Manual control mode (original functionality)
        FORWARD_KEY = "w"
        LEFT_KEY = "a"
        RIGHT_KEY = "d"
        FINISH = "f"
        
        print("\n" + "#" * 40)
        print("MANUAL CONTROL MODE")
        print("Use keyboard to control the agent:")
        print(" w for go forward")
        print(" a for turn left")
        print(" d for turn right")
        print(" f for finish and quit")
        print("#" * 40)
        
        count = 0
        action = "move_forward"
        
        # Take initial observation
        navigate_and_see(action, data_root)
        
        while True:
            keystroke = cv2.waitKey(0)
            if keystroke == ord(FORWARD_KEY):
                action = "move_forward"
                navigate_and_see(action, data_root)
                print("action: FORWARD")
            elif keystroke == ord(LEFT_KEY):
                action = "turn_left"
                navigate_and_see(action, data_root)
                print("action: LEFT")
            elif keystroke == ord(RIGHT_KEY):
                action = "turn_right"
                navigate_and_see(action, data_root)
                print("action: RIGHT")
            elif keystroke == ord(FINISH):
                print("action: FINISH")
                break
            else:
                print("INVALID KEY")
                continue
    
    # Save camera extrinsics
    np.save(data_root + 'GT_pose.npy', np.asarray(cam_extr))
    
    print(f"\n📊 Session completed:")
    print(f"  • Total frames: {count}")
    print(f"  • Data saved to: {data_root}")
    print("✅ All data saved successfully!")


if __name__ == "__main__":
    main()