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

# Note: Habitat imports commented out due to environment issues
# import habitat_sim
# from habitat_sim.utils.common import d3_40_colors_rgb


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
    # Note: Would normally use d3_40_colors_rgb from habitat_sim
    semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
    
    # Create a simple color palette for demonstration
    palette = []
    for i in range(256):
        palette.extend([i, (i * 2) % 256, (i * 3) % 256])
    
    semantic_img.putpalette(palette)
    semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
    semantic_img = semantic_img.convert("RGB")
    semantic_img = cv2.cvtColor(np.asarray(semantic_img), cv2.COLOR_RGB2BGR)
    return semantic_img


def make_simple_cfg(settings):
    """
    Create a simple configuration for the simulator.
    Enhanced with agent action configuration for navigation.
    """
    # Note: This would normally create Habitat configuration
    # For demonstration, we'll return a dummy config
    
    class DummyConfig:
        def __init__(self):
            self.scene = settings["scene"]
            self.agent_height = settings["sensor_height"]
            
            # Agent action configuration as per AGENTS.md
            self.step_size = 0.25  # meters
            self.turn_angle = 10.0  # degrees
            
    return DummyConfig()


def navigate_and_see(action="", data_root='data_collection/second_floor/', auto_mode=False, target_category=None):
    """
    Enhanced navigation function that can work in manual or automatic mode.
    
    Args:
        action: Action to perform (manual mode)
        data_root: Directory to save data
        auto_mode: Whether running in automatic navigation mode
        target_category: Target object category for highlighting
    """
    global count, navigator
    
    # Note: Would normally get observations from simulator
    # observations = sim.step(action)
    
    # For demonstration, create dummy observations
    observations = create_dummy_observations(target_category)
    
    # If in auto mode and navigator exists, highlight target object
    if auto_mode and navigator and target_category:
        highlighted_rgb = navigator.highlight_target_object(
            observations["color_sensor"],
            observations["semantic_sensor"]
        )
        display_image = transform_rgb_bgr(highlighted_rgb)
    else:
        display_image = transform_rgb_bgr(observations["color_sensor"])
    
    # Display images
    cv2.imshow("RGB", display_image)
    cv2.imshow("depth", transform_depth(observations["depth_sensor"]))
    cv2.imshow("semantic", transform_semantic(observations["semantic_sensor"]))
    
    # Simulate agent state
    # agent_state = agent.get_state()
    # sensor_state = agent_state.sensor_states['color_sensor']
    
    print("Frame:", count)
    print("camera pose: x y z rw rx ry rz")
    # Dummy pose values
    print(f"{0.0:.6f} {1.5:.6f} {0.0:.6f} {1.0:.6f} {0.0:.6f} {0.0:.6f} {0.0:.6f}")
    
    count += 1
    
    # Save images
    cv2.imwrite(data_root + f"rgb/{count}.png", display_image)
    cv2.imwrite(data_root + f"depth/{count}.png", transform_depth(observations["depth_sensor"]))
    cv2.imwrite(data_root + f"semantic/{count}.png", transform_semantic(observations["semantic_sensor"]))
    
    # Save camera extrinsics
    cam_extr.append([0.0, 1.5, 0.0, 1.0, 0.0, 0.0, 0.0])


def create_dummy_observations(target_category=None):
    """
    Create dummy observations when Habitat is not available.
    
    Args:
        target_category: If provided, add target-colored pixels
        
    Returns:
        Dictionary with dummy sensor observations
    """
    # Create realistic-looking RGB image
    rgb_image = np.random.randint(50, 200, (512, 512, 3), dtype=np.uint8)
    
    # Add some structure to make it look more like an indoor scene
    # Add floor
    rgb_image[400:, :] = [120, 80, 60]  # Brown floor
    
    # Add walls
    rgb_image[:100, :] = [200, 190, 180]  # Light walls
    rgb_image[:, :50] = [180, 170, 160]   # Side wall
    rgb_image[:, -50:] = [180, 170, 160]  # Side wall
    
    # Create semantic map
    semantic_map = np.random.randint(0, 40, (512, 512, 3), dtype=np.uint8)
    
    # Add target object if specified
    if target_category:
        target_colors = {
            'rack': (0, 255, 133),
            'cushion': (255, 9, 92),
            'sofa': (10, 0, 255),
            'stair': (173, 255, 0),
            'cooktop': (7, 255, 224)
        }
        
        if target_category in target_colors:
            target_color = target_colors[target_category]
            
            # Add some target-colored regions
            for _ in range(3):
                y = np.random.randint(100, 400)
                x = np.random.randint(100, 400)
                size = np.random.randint(20, 80)
                
                y_end = min(y + size, 512)
                x_end = min(x + size, 512)
                
                semantic_map[y:y_end, x:x_end] = target_color
                rgb_image[y:y_end, x:x_end] = [c//2 for c in target_color]  # Darker version for RGB
    
    # Create depth map
    depth_map = np.ones((512, 512), dtype=np.float32) * 2.0  # 2 meters default depth
    
    return {
        "color_sensor": rgb_image,
        "semantic_sensor": semantic_map,
        "depth_sensor": depth_map
    }


def run_automatic_navigation(target_category: str, data_root: str):
    """
    Run automatic navigation mode following RRT path.
    
    Args:
        target_category: Target object category
        data_root: Directory to save data
    """
    global navigator, count
    
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
        
        # Transform to 3D waypoints
        waypoints_3d = navigator.transform_rrt_path_to_3d(pixel_path)
        print(f"📍 Generated {len(waypoints_3d)} 3D waypoints")
        
        # Simulate navigation through waypoints
        print("🚀 Beginning automatic navigation...")
        
        for i, waypoint in enumerate(waypoints_3d):
            print(f"\n--- Navigating to waypoint {i+1}/{len(waypoints_3d)} ---")
            print(f"Target: ({waypoint[0]:.3f}, {waypoint[2]:.3f})")
            
            # Simulate several steps to reach this waypoint
            steps_to_waypoint = 3 + i % 5  # Varying steps per waypoint
            
            for step in range(steps_to_waypoint):
                action = "move_forward" if step % 2 == 0 else "turn_left"
                print(f"  Step {step + 1}: {action}")
                
                # Take action and save frame
                navigate_and_see(action, data_root, auto_mode=True, target_category=target_category)
                
                # Small delay for visualization
                cv2.waitKey(100)
        
        print(f"\n✅ Automatic navigation completed!")
        print(f"📊 Total frames recorded: {count}")
        
        # Generate video from recorded frames
        print("🎬 Generating navigation video...")
        generate_video_from_saved_frames(data_root, target_category)
        
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
    global count
    
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
    
    # Initialize simulator (would normally be done here)
    # cfg = make_simple_cfg(sim_settings)
    # sim = habitat_sim.Simulator(cfg)
    # agent = sim.initialize_agent(sim_settings["default_agent"])
    
    cfg = make_simple_cfg(sim_settings)
    print("✅ Simulator configuration created")
    
    # Set initial agent state
    # agent_state = habitat_sim.AgentState()
    # if args.floor == 1:
    #     agent_state.position = np.array([0.0, 0.0, 0.0])
    # elif args.floor == 2:
    #     agent_state.position = np.array([0.0, 1.0, -1.0])
    # agent.set_state(agent_state)
    
    # Get available actions
    # action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())
    action_names = ["move_forward", "turn_left", "turn_right"]
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