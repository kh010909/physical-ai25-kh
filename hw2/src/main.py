import argparse
import os
import shutil

import cv2
# Habitat imports
import habitat_sim
import numpy as np
from PIL import Image
from habitat_sim.utils.common import d3_40_colors_rgb

# Import our agent navigation module
from agent_navigation import AgentNavigator
from rrt_pathfinder import RRTPathfinder

# Scene configuration
test_scene = "../replica_v1/apartment_0/habitat/mesh_semantic.ply"

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

    # Define action space for navigation
    action_space = {
        "move_forward": habitat_sim.ActionSpec(
            "move_forward",
            habitat_sim.ActuationSpec(amount=0.25)
        ),
        "turn_left": habitat_sim.ActionSpec(
            "turn_left",
            habitat_sim.ActuationSpec(amount=np.deg2rad(10.0))
        ),
        "turn_right": habitat_sim.ActionSpec(
            "turn_right",
            habitat_sim.ActuationSpec(amount=np.deg2rad(10.0))
        ),
    }
    agent_cfg.action_space = action_space
    
    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


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





def select_target_category():
    """
    Interactive menu to select target category from available options.
    
    Returns:
        str: Selected target category
    """
    try:
        # Check for required files
        required_files = [
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
        ]
        
        missing_files = [f for f in required_files if not os.path.exists(f)]
        if missing_files:
            print(f" Missing required files: {missing_files}")
            print(" Please run semantic map generation first (main_semantic_map.py)")
            return None
        
        # Initialize pathfinder to get available categories
        pathfinder = RRTPathfinder(
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
        )
        
        categories = list(pathfinder.target_categories.keys())
        
        print("\n Available target categories:")
        for i, category in enumerate(categories, 1):
            color = pathfinder.target_categories[category]
            print(f"  {i}. {category.upper()} (RGB: {color})")
        
        while True:
            try:
                user_input = input("\n Select target category (name or number): ").strip().lower()
                
                if user_input.isdigit():
                    choice_num = int(user_input)
                    if 1 <= choice_num <= len(categories):
                        return categories[choice_num - 1]
                    else:
                        print(f" Invalid number! Please enter 1-{len(categories)}")
                elif user_input in categories:
                    return user_input
                else:
                    print(f" Invalid choice! Available: {', '.join(categories)}")
                    
            except ValueError:
                print(" Invalid input!")
        
    except Exception as e:
        print(f" Error during target selection: {e}")
        return None


def run_automatic_navigation(target_category: str, data_root: str, interactive_start: bool = True, floor: int = 1):
    """
    Run automatic navigation mode following RRT path with Habitat.
    
    Args:
        target_category: Target object category
        data_root: Directory to save data
        interactive_start: Whether to allow user to click starting point
        floor: Floor number (1 or 2)
    """
    global navigator, count, sim, agent
    
    print(f"\n Starting automatic navigation to: {target_category.upper()}")
    print(f" Floor: {floor}")
    print("=" * 60)
    
    try:
        # Check for required files
        required_files = [
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
        ]
        
        missing_files = [f for f in required_files if not os.path.exists(f)]
        if missing_files:
            print(f" Missing required files: {missing_files}")
            print(" Please run semantic map generation first (main_semantic_map.py)")
            return False
        
        # Initialize RRT pathfinder
        print(" Initializing RRT pathfinder...")
        pathfinder = RRTPathfinder(
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
        )
        
        # Initialize navigator with floor parameter
        navigator = AgentNavigator(pathfinder, target_category, floor=floor)
        
        # Get starting point - interactive or default
        print("\n Starting point selection:")
        if interactive_start:
            print("  You will now select the starting point by clicking on the map...")
            start_point = pathfinder.display_map_for_selection(target_category)
            print(f" Starting point selected: {start_point}")
        else:
            # Use default starting point
            default_starts = {
                'sofa': (300, 500),
                'rack': (600, 400),
                'cushion': (700, 600),
                'stair': (400, 800),
                'cooktop': (200, 300)
            }
            start_point = default_starts.get(target_category, (400, 400))
            print(f" Using default starting point: {start_point}")
        
        # Get RRT path
        print(" Computing RRT path...")
        goal_point = pathfinder.find_goal_point(target_category)
        
        if not goal_point:
            print(f" No {target_category} found in the map")
            return False
        
        pixel_path = pathfinder.run_rrt(start_point, goal_point)
        
        if not pixel_path:
            print(" No path found by RRT algorithm")
            return False
        
        print(f" RRT path found with {len(pixel_path)} waypoints")
        
        # Visualize and save the RRT path
        print(" Generating RRT path visualization...")
        visualization_path = f"{target_category}_rrt_path.png"
        pathfinder.visualize_path(output_path=visualization_path)
        print(f" RRT visualization saved as: {visualization_path}")
        
        # Run navigation using the AgentNavigator with Habitat
        print(" Beginning automatic navigation with Habitat...")
        video_path = navigator.run_navigation(pixel_path, sim, agent)
        
        print(f"\n Automatic navigation completed!")
        print(f" Video saved as: {video_path}")
        
        return True
        
    except Exception as e:
        print(f" Error during automatic navigation: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """
    Main function with enhanced argument parsing for navigation modes.
    """
    global count, sim, agent
    
    parser = argparse.ArgumentParser(description="Enhanced Habitat Agent with Navigation")
    parser.add_argument('-f', '--floor', type=int, default=1, help='Floor number (1 or 2)')
    parser.add_argument('-m', '--mode', type=str, default='auto',
                       choices=['manual', 'auto'], help='Control mode (manual or auto)')
    parser.add_argument('-t', '--target', type=str, default=None,
                       help='Target object category for automatic mode (optional, will prompt if not provided)')
    parser.add_argument('--interactive', action='store_true', default=True,
                       help='Enable interactive starting point selection (default: True)')
    parser.add_argument('--no-interactive', dest='interactive', action='store_false',
                       help='Disable interactive starting point selection')
    
    args = parser.parse_args()
    
    print(" Enhanced Habitat Agent with Navigation")
    print("=" * 50)
    print(f"Mode: {args.mode.upper()}")
    print(f"Floor: {args.floor}")
    print(f"Interactive mode: {'ENABLED' if args.interactive else 'DISABLED'}")
    
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
    
    print(f" Data will be saved to: {data_root}")
    
    # Initialize Habitat simulator
    cfg = make_simple_cfg(sim_settings)
    sim = habitat_sim.Simulator(cfg)
    agent = sim.initialize_agent(sim_settings["default_agent"])
    
    print(" Habitat simulator initialized")
    
    # INITIAL DEBUG VIEW - Show agent position BEFORE anything else
    print("\n" + "="*60)
    print(" INITIAL AGENT POSITION DEBUG VIEW")
    print("="*60)
    
    # Set initial agent state based on floor
    agent_state = habitat_sim.AgentState()
    if args.floor == 1:
        # Y=0 is at floor 2 level, so floor 1 is below at negative Y
        agent_state.position = np.array([0.0, -1.5, 0.0])
    elif args.floor == 2:
        agent_state.position = np.array([0.0, 0.0, 0.0])
    agent.set_state(agent_state)
    
    # Get initial observations
    initial_obs = sim.get_sensor_observations()
    initial_rgb = initial_obs["color_sensor"]
    initial_semantic = initial_obs["semantic_sensor"]
    
    # Get agent state info
    current_state = agent.get_state()
    current_pos = current_state.position
    
    # Display initial view
    display_img = cv2.cvtColor(initial_rgb, cv2.COLOR_RGB2BGR)
    
    # Add info overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(display_img, f"INITIAL AGENT VIEW - Floor {args.floor}", 
               (10, 30), font, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(display_img, f"Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})", 
               (10, 60), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(display_img, f"Mode: {args.mode.upper()}", 
               (10, 90), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(display_img, "Press any key to continue...", 
               (10, display_img.shape[0] - 20), font, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    
    print(f" Agent initial position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
    print(f" Floor: {args.floor}")
    print(f" Mode: {args.mode}")
    print("\n  Showing initial view window...")
    print("   Press any key in the window to continue...")
    
    cv2.imshow("Initial Agent Position - DEBUG", display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print(" Debug view closed. Continuing...\n")
    print("="*60 + "\n")
    
    # Get available actions
    action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())
    print("Discrete action space: ", action_names)
    
    if args.mode == 'auto':
        # Automatic navigation mode
        
        # Get target category - either from args or interactive selection
        if args.target is None:
            print("\n Target category not specified. Starting interactive selection...")
            target_category = select_target_category()
            if target_category is None:
                print(" No target category selected. Exiting.")
                return
        else:
            target_category = args.target.lower()
        
        print(f"\n Starting automatic navigation to {target_category}...")
        success = run_automatic_navigation(target_category, data_root, interactive_start=args.interactive, floor=args.floor)
        
        if success:
            print("\n Automatic navigation completed successfully!")
        else:
            print("\n Automatic navigation failed")
            
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
    
    print(f"\n Session completed:")
    print(f"  • Total frames: {count}")
    print(f"  • Data saved to: {data_root}")
    print(" All data saved successfully!")


if __name__ == "__main__":
    main()
