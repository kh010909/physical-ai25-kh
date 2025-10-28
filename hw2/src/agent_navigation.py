import math
import os
from typing import List, Tuple, Optional

import cv2
# Habitat imports
import habitat_sim
import numpy as np

# Import RRT pathfinder for getting pre-computed paths
from rrt_pathfinder import RRTPathfinder

# Import the VERIFIED coordinate transformation from map_utils
from map_utils import pixel_to_habitat_coords


class AgentNavigator:
    """
    Agent navigation implementation for following RRT paths in Habitat simulator.
    """
    
    def __init__(self, rrt_pathfinder: RRTPathfinder, target_category: str, floor: int = 1):
        """
        Initialize the agent navigator.
        
        Args:
            rrt_pathfinder: Initialized RRT pathfinder instance
            target_category: Target object category (e.g., "chair", "table")
            floor: Floor number (1 or 2), determines agent height
        """
        self.pathfinder = rrt_pathfinder
        self.target_category = target_category
        self.target_color = rrt_pathfinder.target_categories.get(target_category, (255, 0, 0))
        
        # Agent configuration
        self.step_size = 0.25  # meters
        self.turn_angle = 10.0  # degrees
        # Set agent height based on floor
        # IMPORTANT: Y=0.0 is at floor 2 level in this environment!
        # Floor 1: Y = -1.5 (below floor 2)
        # Floor 2: Y = 0.0 (reference level)
        # Note: agent_height is the Y position of the agent base, not the sensor
        self.agent_height = -1.5 if floor == 1 else 0.0  # meters
        self.floor = floor
        self.angle_threshold = np.deg2rad(10.0)  # radians - threshold for alignment
        # Use forward_amount * 0.8 as threshold like the working code
        self.distance_threshold = self.step_size * 0.8  # meters - threshold for reaching waypoint
        
        # Video recording
        self.frames = []
        self.video_fps = 10
        
        # Navigation state
        self.current_waypoint_index = 0
        self.path_3d = []
        
    # def configure_agent_actions(self, config):
    #     """
    #     Configure agent actions with appropriate step sizes.
        
    #     Args:
    #         config: Habitat configuration object
    #     """
    #     # Configure Habitat actions as per AGENTS.md specification
    #     # Note: Habitat uses radians for turn angles
    #     config.TASK.ACTIONS.MOVE_FORWARD.MOTION_ARGS["step_size"] = self.step_size
    #     config.TASK.ACTIONS.TURN_LEFT.MOTION_ARGS["angle"] = np.deg2rad(self.turn_angle)
    #     config.TASK.ACTIONS.TURN_RIGHT.MOTION_ARGS["angle"] = np.deg2rad(self.turn_angle)
        
    #     print(f"Agent configuration:")
    #     print(f"  • Floor: {self.floor}")
    #     print(f"  • Agent height: {self.agent_height} meters")
    #     print(f"  • Step size: {self.step_size} meters")
    #     print(f"  • Turn angle: {self.turn_angle} degrees ({np.deg2rad(self.turn_angle):.3f} radians)")
        
    def pixel_to_world(self, pixel_coord: Tuple[int, int], depth_map: np.ndarray = None, 
                       camera_matrix: np.ndarray = None, camera_transform: np.ndarray = None) -> Tuple[float, float, float]:
        """
        Convert 2D pixel coordinates to 3D world coordinates.
        
        Uses the VERIFIED coordinate transformation from map_utils.py that has been
        tested with test_interactive_transformation.py and confirmed to work correctly.
        
        Args:
            pixel_coord: (x, y) pixel coordinates in TRANSFORMED map space
            depth_map: Depth map from simulator (unused, kept for compatibility)
            camera_matrix: Camera intrinsic matrix (unused, kept for compatibility)
            camera_transform: Camera extrinsic transform (unused, kept for compatibility)
            
        Returns:
            (x, y, z) world coordinates where y is agent_height
        """
        x_pixel, y_pixel = pixel_coord
        
        # Use the VERIFIED transformation function from map_utils.py
        # This is the EXACT same function used in test_interactive_transformation.py
        habitat_x, habitat_z = pixel_to_habitat_coords(x_pixel, y_pixel)
        
        # Set Y coordinate to agent height
        waypoint = (habitat_x, self.agent_height, habitat_z)
        
        return waypoint
    
    def habitat_to_pixel_coords(self, habitat_x: float, habitat_z: float) -> Tuple[int, int]:
        """
        Convert Habitat coordinates back to pixel coordinates.
        
        This is the INVERSE of pixel_to_habitat_coords and uses the same parameters.
        Useful for debugging and visualization.
        
        Args:
            habitat_x: Habitat X coordinate
            habitat_z: Habitat Z coordinate
            
        Returns:
            Tuple of (pixel_x, pixel_y) in TRANSFORMED map space
        """
        # Same parameters as in map_utils.py
        habitat_x_min = -3.089056
        habitat_x_max = 6.220782
        habitat_z_min = -4.929266
        habitat_z_max = 9.912446
        
        TRANSFORMED_WIDTH = 1848
        TRANSFORMED_HEIGHT = 1159
        
        # Convert to normalized coordinates [0, 1]
        norm_x = (habitat_z - habitat_z_min) / (habitat_z_max - habitat_z_min)
        norm_y = (habitat_x_max - habitat_x) / (habitat_x_max - habitat_x_min)
        
        # Map to TRANSFORMED pixel coordinates
        pixel_x = int(norm_x * TRANSFORMED_WIDTH)
        pixel_y = int(norm_y * TRANSFORMED_HEIGHT)
        
        return pixel_x, pixel_y
        
    def transform_rrt_path_to_3d(self, pixel_path: List) -> List[Tuple[float, float, float]]:
        """
        Transform RRT path from 2D pixel coordinates to 3D world coordinates.
        
        Uses the VERIFIED coordinate transformation from map_utils.py.
        
        Args:
            pixel_path: List of nodes from RRT pathfinder
            
        Returns:
            List of 3D waypoints
        """
        waypoints_3d = []
        
        print(f"\n Transforming {len(pixel_path)} RRT nodes to 3D waypoints...")
        print("="*70)
        
        max_error_x = 0
        max_error_y = 0
        
        for i, node in enumerate(pixel_path):
            # Convert node to pixel coordinates
            pixel_coord = (int(node.x), int(node.y))
            
            # Transform to 3D world coordinates using VERIFIED transformation
            world_coord = self.pixel_to_world(pixel_coord)
            waypoints_3d.append(world_coord)
            
            # Verify round-trip transformation for quality check
            verify_pixel = self.habitat_to_pixel_coords(world_coord[0], world_coord[2])
            error_x = abs(verify_pixel[0] - pixel_coord[0])
            error_y = abs(verify_pixel[1] - pixel_coord[1])
            max_error_x = max(max_error_x, error_x)
            max_error_y = max(max_error_y, error_y)
            
            # Print debug info for first, last, and every 10th waypoint
            if i == 0 or i == len(pixel_path) - 1 or i % 10 == 0:
                print(f"  Waypoint {i:3d}: Pixel ({pixel_coord[0]:4d}, {pixel_coord[1]:4d}) "
                      f"-> Habitat ({world_coord[0]:7.3f}, {world_coord[1]:7.3f}, {world_coord[2]:7.3f}) "
                      f"[error: ({error_x}, {error_y}) px]")
        
        print("="*70)
        print(f" Transformation complete: {len(waypoints_3d)} waypoints generated")
        print(f" Max round-trip error: ({max_error_x}, {max_error_y}) pixels")
        if max_error_x > 5 or max_error_y > 5:
            print("  WARNING: High round-trip error detected! Transformation may be inaccurate.")
        else:
            print(" Round-trip verification passed: Transformation is accurate!")
        print()
            
        return waypoints_3d
        
    def calculate_heading_to_waypoint(self, current_pos: Tuple[float, float, float], 
                                    current_rotation: float, 
                                    target_pos: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        Calculate the heading angle needed to face a target waypoint.
        
        Args:
            current_pos: Current agent position (x, y, z)
            current_rotation: Current agent rotation in radians
            target_pos: Target waypoint (x, y, z)
            
        Returns:
            Tuple of (angle_diff in radians, target_yaw in radians)
        """
        # Calculate vector to target in Habitat coordinates
        dx = target_pos[0] - current_pos[0]  # X component (right)
        dz = target_pos[2] - current_pos[2]  # Z component (forward)
        
        # Calculate target yaw angle to face the target
        # In Habitat's coordinate system, we need to use -dx and -dz
        target_yaw = np.arctan2(-dx, -dz)
        
        # Calculate the shortest angle difference
        angle_diff = target_yaw - current_rotation
        
        # Normalize to [-pi, pi] range using atan2 for proper wrapping
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            
        return angle_diff, target_yaw
        
    # def calculate_distance_to_waypoint(self, current_pos: Tuple[float, float, float], 
    #                                  target_pos: Tuple[float, float, float]) -> float:
    #     """
    #     Calculate 2D distance to target waypoint (ignoring Y axis).
        
    #     Args:
    #         current_pos: Current agent position (x, y, z)
    #         target_pos: Target waypoint (x, y, z)
            
    #     Returns:
    #         Distance in meters
    #     """
    #     dx = target_pos[0] - current_pos[0]
    #     dz = target_pos[2] - current_pos[2]
    #     return math.sqrt(dx*dx + dz*dz)
        
    def highlight_target_object(self, rgb_image: np.ndarray, semantic_map: np.ndarray, 
                               semantic_scene: habitat_sim.scene.SemanticScene = None) -> np.ndarray:
        """
        Overlay a transparent mask on the target object in the RGB image.
        
        Args:
            rgb_image: RGB observation from simulator
            semantic_map: Semantic segmentation map (semantic IDs)
            semantic_scene: Habitat semantic scene object for ID mapping
            
        Returns:
            RGB image with highlighted target object
        """
        # Create a copy of the image
        highlighted_image = rgb_image.copy()
        
        # Handle RGBA images by converting to RGB
        if highlighted_image.shape[-1] == 4:
            highlighted_image = highlighted_image[:, :, :3]
        
        # Find target object pixels in semantic map using actual semantic IDs
        target_mask = self._create_target_mask_from_semantics(semantic_map, semantic_scene)
        
        if target_mask is not None and np.any(target_mask):
            # Create colored overlay (semi-transparent red)
            overlay_color = np.array([255, 0, 0], dtype=np.uint8)  # Red
            alpha = 0.3  # Transparency
            
            # Apply overlay where target object is detected
            # Ensure both arrays have the same number of channels
            if len(highlighted_image[target_mask].shape) == 2:
                # Handle grayscale by expanding to 3 channels
                overlay_color = overlay_color[:1]
            
            highlighted_image[target_mask] = (
                (1 - alpha) * highlighted_image[target_mask] + 
                alpha * overlay_color
            ).astype(np.uint8)
            
        return highlighted_image
        
    def _create_target_mask_from_semantics(self, semantic_map: np.ndarray, 
                                          semantic_scene: habitat_sim.scene.SemanticScene = None) -> Optional[np.ndarray]:
        """
        Create a boolean mask for the target object pixels using Habitat semantic data.
        
        Args:
            semantic_map: Semantic segmentation map with semantic IDs
            semantic_scene: Habitat semantic scene object
            
        Returns:
            Boolean mask or None if target not found
        """
        if semantic_scene is None:
            # Fallback to simple approach if semantic scene not available
            print("No semantic scene available, using fallback method.")
            return self._create_target_mask_fallback(semantic_map)
        
        # Get target semantic IDs from Habitat semantic scene
        target_object_ids = []
        
        # Map target category to semantic object IDs
        target_category_lower = self.target_category.lower()
        
        for obj in semantic_scene.objects:
            if obj is not None and hasattr(obj, 'category') and obj.category is not None:
                obj_category = obj.category.name().lower()
                
                # Check if this object matches our target category
                if (target_category_lower in obj_category or 
                    obj_category in target_category_lower or
                    self._is_matching_category(obj_category, target_category_lower)):
                    target_object_ids.append(obj.semantic_id)
        
        if not target_object_ids:
            print(f"No {self.target_category} objects found in semantic scene")
            return None
        
        # Create mask where semantic map contains target object IDs
        mask = np.isin(semantic_map, target_object_ids)
        
        return mask if np.any(mask) else None
    
    def _is_matching_category(self, obj_category: str, target_category: str) -> bool:
        """
        Check if object category matches target category with fuzzy matching.
        
        Args:
            obj_category: Object category from semantic scene
            target_category: Target category we're looking for
            
        Returns:
            True if categories match
        """
        # Define category aliases and variations
        category_aliases = {
            'sofa': ['couch', 'settee', 'loveseat', 'sofa'],
            'chair': ['seat', 'chair', 'armchair'],
            'cushion': ['pillow', 'cushion', 'throw pillow'],
            'rack': ['shelf', 'rack', 'shelving', 'bookshelf'],
            'stair': ['stairs', 'staircase', 'step', 'stair'],
            'cooktop': ['stove', 'cooktop', 'range', 'burner']
        }
        
        target_aliases = category_aliases.get(target_category, [target_category])
        
        return any(alias in obj_category or obj_category in alias for alias in target_aliases)
    
    def _create_target_mask_fallback(self, semantic_map: np.ndarray) -> Optional[np.ndarray]:
        """
        Fallback method using color-based detection when semantic scene unavailable.
        
        Args:
            semantic_map: Semantic segmentation map
            
        Returns:
            Boolean mask or None if target not found
        """
        # If semantic_map has 3 channels, assume it's a color image
        if len(semantic_map.shape) == 3:
            target_color = np.array(self.target_color)
            color_distance = np.sqrt(np.sum((semantic_map - target_color)**2, axis=2))
            mask = color_distance < 50  # Threshold for color similarity
            return mask if np.any(mask) else None
        else:
            # If it's a single channel semantic ID map, we can't do much without scene info
            print("Warning: Semantic scene not available for proper target detection")
            return None
        
    def navigate_to_waypoint(self, simulator: habitat_sim.Simulator, agent: habitat_sim.Agent, 
                            target_waypoint: Tuple[float, float, float]) -> List[np.ndarray]:
        """
        Navigate agent to a specific waypoint and collect frames.
        
        Args:
            simulator: Habitat simulator instance
            agent: Habitat agent instance
            target_waypoint: Target 3D coordinates (x, y, z) in Habitat world space
            
        Returns:
            List of RGB frames collected during navigation
        """
        frames = []
        
        print(f"Navigating to waypoint: ({target_waypoint[0]:.3f}, {target_waypoint[2]:.3f})")
        
        # Navigation loop with Habitat simulator
        max_steps = 5000  # Prevent infinite loops (increased from 500)
        step_count = 0
        
        # Convert turn angle to radians for threshold comparison  
        turn_threshold = np.deg2rad(self.turn_angle)
        
        waypoint_reached = False
        
        while not waypoint_reached and step_count < max_steps:
            # Get current agent state from Habitat
            agent_state = agent.get_state()
            current_pos = agent_state.position
            current_rotation = self._get_agent_rotation_y(agent_state.rotation)  # Now returns radians
            
            current_x, current_z = current_pos[0], current_pos[2]
            target_x, target_z = target_waypoint[0], target_waypoint[2]
            
            # Calculate direction vector to target
            dx = target_x - current_x
            dz = target_z - current_z
            distance = np.sqrt(dx ** 2 + dz ** 2)
            
            # DEBUG: Print detailed navigation state every 10 steps
            if step_count % 10 == 0:
                print(f"\n  === DEBUG Step {step_count} ===")
                print(f"  Current pos: ({current_x:.3f}, {current_z:.3f})")
                print(f"  Target pos: ({target_x:.3f}, {target_z:.3f})")
                print(f"  Direction vector: dx={dx:.3f}, dz={dz:.3f}")
                print(f"  Distance to target: {distance:.3f}m")
                print(f"  Current rotation (yaw): {np.rad2deg(current_rotation):.1f}° ({current_rotation:.3f} rad)")
            
            # Check if we've reached the waypoint
            if distance < self.distance_threshold:
                print(f"  [INFO] Reached waypoint in {step_count} steps (distance: {distance:.2f}m)")
                print(f"  Final position: ({current_pos[0]:.3f}, {current_pos[2]:.3f})")
                waypoint_reached = True
                # Wait for a moment at the waypoint
                cv2.waitKey(2000)
                break
            
            # Calculate heading to waypoint (returns angle_diff and target_yaw in radians)
            angle_diff, target_yaw = self.calculate_heading_to_waypoint(current_pos, current_rotation, target_waypoint)
            
            # DEBUG: Print angle information every 10 steps
            if step_count % 10 == 0:
                print(f"  Target yaw to face waypoint: {np.rad2deg(target_yaw):.1f}° ({target_yaw:.3f} rad)")
                print(f"  Angle difference: {np.rad2deg(angle_diff):.1f}° ({angle_diff:.3f} rad)")
                print(f"  Turn threshold: {np.rad2deg(turn_threshold):.1f}° ({turn_threshold:.3f} rad)")
                print(f"  Need to turn? {abs(angle_diff) > turn_threshold}")
            
            # Decide action based on angle difference
            if abs(angle_diff) > turn_threshold:
                # Need to turn - use sign of angle_diff to determine direction
                if angle_diff > 0:
                    agent.act("turn_left")
                else:
                    agent.act("turn_right")
            else:
                # Aligned enough - move forward
                agent.act("move_forward")
            
            # Increment step counter
            step_count += 1
            
            # Only render and collect frames every N steps for performance
            render_interval = 30  # Collect every 30th frame
            if step_count % render_interval == 0:
                # Get observations only when rendering
                observations = simulator.get_sensor_observations()
                
                # Get semantic scene for proper target detection
                semantic_scene = simulator.semantic_scene
                
                # Highlight target object and collect frame
                highlighted_frame = self.highlight_target_object(
                    observations["color_sensor"], 
                    observations["semantic_sensor"],
                    semantic_scene
                )
                frames.append(highlighted_frame)
                
                # Display frame in real-time for debugging
                display_frame = cv2.cvtColor(highlighted_frame, cv2.COLOR_RGB2BGR)
                
                # Add text overlay with current state
                font = cv2.FONT_HERSHEY_SIMPLEX
                action_text = "turn" if abs(angle_diff) > turn_threshold else "forward"
                cv2.putText(display_frame, f"Step: {step_count} | Mode: {action_text}", 
                           (10, 30), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(display_frame, f"Pos: ({current_x:.2f}, {current_z:.2f})", 
                           (10, 60), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(display_frame, f"Target: ({target_x:.2f}, {target_z:.2f})", 
                           (10, 90), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(display_frame, f"Dist: {distance:.2f}m | Angle: {np.rad2deg(angle_diff):.1f} deg", 
                           (10, 120), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                
                # Show the frame
                cv2.imshow("Agent Navigation (Press 'q' to skip)", display_frame)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    print("  User requested to skip visualization")
                    cv2.destroyAllWindows()
                    break
        
        # Remove duplicate increment
        return frames
    
    def _get_agent_rotation_y(self, quaternion) -> float:
        """
        Extract Y-axis rotation (yaw) from Habitat agent quaternion.
        
        Args:
            quaternion: Habitat agent rotation quaternion
            
        Returns:
            Y-axis rotation in radians
        """
        # Extract quaternion components (w, x, y, z format)
        if hasattr(quaternion, 'components'):
            w, x, y, z = quaternion.components
        else:
            w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        
        # Calculate yaw (Y-axis rotation) around the Y-axis
        yaw = np.arctan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + x * x))
        
        return yaw
        
    def _create_dummy_observations(self) -> dict:
        """
        Create dummy observations for simulation when Habitat is not available.
        
        Returns:
            Dictionary with dummy sensor observations
        """
        # Create dummy RGB image
        rgb_image = np.random.randint(0, 255, (512, 512, 3), dtype=np.uint8)
        
        # Create dummy semantic map
        semantic_map = np.random.randint(0, 255, (512, 512, 3), dtype=np.uint8)
        
        # Add some target-colored pixels for demonstration
        target_pixels = np.random.choice([True, False], size=(512, 512), p=[0.05, 0.95])
        semantic_map[target_pixels] = self.target_color
        
        return {
            "color_sensor": rgb_image,
            "semantic_sensor": semantic_map,
            "depth_sensor": np.ones((512, 512), dtype=np.float32)
        }
        
    def run_navigation(self, pixel_path: List, simulator: habitat_sim.Simulator = None, 
                      agent: habitat_sim.Agent = None) -> str:
        """
        Execute the complete navigation pipeline with Habitat.
        
        Args:
            pixel_path: RRT path as list of nodes
            simulator: Habitat simulator instance
            agent: Habitat agent instance
            
        Returns:
            Path to generated video file
        """
        print(" Starting agent navigation...")
        
        # Step 1: Transform pixel path to 3D waypoints
        print(" Transforming RRT path to 3D waypoints...")
        self.path_3d = self.transform_rrt_path_to_3d(pixel_path)
        print(f"   Generated {len(self.path_3d)} waypoints")
        
        # if simulator is None or agent is None:
        #     print(" No Habitat simulator provided - using simulation mode")
        #     return self._run_navigation_simulation(pixel_path)
        
        print(" Beginning navigation with Habitat...")
        
        # Set initial agent position to first waypoint
        if self.path_3d:
            start_x, start_y, start_z = self.path_3d[0]
            agent_state = habitat_sim.AgentState()
            agent_state.position = np.array([start_x, self.agent_height, start_z])
            agent.set_state(agent_state)
            print(f"   Set initial position to: ({start_x:.3f}, {self.agent_height:.3f}, {start_z:.3f})")
            
            # Verify agent position was set correctly
            actual_state = agent.get_state()
            actual_pos = actual_state.position
            actual_rot = self._get_agent_rotation_y(actual_state.rotation)
            print(f"   Verified agent position: ({actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f})")
            print(f"   Agent rotation: {np.rad2deg(actual_rot):.1f}°")
            print(f"\n    Path waypoints (Habitat coords):")
            for idx, wp in enumerate(self.path_3d[:5]):  # Show first 5
                print(f"      {idx}: ({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})")
            if len(self.path_3d) > 5:
                print(f"      ... ({len(self.path_3d) - 5} more waypoints)")
        
        # Step 2: Navigate through waypoints (skip first waypoint as agent starts there)
        all_frames = []
        for i, waypoint in enumerate(self.path_3d[1:], 1):
            print(f"\n--- Waypoint {i+1}/{len(self.path_3d)} ---")
            
            # Navigate to waypoint and collect frames using Habitat
            waypoint_frames = self.navigate_to_waypoint(simulator, agent, waypoint)
            all_frames.extend(waypoint_frames)
            
        # Step 3: Generate video
        print(f"\n🎬 Generating video with {len(all_frames)} frames...")
        video_path = self.generate_video(all_frames)
        
        print(f"✅ Navigation completed! Video saved as: {video_path}")
        return video_path
    
    # def _run_navigation_simulation(self, pixel_path: List) -> str:
    #     """
    #     Run navigation in simulation mode when Habitat is not available.
        
    #     Args:
    #         pixel_path: RRT path as list of nodes
            
    #     Returns:
    #         Path to generated video file
    #     """
    #     print("🔄 Running in simulation mode...")
        
    #     # Navigate through waypoints in simulation
    #     all_frames = []
    #     for i, waypoint in enumerate(self.path_3d):
    #         print(f"\n--- Waypoint {i+1}/{len(self.path_3d)} ---")
            
    #         # Simulate navigation to waypoint
    #         waypoint_frames = self._simulate_waypoint_navigation(waypoint, i)
    #         all_frames.extend(waypoint_frames)
            
    #     # Generate video
    #     print(f"\n🎬 Generating video with {len(all_frames)} frames...")
    #     video_path = self.generate_video(all_frames)
        
    #     print(f"✅ Simulation completed! Video saved as: {video_path}")
    #     return video_path
        
    # def _simulate_waypoint_navigation(self, waypoint: Tuple[float, float, float], waypoint_index: int) -> List[np.ndarray]:
    #     """
    #     Simulate navigation to a waypoint when Habitat is not available.
        
    #     Args:
    #         waypoint: Target waypoint coordinates
    #         waypoint_index: Index of current waypoint
            
    #     Returns:
    #         List of simulated frames
    #     """
    #     print(f"  Simulating navigation to waypoint ({waypoint[0]:.3f}, {waypoint[2]:.3f})")
        
    #     frames = []
    #     num_frames = 5 + waypoint_index  # Varying number of frames per waypoint
        
    #     for frame_idx in range(num_frames):
    #         # Create dummy observations
    #         observations = self._create_dummy_observations()
            
    #         # Highlight target object
    #         highlighted_frame = self.highlight_target_object(
    #             observations["color_sensor"],
    #             observations["semantic_sensor"]
    #         )
            
    #         # Add waypoint info to frame (for visualization)
    #         self._add_waypoint_info_to_frame(highlighted_frame, waypoint_index, frame_idx)
            
    #         frames.append(highlighted_frame)
            
    #     print(f"  Generated {len(frames)} frames for waypoint {waypoint_index + 1}")
    #     return frames
        
    # def _add_waypoint_info_to_frame(self, frame: np.ndarray, waypoint_idx: int, frame_idx: int):
    #     """
    #     Add waypoint information text overlay to frame.
        
    #     Args:
    #         frame: RGB frame to modify
    #         waypoint_idx: Current waypoint index
    #         frame_idx: Current frame index within waypoint
    #     """
    #     # Add text overlay
    #     text = f"Waypoint {waypoint_idx + 1} | Frame {frame_idx + 1}"
    #     cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
    #     # Add target category info
    #     target_text = f"Target: {self.target_category.upper()}"
    #     cv2.putText(frame, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
    def generate_video(self, frames: List[np.ndarray]) -> str:
        """
        Generate video output from collected frames.
        
        Args:
            frames: List of RGB frames
            
        Returns:
            Path to generated video file
        """
        if not frames:
            raise ValueError("No frames to generate video")
            
        # Video configuration
        height, width, layers = frames[0].shape
        video_name = f"{self.target_category}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        # Initialize video writer
        video_writer = cv2.VideoWriter(video_name, fourcc, self.video_fps, (width, height))
        
        print(f" Writing {len(frames)} frames to {video_name}...")
        
        # Write frames to video
        for i, frame in enumerate(frames):
            # Convert RGB (from Habitat/simulation) to BGR (for OpenCV)
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(bgr_frame)
            
            if (i + 1) % 10 == 0:
                print(f"   Written {i + 1}/{len(frames)} frames...")
                
        # Finalize video
        video_writer.release()
        
        print(f" Video saved successfully: {video_name}")
        return video_name


# def main():
#     """
#     Main function to demonstrate the agent navigation system.
#     """
#     print("🤖 Agent Navigation Demo")
#     print("=" * 50)
    
#     # Note: Check for required files
#     required_files = [
#         'map.png',
#         '../color_coding_semantic_segmentation_classes.xlsx',
#         'coordinate_transformation.txt'
#     ]
    
#     missing_files = [f for f in required_files if not os.path.exists(f)]
#     if missing_files:
#         print(f"❌ Missing required files: {missing_files}")
#         print("💡 Please ensure RRT pathfinding has been run first")
#         return
    
#     try:
#         # Initialize RRT pathfinder
#         print("🔧 Initializing RRT pathfinder...")
#         pathfinder = RRTPathfinder(
#             'map.png',
#             '../color_coding_semantic_segmentation_classes.xlsx',
#             'coordinate_transformation.txt'
#         )
        
#         # Select target category
#         target_category = 'sofa'  # Can be changed to any available category
#         print(f"🎯 Target category: {target_category.upper()}")
        
#         # Get pre-computed path from RRT
#         print("🗺️ Getting RRT path...")
        
#         # For demo, use a predefined start point
#         start_point = (300, 500)
#         goal_point = pathfinder.find_goal_point(target_category)
        
#         if not goal_point:
#             print(f"❌ No {target_category} found in the map")
#             return
            
#         print(f"📍 Start: {start_point}, Goal: {goal_point}")
        
#         # Run RRT pathfinding
#         pixel_path = pathfinder.run_rrt(start_point, goal_point)
        
#         if not pixel_path:
#             print("❌ No path found by RRT algorithm")
#             return
            
#         print(f"✅ RRT path found with {len(pixel_path)} waypoints")
        
#         # Initialize agent navigator
#         navigator = AgentNavigator(pathfinder, target_category)
        
#         # Run navigation simulation
#         video_path = navigator.run_navigation(pixel_path)
        
#         print(f"\n🎉 Demo completed successfully!")
#         print(f"📽️ Video output: {video_path}")
        
#     except Exception as e:
#         print(f"❌ Error during navigation demo: {e}")
#         import traceback
#         traceback.print_exc()


# if __name__ == "__main__":
#     main()
