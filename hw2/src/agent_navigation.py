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
        self.step_size = 0.40  # meters
        self.turn_angle = 10.0  # degrees
        # Set agent height based on floor
        # Floor 1: Y = -1.5 (below floor 2)
        # Floor 2: Y = 0.0 (reference level)
        # Note: agent_height is the Y position of the agent base, not the sensor
        self.agent_height = -1.5 if floor == 1 else 0.0  # meters
        self.floor = floor
        self.angle_threshold = np.deg2rad(10.0)  # radians - threshold for alignment
        self.distance_threshold = self.step_size * 0.9  # meters - threshold for reaching waypoint
        
        # Video recording
        self.frames = []
        self.video_fps = 5 
        
        # Navigation state
        self.current_waypoint_index = 0
        self.path_3d = []
        
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
            
            # Transform to 3D world coordinates
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
        
        # Ensure we have a 3-channel RGB image
        if len(highlighted_image.shape) != 3 or highlighted_image.shape[2] != 3:
            print(f"Warning: Unexpected image shape {highlighted_image.shape}, converting to RGB")
            if len(highlighted_image.shape) == 2:
                highlighted_image = np.stack([highlighted_image] * 3, axis=2)
            elif highlighted_image.shape[2] == 1:
                highlighted_image = np.repeat(highlighted_image, 3, axis=2)
        
        # Debug: Print semantic map info
        unique_ids = np.unique(semantic_map)
        print(f"  Semantic map contains IDs: {unique_ids[:10]}{'...' if len(unique_ids) > 10 else ''}")
        
        # Find target object pixels in semantic map using actual semantic IDs
        target_mask = self._create_target_mask_from_semantics(semantic_map, semantic_scene)
        
        if target_mask is not None and np.any(target_mask):
            num_target_pixels = np.sum(target_mask)
            total_pixels = target_mask.size
            percentage = (num_target_pixels / total_pixels) * 100
            print(f"  Found {num_target_pixels} target pixels ({percentage:.2f}% of image)")
            
            # Create colored overlay (semi-transparent red)
            overlay_color = np.array([255, 0, 0], dtype=np.uint8)  # Red
            alpha = 0.4  # Increased transparency for better visibility
            
            # Apply overlay where target object is detected
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
        max_steps = 500  # Prevent infinite loops (increased from 500)
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
                
                # Capture final frame at waypoint
                observations = simulator.get_sensor_observations()
                semantic_scene = simulator.semantic_scene
                highlighted_frame = self.highlight_target_object(
                    observations["color_sensor"], 
                    observations["semantic_sensor"],
                    semantic_scene
                )
                frames.append(highlighted_frame)
                
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
            
            # Collect frames more frequently for smooth video
            render_interval = 5  # Collect every 5th frame instead of 30
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
            
            print(f"  Collected {len(waypoint_frames)} frames for waypoint {i+1}")
            
        # Step 3: Generate video
        print(f"\n Generating video with {len(all_frames)} frames...")
        
        if len(all_frames) < 10:
            print("  Warning: Very few frames collected. Video may be very short.")
            print("   Consider reducing render_interval or increasing navigation steps.")
        
        video_path = self.generate_video(all_frames)
        
        print(f"✅ Navigation completed! Video saved as: {video_path}")
        return video_path
        
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
        
        # Validate all frames have the same shape
        first_shape = frames[0].shape
        for i, frame in enumerate(frames):
            if frame.shape != first_shape:
                print(f"Warning: Frame {i} has different shape {frame.shape} vs {first_shape}")
                # Resize to match first frame
                frames[i] = cv2.resize(frame, (first_shape[1], first_shape[0]))
        
        # Video configuration
        height, width, layers = frames[0].shape
        video_name = f"{self.target_category}_navigation.mp4"
        
        # Try different codecs if mp4v fails
        codecs_to_try = ['mp4v', 'avc1', 'XVID', 'MJPG']
        video_writer = None
        
        for codec in codecs_to_try:
            try:
                fourcc = cv2.VideoWriter_fourcc(*codec)
                video_writer = cv2.VideoWriter(video_name, fourcc, self.video_fps, (width, height))
                if video_writer.isOpened():
                    print(f"Using codec: {codec}")
                    break
                else:
                    video_writer.release()
                    video_writer = None
            except Exception as e:
                print(f"Codec {codec} failed: {e}")
                continue
        
        if video_writer is None or not video_writer.isOpened():
            raise RuntimeError("Failed to initialize video writer with any codec")
        
        print(f"Writing {len(frames)} frames to {video_name} ({width}x{height} @ {self.video_fps}fps)...")
        
        # Write frames to video
        frames_written = 0
        for i, frame in enumerate(frames):
            try:
                # Ensure frame is RGB and convert to BGR for OpenCV
                if frame.shape[-1] == 3:  # RGB
                    bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:  # Already BGR or other format
                    bgr_frame = frame
                
                video_writer.write(bgr_frame)
                frames_written += 1
                
                if (i + 1) % 20 == 0:
                    print(f"   Written {i + 1}/{len(frames)} frames...")
                    
            except Exception as e:
                print(f"Error writing frame {i}: {e}")
                continue
        
        # Finalize video
        video_writer.release()
        
        if frames_written == 0:
            raise RuntimeError("No frames were successfully written to video")
        
        print(f"Video saved successfully: {video_name} ({frames_written} frames)")
        return video_name
