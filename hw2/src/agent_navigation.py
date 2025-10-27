#!/usr/bin/env python3
"""
Agent Navigation Implementation for Habitat Simulator

This module implements a navigation agent that follows a pre-computed RRT path,
highlights target objects during navigation, and records the journey as a video.

Based on the AGENTS.md task specification.
"""

import numpy as np
import cv2
import math
from typing import List, Tuple, Optional
from PIL import Image
import os
import sys

# Import RRT pathfinder for getting pre-computed paths
from rrt_pathfinder import RRTPathfinder

# Habitat imports
import habitat_sim
from habitat_sim.utils.common import d3_40_colors_rgb


class AgentNavigator:
    """
    Agent navigation implementation for following RRT paths in Habitat simulator.
    """
    
    def __init__(self, rrt_pathfinder: RRTPathfinder, target_category: str):
        """
        Initialize the agent navigator.
        
        Args:
            rrt_pathfinder: Initialized RRT pathfinder instance
            target_category: Target object category (e.g., "chair", "table")
        """
        self.pathfinder = rrt_pathfinder
        self.target_category = target_category
        self.target_color = rrt_pathfinder.target_categories.get(target_category, (255, 0, 0))
        
        # Agent configuration
        self.step_size = 0.25  # meters
        self.turn_angle = 10.0  # degrees
        self.agent_height = 1.5  # meters
        self.angle_threshold = 5.0  # degrees - threshold for alignment
        self.distance_threshold = 0.1  # meters - threshold for reaching waypoint
        
        # Video recording
        self.frames = []
        self.video_fps = 10
        
        # Navigation state
        self.current_waypoint_index = 0
        self.path_3d = []
        
    def configure_agent_actions(self, config):
        """
        Configure agent actions with appropriate step sizes.
        
        Args:
            config: Habitat configuration object
        """
        # Configure Habitat actions as per AGENTS.md specification
        config.TASK.ACTIONS.MOVE_FORWARD.MOTION_ARGS["step_size"] = self.step_size
        config.TASK.ACTIONS.TURN_LEFT.MOTION_ARGS["angle"] = self.turn_angle
        config.TASK.ACTIONS.TURN_RIGHT.MOTION_ARGS["angle"] = self.turn_angle
        
        print(f"Agent configuration:")
        print(f"  • Step size: {self.step_size} meters")
        print(f"  • Turn angle: {self.turn_angle} degrees")
        print(f"  • Agent height: {self.agent_height} meters")
        
    def pixel_to_world(self, pixel_coord: Tuple[int, int], depth_map: np.ndarray, 
                       camera_matrix: np.ndarray = None, camera_transform: np.ndarray = None) -> Tuple[float, float, float]:
        """
        Convert 2D pixel coordinates to 3D world coordinates using depth information.
        
        Args:
            pixel_coord: (x, y) pixel coordinates
            depth_map: Depth map from simulator
            camera_matrix: Camera intrinsic matrix (optional, uses transform if not provided)
            camera_transform: Camera extrinsic transform (optional)
            
        Returns:
            (x, y, z) world coordinates
        """
        x_pixel, y_pixel = pixel_coord
        
        # Get depth value at the pixel coordinate
        if depth_map is not None:
            depth_value = depth_map[y_pixel, x_pixel]
            
            if camera_matrix is not None and camera_transform is not None:
                # Use proper camera projection if available
                # Unproject the 2D point + depth to a 3D point in the camera frame
                fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
                cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
                
                camera_x = (x_pixel - cx) * depth_value / fx
                camera_y = (y_pixel - cy) * depth_value / fy
                camera_z = depth_value
                
                camera_point_3d = np.array([camera_x, camera_y, camera_z, 1.0])
                
                # Transform from camera frame to world frame
                world_point_3d = camera_transform @ camera_point_3d
                waypoint = (world_point_3d[0], world_point_3d[1], world_point_3d[2])
            else:
                # Fallback to coordinate transformation method
                world_x = x_pixel / self.pathfinder.transform_params['x_scale'] + self.pathfinder.transform_params['x_offset']
                world_z = y_pixel / self.pathfinder.transform_params['z_scale'] + self.pathfinder.transform_params['z_offset']
                waypoint = (world_x, self.agent_height, world_z)
        else:
            # Use the coordinate transformation from the pathfinder as fallback
            world_x = x_pixel / self.pathfinder.transform_params['x_scale'] + self.pathfinder.transform_params['x_offset']
            world_z = y_pixel / self.pathfinder.transform_params['z_scale'] + self.pathfinder.transform_params['z_offset']
            waypoint = (world_x, self.agent_height, world_z)
        
        return waypoint
        
    def transform_rrt_path_to_3d(self, pixel_path: List) -> List[Tuple[float, float, float]]:
        """
        Transform RRT path from 2D pixel coordinates to 3D world coordinates.
        
        Args:
            pixel_path: List of nodes from RRT pathfinder
            
        Returns:
            List of 3D waypoints
        """
        waypoints_3d = []
        
        for node in pixel_path:
            # Convert node to pixel coordinates
            pixel_coord = (int(node.x), int(node.y))
            
            # Transform to 3D world coordinates using coordinate transformation
            # Note: In real usage with depth data, pass the depth_map parameter
            world_coord = self.pixel_to_world(pixel_coord, None)
            waypoints_3d.append(world_coord)
            
        return waypoints_3d
        
    def calculate_heading_to_waypoint(self, current_pos: Tuple[float, float, float], 
                                    current_rotation: float, 
                                    target_pos: Tuple[float, float, float]) -> float:
        """
        Calculate the heading angle needed to face a target waypoint.
        
        Args:
            current_pos: Current agent position (x, y, z)
            current_rotation: Current agent rotation in degrees
            target_pos: Target waypoint (x, y, z)
            
        Returns:
            Angle difference in degrees (-180 to 180)
        """
        # Calculate vector to target
        dx = target_pos[0] - current_pos[0]
        dz = target_pos[2] - current_pos[2]
        
        # Calculate target heading in degrees
        target_heading = math.degrees(math.atan2(dx, dz))
        
        # Calculate angle difference
        angle_diff = target_heading - current_rotation
        
        # Normalize to [-180, 180] range
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
            
        return angle_diff
        
    def calculate_distance_to_waypoint(self, current_pos: Tuple[float, float, float], 
                                     target_pos: Tuple[float, float, float]) -> float:
        """
        Calculate 2D distance to target waypoint (ignoring Y axis).
        
        Args:
            current_pos: Current agent position (x, y, z)
            target_pos: Target waypoint (x, y, z)
            
        Returns:
            Distance in meters
        """
        dx = target_pos[0] - current_pos[0]
        dz = target_pos[2] - current_pos[2]
        return math.sqrt(dx*dx + dz*dz)
        
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
        
        # Find target object pixels in semantic map using actual semantic IDs
        target_mask = self._create_target_mask_from_semantics(semantic_map, semantic_scene)
        
        if target_mask is not None and np.any(target_mask):
            # Create colored overlay (semi-transparent red)
            overlay_color = np.array([255, 0, 0], dtype=np.uint8)  # Red
            alpha = 0.3  # Transparency
            
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
            target_waypoint: Target 3D coordinates
            
        Returns:
            List of RGB frames collected during navigation
        """
        frames = []
        
        print(f"Navigating to waypoint: ({target_waypoint[0]:.3f}, {target_waypoint[2]:.3f})")
        
        # Navigation loop with Habitat simulator
        max_steps = 50  # Prevent infinite loops
        step_count = 0
        
        while step_count < max_steps:
            # Get current agent state from Habitat
            agent_state = agent.get_state()
            current_pos = agent_state.position
            current_rotation = self._get_agent_rotation_y(agent_state.rotation)
            
            # Calculate heading to waypoint
            angle_diff = self.calculate_heading_to_waypoint(current_pos, current_rotation, target_waypoint)
            distance = self.calculate_distance_to_waypoint(current_pos, target_waypoint)
            
            # Check if we've reached the waypoint
            if distance < self.distance_threshold:
                print(f"  Reached waypoint in {step_count} steps")
                break
                
            # Turn towards waypoint if needed
            if abs(angle_diff) > self.angle_threshold:
                action = "turn_left" if angle_diff > 0 else "turn_right"
                print(f"  Step {step_count}: {action} (angle diff: {angle_diff:.1f}°)")
                
                # Execute turn action in Habitat
                observations = simulator.step(action)
                
            else:
                # Move forward
                action = "move_forward"
                print(f"  Step {step_count}: {action} (distance: {distance:.3f}m)")
                
                # Execute move action in Habitat
                observations = simulator.step(action)
            
            # Get semantic scene for proper target detection
            semantic_scene = simulator.semantic_scene
            
            # Highlight target object and collect frame
            highlighted_frame = self.highlight_target_object(
                observations["color_sensor"], 
                observations["semantic_sensor"],
                semantic_scene
            )
            frames.append(highlighted_frame)
            
            step_count += 1
            
        return frames
    
    def _get_agent_rotation_y(self, quaternion: habitat_sim.agent.AgentState.rotation) -> float:
        """
        Extract Y-axis rotation (yaw) from Habitat agent quaternion.
        
        Args:
            quaternion: Habitat agent rotation quaternion
            
        Returns:
            Y-axis rotation in degrees
        """
        # Convert quaternion to yaw angle in degrees
        import math
        
        # Extract yaw from quaternion (assuming w, x, y, z format)
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        
        # Calculate yaw (Y-axis rotation)
        yaw = math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))
        yaw_degrees = math.degrees(yaw)
        
        return yaw_degrees
        
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
        print("🤖 Starting agent navigation...")
        
        # Step 1: Transform pixel path to 3D waypoints
        print("📍 Transforming RRT path to 3D waypoints...")
        self.path_3d = self.transform_rrt_path_to_3d(pixel_path)
        print(f"   Generated {len(self.path_3d)} waypoints")
        
        if simulator is None or agent is None:
            print("⚠️ No Habitat simulator provided - using simulation mode")
            return self._run_navigation_simulation(pixel_path)
        
        print("🚀 Beginning navigation with Habitat...")
        
        # Step 2: Navigate through waypoints
        all_frames = []
        for i, waypoint in enumerate(self.path_3d):
            print(f"\n--- Waypoint {i+1}/{len(self.path_3d)} ---")
            
            # Navigate to waypoint and collect frames using Habitat
            waypoint_frames = self.navigate_to_waypoint(simulator, agent, waypoint)
            all_frames.extend(waypoint_frames)
            
        # Step 3: Generate video
        print(f"\n🎬 Generating video with {len(all_frames)} frames...")
        video_path = self.generate_video(all_frames)
        
        print(f"✅ Navigation completed! Video saved as: {video_path}")
        return video_path
    
    def _run_navigation_simulation(self, pixel_path: List) -> str:
        """
        Run navigation in simulation mode when Habitat is not available.
        
        Args:
            pixel_path: RRT path as list of nodes
            
        Returns:
            Path to generated video file
        """
        print("🔄 Running in simulation mode...")
        
        # Navigate through waypoints in simulation
        all_frames = []
        for i, waypoint in enumerate(self.path_3d):
            print(f"\n--- Waypoint {i+1}/{len(self.path_3d)} ---")
            
            # Simulate navigation to waypoint
            waypoint_frames = self._simulate_waypoint_navigation(waypoint, i)
            all_frames.extend(waypoint_frames)
            
        # Generate video
        print(f"\n🎬 Generating video with {len(all_frames)} frames...")
        video_path = self.generate_video(all_frames)
        
        print(f"✅ Simulation completed! Video saved as: {video_path}")
        return video_path
        
    def _simulate_waypoint_navigation(self, waypoint: Tuple[float, float, float], waypoint_index: int) -> List[np.ndarray]:
        """
        Simulate navigation to a waypoint when Habitat is not available.
        
        Args:
            waypoint: Target waypoint coordinates
            waypoint_index: Index of current waypoint
            
        Returns:
            List of simulated frames
        """
        print(f"  Simulating navigation to waypoint ({waypoint[0]:.3f}, {waypoint[2]:.3f})")
        
        frames = []
        num_frames = 5 + waypoint_index  # Varying number of frames per waypoint
        
        for frame_idx in range(num_frames):
            # Create dummy observations
            observations = self._create_dummy_observations()
            
            # Highlight target object
            highlighted_frame = self.highlight_target_object(
                observations["color_sensor"],
                observations["semantic_sensor"]
            )
            
            # Add waypoint info to frame (for visualization)
            self._add_waypoint_info_to_frame(highlighted_frame, waypoint_index, frame_idx)
            
            frames.append(highlighted_frame)
            
        print(f"  Generated {len(frames)} frames for waypoint {waypoint_index + 1}")
        return frames
        
    def _add_waypoint_info_to_frame(self, frame: np.ndarray, waypoint_idx: int, frame_idx: int):
        """
        Add waypoint information text overlay to frame.
        
        Args:
            frame: RGB frame to modify
            waypoint_idx: Current waypoint index
            frame_idx: Current frame index within waypoint
        """
        # Add text overlay
        text = f"Waypoint {waypoint_idx + 1} | Frame {frame_idx + 1}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add target category info
        target_text = f"Target: {self.target_category.upper()}"
        cv2.putText(frame, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
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
        
        print(f"📹 Writing {len(frames)} frames to {video_name}...")
        
        # Write frames to video
        for i, frame in enumerate(frames):
            # Convert RGB (from Habitat/simulation) to BGR (for OpenCV)
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(bgr_frame)
            
            if (i + 1) % 10 == 0:
                print(f"   Written {i + 1}/{len(frames)} frames...")
                
        # Finalize video
        video_writer.release()
        
        print(f"✅ Video saved successfully: {video_name}")
        return video_name


def main():
    """
    Main function to demonstrate the agent navigation system.
    """
    print("🤖 Agent Navigation Demo")
    print("=" * 50)
    
    # Note: Check for required files
    required_files = [
        'map.png',
        '../color_coding_semantic_segmentation_classes.xlsx',
        'coordinate_transformation.txt'
    ]
    
    missing_files = [f for f in required_files if not os.path.exists(f)]
    if missing_files:
        print(f"❌ Missing required files: {missing_files}")
        print("💡 Please ensure RRT pathfinding has been run first")
        return
    
    try:
        # Initialize RRT pathfinder
        print("🔧 Initializing RRT pathfinder...")
        pathfinder = RRTPathfinder(
            'map.png',
            '../color_coding_semantic_segmentation_classes.xlsx',
            'coordinate_transformation.txt'
        )
        
        # Select target category
        target_category = 'sofa'  # Can be changed to any available category
        print(f"🎯 Target category: {target_category.upper()}")
        
        # Get pre-computed path from RRT
        print("🗺️ Getting RRT path...")
        
        # For demo, use a predefined start point
        start_point = (300, 500)
        goal_point = pathfinder.find_goal_point(target_category)
        
        if not goal_point:
            print(f"❌ No {target_category} found in the map")
            return
            
        print(f"📍 Start: {start_point}, Goal: {goal_point}")
        
        # Run RRT pathfinding
        pixel_path = pathfinder.run_rrt(start_point, goal_point)
        
        if not pixel_path:
            print("❌ No path found by RRT algorithm")
            return
            
        print(f"✅ RRT path found with {len(pixel_path)} waypoints")
        
        # Initialize agent navigator
        navigator = AgentNavigator(pathfinder, target_category)
        
        # Run navigation simulation
        video_path = navigator.run_navigation(pixel_path)
        
        print(f"\n🎉 Demo completed successfully!")
        print(f"📽️ Video output: {video_path}")
        
    except Exception as e:
        print(f"❌ Error during navigation demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()