import json
import os
from typing import List, Tuple, Dict, Optional
import numpy as np


def pixel_to_habitat_coords(
    pixel_x: int, pixel_y: int, x_limit: float = None, y_limit: float = None, data_bounds: dict = None
) -> Tuple[float, float]:
    """
    Convert pixel coordinates to Habitat world coordinates.
    Uses COMPUTED transformation parameters from point cloud data.
    
    CRITICAL REQUIREMENT:
    This function expects pixel coordinates from a TRANSFORMED map:
    1. Original map → Flip horizontally (cv2.flip(map, 1))
    2. Flipped map → Rotate 90° clockwise (cv2.rotate(map, cv2.ROTATE_90_CLOCKWISE))
    
    After transformation:
    - Map X-axis (horizontal) → Habitat Z-axis (forward/back)
    - Map Y-axis (vertical) → Habitat X-axis (left/right, inverted)
    
    This transformation is applied in:
    - test_interactive_transformation.py (for verification)
    - rrt_pathfinder.py (for path planning)
    - Any other tool that needs to map between 2D and 3D coordinates

    Args:
        pixel_x: X coordinate in TRANSFORMED map (horizontal)
        pixel_y: Y coordinate in TRANSFORMED map (vertical)
        x_limit: X axis limit (unused, for compatibility)
        y_limit: Y axis limit (unused, for compatibility)
        data_bounds: Dictionary containing coordinate bounds (unused, for compatibility)

    Returns:
        Tuple of (habitat_x, habitat_z) coordinates
    """
    # Computed parameters from point cloud (Floor 1)
    habitat_x_min = -3.089056
    habitat_x_max = 6.220782
    habitat_z_min = -4.929266
    habitat_z_max = 9.912446
    
    # Transformed map dimensions (after flip + rotate 90° CW)
    TRANSFORMED_WIDTH = 1848
    TRANSFORMED_HEIGHT = 1159
    
    # Normalize pixel coordinates
    norm_x = pixel_x / TRANSFORMED_WIDTH
    norm_y = pixel_y / TRANSFORMED_HEIGHT
    
    # Map to Habitat coordinates
    habitat_z = habitat_z_min + norm_x * (habitat_z_max - habitat_z_min)
    habitat_x = habitat_x_max - norm_y * (habitat_x_max - habitat_x_min)  # Inverted because image Y increases downward

    return habitat_x, habitat_z


def load_map_limits(pointcloud_path: str) -> Tuple[float, float, float, dict]:
    """
    Load map limits and data bounds from point cloud data.

    Args:
        pointcloud_path: Path to the point cloud data directory

    Returns:
        Tuple of (x_limit, y_limit, image_size, data_bounds)
    """
    # Load point cloud data
    color_file = os.path.join(pointcloud_path, 'color0255.npy')
    point_file = os.path.join(pointcloud_path, 'point.npy')

    if not os.path.exists(color_file) or not os.path.exists(point_file):
        raise FileNotFoundError(f"Point cloud files not found in {pointcloud_path}")

    colors = np.load(color_file)
    points = np.load(point_file)

    # Calculate bounds
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()
    z_min, z_max = points[:, 2].min(), points[:, 2].max()

    data_bounds = {
        'x': (x_min, x_max),
        'y': (y_min, y_max),
        'z': (z_min, z_max)
    }

    # Use image dimensions from coordinate_transformation.txt
    image_width = 3600
    image_height = 2400

    return x_max - x_min, z_max - z_min, (image_width, image_height), data_bounds


def apply_semantic_highlighting(
    rgb_image: np.ndarray,
    semantic_obs: np.ndarray,
    target_name: str,
    name_to_instance_ids: Dict[str, List[int]],
    is_depth: bool = False
) -> np.ndarray:
    """
    Apply semantic highlighting to RGB image for target object.

    Args:
        rgb_image: RGB image array
        semantic_obs: Semantic segmentation observation
        target_name: Name of the target object to highlight
        name_to_instance_ids: Mapping from names to instance IDs
        is_depth: Whether this is a depth image

    Returns:
        RGB image with target object highlighted
    """
    highlighted_image = rgb_image.copy()

    if target_name not in name_to_instance_ids:
        print(f"Warning: Target '{target_name}' not found in semantic mapping")
        return highlighted_image

    target_instance_ids = name_to_instance_ids[target_name]

    # Create mask for target object pixels
    target_mask = np.isin(semantic_obs, target_instance_ids)

    if not np.any(target_mask):
        print(f"Warning: No pixels found for target '{target_name}'")
        return highlighted_image

    # Apply highlighting (semi-transparent overlay)
    overlay_color = np.array([255, 0, 0], dtype=np.uint8)  # Red
    alpha = 0.3  # Transparency

    # Apply overlay where target is present
    highlighted_image[target_mask] = (
        (1 - alpha) * highlighted_image[target_mask] +
        alpha * overlay_color
    ).astype(np.uint8)

    return highlighted_image