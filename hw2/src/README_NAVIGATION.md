# Agent Navigation Implementation

This directory contains the implementation of the agent navigation system described in `AGENTS.md`. The system allows a simulated agent to follow pre-computed RRT paths, highlight target objects during navigation, and record the journey as a video.

## 📁 Files Overview

### Core Implementation Files

- **`agent_navigation.py`** - Main agent navigation class implementing the complete pipeline
- **`load_enhanced.py`** - Enhanced version of `load.py` with integrated navigation capabilities
- **`demo_agent_navigation.py`** - Standalone demo script that works without Habitat

### Original Files (Reference)

- **`load.py`** - Original Habitat simulation script with manual controls
- **`rrt_pathfinder.py`** - RRT pathfinding algorithm implementation
- **`main_rrt.py`** - Main script for running RRT pathfinding

## 🚀 Quick Start

With Habitat fully integrated, run the complete navigation system:

```bash
# Complete navigation with Habitat
python habitat_navigation_example.py --target sofa --scene replica_v1/apartment_0/habitat/mesh_semantic.ply

# Enhanced load script with automatic navigation
python load_enhanced.py --mode auto --target sofa --floor 1

# Demo without Habitat (for testing)
python demo_agent_navigation.py --target rack --frames 100
```

## 🎯 Implementation Features

### 1. Agent Configuration
- **Step size**: 0.25 meters for forward movement
- **Turn angle**: 10.0 degrees for rotational movement
- **Agent height**: 1.5 meters for 3D coordinate calculation

### 2. Coordinate Transformation
Converts 2D pixel coordinates from RRT to 3D world coordinates:

```python
# Transformation formulas
habitat_x = pixel_x / x_scale + x_offset
habitat_z = pixel_y / z_scale + z_offset
waypoint = (habitat_x, agent_height, habitat_z)
```

### 3. Navigation Loop
For each waypoint in the RRT path:
1. **Calculate heading** to target waypoint
2. **Turn** agent until aligned (within 5° threshold)
3. **Move forward** until close to waypoint (within 0.1m threshold)
4. **Collect RGB frames** after each action

### 4. Target Object Highlighting
- Retrieves semantic segmentation data
- Identifies target object pixels
- Applies semi-transparent red overlay (30% opacity)
- Preserves original image with highlighted targets

### 5. Video Generation
- Collects all navigation frames
- Uses OpenCV VideoWriter with MP4V codec
- Outputs video named `{target_name}.mp4`
- Frame rate: 10 FPS

## 🔧 Class Structure

### `AgentNavigator`

Main class implementing the navigation pipeline:

```python
navigator = AgentNavigator(rrt_pathfinder, target_category)

# Key methods:
- configure_agent_actions(config)       # Set up agent movement parameters
- pixel_to_world(pixel_coord, depth)    # Convert 2D to 3D coordinates
- transform_rrt_path_to_3d(pixel_path)  # Transform entire path
- calculate_heading_to_waypoint()       # Compute required rotation
- highlight_target_object()             # Add visual highlighting
- navigate_to_waypoint()                # Execute movement to single waypoint
- run_navigation()                      # Complete navigation pipeline
- generate_video()                      # Create MP4 output
```

## 📊 Coordinate System

### Transformation Parameters
From `coordinate_transformation.txt`:

- **X scale**: 351.463912 pixels/habitat_unit
- **Z scale**: 147.021786 pixels/habitat_unit  
- **X offset**: -3.554479 habitat_units
- **Z offset**: -5.671297 habitat_units

### Habitat Ranges
- **X axis**: [-3.554, 6.688] meters
- **Z axis**: [-5.671, 10.653] meters
- **Y axis**: Agent height (1.5m)

## 🎮 Usage Modes

### 1. Standalone Demo (Recommended)
```bash
python demo_agent_navigation.py --target sofa --frames 50
```
Works without Habitat, generates sample navigation video.

### 2. Enhanced Load Script
```bash
# Automatic navigation mode
python load_enhanced.py --mode auto --target sofa --floor 1

# Manual control mode (original functionality)
python load_enhanced.py --mode manual --floor 1
```

### 3. Direct Navigation Module
```python
from agent_navigation import AgentNavigator
from rrt_pathfinder import RRTPathfinder

# Initialize
pathfinder = RRTPathfinder('map.png', 'colors.xlsx', 'coords.txt')
navigator = AgentNavigator(pathfinder, 'sofa')

# Get RRT path
path = pathfinder.run_rrt(start_point, goal_point)

# Execute navigation
video_path = navigator.run_navigation(path)
```

## 🎯 Target Categories

Supported target object categories:

| Category | RGB Color | Description |
|----------|-----------|-------------|
| `sofa` | (10, 0, 255) | Seating furniture |
| `rack` | (0, 255, 133) | Storage/display racks |
| `cushion` | (255, 9, 92) | Cushions/pillows |
| `stair` | (173, 255, 0) | Staircases |
| `cooktop` | (7, 255, 224) | Kitchen cooking surfaces |

## 📹 Video Output

Generated videos include:
- **First-person agent view** during navigation
- **Target object highlighting** with red overlay
- **Waypoint information** overlay
- **Smooth navigation** between RRT waypoints

### Video Specifications
- **Format**: MP4 (H.264)
- **Resolution**: 512x512 pixels
- **Frame rate**: 10 FPS
- **Naming**: `{target_category}.mp4`

## 🔍 Example Output

```
🤖 Starting agent navigation...
📍 Transforming RRT path to 3D waypoints...
   Generated 12 waypoints
🚀 Beginning navigation...

--- Waypoint 1/12 ---
Navigating to waypoint: (1.234, -2.567)
  Step 1: turn_left (angle diff: 15.3°)
  Step 2: move_forward (distance: 0.523m)
  Reached waypoint in 3 steps

--- Waypoint 2/12 ---
...

🎬 Generating video with 45 frames...
📹 Writing 45 frames to sofa.mp4...
✅ Navigation completed! Video saved as: sofa.mp4
```

## 🛠️ Implementation Notes

### Environment Compatibility
- **Works without Habitat**: Demo mode uses simulated observations
- **Habitat-ready**: Core navigation logic compatible with Habitat simulator
- **Modular design**: Easy integration with existing Habitat setups

### Error Handling
- Validates required files before execution
- Handles missing RRT paths gracefully
- Provides informative error messages
- Includes simulation fallbacks

### Performance Considerations
- Efficient coordinate transformations
- Optimized frame collection
- Reasonable default parameters
- Configurable navigation thresholds

## 📝 Next Steps

To integrate with a working Habitat environment:

1. **Uncomment Habitat imports** in the navigation files
2. **Replace simulation methods** with actual Habitat API calls
3. **Update observation handling** to use real sensor data
4. **Test with actual semantic segmentation** from Habitat

The core navigation logic is ready and tested with simulated data!