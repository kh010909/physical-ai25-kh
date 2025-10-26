# Part 1 & 2: 2D Semantic Map and RRT Pathfinding

This directory contains solutions for:
- **Part 1**: Building a 2D semantic map from 3D point cloud data
- **Part 2**: Implementing RRT (Rapidly-exploring Random Trees) algorithm for pathfinding

## Part 1: 2D Semantic Map Construction

### Overview
Generates a 2D semantic map from a 3D point cloud of the first floor of apartment_0.

### Files
- `generate_semantic_map.py` - Main script to generate the 2D semantic map
- `map.png` - Generated 2D semantic map
- `calibration_info.npy` - Coordinate calibration data for Habitat conversion

### Process
1. **Load 3D Point Cloud**: Loads `semantic_3d_pointcloud/point.npy` and color data
2. **Filter Points**:
   - Removes ceiling points (color: [8, 255, 214])
   - Removes floor points (color: [255, 194, 7])
   - Applies height threshold filter (Y: [-0.03, 0.01])
3. **Scale Transformation**: Applies `apartment_0 = points * 10000 / 255`
4. **Generate Visualization**: Creates scatter plot using X and Z coordinates
5. **Save Output**: Saves as `map.png` with semantic colors preserved

### Running
```bash
python3 generate_semantic_map.py
```

### Output
- **map.png** (1510 x 974 pixels): 2D top-down view colored by semantic category
- **calibration_info.npy**: Contains:
  - `scale_factor`: 39.216 (points to habitat conversion)
  - `x_min`, `x_max`: X coordinate range in habitat space
  - `z_min`, `z_max`: Z coordinate range in habitat space

## Part 2: RRT Pathfinding

### Overview
Implements the RRT algorithm to find navigable paths from a starting point to target object categories.

### Target Categories
- **rack** - RGB: [0, 255, 133]
- **cushion** - RGB: [255, 9, 92]
- **sofa** - RGB: [10, 0, 255]
- **stair** - RGB: [173, 255, 0]
- **cooktop** - RGB: [7, 255, 224]

### Files
- `rrt_pathfinding.py` - Main RRT implementation
- `rrt_demo.py` - Demo script showing how to use RRT
- `rrt_path.png` - Generated path visualization
- `color_coding_semantic_segmentation_classes.xlsx` - Category to color mapping

### Components

#### RRTPlanner Class
Implements the RRT algorithm:
- **Distance calculation**: Euclidean distance between points
- **Nearest node search**: Finds closest node in the tree
- **Steer function**: Moves toward random samples with bounded step size
- **Collision checking**: Verifies path segment is obstacle-free
- **Path extraction**: Backtracks from goal to start through parent pointers

**Algorithm Parameters**:
- `max_iterations`: 10,000 (can be increased for more complex environments)
- `step_size`: 100 pixels (affects how much the tree grows per iteration)
- `goal_bias`: 10% (probability of sampling goal directly)

#### InteractiveMapUI Class
Provides interactive interface for pathfinding:
- Loads semantic map and color categories
- Creates occupancy grid from the map
- Finds target points automatically
- Converts between pixel and Habitat coordinates
- Provides click-based UI for setting start points

### Coordinate System Mapping

The UI provides bidirectional conversion between:
1. **Pixel Coordinates** (map.png): (0-974, 0-1510) pixels
2. **Habitat Coordinates** (apartment_0): Uses calibration data

Conversion formulas:
```
x_ratio = pixel_x / width
z_ratio = pixel_y / height
habitat_x = x_min + x_ratio * (x_max - x_min)
habitat_z = z_min + z_ratio * (z_max - z_min)
```

### Usage

#### 1. Non-Interactive Demo
```bash
python3 rrt_demo.py
```
This runs an example pathfinding:
- Start point: (200, 800) in pixel coordinates
- Target: "sofa"
- Output: Path visualization and waypoint coordinates

#### 2. Interactive Mode
```python
from rrt_pathfinding import InteractiveMapUI

ui = InteractiveMapUI()
ui.interactive_search('sofa')  # or any target: rack, cushion, stair, cooktop
```

When running interactively:
1. A map window opens with the target highlighted in green
2. Click on the map to set your starting point
3. RRT automatically computes the path to the target
4. Path is displayed and printed

### Example Output
```
Path waypoints:
Pixel Coordinates -> Habitat Coordinates
  0: ( 200,  800) -> (  -1.18,    2.92)
  1: ( 219,  754) -> (  -1.00,    2.47)
  2: ( 254,  718) -> (  -0.66,    2.11)
  ...
  13: ( 440,  206) -> (   1.12,   -2.92)
```

### How It Works

1. **Occupancy Grid Creation**: 
   - Colored pixels (obstacles) are marked as True
   - White/light pixels (free space) are marked as False
   - Grid is padded for safety buffer

2. **Path Planning**:
   - Randomly samples points in the environment
   - 10% of samples are the goal (goal biasing)
   - Extends tree toward random samples by max `step_size`
   - Checks collision with occupancy grid
   - Terminates when goal is reached

3. **Path Extraction**:
   - Backtracks from goal node through parent pointers
   - Returns waypoints in order from start to goal
   - Converts pixel coordinates to Habitat coordinates

### Key Features

✅ **Robust RRT Implementation**
- Goal biasing for faster convergence
- Collision checking with line segment interpolation
- Efficient nearest-node search

✅ **Coordinate System Integration**
- Automatic conversion between pixel and Habitat coordinates
- Calibration information saved and reused
- Support for Part 3 navigation in Habitat simulator

✅ **Interactive UI**
- Visual highlighting of target objects
- Click-to-set starting point
- Automatic target point detection
- Real-time path visualization

✅ **Multiple Target Support**
- Pre-defined target categories (5 types)
- Automatic color detection and point finding
- Extensible to add more categories

### Notes

- The algorithm may occasionally fail to find a path if the environment is complex
- Increasing `max_iterations` or decreasing `step_size` can improve success rate
- The occupancy grid detection may mark some narrow passages as obstacles
- For actual Habitat navigation, additional collision checking with the 3D environment may be needed

### Future Improvements

- RRT* for path optimization
- Bidirectional RRT for faster planning
- Custom occupancy grid tuning per environment
- Path smoothing post-processing
- Sampling-based planning with different distributions
