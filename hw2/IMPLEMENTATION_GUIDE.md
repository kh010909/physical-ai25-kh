# Enhanced RRT Pathfinding Implementation

## Overview
This implementation provides a complete RRT (Rapidly-exploring Random Tree) pathfinding solution for 2D semantic maps with robot radius consideration and safety margins.

## Key Features

### 1. Robot Radius Consideration
- **Robot Radius**: Configurable robot size in pixels
- **Safety Margin**: Additional clearance beyond robot radius
- **Obstacle Dilation**: Obstacles are expanded by robot radius + safety margin
- **Collision Detection**: Fast collision checking using precomputed obstacle maps

### 2. Target Categories Supported
- `rack` - RGB(0, 255, 133)
- `cushion` - RGB(255, 9, 92) 
- `sofa` - RGB(10, 0, 255)
- `stair` - RGB(173, 255, 0)
- `cooktop` - RGB(7, 255, 224)

### 3. Coordinate Systems
- **Pixel Coordinates**: Used internally for pathfinding on the map image
- **Habitat Coordinates**: Transformed output for robot navigation in 3D world

## Implementation Details

### Core Classes

#### `Node`
- Represents points in the RRT tree
- Stores (x, y) coordinates and parent references
- Provides distance calculation methods

#### `RRTPathfinder`
- Main pathfinding class
- Handles map loading, obstacle processing, and path planning
- Configurable robot parameters

### Key Algorithms

#### Obstacle Map Creation
```python
def _create_obstacle_map(self):
    # 1. Identify obstacle pixels from semantic colors
    # 2. Create binary obstacle map
    # 3. Dilate obstacles by robot_radius + safety_margin
    # 4. Store for efficient collision checking
```

#### RRT Algorithm
```python
def run_rrt(self, start_point, goal_point):
    # 1. Validate start/goal points are collision-free
    # 2. Initialize tree with start node
    # 3. For max_iterations:
    #    a. Sample random point
    #    b. Find nearest tree node
    #    c. Extend tree toward sample
    #    d. Check if goal reached
    # 4. Reconstruct path from goal to start
```

#### Collision Detection
```python
def _is_path_clear(self, from_node, to_x, to_y):
    # 1. Sample points along path
    # 2. Check each point against dilated obstacle map
    # 3. Return True if entire path is clear
```

## Configuration Parameters

### Robot Parameters
- `robot_radius_pixels`: Physical robot radius (default: 20px)
- `safety_margin_pixels`: Additional safety clearance (default: 5px)

### RRT Parameters
- `step_size`: Maximum extension distance (default: 10px)
- `max_iterations`: Maximum algorithm iterations (default: 5000)
- `goal_tolerance`: Goal region radius (default: 15px)

## Usage Examples

### Basic Usage
```python
# Initialize pathfinder
pathfinder = RRTPathfinder('map.png', 'colors.xlsx', 'transform.txt')

# Set robot parameters
pathfinder.robot_radius_pixels = 15.0
pathfinder.safety_margin_pixels = 5.0

# Find path to target
goal_point = pathfinder.find_goal_point('sofa')
path = pathfinder.run_rrt(start_point, goal_point)

# Get world coordinates
habitat_coords = pathfinder.pixel_to_habitat_coordinates(path)
```

### Interactive Selection
```python
# User clicks on map to select start point
start_point = pathfinder.display_map_for_selection('rack')
pixel_path, habitat_coords = pathfinder.run_pathfinding('rack')
```

## File Structure

```
src/
├── rrt_pathfinder.py      # Main implementation
├── main_rrt.py           # Interactive application
├── demo_rrt.py           # Demo with predefined points
├── test_enhanced_rrt.py  # Enhanced testing with robot radius
├── map.png               # Semantic map image
└── coordinate_transformation.txt  # Pixel-to-world transforms
```

## Output Files

### Path Visualizations
- `path_to_{category}.png` - Path visualization on map
- `obstacle_map.png` - Dilated obstacle map visualization

### Coordinate Data
- Pixel coordinates: List of (x, y) points in image space
- Habitat coordinates: List of (x, z) points in world space

## Performance Characteristics

### Typical Results
- **Success Rate**: High for reachable targets with adequate clearance
- **Path Quality**: Maintains safe distances from obstacles
- **Computation Time**: 1-5 seconds for most scenarios
- **Path Length**: Varies based on robot size and obstacle density

### Robot Size Impact
- **Small robots** (r < 10px): Can navigate tight spaces, faster convergence
- **Medium robots** (r = 15-20px): Balanced performance, good clearance
- **Large robots** (r > 25px): May fail in constrained environments

## Safety Considerations

1. **Minimum Clearance**: Always maintains robot_radius + safety_margin from obstacles
2. **Path Validation**: All path segments are collision-checked
3. **Start/Goal Validation**: Ensures endpoints are in safe locations
4. **Obstacle Dilation**: Conservative approach prevents robot-obstacle contact

## Limitations

1. **Holonomic Assumption**: Does not consider robot dynamics or turning radius
2. **2D Planning**: No consideration of height variations or 3D obstacles
3. **Static Environment**: Assumes obstacles do not move during planning
4. **Local Planner**: May get trapped in local minima for complex environments

## Future Enhancements

1. **RRT***: Implement optimal variant for improved path quality
2. **Dynamic Windows**: Add velocity and acceleration constraints
3. **Multi-Query**: Reuse tree for multiple planning queries
4. **Adaptive Sampling**: Bias sampling toward goal or difficult regions