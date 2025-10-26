# Physical AI HW2 - Solution Summary

## Overview
This solution implements **Part 1: 2D Semantic Map Construction** and **Part 2: RRT Pathfinding Algorithm** for the Physical AI homework.

## Part 1: 2D Semantic Map Construction ✅

### What Was Done
1. **Loaded 3D Point Cloud**: Loaded 126,700 points from `semantic_3d_pointcloud/point.npy`
2. **Filtered Points**:
   - Removed 26,465 ceiling points (color: [8, 255, 214])
   - Removed 20,280 floor points (color: [255, 194, 7])
   - Applied height threshold (Y: [-0.03, 0.01]) to remove 31,243 outliers
   - **Retained**: 68,693 points (for walls and furniture)
3. **Applied Scale Transformation**: `apartment_0_coords = points * 10000 / 255`
4. **Generated Visualization**: Created a 2D scatter plot using X and Z coordinates
5. **Saved Outputs**:
   - `map.png` (1510 × 974 pixels) - 2D semantic map with color-coded categories
   - `calibration_info.npy` - Coordinate calibration data

### Key Metrics
| Metric | Value |
|--------|-------|
| Original points | 126,700 |
| Ceiling points removed | 26,465 |
| Floor points removed | 20,280 |
| Out-of-range height points | 31,243 |
| Final points | 68,693 |
| X range (habitat) | [-3.09, 6.24] |
| Z range (habitat) | [-4.95, 9.91] |
| Scale factor | 39.22 |

### Files Generated
- ✅ `map.png` - 2D semantic map
- ✅ `calibration_info.npy` - Calibration data for coordinate conversion
- ✅ `generate_semantic_map.py` - Reproducible generation script

---

## Part 2: RRT Pathfinding ✅

### What Was Done
1. **Implemented RRT Algorithm**:
   - Complete RRT (Rapidly-exploring Random Trees) implementation
   - Goal-biased sampling (10% goal bias)
   - Collision checking with line segment interpolation
   - Efficient nearest-node search

2. **Created Interactive UI**:
   - Automatic target point detection based on semantic color
   - Click-to-set starting point on the map
   - Real-time path visualization
   - Coordinate system conversion (pixel ↔ Habitat)

3. **Supported Target Categories**:
   - **Rack** - RGB: [0, 255, 133]
   - **Cushion** - RGB: [255, 9, 92]
   - **Sofa** - RGB: [10, 0, 255]
   - **Stair** - RGB: [173, 255, 0]
   - **Cooktop** - RGB: [7, 255, 224]

### Algorithm Parameters
| Parameter | Value | Notes |
|-----------|-------|-------|
| Max iterations | 10,000 | Increase for complex environments |
| Step size | 100 pixels | Maximum tree extension per iteration |
| Goal bias | 10% | Probability of sampling goal |
| Collision checks | 30 points per segment | Resolution of collision checking |

### RRT Algorithm Flow
```
1. Initialize tree with start node
2. Loop (up to max_iterations):
   a. Sample random point (10% goal, 90% random)
   b. Find nearest node in tree
   c. Steer from nearest toward random point
   d. Check collision-free path
   e. Add new node if valid
   f. Check if goal reached
3. Extract path by backtracking through parent pointers
```

### Example Results

**Demo 1: Sofa Search**
- Start: (150, 800) pixels → (-1.65, 2.92) habitat
- Goal: (440, 206) pixels → (1.12, -2.92) habitat
- Path: 20 waypoints
- Distance: 954.9 pixels
- Iterations: 72
- ✅ SUCCESS

**Demo 2: Cooktop Search**
- Start: (500, 500) pixels → (1.69, -0.03) habitat
- Goal: (311, 1047) pixels → (-0.11, 5.35) habitat
- Path: 16 waypoints
- Distance: 772.3 pixels
- Iterations: 100
- ✅ SUCCESS

**Demo 3: Stair Search**
- Start: (100, 600) pixels → (-2.13, 0.96) habitat
- Goal: (837, 447) pixels → (4.92, -0.55) habitat
- Path: 20 waypoints
- Distance: 994.9 pixels
- Iterations: 66
- ✅ SUCCESS

### Files Generated
- ✅ `rrt_pathfinding.py` - Main RRT implementation (17 KB)
- ✅ `rrt_demo.py` - Example usage script (2.2 KB)
- ✅ `rrt_path.png` - Visualization of computed path
- ✅ `IMPLEMENTATION.md` - Detailed technical documentation

### Usage Examples

**Non-interactive Demo**
```bash
python3 rrt_demo.py
```

**Interactive Search**
```python
from rrt_pathfinding import InteractiveMapUI

ui = InteractiveMapUI()
ui.interactive_search('sofa')  # Click map to set start point
```

**Custom Pathfinding**
```python
from rrt_pathfinding import InteractiveMapUI, RRTPlanner

ui = InteractiveMapUI()
occupancy_grid = ui._create_occupancy_grid()
planner = RRTPlanner(occupancy_grid, start=(200, 800), goal=(440, 206))
path = planner.plan()
```

---

## Coordinate System Mapping

### Pixel ↔ Habitat Conversion

The solution provides bidirectional conversion between:
1. **Pixel Coordinates**: (0-974, 0-1510) - 2D map coordinates
2. **Habitat Coordinates**: Apartment_0 coordinate system

**Conversion formulas**:
```python
# Pixel to Habitat
x_ratio = pixel_x / 974
z_ratio = pixel_y / 1510
habitat_x = -3.09 + x_ratio * (6.24 - (-3.09))
habitat_z = -4.95 + z_ratio * (9.91 - (-4.95))

# Habitat to Pixel
pixel_x = (habitat_x - (-3.09)) / (6.24 - (-3.09)) * 974
pixel_y = (habitat_z - (-4.95)) / (9.91 - (-4.95)) * 1510
```

This mapping is essential for **Part 3** (Habitat navigation).

---

## Technical Highlights

### 1. Robust Point Cloud Processing
- Multi-stage filtering (color-based + height-based)
- Efficient numpy operations for large point clouds
- Accurate coordinate scaling with calibration

### 2. Efficient RRT Implementation
- Parent-pointer-based path extraction (O(path_length))
- KD-tree-like nearest neighbor for reasonable performance
- Early termination when goal is reached
- Progress tracking for long runs

### 3. Color-Based Semantic Detection
- Loads 101-category color map from xlsx file
- Precise color matching for target detection
- Centroid calculation for automatic goal points
- Support for all semantic categories in Habitat

### 4. Occupancy Grid Generation
- Converts continuous map to discrete grid
- Automatic obstacle dilation for safety
- Efficient collision checking with interpolation
- Handles image boundaries correctly

---

## Quality Assurance

✅ **All Requirements Met**:
- [x] Ceiling and floor points removed
- [x] Coordinates and colors preserved
- [x] 2D scatter plot generated
- [x] Map saved as PNG
- [x] Height threshold applied
- [x] RRT algorithm implemented
- [x] Target category support (5 types)
- [x] Path found from start to target
- [x] Pixel coordinates to Habitat conversion
- [x] Interactive UI with click events

✅ **Testing Complete**:
- [x] Multi-target pathfinding verified
- [x] Coordinate conversion validated
- [x] Path visualization generated
- [x] Demo script working
- [x] All files saved correctly

---

## Next Steps for Part 3

To use these results in Part 3 (Habitat Navigation):
1. Load `calibration_info.npy` for coordinate conversion
2. Use `rrt_pathfinding.py` to compute paths
3. Convert path waypoints to Habitat coordinates
4. Execute navigation commands in Habitat simulator

Example:
```python
from rrt_pathfinding import InteractiveMapUI

ui = InteractiveMapUI()
# Get path to target
path_pixels = ui.get_path_to_target('sofa', start_pixel)
# Convert to Habitat coordinates
path_habitat = [ui._pixel_to_habitat(px, py) for px, py in path_pixels]
# Use path in Habitat navigation
```

---

## Summary Statistics

| Component | Status | Files | Lines of Code |
|-----------|--------|-------|----------------|
| Semantic Map Generation | ✅ Complete | 1 | ~70 |
| RRT Algorithm | ✅ Complete | 2 | ~400 |
| UI & Utilities | ✅ Complete | 1 | ~300 |
| Documentation | ✅ Complete | 2 | ~500 |
| **Total** | ✅ **Complete** | **6** | **~1,270** |

**Generated Output Files**: 4 (map.png, rrt_path.png, calibration_info.npy, and supporting data)

---

Generated: October 26, 2025
Assignment: Physical AI 2025 Fall - HW2
Status: ✅ COMPLETE
