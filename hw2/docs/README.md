# Main.py Documentation

## Overview

`main.py` is the primary entry point for the Enhanced Habitat Agent Navigation system. It provides both manual and automatic navigation capabilities in a Habitat simulator environment, with support for RRT path planning and target object highlighting.

## Table of Contents

- [Features](#features)
- [Dependencies](#dependencies)
- [Configuration](#configuration)
- [Function Reference](#function-reference)
- [Usage Examples](#usage-examples)
- [Command Line Arguments](#command-line-arguments)
- [Data Collection](#data-collection)
- [Navigation Modes](#navigation-modes)

## Features

### Core Capabilities
- **Manual Navigation**: Real-time keyboard-controlled agent movement
- **Automatic Navigation**: RRT-based pathfinding to target objects
- **Multi-Floor Support**: Floor 1 and Floor 2 navigation
- **Target Object Highlighting**: Visual highlighting of target objects during navigation
- **Multi-Sensor Data Collection**: RGB, depth, and semantic sensor data
- **Video Generation**: Automatic video creation from collected frames
- **Interactive Starting Point Selection**: Click-to-select starting positions

### Supported Target Objects
- Sofa
- Rack
- Cushion
- Stair
- Cooktop

## Dependencies

```python
import argparse
import os
import shutil
import cv2
import habitat_sim
import numpy as np
from PIL import Image
from habitat_sim.utils.common import d3_40_colors_rgb
from agent_navigation import AgentNavigator
from rrt_pathfinder import RRTPathfinder
```

### External Requirements
- Habitat-Sim
- OpenCV (cv2)
- NumPy
- PIL (Pillow)
- Matplotlib (for pathfinding visualization)
- Pandas (for color coding data)

## Configuration

### Scene Configuration
```python
test_scene = "../replica_v1/apartment_0/habitat/mesh_semantic.ply"

sim_settings = {
    "scene": test_scene,
    "default_agent": 0,
    "sensor_height": 1.5,
    "width": 512,
    "height": 512,
    "sensor_pitch": 0,
}
```

### Agent Action Configuration
- **Move Forward**: 0.25 meters per step
- **Turn Left/Right**: 10 degrees per step
- **Sensor Resolution**: 512x512 pixels
- **Sensor Height**: 1.5 meters

## Function Reference

### Core Functions

#### `main()`
**Purpose**: Entry point for the application with argument parsing and mode selection.

**Features**:
- Command-line argument parsing
- Habitat simulator initialization
- Mode selection (manual/automatic)
- Data directory setup
- Debug view display

#### `make_simple_cfg(settings)`
**Purpose**: Creates Habitat simulator configuration with multi-sensor setup.

**Parameters**:
- `settings` (dict): Simulator configuration parameters

**Returns**: `habitat_sim.Configuration` object

**Sensors Configured**:
- RGB Camera (`color_sensor`)
- Depth Sensor (`depth_sensor`)
- Semantic Segmentation (`semantic_sensor`)

### Navigation Functions

#### `navigate_and_see(action, data_root, auto_mode, target_category)`
**Purpose**: Core navigation function supporting both manual and automatic modes.

**Parameters**:
- `action` (str): Action to perform ("move_forward", "turn_left", "turn_right")
- `data_root` (str): Directory path for saving data
- `auto_mode` (bool): Whether running in automatic navigation mode
- `target_category` (str): Target object category for highlighting

**Features**:
- Real-time sensor data collection
- Target object highlighting (automatic mode)
- Image display and saving
- Camera pose tracking

#### `run_automatic_navigation(target_category, data_root, interactive_start, floor)`
**Purpose**: Orchestrates the complete automatic navigation pipeline.

**Parameters**:
- `target_category` (str): Target object to navigate to
- `data_root` (str): Data collection directory
- `interactive_start` (bool): Enable interactive starting point selection
- `floor` (int): Floor number (1 or 2)

**Returns**: `bool` - Success status

**Pipeline**:
1. Initialize RRT pathfinder
2. Select starting point (interactive or default)
3. Compute RRT path to target
4. Execute navigation with Habitat
5. Generate navigation video

#### `select_target_category()`
**Purpose**: Interactive menu for target category selection.

**Returns**: `str` - Selected target category

**Features**:
- Dynamic category listing with RGB colors
- Input validation
- Error handling for missing files

### Utility Functions

#### `transform_rgb_bgr(image)`
**Purpose**: Convert RGB image to BGR format for OpenCV display.

**Parameters**:
- `image` (np.ndarray): RGB image array

**Returns**: `np.ndarray` - BGR image array

#### `transform_depth(image)`
**Purpose**: Convert depth image to visualization format.

**Parameters**:
- `image` (np.ndarray): Raw depth data

**Returns**: `np.ndarray` - Normalized depth image (0-255)

#### `transform_semantic(semantic_obs)`
**Purpose**: Convert semantic segmentation to colored visualization.

**Parameters**:
- `semantic_obs` (np.ndarray): Semantic segmentation data

**Returns**: `np.ndarray` - Colored semantic image

## Usage Examples

### Automatic Navigation
```bash
# Simple automatic navigation based on default settings
python main.py 
```



## Command Line Arguments

| Argument | Short | Type | Default | Description |
|----------|-------|------|---------|-------------|
| `--floor` | `-f` | int | 1 | Floor number (1 or 2) |
| `--mode` | `-m` | str | 'auto' | Control mode ('manual' or 'auto') |
| `--target` | `-t` | str | None | Target object category |
| `--interactive` | | flag | True | Enable interactive starting point selection |
| `--no-interactive` | | flag | False | Disable interactive starting point selection |

## Data Collection

### Directory Structure
```
data_collection/
├── first_floor/
│   ├── rgb/           # RGB images
│   ├── depth/         # Depth images
│   └── semantic/      # Semantic segmentation images
├── second_floor/
│   ├── rgb/
│   ├── depth/
│   └── semantic/
└── GT_pose.npy        # Camera poses
```

### File Formats
- **Images**: PNG format (512x512 resolution)
- **Poses**: NumPy array with 7D poses (x, y, z, qw, qx, qy, qz)
- **Videos**: MP4 format (10 FPS)

### Frame Naming Convention
- RGB: `rgb/{frame_number}.png`
- Depth: `depth/{frame_number}.png`
- Semantic: `semantic/{frame_number}.png`

## Navigation Modes

### Manual Mode
**Controls**:
- `W`: Move forward
- `A`: Turn left
- `D`: Turn right
- `F`: Finish and quit

**Features**:
- Real-time keyboard control
- Live sensor data display
- Frame-by-frame data collection

### Automatic Mode
**Features**:
- RRT-based path planning
- Target object highlighting
- Automated navigation execution
- Video generation
- Interactive or programmatic starting point selection

**Process Flow**:
1. **Initialization**: Load map and color coding data
2. **Target Selection**: Choose target object category
3. **Starting Point**: Interactive selection or default position
4. **Path Planning**: RRT algorithm computes optimal path
5. **Navigation**: Agent follows path with obstacle avoidance
6. **Data Collection**: Continuous sensor data recording
7. **Video Generation**: Create navigation video with highlighting

## Floor Configuration

### Floor 1 (Y = -1.5)
- Located below the reference level
- Agent positioned at Y = -1.5 meters
- Suitable for lower level navigation

### Floor 2 (Y = 0.0)
- Reference level
- Agent positioned at Y = 0.0 meters
- Default floor configuration

## Error Handling

### Common Issues
1. **Missing Files**: Automatic detection of required files
2. **Navigation Failures**: Graceful handling of pathfinding errors
3. **Sensor Errors**: Robust sensor data processing
4. **Invalid Input**: User input validation and error messages

### Required Files
- `map.png`: Semantic map for navigation, run "python main_semantic_map.py" to generate
- `../color_coding_semantic_segmentation_classes.xlsx`: Object color mapping
- `../replica_v1/apartment_0/habitat/mesh_semantic.ply`: 3D scene mesh

## Integration with Other Modules

### AgentNavigator
- Provides coordinate transformation between pixel and world space
- Handles target object highlighting
- Manages navigation execution

### RRTPathfinder
- Implements Rapidly-exploring Random Tree algorithm
- Provides interactive map selection
- Handles obstacle detection and collision checking

## Version Information
- **Created**: October 28, 2025
- **Language**: Python 3.7+
- **Dependencies**: Habitat-Sim, OpenCV, NumPy
- **License**: Project-specific

## Related Documentation
- [README.md](../README.md): Project overview and setup instructions