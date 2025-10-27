# 2D Semantic Map Generator

## Overview

This project implements a 2D semantic map generator that projects a 3D semantic point cloud onto a top-down (X-Z plane) view. The system filters out unwanted elements like ceilings, floors, and objects at specific heights to create clean 2D representations suitable for navigation and spatial analysis.

## Implementation Summary

### Core Components

#### 1. `SemanticMapGenerator` Class (`semantic_map_generator.py`)
The main class that handles the complete pipeline for generating 2D semantic maps.

**Key Features:**
- Loads 3D point cloud data and color mappings
- Applies semantic filtering (removes ceiling/floor points)
- Implements height-based filtering for fine-grained control
- Generates clean 2D scatter plots without axes or labels
- Calculates and saves coordinate transformation parameters

#### 2. Main Execution Script (`main.py`)
Entry point that allows manual adjustment of filtering parameters.

**Configuration Options:**
- Manual absolute height ranges
- Automatic percentile-based filtering
- Conservative filtering presets

#### 3. Alternative Runner (`run_with_custom_height.py`)
Provides multiple preset configurations and detailed height statistics.

### Data Processing Pipeline

#### Step 1: Data Loading
- **Input Files:**
  - `point.npy`: 3D coordinates (126,700 points)
  - `color01.npy`: RGB values in [0,1] range
  - `color0255.npy`: RGB values in [0,255] range
  - `color_coding_semantic_segmentation_classes.xlsx`: Semantic class mappings

- **Coordinate Scaling:** `apartment_0_coordinates = point_array * 10000.0 / 255.0`

#### Step 2: Semantic Filtering
- **Ceiling Color:** RGB(8, 255, 214)
- **Floor Color:** RGB(255, 194, 7)
- **Removal:** All points matching ceiling/floor colors are filtered out

#### Step 3: Height Filtering
- **Purpose:** Remove mattresses, door frames, and other height-specific objects
- **Method:** Filter points based on Y-coordinate (height) values
- **Options:**
  - Manual absolute ranges (e.g., Y ∈ [-1.2, -0.2])
  - Automatic percentile-based (10th to 90th percentile)
  - Custom presets

#### Step 4: 2D Projection
- **Mapping:** X-coordinate → Plot X-axis, Z-coordinate → Plot Y-axis
- **Visualization:** Scatter plot with original RGB colors
- **Output:** Clean image without axes, labels, or borders

#### Step 5: Coordinate Transformation
- **Purpose:** Enable conversion between pixel coordinates and Habitat coordinates
- **Parameters:** Scaling factors, offsets, image dimensions
- **Output:** Saved to `coordinate_transformation.txt`

### Output Files

#### Generated Files (in `src/` directory):
1. **`map.png`** - Clean 2D semantic map image
2. **`coordinate_transformation.txt`** - Transformation parameters for pixel↔Habitat coordinate conversion

#### Transformation Parameters:
- Habitat coordinate ranges (X, Z)
- Image dimensions (width, height in pixels)
- Scaling factors (pixels per Habitat unit)
- Offset values for coordinate conversion

### Filtering Results

#### Example Filtering Performance:
- **Original Points:** 126,700
- **Semantic Filtering:** Removes ~46,745 ceiling/floor points
- **Height Filtering:** Removes additional points based on height range
- **Final Output:** Typically 60,000-80,000 filtered points

#### Height Statistics (Example):
- **Full Y Range:** [-1.775, 2.365]
- **Mean Y:** -0.264
- **Current Filter:** Y ∈ [-1.2, -0.2] (floor-level objects only)

### Usage Instructions

#### Basic Usage:
```bash
cd src/
python main.py
```

#### Customization:
1. **Edit Height Range:** Modify `HEIGHT_MIN` and `HEIGHT_MAX` in `main.py`
2. **Use Presets:** Run `python run_with_custom_height.py` with different configurations
3. **View Statistics:** Check height distribution before filtering

#### Configuration Examples:
```python
# Floor-level objects only
HEIGHT_MIN = -1.2
HEIGHT_MAX = -0.2

# Human-height objects
HEIGHT_MIN = -0.8
HEIGHT_MAX = 1.8

# Conservative middle range
HEIGHT_MIN = -0.5
HEIGHT_MAX = 0.5
```

### Technical Requirements

#### Dependencies:
- `numpy` - Numerical operations and data handling
- `matplotlib` - 2D plotting and image generation
- `pandas` - Excel file reading and data manipulation
- `openpyxl` - Excel file format support

#### Environment:
- Designed for `conda habitat` environment
- Compatible with Linux systems
- Tested with Python 3.7+

### Key Features

#### 1. **Clean Visualization**
- No axes, labels, or visual clutter
- Pure semantic point representation
- White background with tight cropping

#### 2. **Flexible Filtering**
- Dual-layer filtering (semantic + height)
- Manual and automatic height range selection
- Preserves spatial relationships

#### 3. **Coordinate Transformation**
- Precise pixel-to-Habitat coordinate mapping
- Enables future navigation and localization tasks
- Documented transformation parameters

#### 4. **Modular Design**
- Reusable `SemanticMapGenerator` class
- Configurable entry points
- Easy parameter adjustment

### Future Applications

This implementation provides the foundation for:
- **Part 3:** Navigation and path planning using coordinate transformations
- **Robot Localization:** Converting between image and world coordinates
- **Spatial Analysis:** Understanding environment layout and object relationships
- **Map Updates:** Real-time semantic map generation and updates

### Performance Notes

- **Memory Efficient:** Processes large point clouds without excessive memory usage
- **Fast Filtering:** Vectorized operations for efficient point filtering
- **High Quality Output:** 300 DPI images suitable for detailed analysis
- **Scalable:** Handles point clouds of varying sizes