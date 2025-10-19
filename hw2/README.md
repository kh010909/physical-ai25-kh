# physical-ai-hw2
NYCU Physical AI 2025 Fall

Spec: https://drive.google.com/file/d/1jg5wRDpTQcx7Ux01hNzPmMdKGN-Mhxc0/view?usp=sharing

## Overview

This assignment focuses on 2D semantic map construction and navigation using semantic information from 3D point clouds.

### Components

- **Part 1**: 2D Semantic Map Construction from 3D point cloud ✅ (Complete)
- **Part 2**: TBD
- **Part 3**: Navigation using semantic maps

## 🚀 Quick Start

**New to this assignment? Start here:**

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Verify your data:**
   ```bash
   python test_data.py
   ```

3. **Generate the 2D semantic map:**
   ```bash
   python semantic_map.py
   ```

4. **Check the output:** Open `map.png` to see your 2D semantic map!

📖 **For detailed instructions, see [QUICKSTART.md](QUICKSTART.md)**  
📚 **For complete documentation, continue reading below**

## Preparation

In your original physical-ai25 directory, `git pull` to get new `hw2` directory.

### Required Libraries

```bash
pip install numpy matplotlib open3d
```

### Dataset

Download the 3D semantic point cloud data for apartment_0 (first floor) and place it in the `semantic_3d_pointcloud/` directory:
- `point.npy` - 3D point coordinates
- `color01.npy` - RGB colors in [0, 1] range
- `color0255.npy` - RGB colors in [0, 255] range

## Part 1: 2D Semantic Map Construction

### Objective

Generate a 2D semantic map (top-down view) from a 3D semantic point cloud by projecting points onto the X-Z plane.

### Steps

1. **Load 3D semantic point cloud** from provided data
2. **Remove ceiling and floor points** based on Y-coordinate thresholds
3. **Convert coordinates** to Habitat coordinate system
4. **Project to 2D** by using X and Z coordinates
5. **Save the map** as `map.png` with semantic colors

### Usage

**Basic usage:**

```bash
python semantic_map.py
```

**Advanced options:**

```bash
# Specify custom output filename
python semantic_map.py --output my_map.png

# Adjust ceiling/floor thresholds
python semantic_map.py --floor_threshold 0.15 --ceiling_threshold 0.85

# Use color_0255.npy instead of color01.npy (avoid floating-point errors)
python semantic_map.py --use_color_0255

# Set higher resolution
python semantic_map.py --dpi 300

# Specify data directory
python semantic_map.py --data_dir path/to/semantic_3d_pointcloud
```

**All options:**

```
--data_dir          Directory containing point cloud data (default: semantic_3d_pointcloud)
--output            Output filename for 2D map (default: map.png)
--floor_threshold   Y-coordinate threshold for floor removal (default: 0.1)
--ceiling_threshold Y-coordinate threshold for ceiling removal (default: 0.9)
--dpi               Output image resolution (default: 150)
--use_color_0255    Use color_0255.npy to avoid floating-point errors
```

### Key Concepts

#### Coordinate Systems

**Point Cloud Coordinates (Normalized)**
- Raw data from `point.npy`
- Range: typically [0, 255] normalized

**Habitat Coordinates (Meters)**
- Conversion formula: `habitat_coords = point_coords * 10000.0 / 255.0`
- This is the coordinate system used in Habitat simulator
- Unit: meters

**Map Pixel Coordinates**
- For visualization and navigation
- Conversion: `pixel = (habitat_coord - min_coord) * scale`
- The script calculates and saves mapping information for Part 3

#### Ceiling and Floor Removal

Points are filtered based on Y-coordinate (vertical axis):
- **Floor points**: Y < floor_threshold (default: 0.1)
- **Ceiling points**: Y > ceiling_threshold (default: 0.9)
- **Kept points**: floor_threshold ≤ Y ≤ ceiling_threshold

You may need to adjust these thresholds based on your data.

#### Color Arrays

Two versions are provided:
- `color01.npy`: RGB in [0, 1] - standard format for matplotlib
- `color0255.npy`: RGB in [0, 255] - use this if you encounter floating-point precision issues

Colors correspond to 101 semantic categories in the dataset.

### Output Files

The script generates several output files:

1. **map.png** - 2D semantic map visualization
   - Top-down view of the scene
   - Each point colored by its semantic category
   - Includes coordinate range information

2. **filtered_points.npy** - 3D points after ceiling/floor removal
   - Shape: (N, 3) in Habitat coordinates
   - Use for further processing

3. **filtered_colors.npy** - Colors corresponding to filtered points
   - Shape: (N, 3)
   - Same color format as input

4. **mapping_info.npy** - Coordinate mapping parameters
   - Contains: x_min, x_max, z_min, z_max, x_range, z_range
   - Essential for Part 3 (coordinate conversion)

### Coordinate Mapping (Important for Part 3!)

The script calculates the relationship between:
- **Habitat coordinates** (x_h, z_h) in meters
- **Map pixel coordinates** (x_p, z_p) in pixels

**Conversion formulas:**
```python
# Habitat to pixel
x_pixel = (x_habitat - x_min) * scale
z_pixel = (z_habitat - z_min) * scale

# Pixel to Habitat
x_habitat = x_pixel / scale + x_min
z_habitat = z_pixel / scale + z_min
```

Where `scale` depends on your desired map resolution (e.g., pixels per meter).

The mapping information is printed during execution and saved in `mapping_info.npy`.

### Helper Tools

#### Y-Distribution Analyzer

To help choose optimal threshold values, use the distribution analyzer:

```bash
python visualize_3d.py --distribution
```

This will:
- Show histogram of Y-coordinate distribution
- Display percentiles and statistics
- Suggest appropriate threshold values
- Save plot as `y_distribution.png`

#### 3D Point Cloud Viewer (Optional)

If you have Open3D installed, you can visualize the 3D point cloud:

```bash
# View with default thresholds
python visualize_3d.py

# View with custom thresholds
python visualize_3d.py --floor_threshold 0.12 --ceiling_threshold 0.88
```

This helps you:
- Understand the 3D structure
- Verify threshold choices visually
- See what gets removed

### Troubleshooting

**Issue: Map looks incorrect or missing points**
- Solution 1: Run `python visualize_3d.py --distribution` to see Y-distribution
- Solution 2: Adjust `--floor_threshold` and `--ceiling_threshold` based on distribution
- Solution 3: Try different values between 0.05 and 0.95

**Issue: Floating-point errors with colors**
- Solution: Use `--use_color_0255` flag to use integer color values

**Issue: Map resolution too low**
- Solution: Increase `--dpi` value (e.g., 300 or 600)

**Issue: Memory error**
- Solution: The point cloud might be very large; consider downsampling or processing in chunks

**Issue: How do I choose the right thresholds?**
- Solution: Run `python test_data.py` to see suggested values
- Or run `python visualize_3d.py --distribution` for detailed analysis

### Example Output

After running the script, you should see:
- A scatter plot showing the 2D top-down view of the apartment
- Different colors representing different semantic categories (walls, floor, furniture, etc.)
- Coordinate information displayed on the plot
- Console output with coordinate ranges and mapping information

## Files Overview

### Main Scripts

| File | Purpose | When to Use |
|------|---------|-------------|
| `semantic_map.py` | Main script for Part 1 - generates 2D semantic map | Run this to complete Part 1 |
| `test_data.py` | Data verification and statistics | Run first to check setup |
| `visualize_3d.py` | 3D visualization and Y-distribution analysis | Use to choose thresholds |
| `coordinate_utils.py` | Coordinate conversion utilities | Use for Part 3 integration |
| `example_usage.py` | Examples of using coordinate utilities | Learn how to use for Part 3 |

### Documentation

| File | Purpose |
|------|---------|
| `README.md` | Complete documentation with technical details (this file) |
| `QUICKSTART.md` | Step-by-step guide for beginners |
| `SOLUTION_SUMMARY.md` | Overview of the complete solution |
| `requirements.txt` | Python package dependencies |

### Data Files

| File | Type | Description |
|------|------|-------------|
| `point.npy` | Input | 3D point coordinates (normalized) |
| `color01.npy` | Input | RGB colors in [0, 1] range |
| `color0255.npy` | Input | RGB colors in [0, 255] range |
| `filtered_points.npy` | Output | 3D points after ceiling/floor removal |
| `filtered_colors.npy` | Output | Colors for filtered points |
| `mapping_info.npy` | Output | Coordinate mapping parameters for Part 3 |
| `map.png` | Output | Final 2D semantic map visualization |

## File Structure

```
hw2/
├── semantic_map.py              # Part 1: Main solution script
├── test_data.py                 # Data verification utility
├── visualize_3d.py              # 3D visualization and distribution analysis
├── coordinate_utils.py          # Coordinate conversion utilities (for Part 3)
├── example_usage.py             # Example code for Part 3 integration
├── README.md                    # Complete documentation (this file)
├── QUICKSTART.md                # Quick start guide
├── SOLUTION_SUMMARY.md          # Solution overview and summary
├── requirements.txt             # Python dependencies
├── semantic_3d_pointcloud/      # Point cloud data directory
│   ├── point.npy               # Input: 3D coordinates
│   ├── color01.npy             # Input: Colors [0, 1]
│   ├── color0255.npy           # Input: Colors [0, 255]
│   ├── filtered_points.npy     # Output: Filtered 3D points
│   ├── filtered_colors.npy     # Output: Filtered colors
│   └── mapping_info.npy        # Output: Coordinate mapping info
├── map.png                      # Output: 2D semantic map
└── y_distribution.png           # Output: Y-coordinate distribution plot
```

## Technical Details

### Algorithm Overview

1. **Data Loading**
   - Load point cloud coordinates and semantic colors
   - Support both [0,1] and [0,255] color ranges

2. **Filtering**
   - Remove outliers based on Y-coordinate
   - Keep only points representing walls, furniture, and objects

3. **Coordinate Transformation**
   - Apply scale factor: 10000.0 / 255.0
   - Convert to Habitat's metric coordinate system

4. **2D Projection**
   - Project 3D points (X, Y, Z) to 2D (X, Z)
   - Y-axis is vertical (removed in top-down view)

5. **Visualization**
   - Scatter plot with semantic colors
   - Equal aspect ratio to preserve spatial relationships
   - Grid and labels for reference

### Performance Notes

- Processing time depends on the number of points
- Typical point clouds: 100K-1M points
- Rendering time: 5-30 seconds depending on DPI and point count
- Memory usage: ~100-500 MB for typical datasets

## Preparing for Part 3

The coordinate mapping utilities are essential for Part 3. Here's how to use them:

### Loading the Coordinate Mapper

```python
from coordinate_utils import create_mapper_from_file

# Create mapper from the mapping info generated in Part 1
mapper = create_mapper_from_file('semantic_3d_pointcloud/mapping_info.npy', 
                                 map_width_pixels=1000)
```

### Common Use Cases

**1. Convert agent position to map coordinates:**
```python
# Agent at position (2.5, 3.0) in Habitat
x_pixel, z_pixel = mapper.habitat_to_pixel(2.5, 3.0)
```

**2. Convert map click to Habitat coordinates:**
```python
# User clicked at pixel (500, 600)
x_habitat, z_habitat = mapper.pixel_to_habitat(500, 600)
```

**3. Plan navigation path:**
```python
# Path in Habitat coordinates
path = [(0.0, 0.0), (1.0, 2.0), (2.5, 3.5)]

# Convert to pixels for visualization
path_pixels = [mapper.habitat_to_pixel(x, z) for x, z in path]
```

For more examples, see `example_usage.py`.

## References

- [Habitat-Sim Documentation](https://aihabitat.org/docs/habitat-sim/)
- [Matplotlib Scatter Plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.scatter.html)
- [NumPy Documentation](https://numpy.org/doc/stable/)

---

**Questions or issues?** Check the [QUICKSTART.md](QUICKSTART.md) or [SOLUTION_SUMMARY.md](SOLUTION_SUMMARY.md) for more information.
