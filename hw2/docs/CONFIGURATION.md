# Configuration Guide

## Height Filtering Configuration

### Understanding Height Values

The Y-coordinate in the Habitat coordinate system represents vertical height:
- **Negative values**: Below the reference level (typically floor areas)
- **Positive values**: Above the reference level (walls, furniture, ceiling)
- **Zero**: Reference ground level

### Common Height Ranges

#### Floor-Level Objects Only
```python
HEIGHT_MIN = -1.2
HEIGHT_MAX = -0.2
```
**Use Case:** Navigation maps focusing on floor boundaries and low obstacles

#### Human-Height Objects
```python
HEIGHT_MIN = -0.8
HEIGHT_MAX = 1.8
```
**Use Case:** Objects at human interaction level (furniture, walls, doors)

#### Conservative Middle Range
```python
HEIGHT_MIN = -0.5
HEIGHT_MAX = 0.5
```
**Use Case:** Core structural elements, avoiding floor and ceiling artifacts

#### Full Automatic Range
```python
# Use None values for automatic percentile-based filtering
HEIGHT_MIN = None
HEIGHT_MAX = None
```
**Use Case:** Let the system automatically determine optimal range

### Height Statistics Interpretation

When you run the script, it shows height statistics:
```
Height statistics:
  Min Y: -1.775    # Lowest point (deep floor level)
  Max Y: 2.365     # Highest point (ceiling level)
  Mean Y: -0.264   # Average height (slightly below reference)
  Median Y: -1.093 # Median height (floor-heavy distribution)
```

**Analysis:**
- Most points are below reference level (floor-focused)
- Wide range suggests multi-level environment
- Use statistics to inform filtering decisions

### Filtering Strategy Examples

#### Remove Only Extreme Heights
```python
HEIGHT_MIN = -1.5  # Keep most floor points
HEIGHT_MAX = 2.0   # Keep most ceiling points
```

#### Focus on Navigation Level
```python
HEIGHT_MIN = -1.0  # Just above deep floor
HEIGHT_MAX = 0.2   # Just below wall tops
```

#### Remove Furniture, Keep Structure
```python
HEIGHT_MIN = -0.8  # Above floor clutter
HEIGHT_MAX = -0.1  # Below most furniture
```

## Semantic Filtering

### Predefined Semantic Classes

The system automatically removes these semantic classes:
- **Ceiling**: RGB(8, 255, 214)
- **Floor**: RGB(255, 194, 7)

### Adding Custom Semantic Filtering

To filter additional semantic classes, modify the `filter_ceiling_floor_height()` method:

```python
# Example: Also remove 'wall' class
wall_color = (120, 120, 120)  # Get from Excel file
wall_rgb = np.array(wall_color, dtype=np.uint8)
wall_mask = np.all(colors_uint8 == wall_rgb, axis=1)

# Add to existing masks
semantic_remove_mask = ceiling_mask | floor_mask | wall_mask
```

## Output Configuration

### Image Quality Settings

#### Standard Quality
```python
transformation = generator.generate_2d_map(
    filtered_points, filtered_colors,
    figsize=(12, 8), 
    dpi=300
)
```

#### High Quality for Printing
```python
transformation = generator.generate_2d_map(
    filtered_points, filtered_colors,
    figsize=(16, 12), 
    dpi=600
)
```

#### Large Format for Analysis
```python
transformation = generator.generate_2d_map(
    filtered_points, filtered_colors,
    figsize=(20, 16), 
    dpi=300
)
```

### Point Density Control

Adjust point size for different visualizations:

```python
# In the generate_2d_map method, modify:
scatter = ax.scatter(x_coords, z_coords, c=colors, s=0.1, alpha=0.8)

# For denser visualization
scatter = ax.scatter(x_coords, z_coords, c=colors, s=0.05, alpha=1.0)

# For sparser visualization  
scatter = ax.scatter(x_coords, z_coords, c=colors, s=0.2, alpha=0.6)
```

## Performance Optimization

### Memory Management

For large point clouds:
```python
# Process in chunks if memory is limited
chunk_size = 50000
for i in range(0, len(points), chunk_size):
    chunk_points = points[i:i+chunk_size]
    chunk_colors = colors[i:i+chunk_size]
    # Process chunk
```

### Speed Optimization

For faster processing:
```python
# Reduce resolution for quick previews
transformation = generator.generate_2d_map(
    filtered_points[::10], filtered_colors[::10],  # Every 10th point
    figsize=(8, 6), 
    dpi=150
)
```

## File Organization

### Recommended Directory Structure
```
hw2/
├── src/
│   ├── main.py
│   ├── semantic_map_generator.py
│   ├── run_with_custom_height.py
│   ├── map.png
│   └── coordinate_transformation.txt
├── docs/
│   ├── README.md
│   ├── API.md
│   └── CONFIGURATION.md
├── semantic_3d_pointcloud/
│   ├── point.npy
│   ├── color01.npy
│   └── color0255.npy
└── color_coding_semantic_segmentation_classes.xlsx
```

### Output Management

Create timestamped outputs:
```python
import datetime
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = f"map_{timestamp}.png"
```