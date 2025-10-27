# API Reference

## SemanticMapGenerator Class

### Constructor
```python
SemanticMapGenerator(data_dir="../", output_dir="./")
```
**Parameters:**
- `data_dir` (str): Directory containing input data files
- `output_dir` (str): Directory for output files

### Methods

#### `load_data()`
Loads all required data files including point cloud and color mappings.

**Returns:** None

**Side Effects:**
- Populates `self.points`, `self.colors01`, `self.colors0255`
- Applies coordinate scaling transformation
- Loads semantic class mappings

#### `filter_ceiling_floor_height(height_min=None, height_max=None)`
Filters point cloud by semantic labels and height range.

**Parameters:**
- `height_min` (float, optional): Minimum Y-coordinate to keep
- `height_max` (float, optional): Maximum Y-coordinate to keep

**Returns:**
- `tuple`: (filtered_points, filtered_colors) as numpy arrays

**Behavior:**
- If height parameters are None, uses 10th-90th percentile range
- Removes points matching ceiling/floor colors
- Applies height-based filtering

#### `generate_2d_map(points, colors, figsize=(12,8), dpi=300)`
Creates 2D scatter plot from filtered point cloud.

**Parameters:**
- `points` (np.ndarray): 3D coordinates array
- `colors` (np.ndarray): RGB color values [0,1]
- `figsize` (tuple): Figure size in inches
- `dpi` (int): Image resolution

**Returns:**
- `dict`: Transformation parameters

#### `save_transformation(transformation)`
Saves coordinate transformation parameters to file.

**Parameters:**
- `transformation` (dict): Transformation parameters from `generate_2d_map()`

**Output File:** `coordinate_transformation.txt`

#### `run(height_min=None, height_max=None)`
Executes complete pipeline with optional height filtering.

**Parameters:**
- `height_min` (float, optional): Minimum height to keep
- `height_max` (float, optional): Maximum height to keep

## Data Formats

### Input Files
- **`point.npy`**: Shape (N, 3) - XYZ coordinates
- **`color01.npy`**: Shape (N, 3) - RGB values [0,1]
- **`color0255.npy`**: Shape (N, 3) - RGB values [0,255]
- **`color_coding_semantic_segmentation_classes.xlsx`**: Semantic class mappings

### Output Files
- **`map.png`**: 2D semantic map image
- **`coordinate_transformation.txt`**: Transformation parameters

### Transformation Dictionary
```python
{
    'habitat_x_range': (x_min, x_max),
    'habitat_z_range': (z_min, z_max),
    'image_width_px': float,
    'image_height_px': float,
    'x_scale': float,  # pixels per habitat unit
    'z_scale': float,  # pixels per habitat unit
    'x_offset': float,
    'z_offset': float,
    'dpi': int
}
```

## Configuration Constants

### Semantic Filtering
```python
ceiling_color = (8, 255, 214)    # RGB values
floor_color = (255, 194, 7)      # RGB values
```

### Coordinate Scaling
```python
apartment_0_coordinates = point_array * 10000.0 / 255.0
```

## Usage Examples

### Basic Usage
```python
generator = SemanticMapGenerator()
generator.run()
```

### Custom Height Range
```python
generator = SemanticMapGenerator()
generator.load_data()
filtered_points, filtered_colors = generator.filter_ceiling_floor_height(-1.0, 1.0)
transformation = generator.generate_2d_map(filtered_points, filtered_colors)
generator.save_transformation(transformation)
```

### High Resolution Output
```python
transformation = generator.generate_2d_map(
    filtered_points, 
    filtered_colors, 
    figsize=(16, 12), 
    dpi=600
)
```