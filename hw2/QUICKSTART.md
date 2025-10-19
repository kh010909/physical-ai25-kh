# Quick Start Guide - HW2 Part 1

## Step-by-Step Instructions

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install numpy matplotlib open3d
```

### 2. Verify Data

Run the test script to check if all data files are present and valid:

```bash
python test_data.py
```

This will:
- Check if all required files exist
- Load and verify the data
- Display statistics about the point cloud
- Suggest optimal threshold values for your data

### 3. Generate 2D Semantic Map

Run the main script with default settings:

```bash
python semantic_map.py
```

This will:
- Load the 3D semantic point cloud
- Remove ceiling and floor points
- Convert to Habitat coordinates
- Generate a 2D top-down semantic map
- Save as `map.png`

### 4. Adjust Parameters (Optional)

If the default parameters don't work well, try adjusting:

```bash
# Example: Custom thresholds based on test_data.py output
python semantic_map.py --floor_threshold 0.12 --ceiling_threshold 0.88

# Example: Higher resolution output
python semantic_map.py --dpi 300

# Example: Use integer colors to avoid floating-point errors
python semantic_map.py --use_color_0255
```

## Expected Output

### Console Output
You should see:
```
==============================================================
2D SEMANTIC MAP CONSTRUCTION - PART 1
==============================================================

Step 1: Loading point cloud data...
Loaded point cloud with XXXXX points
...

Step 5: Creating 2D semantic map...
2D semantic map saved to: map.png

COMPLETED SUCCESSFULLY!
==============================================================
```

### Visual Output
- A matplotlib window showing the 2D semantic map
- The map shows a top-down view of the apartment
- Different colors represent different semantic categories
- Coordinate information is displayed on the plot

### Generated Files
1. `map.png` - Your 2D semantic map
2. `semantic_3d_pointcloud/filtered_points.npy` - Filtered 3D points
3. `semantic_3d_pointcloud/filtered_colors.npy` - Corresponding colors
4. `semantic_3d_pointcloud/mapping_info.npy` - Coordinate mapping info (for Part 3)

## Common Issues

### Issue 1: "No module named 'numpy'"
**Solution:** Install dependencies with `pip install -r requirements.txt`

### Issue 2: "Directory 'semantic_3d_pointcloud' not found"
**Solution:** 
- Create the directory: `mkdir semantic_3d_pointcloud`
- Download the data files from the link in the spec
- Place them in the directory

### Issue 3: Map looks strange or missing large areas
**Solution:** 
- Run `python test_data.py` to see suggested threshold values
- Adjust `--floor_threshold` and `--ceiling_threshold` accordingly
- Try values between 0.05 and 0.95

### Issue 4: Floating-point color errors
**Solution:** Use `python semantic_map.py --use_color_0255`

## What's Next?

After completing Part 1, you should have:
- ✓ A 2D semantic map (`map.png`)
- ✓ Filtered point cloud data
- ✓ Coordinate mapping information

This mapping information will be crucial for Part 3 when you need to:
- Convert between Habitat coordinates and map pixel coordinates
- Plan navigation paths
- Locate objects on the map

## Quick Reference

### File Purposes
| File | Purpose |
|------|---------|
| `semantic_map.py` | Main script for Part 1 |
| `test_data.py` | Verify data and get statistics |
| `requirements.txt` | Python dependencies |
| `map.png` | Output: 2D semantic map |
| `filtered_points.npy` | Output: Cleaned 3D points |
| `mapping_info.npy` | Output: Coordinate conversion parameters |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--floor_threshold` | 0.1 | Y-value below which points are removed (floor) |
| `--ceiling_threshold` | 0.9 | Y-value above which points are removed (ceiling) |
| `--dpi` | 150 | Output image resolution |
| `--use_color_0255` | False | Use integer colors instead of float |

## Need Help?

1. Run `python test_data.py` first to diagnose data issues
2. Check the console output for error messages
3. Review the README.md for detailed explanations
4. Try adjusting parameters based on `test_data.py` suggestions
