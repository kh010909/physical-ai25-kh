import numpy as np
import matplotlib.pyplot as plt

# Load the data files
points = np.load('semantic_3d_pointcloud/point.npy')
colors_0255 = np.load('semantic_3d_pointcloud/color0255.npy')

# Define ceiling and floor colors
ceiling_color = np.array([8, 255, 214])
floor_color = np.array([255, 194, 7])

# Create masks for ceiling and floor points
ceiling_mask = np.all(colors_0255 == ceiling_color, axis=1)
floor_mask = np.all(colors_0255 == floor_color, axis=1)

# Apply height threshold (Y coordinate)
# Keep points within a reasonable height range to filter out noise
height_min = -0.03  # Filter out floor level
height_max = 0.01   # Filter out ceiling level
height_mask = (points[:, 1] >= height_min) & (points[:, 1] <= height_max)

# Combine masks to remove ceiling, floor, and out-of-range heights
remove_mask = ceiling_mask | floor_mask | ~height_mask

# Filter out ceiling, floor, and out-of-range points
filtered_points = points[~remove_mask]
filtered_colors = colors_0255[~remove_mask]

print(f"Original number of points: {len(points)}")
print(f"Number of ceiling points: {np.sum(ceiling_mask)}")
print(f"Number of floor points: {np.sum(floor_mask)}")
print(f"Number of out-of-range height points: {np.sum(~height_mask)}")
print(f"Filtered number of points: {len(filtered_points)}")
print(f"Height range filter: [{height_min}, {height_max}]")

# Apply the scale transformation: apartment_0 = points array * 10000 / 255
scaled_points = filtered_points * 10000.0 / 255.0

print(f"\nOriginal point range: x=[{points[:, 0].min():.4f}, {points[:, 0].max():.4f}]")
print(f"Original point range: z=[{points[:, 2].min():.4f}, {points[:, 2].max():.4f}]")
print(f"Scaled point range: x=[{scaled_points[:, 0].min():.4f}, {scaled_points[:, 0].max():.4f}]")
print(f"Scaled point range: z=[{scaled_points[:, 2].min():.4f}, {scaled_points[:, 2].max():.4f}]")

# Normalize colors to [0, 1] for matplotlib
normalized_colors = filtered_colors / 255.0

# Create 2D scatter plot (x-coordinate vs z-coordinate)
fig, ax = plt.subplots(figsize=(12, 12), dpi=100)
scatter = ax.scatter(scaled_points[:, 0], scaled_points[:, 2], 
                     c=normalized_colors, s=1, alpha=0.8)

ax.set_xlabel('X Coordinate (Habitat Scale)')
ax.set_ylabel('Z Coordinate (Habitat Scale)')
ax.set_title('2D Semantic Map - First Floor of Apartment 0')
ax.set_aspect('equal', adjustable='box')
ax.grid(True, alpha=0.3)

# Save the map
plt.savefig('map.png', dpi=150, bbox_inches='tight')
print("\nMap saved as 'map.png'")

# Print some statistics for calibration
print(f"\nStatistics for coordinate calibration:")
print(f"X range: [{scaled_points[:, 0].min():.2f}, {scaled_points[:, 0].max():.2f}]")
print(f"Z range: [{scaled_points[:, 2].min():.2f}, {scaled_points[:, 2].max():.2f}]")
print(f"Image dimensions in generated plot: should correspond to these ranges")

# Store the calibration information
calibration_info = {
    'scale_factor': 10000.0 / 255.0,
    'x_min': scaled_points[:, 0].min(),
    'x_max': scaled_points[:, 0].max(),
    'z_min': scaled_points[:, 2].min(),
    'z_max': scaled_points[:, 2].max(),
}

print(f"\nCalibration info:")
for key, value in calibration_info.items():
    print(f"  {key}: {value}")

# Save calibration info for Part 3
np.save('calibration_info.npy', calibration_info, allow_pickle=True)
print("\nCalibration info saved as 'calibration_info.npy'")
