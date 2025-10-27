# TASK

Construct a 2D semantic map by projecting a 3D semantic point cloud onto a top-down (X-Z plane) view.

---

# REQUIREMENTS

-   **Files:**
    -   3D semantic map for `apartment_0`
    -   `color_01.npy` (Color map with RGB values in [0, 1])
    -   `color_0255.npy` (Color map with RGB values in [0, 255])
    -   `color_coding_semantic_segmentation_classes.xlsx`: The colors and their corresponding semantic labels (Use for remove items).
-   **Libraries:**
    -   A library to read and process point clouds (e.g., `open3d`, `plyfile`).
    -   `numpy` for numerical operations.
    -   `matplotlib` (or a similar plotting library) to generate the 2D map.

---

# STEPS

1.  **Load Data:**
    -   Load the 3D semantic map file (e.g., a `.ply` file) for `apartment_0`.
    -   Load the color map files (`color_01.npy` or `color_0255.npy`).
2.  **Filter Point Cloud:**
    -   Identify the semantic labels (and corresponding colors) for 'ceiling' and 'floor' from the provided color maps.
    -   Remove all points from the 3D point cloud that are classified as 'ceiling' or 'floor'.
3.  **Extract Data:**
    -   From the *filtered* point cloud, extract the coordinates (X, Y, Z) and the corresponding RGB color values for each point.
4.  **Generate 2D Map:**
    -   Create a 2D scatter plot using the **X-coordinates** (as the plot's x-axis) and the **Z-coordinates** (as the plot's y-axis) of the filtered points.
    -   Set the color of each plotted point $(x, z)$ to its corresponding RGB color extracted in Step 3.
5.  **Save Map:**
    -   Save the generated 2D plot as a PNG file named `map.png`.
6.  **Determine Coordinate Transformation:**
    -   Establish and document the relationship (e.g., scaling and offset) between the pixel coordinates of the saved `map.png` and the original 3D Habitat coordinates (X, Z). This is critical for future tasks. You may need to add known reference points or analyze plot boundaries to calculate this transformation.

---

# NOTES

-   **Color Map Usage:** You are provided with two color map versions. `color_01.npy` has RGB values in the [0, 1] range. `color_0255.npy` has values in the [0, 255] range. If you encounter floating-point precision issues when matching colors with `color_01.npy`, use `color_0255.npy` instead.
-   **Coordinate Scaling (if applicable):** If you are using a file named `point.npy` in relation to `apartment_0`, be aware of the scale relationship: `apartment_0_coordinates = point_array * 10000.0 / 255.0`.
-   **Final Goal:** The transformation calculated in Step 6 is necessary for "Part 3" (a subsequent task). Ensure this relationship is clearly defined.
