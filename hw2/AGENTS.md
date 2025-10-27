It looks like you've pasted an incomplete version of the file, which confirms it's getting cut off for you.

My apologies again. Let's provide the **full and complete content** one more time. Please copy the *entire* text block below, from the very beginning (` ```markdown `) to the very end (` ``` `), and paste that into your `.md` file.

-----

````markdown
# Code Agent Task: RRT Path Navigation and Visualization in Habitat

## 🎯 Objective

The goal is to implement a navigation agent in the Habitat simulator. The agent will follow a pre-computed path from an RRT algorithm, visually highlight the target object during navigation, and record its journey as a video file.

---

## ⚙️ Prerequisites

* A pre-calculated path from the **RRT algorithm** (Part 2), available as a list of 2D pixel coordinates.
* A functional **Habitat simulation environment**.
* The **target category/name** is known (e.g., "chair", "table").

---

## 📝 Implementation Plan

The implementation can be done primarily by modifying the `load.py` script or a similar entry point for the simulation.

### 1. Configure Agent Actions

In the function responsible for setting up the environment configuration (e.g., `make_simple_cfg`), define the discrete step sizes for the agent's actions.

**Example Configuration:**
```python
# In your environment configuration function
config.TASK.ACTIONS.MOVE_FORWARD.MOTION_ARGS["step_size"] = 0.25  # meters
config.TASK.ACTIONS.TURN_LEFT.MOTION_ARGS["angle"] = 10.0      # degrees
config.TASK.ACTIONS.TURN_RIGHT.MOTION_ARGS["angle"] = 10.0     # degrees
````

### 2\. Implement Coordinate Transformation

Create a function to convert the 2D pixel coordinates from the RRT path into 3D world coordinates (`xyz`) usable by the Habitat agent. This typically involves using the simulation's projection and depth information.

**Pseudocode:**

```python
function pixel_to_world(pixel_coord, depth_map):
  # Get depth value at the pixel coordinate
  depth_value = depth_map[pixel_coord.y, pixel_coord.x]

  # Unproject the 2D point + depth to a 3D point in the camera frame
  camera_point_3d = unproject(pixel_coord, depth_value)

  # Transform the 3D point from the camera frame to the world frame
  world_point_3d = camera_to_world_transform(camera_point_3d)

  # Set the y-coordinate to the agent's height to create a navigation waypoint
  waypoint = (world_point_3d.x, agent_height, world_point_3d.z)

  return waypoint
```

### 3\. Implement the Navigation Loop

The core logic will iterate through the transformed 3D waypoints and control the agent.

**Steps:**

1.  **Initialize**: Load the RRT path and transform all points into a list of 3D waypoints. Initialize an empty list to store RGB frames for the video.
2.  **Loop through Waypoints**: For each waypoint in the list:
    a.  **Calculate Heading**: Determine the angle between the agent's current forward direction and the vector pointing to the next waypoint.
    b.  **Turn**: If the angle is greater than a small threshold, issue `turn_left` or `turn_right` commands repeatedly until the agent is facing the waypoint. Collect the RGB frame after each turn action.
    c.  **Move Forward**: Once aligned, calculate the distance to the waypoint. Issue `move_forward` commands repeatedly until the agent is close to the waypoint. Collect the RGB frame after each move action.
3.  **Collect Frames**: In each step of the loop (after every action), get the current RGB observation and append it to your frame list.

### 4\. Highlight the Target Object

At each step, before saving the RGB frame, overlay a transparent mask on the target object.

**Steps:**

1.  **Get Semantic Data**: For each observation, retrieve the semantic segmentation map from the simulator.
2.  **Find Target Pixels**: Identify the pixels in the semantic map that correspond to the target object's ID.
3.  **Create Mask**: Generate a boolean mask where `True` values correspond to the target object's pixels.
4.  **Overlay Mask**: Create a colored overlay (e.g., semi-transparent red) and apply it to the RGB image using the mask. The reference `tutorial_adding_images` should provide guidance on blending images.

### 5\. Generate Video Output

After the navigation loop is complete, compile the collected frames into a video.

**Steps:**

1.  **Get Target Name**: Use the known target name for the output filename.
2.  **Initialize Video Writer**: Use a library like `cv2` or `imageio` to set up a video writer.
    ```python
    import cv2
    import numpy as np

    height, width, layers = frames[0].shape
    video_name = f"{target_name}.mp4"
    # Use 'mp4v' codec for .mp4 files
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_name, fourcc, 10, (width, height))
    ```
3.  **Write Frames**: Iterate through the list of collected frames (with the highlighted target) and write each one to the video file.
    ```python
    for frame in frames:
      # Convert RGB (from Habitat) to BGR (for OpenCV)
      video.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    ```
4.  **Save Video**: Close the video writer to save the file.
    ```python
    video.release()
    ```

-----

## ✅ Deliverables

  * A modified Python script (e.g., `load.py`) containing the full navigation and visualization logic.
  * An output video file named **`{target_name}.mp4`** showing the agent's first-person view as it navigates the path with the target highlighted.

<!-- end list -->

```
```