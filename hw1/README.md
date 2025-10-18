# physical-ai-hw1

NYCU Physical AI 2025 Fall

Spec: [Google Docs](https://docs.google.com/document/d/1UqDRjh7qwQVzz2iN9Abdu4-NIl-xEO4G/edit)

## Overview

This assignment focuses on 3D computer vision tasks including data collection in a simulated environment, Bird's Eye View (BEV) projection, and 3D scene reconstruction from RGB-D sequences.

### Components

1. **Data Collection** (`load.py`): Interactive navigation in Habitat-Sim to collect RGB-D-Semantic data
2. **BEV Projection** (`bev.py`): Project points from top-view (BEV) to front-view perspective
3. **3D Reconstruction** (`reconstruct.py`): Reconstruct 3D scenes from RGB-D sequences using ICP

## Preparation

### Dataset
The replica dataset, you can use the same one in `hw0`.

## Usage

### 1. Data Collection (`load.py`)

Interactive tool for navigating through a simulated environment and collecting sensor data.

**Run the script:**

```bash
# Activate conda environment
conda activate habitat

# For collecting Floor 1 data
python load.py -f 1

# For collecting Floor 2 data
python load.py -f 2
```

**Controls:**
- `W` - Move forward
- `A` - Turn left
- `D` - Turn right
- `F` - Finish and save data

### 2. BEV Projection (`bev.py`)

Projects selected pixels from a top-view (BEV) image to a front-view perspective image.

**Run the script:**

```bash
# Activate conda environment
conda activate habitat

# Run the projection
python bev.py
```

**Instructions:**

- A window with the BEV image will pop up
- Click on the image to select points via left mouse click
- Close the window when done selecting points
- The projected region will be highlighted on the front view image

### 3. 3D Reconstruction (`reconstruct.py`)

Reconstructs a 3D point cloud scene from sequential RGB-D images using point cloud registration.

**Run the script:**

```bash
# Activate conda environment
conda activate habitat

# Using custom ICP implementation on floor 1(default)
python reconstruct.py --floor 1 --version my_icp

# Using Open3D's ICP on floor 1
python reconstruct.py --floor 1 --version open3d

# Using custom ICP implementation on floor 2
python reconstruct.py --floor 2 --version my_icp
```

**Arguments:**
- `-f, --floor`: Floor number (1 or 2)
- `-v, --version`: ICP version (`my_icp` or `open3d`)
- `--data_root`: Path to data directory (auto-set based on floor)

**Features:**
- RGBD to point cloud conversion
- RANSAC-based global registration (coarse alignment)
- ICP refinement (fine alignment)
- Custom ICP implementation option
- Camera trajectory estimation
- Trajectory accuracy evaluation (L2 distance)