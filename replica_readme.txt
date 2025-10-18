## Algorithm Details

### BEV to Front-View Projection

The projection follows these steps:

1. **Unproject BEV pixel to 3D ray** using inverse camera intrinsics
2. **Transform ray to world coordinates** using BEV camera pose
3. **Intersect with ground plane** (Y=0) to get 3D point
4. **Transform to front camera coordinates**
5. **Project onto front image plane** using camera intrinsics

### 3D Reconstruction Pipeline

1. **Point Cloud Generation**
   - Convert RGB-D pairs to colored point clouds
   - Camera intrinsics computed from 90° FOV

2. **Preprocessing**
   - Voxel downsampling (voxel size: 0.2m)
   - Normal estimation
   - FPFH feature computation

3. **Global Registration (RANSAC)**
   - Feature-based correspondence matching
   - Coarse alignment between consecutive frames

4. **Local Refinement (ICP)**
   - Point-to-point ICP
   - Iterative closest point matching (8 iterations)
   - SVD-based transformation estimation

5. **Trajectory Accumulation**
   - Transform point clouds to global coordinate frame
   - Accumulate poses for camera trajectory

6. **Evaluation**
   - Compare with ground truth poses
   - Compute mean L2 distance error

### Custom ICP Implementation

The `my_local_icp_algorithm` implements:

1. **Correspondence Finding**: Nearest neighbor search for each source point
2. **Outlier Filtering**: Distance threshold-based filtering
3. **Transformation Estimation**:
   - Compute centroids of matched point sets
   - Center point clouds
   - Compute cross-covariance matrix
   - SVD decomposition: H = U·S·Vᵀ
   - Rotation: R = Vᵀ·Uᵀ
   - Handle reflection case (det(R) < 0)
   - Translation: t = μ_target - R·μ_source
4. **Iterative Refinement**: Repeat for 8 iterations

**Key Parameters:**
- Voxel size: 0.2m (for downsampling)
- ICP iterations: 8
- Depth range: 0-10m
- Ceiling removal offset: 0.2m

## Visualization

**Reconstruction Output:**
- Reconstructed 3D point cloud (colored)
- Estimated trajectory (red line)
- Ground truth trajectory (black line)

## Expected Results

- **Data Collection**: RGB-D-Semantic sequences with ground truth poses
- **BEV Projection**: Accurate geometric mapping between views
- **3D Reconstruction**: 
  - Dense colored point cloud reconstruction
  - Low L2 trajectory error (< 0.1m for good alignment)
  - Visually consistent 3D model

## Tips and Troubleshooting

1. **Data Collection**:
   - Navigate slowly for better coverage
   - Avoid rapid rotations
   - Cover the full floor systematically

2. **BEV Projection**:
   - Select points carefully to define the region of interest
   - Points should form a closed polygon

3. **Reconstruction**:
   - More frames = better reconstruction but slower processing
   - `my_icp` may be faster but less robust than `open3d`
   - Adjust voxel size for speed/quality tradeoff
   - If reconstruction fails, check that data was collected properly

## File Structure

```
hw1/
├── bev.py                    # BEV projection script
├── load.py                   # Data collection script
├── reconstruct.py            # 3D reconstruction script
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── bev_data/                 # BEV projection test data
│   ├── front1.png
│   └── bev1.png
├── replica_v1/               # Replica dataset
│   └── apartment_0/
│       └── habitat/
│           └── mesh_semantic.ply
└── data_collection/          # Collected data (generated)
    ├── first_floor/
    └── second_floor/
```

## References

- [Habitat-Sim](https://github.com/facebookresearch/habitat-sim): 3D simulation platform
- [Open3D](http://www.open3d.org/): 3D data processing library
- [Replica Dataset](https://github.com/facebookresearch/Replica-Dataset): High-quality 3D reconstructions
