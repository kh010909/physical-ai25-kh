import numpy as np
import open3d as o3d
import argparse
import os
import glob
import copy
from tqdm import tqdm

def depth_image_to_point_cloud(rgb, depth):
    """
    Converts an RGB and a depth image into a 3D point cloud.
    """
    # Using Open3D's built-in RGBD to point cloud conversion
    h, w = depth.shape
    f = w / (2 * np.tan(np.radians(90) / 2))
    cx, cy = w / 2.0, h / 2.0
    
    # Create camera intrinsic parameters
    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
    camera_intrinsic.set_intrinsics(w, h, f, f, cx, cy)

    # Convert depth to float32 and scale to meters (0-255 -> 0-10m)
    depth_meters = depth.astype(np.float32) / 255.0 * 10.0
    depth_image = o3d.geometry.Image(depth_meters)
    
    # Convert RGB to Open3D format
    color_image = o3d.geometry.Image(rgb)
    
    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, 
        depth_scale=1.0,  # Already scaled to meters
        depth_trunc=10.0,  # Truncate depth at 10m
        convert_rgb_to_intensity=False
    )
    
    # Create point cloud from RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, camera_intrinsic
    )
    
    return pcd

def preprocess_point_cloud(pcd, voxel_size):
    """
    Use voxelization to reduce the number of points.
    """
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down


def compute_fpfh(pcd, voxel_size):
    """
    Compute FPFH feature for a point cloud.
    """
    radius_normal = voxel_size * 2
    radius_feature = voxel_size * 5

    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    """
    Performs global registration on two downsampled point clouds using their FPFH features.
    """
    # RANSAC distance threshold
    distance_threshold = voxel_size * 1.5
    
    # Execute RANSAC based on feature matching
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    return result


def local_icp_algorithm(source_down, target_down, trans_init, threshold):
    """
    Use Open3D function to implement ICP.
    """
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return result


def my_local_icp_algorithm(source_down, target_down, trans_init, voxel_size):
    """
    Use my own function to implement ICP.
    """
    # ICP parameters
    max_iterations = 8
    threshold = voxel_size
    source_pts = np.asarray(source_down.points)
    target_pts = np.asarray(target_down.points)
    T = np.copy(trans_init)

    # Initialize mask for valid correspondences
    inlier_mask = np.ones(len(source_pts), dtype=bool)

    # ICP iterations
    for iteration in range(max_iterations):
        # Apply transformation to source points
        source_transformed = (T[:3, :3] @ source_pts.T).T + T[:3, 3]

        # Find nearest neighbors in target
        diffs = source_transformed[:, None, :] - target_pts[None, :, :]
        distances = np.sum(diffs ** 2, axis=2)
        nearest_indices = np.argmin(distances, axis=1)

        # Filter correspondences by distance threshold
        inlier_mask = np.sqrt(distances[np.arange(len(source_pts)), nearest_indices]) < threshold
        src_inliers = source_transformed[inlier_mask]
        tgt_inliers = target_pts[nearest_indices[inlier_mask]]

        # Compute optimal transformation using SVD
        # Calculate centroids
        src_mean = np.mean(src_inliers, axis=0)
        tgt_mean = np.mean(tgt_inliers, axis=0)

        # Center point sets
        src_centered = src_inliers - src_mean
        tgt_centered = tgt_inliers - tgt_mean

        # Compute cross-covariance and apply SVD
        covariance = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(covariance)
        R = Vt.T @ U.T

        # Correct for reflection case
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        # Calculate translation vector
        t = tgt_mean - R @ src_mean

        # Construct incremental transformation matrix
        T_delta = np.eye(4)
        T_delta[:3, :3] = R
        T_delta[:3, 3] = t

        # Update cumulative transformation
        T = T_delta @ T

    # Prepare output result object
    reg_result = o3d.pipelines.registration.RegistrationResult()
    reg_result.transformation = T
    reg_result.fitness = np.mean(inlier_mask)
    reg_result.inlier_rmse = threshold

    return reg_result


def reconstruct(args):
    """
    The main reconstruct function.
    """
    data_dir = args.data_root
    
    # Gather RGB and depth image paths
    rgb_image_paths = glob.glob(os.path.join(data_dir, 'rgb', '*.png'))
    depth_image_paths = glob.glob(os.path.join(data_dir, 'depth', '*.png'))

    # Map paths by frame number
    rgb_dict = {int(os.path.splitext(os.path.basename(p))[0]): p for p in rgb_image_paths}
    depth_dict = {int(os.path.splitext(os.path.basename(p))[0]): p for p in depth_image_paths}

    # Get intersection of frame IDs and sort
    frame_ids = sorted(set(rgb_dict.keys()) & set(depth_dict.keys()))
    rgb_sequence = [rgb_dict[fid] for fid in frame_ids]
    depth_sequence = [depth_dict[fid] for fid in frame_ids]
    
    # Configuration
    downsample_voxel_size = 0.2
    
    # Accumulation structures
    merged_pcd = o3d.geometry.PointCloud()
    camera_trajectory = [np.eye(4)]
    accumulated_pose = np.eye(4)
    
    progress_bar = tqdm(range(1, len(rgb_sequence)), desc="Processing frames")
    for idx in progress_bar:
        # Read consecutive frame pairs
        rgb_target = o3d.io.read_image(rgb_sequence[idx - 1])
        depth_target = o3d.io.read_image(depth_sequence[idx - 1])
        rgb_source = o3d.io.read_image(rgb_sequence[idx])
        depth_source = o3d.io.read_image(depth_sequence[idx])

        # Generate point clouds from RGB-D data
        pcd_target = depth_image_to_point_cloud(np.asarray(rgb_target), np.asarray(depth_target))
        pcd_source = depth_image_to_point_cloud(np.asarray(rgb_source), np.asarray(depth_source))

        # Preprocessing: downsample and extract features
        pcd_target_processed = preprocess_point_cloud(pcd_target, downsample_voxel_size)
        pcd_source_processed = preprocess_point_cloud(pcd_source, downsample_voxel_size)
        features_target = compute_fpfh(pcd_target_processed, downsample_voxel_size)
        features_source = compute_fpfh(pcd_source_processed, downsample_voxel_size)

        # Coarse alignment via global registration
        coarse_registration = execute_global_registration(
            pcd_source_processed, pcd_target_processed, 
            features_source, features_target, downsample_voxel_size
        )
        initial_alignment = coarse_registration.transformation

        # Fine alignment via ICP refinement
        if args.version == 'open3d':
            refined_registration = local_icp_algorithm(
                pcd_source_processed, pcd_target_processed,
                initial_alignment, downsample_voxel_size * 1.5
            )
        else:
            refined_registration = my_local_icp_algorithm(
                pcd_source_processed, pcd_target_processed,
                initial_alignment, downsample_voxel_size
            )

        # Get relative transformation between frames
        relative_transform = refined_registration.transformation

        # Accumulate transformation in global coordinate frame
        accumulated_pose = accumulated_pose @ relative_transform

        # Store camera pose
        camera_trajectory.append(accumulated_pose.copy())

        # Transform source point cloud to global frame and merge
        pcd_source_global = copy.deepcopy(pcd_source_processed)
        pcd_source_global.transform(accumulated_pose)
        merged_pcd += pcd_source_global

    # Convert trajectory list to array
    camera_trajectory = np.array(camera_trajectory)

    return merged_pcd, camera_trajectory


def remove_ceiling_points(pcd, starting_height, offset):
    """
    Remove ceiling points from the point cloud based on relative height from starting point.
    """
    if len(pcd.points) == 0:
        return pcd

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None

    # Remove points higher than starting_height + offset
    ceiling_threshold = starting_height - offset
    mask = points[:, 1] > ceiling_threshold

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    if colors is not None:
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

    return filtered_pcd


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--floor', type=int, default=1)
    parser.add_argument('-v', '--version', type=str, default='my_icp', help='open3d or my_icp')
    parser.add_argument('--data_root', type=str, default='data_collection/first_floor/')
    args = parser.parse_args()

    if args.floor == 1:
        args.data_root = "data_collection/first_floor/"
    elif args.floor == 2:
        args.data_root = "data_collection/second_floor/"
    

    result_pcd, pred_cam_pos = reconstruct(args)
    
    # Load ground truth poses (shape: Nx7 [x, y, z, qw, qx, qy, qz])
    gt_poses = np.load(os.path.join(args.data_root, 'GT_pose.npy'))

    # Reflect positions across the XY plane to align with reconstruction (position only)
    gt_poses[:, 2] *= -1


    pred_positions = np.array([pose[:3, 3] for pose in pred_cam_pos])
    gt_positions = gt_poses[:, :3]
    assert len(pred_positions) == len(gt_positions)
    min_len = len(pred_positions)

    # Align predicted trajectory to ground truth
    offset = gt_positions[0] - pred_positions[0]
    pred_positions_aligned = pred_positions + offset

    # Compute L2 distance with alignment
    distances = np.linalg.norm(pred_positions_aligned - gt_positions, axis=1)
    mean_l2 = np.mean(distances)
    print(f"Mean L2 distance: {mean_l2:.6f} m")

    # Apply the same offset to the point cloud
    alignment_transform = np.eye(4)
    alignment_transform[:3, 3] = offset
    result_pcd.transform(alignment_transform)


    # Remove ceiling points before visualization
    starting_height = gt_positions[0, 1]
    result_pcd = remove_ceiling_points(
        result_pcd,
        starting_height=starting_height,
        offset=0.2
    )

    # Visualize the trajectory
    lines = [[i, i + 1] for i in range(min_len - 1)]

    # Estimated trajectory (red)
    est_traj_lines = o3d.geometry.LineSet()
    est_traj_lines.points = o3d.utility.Vector3dVector(pred_positions_aligned)
    est_traj_lines.lines = o3d.utility.Vector2iVector(lines)
    est_traj_lines.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])

    # Ground truth trajectory (black)
    gt_traj_lines = o3d.geometry.LineSet()
    gt_traj_lines.points = o3d.utility.Vector3dVector(gt_positions)
    gt_traj_lines.lines = o3d.utility.Vector2iVector(lines)
    gt_traj_lines.colors = o3d.utility.Vector3dVector([[0, 0, 0] for _ in lines])

    # Visualize together
    print("Displaying reconstructed scene with camera trajectories...")
    o3d.visualization.draw_geometries(
        [result_pcd, est_traj_lines, gt_traj_lines],
        window_name=f"Reconstruction - Floor {args.floor} ({args.version})"
    )
