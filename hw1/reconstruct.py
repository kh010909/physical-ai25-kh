import numpy as np
import open3d as o3d
import argparse
import os
from tqdm import tqdm

def depth_image_to_point_cloud(rgb, depth):
    # TODO: Get point cloud from rgb and depth image 
    rgb_img = o3d.io.read_image(rgb_path)
    depth_img = o3d.io.read_image(depth_path)
    
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_img, depth_img, depth_scale=DEPTH_SCALE, depth_trunc=5.0, convert_rgb_to_intensity=False)
    
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, INTRINSICS)
    
    # Flip the point cloud to an upright orientation
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd


def preprocess_point_cloud(pcd, voxel_size):
    # TODO: Do voxelization to reduce the number of points for less memory usage and speedup
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    return pcd_down


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
       # RANSAC 的距離閾值，通常設為 voxel_size 的倍數
    distance_threshold = voxel_size * 1.5
    
    # 1. 計算 FPFH 特徵
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))

    # 3. 執行基於特徵匹配的 RANSAC
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def local_icp_algorithm(source_down, target_down, trans_init, threshold):
    # TODO: Use Open3D ICP function to implement
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    return result


def my_local_icp_algorithm(source_down, target_down, trans_init, voxel_size):
    # TODO: Write your own ICP function
    current_transformation = trans_init
    source_points_orig = np.asarray(source_down.points)
    target_kdtree = o3d.geometry.KDTreeFlann(target_down)
    
    max_iterations = 50
    convergence_threshold = 1e-6
    prev_rmse = float('inf')

    for _ in range(max_iterations):
        source_points_transformed = o3d.geometry.PointCloud()
        source_points_transformed.points = o3d.utility.Vector3dVector(source_points_orig)
        source_points_transformed.transform(current_transformation)
        
        # 1. Find correspondences using KD-Tree
        correspondences = []
        for i, point in enumerate(np.asarray(source_points_transformed.points)):
            [k, idx, _] = target_kdtree.search_knn_vector_3d(point, 1)
            correspondences.append([i, idx[0]])
        
        corr = o3d.utility.Vector2iVector(correspondences)
        
        # 2. Calculate RMSE and check for convergence
        pcd_corr = source_points_transformed.select_by_index(list(range(len(source_points_transformed.points))))
        corr_dist = pcd_corr.compute_point_cloud_distance(target_down.select_by_index([c[1] for c in correspondences]))
        rmse = np.sqrt(np.mean(np.asarray(corr_dist)**2))
        
        if abs(prev_rmse - rmse) < convergence_threshold:
            break
        prev_rmse = rmse
        
        # 3. Solve for transformation using point-to-point error metric
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        delta_transform = estimation.compute_transformation(source_down, target_down, corr)
        
        # 4. Update transformation
        current_transformation = delta_transform @ current_transformation

    result = o3d.pipelines.registration.RegistrationResult()
    result.transformation = current_transformation
    result.inlier_rmse = prev_rmse
    return result


def reconstruct(args):
    # TODO: Return results
    """
    For example:
        ...
        args.version == 'open3d':
            trans = local_icp_algorithm()
        args.version == 'my_icp':
            trans = my_local_icp_algorithm()
        ...
    """
    data_root = args.data_root
    rgb_files = sorted([os.path.join(data_root, 'rgb', f) for f in os.listdir(os.path.join(data_root, 'rgb'))])
    depth_files = sorted([os.path.join(data_root, 'depth', f) for f in os.listdir(os.path.join(data_root, 'depth'))])
    
    voxel_size = 0.05
    
    result_pcd = o3d.geometry.PointCloud()
    pred_cam_poses = []
    
    # Initialize with the first frame
    cumulative_transform = np.identity(4)
    pred_cam_poses.append(np.linalg.inv(cumulative_transform))
    
    pcd_prev = depth_image_to_point_cloud(rgb_files[0], depth_files[0])
    result_pcd += pcd_prev
    
    pbar = tqdm(range(1, len(rgb_files)))
    for i in pbar:
        pbar.set_description(f"Aligning frame {i+1}/{len(rgb_files)}")
        pcd_curr = depth_image_to_point_cloud(rgb_files[i], depth_files[i])
        
        # Source is the current frame, target is the accumulated previous frames
        source_down = preprocess_point_cloud(pcd_curr, voxel_size)
        target_down = preprocess_point_cloud(pcd_prev, voxel_size)
        
        trans_init = np.identity(4)
        threshold = voxel_size * 1.5

        if args.version == 'open3d':
            reg_result = local_icp_algorithm(source_down, target_down, trans_init, threshold)
        elif args.version == 'my_icp':
            reg_result = my_local_icp_algorithm(source_down, target_down, trans_init, voxel_size)
        else:
            raise NotImplementedError("Version must be 'open3d' or 'my_icp'")

        # The result transforms source to target; we need its inverse for the camera pose update
        relative_transform = reg_result.transformation
        cumulative_transform = cumulative_transform @ np.linalg.inv(relative_transform)
        pred_cam_poses.append(np.linalg.inv(cumulative_transform))
        
        pcd_curr.transform(cumulative_transform)
        result_pcd += pcd_curr
        # Voxel down-sample the merged cloud to manage memory
        result_pcd = result_pcd.voxel_down_sample(voxel_size)
        
        pcd_prev = pcd_curr
    return result_pcd, np.array(pred_cam_poses)

def create_trajectory_lineset(poses, color):
    """ Creates an Open3D LineSet object for visualizing a camera trajectory. """
    points = [pose[:3, 3] for pose in poses]
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    lineset.paint_uniform_color(color)
    return lineset

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
    
    # TODO: Output result point cloud and estimated camera pose
    '''
    Hint: Follow the steps on the spec
    '''
    result_pcd, pred_cam_pos = reconstruct()
    # Load ground truth poses for comparison
    gt_poses = np.load(os.path.join(args.data_root, 'GT_pose.npy'))

    # Align estimated trajectory to ground truth trajectory for accurate comparison [cite: 64]
    alignment_transform = gt_poses[0] @ np.linalg.inv(pred_cam_pos[0])
    pred_pos_aligned = np.array([alignment_transform @ pose for pose in pred_cam_pos])
    # TODO: Calculate and print L2 distance
    '''
    Hint: Mean L2 distance = mean(norm(ground truth - estimated camera trajectory))
    '''
    min_len = min(len(pred_pos_aligned), len(gt_poses))
    trans_pred = pred_pos_aligned[:min_len, :3, 3]
    trans_gt = gt_poses[:min_len, :3, 3]
    l2_distances = np.linalg.norm(trans_pred - trans_gt, axis=1)
    print(f"Mean L2 distance: {np.mean(l2_distances):.4f}")

    # TODO: Visualize result
    '''
    Hint: Sould visualize
    1. Reconstructed point cloud
    2. Red line: estimated camera pose
    3. Black line: ground truth camera pose
    '''
     # Remove ceiling from the point cloud for better visualization [cite: 66]
    max_bound = result_pcd.get_max_bound()
    min_bound = result_pcd.get_min_bound()
    # Crop points that are in the top 20% of the Y-axis range
    crop_y = min_bound[1] + (max_bound[1] - min_bound[1]) * 0.80
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, [max_bound[0], crop_y, max_bound[2]])
    result_pcd_no_ceiling = result_pcd.crop(bbox)

    # Visualize result [cite: 62, 63, 69, 70]
    est_traj_lines = create_trajectory_lineset(pred_pos_aligned, color=[1, 0, 0]) # Red
    gt_traj_lines = create_trajectory_lineset(gt_poses, color=[0, 0, 0]) # Black
    
    print("Displaying reconstructed scene with camera trajectories...")
    o3d.visualization.draw_geometries(
        [result_pcd_no_ceiling, est_traj_lines, gt_traj_lines],
        window_name=f"Reconstruction - Floor {args.floor} ({args.version})")
    # o3d.visualization.draw_geometries()
