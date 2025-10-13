import argparse
import copy
import glob
import os

import numpy as np
import open3d as o3d
from tqdm import tqdm

def create_point_cloud_from_rgbd(rgb_image, depth_image):
    """
    根據給定的 RGB 和深度影像，生成一個 Open3D 點雲物件。

    Args:
        rgb_image (np.ndarray): 彩色影像。
        depth_image (np.ndarray): 深度影像 (通常為 8-bit)。

    Returns:
        o3d.geometry.PointCloud: 產生的彩色點雲。
    """
    # 從影像尺寸推斷相機內參
    image_height, image_width = depth_image.shape
    # 假設水平視野 (FOV) 為 90 度，以此計算焦距
    focal_length = image_width / (2 * np.tan(np.radians(90) / 2))
    camera_center_x = image_width / 2.0
    camera_center_y = image_height / 2.0

    # 設定 Open3D 相機內參物件
    camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        width=image_width,
        height=image_height,
        fx=focal_length,
        fy=focal_length,
        cx=camera_center_x,
        cy=camera_center_y
    )

    # 將 8-bit 深度影像轉換為浮點數，並將其範圍 (0-255) 縮放到指定的公尺單位 (0-10m)
    depth_scaled = depth_image.astype(np.float32) / 255.0 * 10.0
    
    # 將 NumPy 陣列轉換為 Open3D 影像格式
    o3d_color_image = o3d.geometry.Image(rgb_image)
    o3d_depth_image = o3d.geometry.Image(depth_scaled)

    # 建立 RGBD 影像物件
    # depth_scale=1.0 是因為深度值已經被縮放到公尺單位
    # depth_trunc=10.0 會忽略深度超過 10 公尺的點
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color_image,
        o3d_depth_image,
        depth_scale=1.0,
        depth_trunc=10.0,
        convert_rgb_to_intensity=False
    )

    # 從 RGBD 影像和相機內參生成點雲
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, camera_intrinsics
    )

    return point_cloud

def downsample_point_cloud(pcd, voxel_size):
    """
    使用體素化 (Voxelization) 對點雲進行降採樣，以減少計算負擔。

    Args:
        pcd (o3d.geometry.PointCloud): 原始點雲。
        voxel_size (float): 體素的大小。

    Returns:
        o3d.geometry.PointCloud: 降採樣後的點雲。
    """
    # 執行體素降採樣
    pcd_downsampled = pcd.voxel_down_sample(voxel_size)
    return pcd_downsampled

def calculate_fpfh_features(pcd, voxel_size):
    """
    計算點雲的 FPFH (Fast Point Feature Histograms) 特徵。

    Args:
        pcd (o3d.geometry.PointCloud): 輸入的點雲。
        voxel_size (float): 用於定義搜尋半徑的體素大小。

    Returns:
        o3d.pipelines.registration.Feature: 計算出的 FPFH 特徵。
    """
    # 設定計算法向量和 FPFH 特徵的搜尋半徑
    radius_for_normals = voxel_size * 2
    radius_for_features = voxel_size * 5

    # 估計法向量，這是計算 FPFH 的前提
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_for_normals, max_nn=30)
    )

    # 計算 FPFH 特徵
    fpfh_features = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_for_features, max_nn=100)
    )
    return fpfh_features

def perform_global_registration(source_pcd, target_pcd, source_features, target_features, voxel_size):
    """
    使用 RANSAC 演算法基於 FPFH 特徵進行全域註冊，以獲得一個粗略的變換矩陣。

    Args:
        source_pcd (o3d.geometry.PointCloud): 來源點雲。
        target_pcd (o3d.geometry.PointCloud): 目標點雲。
        source_features (o3d.pipelines.registration.Feature): 來源點雲的 FPFH 特徵。
        target_features (o3d.pipelines.registration.Feature): 目標點雲的 FPFH 特徵。
        voxel_size (float): 體素大小，用於設定距離閾值。

    Returns:
        o3d.pipelines.registration.RegistrationResult: 全域註冊的結果。
    """
    distance_threshold = voxel_size * 1.5

    # 執行基於特徵匹配的 RANSAC 註冊
    registration_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_pcd, target_pcd, source_features, target_features,
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
    return registration_result

def refine_registration_with_open3d_icp(source_pcd, target_pcd, initial_transform, distance_threshold):
    """
    使用 Open3D 內建的 ICP 演算法對齊點雲，以優化初始變換。

    Args:
        source_pcd (o3d.geometry.PointCloud): 來源點雲。
        target_pcd (o3d.geometry.PointCloud): 目標點雲。
        initial_transform (np.ndarray): 初始的 4x4 變換矩陣。
        distance_threshold (float): ICP 的最大對應點距離。

    Returns:
        o3d.pipelines.registration.RegistrationResult: ICP 註冊的結果。
    """
    # 使用 Open3D 的 ICP 函式進行局部對齊
    icp_result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, distance_threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return icp_result

def refine_registration_with_custom_icp(source_pcd, target_pcd, initial_transform, voxel_size):
    """
    使用自訂的 ICP 演算法實現，對齊點雲。

    Args:
        source_pcd (o3d.geometry.PointCloud): 來源點雲。
        target_pcd (o3d.geometry.PointCloud): 目標點雲。
        initial_transform (np.ndarray): 初始的 4x4 變換矩陣。
        voxel_size (float): 體素大小，用於設定距離閾值。

    Returns:
        o3d.pipelines.registration.RegistrationResult: 自訂 ICP 註冊的結果。
    """
    # ICP 演算法參數
    max_iterations = 8
    distance_threshold = voxel_size
    current_transform = np.copy(initial_transform)

    source_points = np.asarray(source_pcd.points)
    target_points = np.asarray(target_pcd.points)

    # 建立 KDTree 以加速最近鄰搜索
    target_kdtree = o3d.geometry.KDTreeFlann(target_pcd)

    # ICP 迭代過程
    for _ in range(max_iterations):
        # 應用當前變換到來源點雲
        source_transformed = copy.deepcopy(source_pcd).transform(current_transform)
        source_transformed_points = np.asarray(source_transformed.points)

        # 為每個轉換後的來源點尋找其在目標點雲中的最近鄰
        correspondences = []
        for i, point in enumerate(source_transformed_points):
            [k, idx, _] = target_kdtree.search_knn_vector_3d(point, 1)
            if k > 0:
                distance = np.linalg.norm(point - target_points[idx[0]])
                if distance < distance_threshold:
                    correspondences.append((i, idx[0]))

        # 從對應點中提取有效的點對
        source_corr_points = source_points[[corr[0] for corr in correspondences]]
        target_corr_points = target_points[[corr[1] for corr in correspondences]]

        if len(source_corr_points) < 10:  # 如果有效對應點太少，則提前終止
            break

        # 使用 SVD (奇異值分解) 估計新的變換
        # 1. 計算質心
        source_centroid = np.mean(source_corr_points, axis=0)
        target_centroid = np.mean(target_corr_points, axis=0)

        # 2. 將點去中心化
        source_centered = source_corr_points - source_centroid
        target_centered = target_corr_points - target_centroid

        # 3. 計算協方差矩陣 H
        H = source_centered.T @ target_centered

        # 4. 執行 SVD
        U, _, Vt = np.linalg.svd(H)
        
        # 5. 計算旋轉矩陣 R
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0: # 處理反射情況，確保是正規的旋轉矩陣
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        
        # 6. 計算平移向量 t
        t = target_centroid - R @ source_centroid

        # 建立此次迭代的增量變換矩陣
        delta_transform = np.identity(4)
        delta_transform[:3, :3] = R
        delta_transform[:3, 3] = t

        # 更新總變換矩陣
        current_transform = delta_transform @ current_transform

    # 建構最終的註冊結果
    registration_result = o3d.pipelines.registration.RegistrationResult()
    registration_result.transformation = current_transform
    # 這裡的 fitness 和 inlier_rmse 是簡化估計，非精確計算
    registration_result.fitness = len(correspondences) / len(source_points) if len(source_points) > 0 else 0.0
    registration_result.inlier_rmse = distance_threshold

    return registration_result


def run_reconstruction_pipeline(config):
    """
    執行完整的 3D 重建流程。

    Args:
        config (argparse.Namespace): 包含執行參數的設定物件。

    Returns:
        tuple: (重建的點雲, 預測的相機軌跡)。
    """
    data_path = config.data_root
    
    # 讀取並排序影像檔案路徑
    rgb_files = sorted(glob.glob(os.path.join(data_path, 'rgb', '*.png')), key=lambda x: int(os.path.basename(x).split('.')[0]))
    depth_files = sorted(glob.glob(os.path.join(data_path, 'depth', '*.png')), key=lambda x: int(os.path.basename(x).split('.')[0]))

    # 初始化參數
    voxel_size = 0.2
    accumulated_pcd = o3d.geometry.PointCloud()
    camera_poses = [np.identity(4)]
    global_transform = np.identity(4)

    # 迭代處理每對連續的影像幀
    progress_bar = tqdm(range(1, len(rgb_files)), desc="Processing frames")
    for i in progress_bar:
        # 讀取前後兩幀的影像
        prev_rgb = o3d.io.read_image(rgb_files[i - 1])
        prev_depth = o3d.io.read_image(depth_files[i - 1])
        curr_rgb = o3d.io.read_image(rgb_files[i])
        curr_depth = o3d.io.read_image(depth_files[i])

        # 將影像轉換為點雲
        pcd_prev = create_point_cloud_from_rgbd(np.asarray(prev_rgb), np.asarray(prev_depth))
        pcd_curr = create_point_cloud_from_rgbd(np.asarray(curr_rgb), np.asarray(curr_depth))

        # 預處理：降採樣與特徵計算
        pcd_prev_down = downsample_point_cloud(pcd_prev, voxel_size)
        pcd_curr_down = downsample_point_cloud(pcd_curr, voxel_size)
        fpfh_prev = calculate_fpfh_features(pcd_prev_down, voxel_size)
        fpfh_curr = calculate_fpfh_features(pcd_curr_down, voxel_size)

        # 步驟 1: 全域註冊 (粗略對齊)
        global_reg_result = perform_global_registration(
            pcd_curr_down, pcd_prev_down, fpfh_curr, fpfh_prev, voxel_size
        )

        # 步驟 2: 局部註冊 (精細對齊)
        if config.version == 'open3d':
            local_reg_result = refine_registration_with_open3d_icp(
                pcd_curr_down, pcd_prev_down,
                global_reg_result.transformation, voxel_size * 1.5
            )
        else: # 'my_icp'
            local_reg_result = refine_registration_with_custom_icp(
                pcd_curr_down, pcd_prev_down,
                global_reg_result.transformation, voxel_size
            )
        
        # 獲得從當前幀到前一幀的變換
        transform_curr_to_prev = local_reg_result.transformation
        
        # 累積變換，得到當前幀相對於世界座標系 (第一幀) 的變換
        global_transform = global_transform @ transform_curr_to_prev
        camera_poses.append(np.copy(global_transform))

        # 將當前降採樣後的點雲轉換到世界座標系並加入到總點雲中
        pcd_curr_world = copy.deepcopy(pcd_curr_down).transform(global_transform)
        accumulated_pcd += pcd_curr_world

    return accumulated_pcd, np.array(camera_poses)


def filter_ceiling(pcd, initial_camera_y, height_offset):
    """
    從點雲中移除天花板的點。

    Args:
        pcd (o3d.geometry.PointCloud): 原始點雲。
        initial_camera_y (float): 起始相機的 y 座標 (高度)。
        height_offset (float): 允許的高度偏移量。

    Returns:
        o3d.geometry.PointCloud: 過濾後的點雲。
    """
    if not pcd.has_points():
        return pcd

    points_np = np.asarray(pcd.points)
    
    # 定義天花板的高度閾值
    ceiling_y_threshold = initial_camera_y - height_offset
    
    # 建立一個遮罩，保留所有 y 座標大於 (即低於) 閾值的點
    mask = points_np[:, 1] > ceiling_y_threshold
    
    filtered_pcd = pcd.select_by_index(np.where(mask)[0])
    return filtered_pcd


def main():
    parser = argparse.ArgumentParser(description="3D reconstruction from RGB-D sequence.")
    parser.add_argument('-f', '--floor', type=int, default=1, choices=[1, 2], help="Floor number to process.")
    parser.add_argument('-v', '--version', type=str, default='my_icp', choices=['open3d', 'my_icp'], help="ICP implementation version.")
    parser.add_argument('--data_root', type=str, help="Override default data root path.")
    args = parser.parse_args()

    # 根據樓層設定資料路徑
    if args.data_root is None:
        if args.floor == 1:
            args.data_root = "data_collection/first_floor/"
        else:
            args.data_root = "data_collection/second_floor/"

    # 執行重建
    reconstructed_pcd, predicted_poses = run_reconstruction_pipeline(args)

    # 載入真實相機位姿 (Ground Truth)
    gt_poses_path = os.path.join(args.data_root, 'GT_pose.npy')
    gt_poses = np.load(gt_poses_path)
    
    # 反轉 z 軸以匹配重建座標系
    gt_poses[:, 2] *= -1

    # 提取位移向量
    predicted_positions = predicted_poses[:, :3, 3]
    gt_positions = gt_poses[:, :3]

    # 確保軌跡長度一致
    min_len = min(len(predicted_positions), len(gt_positions))
    predicted_positions = predicted_positions[:min_len]
    gt_positions = gt_positions[:min_len]

    # 對齊軌跡：將預測軌跡的起點與真實軌跡的起點對齊
    alignment_offset = gt_positions[0] - predicted_positions[0]
    predicted_positions_aligned = predicted_positions + alignment_offset

    # 計算平均 L2 距離誤差
    l2_distances = np.linalg.norm(predicted_positions_aligned - gt_positions, axis=1)
    mean_l2_distance = np.mean(l2_distances)
    print(f"Mean L2 distance after alignment: {mean_l2_distance:.6f} meters")

    # 將相同的對齊位移應用到點雲上
    alignment_transform = np.identity(4)
    alignment_transform[:3, 3] = alignment_offset
    reconstructed_pcd.transform(alignment_transform)
    
    # 移除天花板點以改善視覺效果
    start_height = gt_positions[0, 1]
    reconstructed_pcd_filtered = filter_ceiling(reconstructed_pcd, start_height, offset=0.2)

    # 準備視覺化物件
    # 1. 預測軌跡 (紅色線條)
    pred_lineset = o3d.geometry.LineSet()
    pred_lineset.points = o3d.utility.Vector3dVector(predicted_positions_aligned)
    line_indices = [[i, i + 1] for i in range(min_len - 1)]
    pred_lineset.lines = o3d.utility.Vector2iVector(line_indices)
    pred_lineset.paint_uniform_color([1, 0, 0]) # Red

    # 2. 真實軌跡 (黑色線條)
    gt_lineset = o3d.geometry.LineSet()
    gt_lineset.points = o3d.utility.Vector3dVector(gt_positions)
    gt_lineset.lines = o3d.utility.Vector2iVector(line_indices)
    gt_lineset.paint_uniform_color([0, 0, 0]) # Black

    # 視覺化重建結果
    print("Visualizing the reconstructed scene, predicted trajectory (red), and ground truth (black)...")
    o3d.visualization.draw_geometries(
        [reconstructed_pcd_filtered, pred_lineset, gt_lineset],
        window_name=f"Reconstruction Result - Floor {args.floor} ({args.version})"
    )

if __name__ == '__main__':
    main()