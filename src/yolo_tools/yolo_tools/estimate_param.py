import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

def draw_coodinate():
    pcd = o3d.geometry.PointCloud()
    num_points = 100  # 点の数
    colors = np.zeros((num_points, 3))  # (num_points, 3) の配列を作成
    points = np.linspace([0, 0, 0], [1, 0, 0], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [1, 0, 0]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    points = np.linspace([0, 0, 0], [0, 1, 0], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [0, 1, 0]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    points = np.linspace([0, 0, 0], [0, 0, 1], num=num_points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    colors[:] = [0, 0, 1]  # 全ての点に赤色を設定
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    pcd += point_cloud
    return pcd

def Colored_PCD_Registration(src,tgt):
    trans_icp = np.identity(4)
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14] * 2
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        #print([iter, radius, scale])

        #print("3-1. Downsample with a voxel size %.2f" % radius)
        #source_down = src
        #target_down = tgt
        source_down = src.voxel_down_sample(radius)
        target_down = tgt.voxel_down_sample(radius)
        #print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        #print("3-3. Applying colored point cloud registration")
        icp_result = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, trans_icp,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))

        return icp_result.transformation

def evaluate_trans(trans):
    
    t_vec = trans[:3,3]
    R_trans = np.copy(trans[:3,:3])
    rot =  Rotation.from_matrix(R_trans)
    quat = rot.as_quat()
    
    t = np.linalg.norm(t_vec,ord=2)
    th = 2*np.arccos(quat[3])
    print(t,th)
    #print(quat)



#pcd = draw_coodinate()
pcd = o3d.geometry.PointCloud()
pcd.estimate_normals()

for id in range(3):
    dir0 = f'./dataset_paramEst/data{id}'

    # PCDファイルを読み込む
    new_pcd = o3d.io.read_point_cloud(f"{dir0}/new_pcd.ply")
    new_pcd.estimate_normals()
    T1 = np.load(f"{dir0}/trans1.npy")
    T2 = np.load(f"{dir0}/trans2.npy")
    T3 = np.load(f"{dir0}/trans3.npy")
    T4 = np.load(f"{dir0}/trans4.npy")

    Td = np.identity(4)
    Td[:3,3] = np.array([0,0,0])

    Tc = T3@Td@T2@T1
    new_pcd.transform(Tc)

    #o3d.visualization.draw_geometries([self.pcd])
    if id != 0:
        trans_icp = Colored_PCD_Registration(new_pcd,pcd)
        norm =  np.linalg.norm(trans_icp[:3,3])
        evaluate_trans(trans_icp)
        new_pcd.transform(trans_icp)

    pcd += new_pcd

    # ポイントクラウドを可視化
    o3d.visualization.draw_geometries([pcd])
