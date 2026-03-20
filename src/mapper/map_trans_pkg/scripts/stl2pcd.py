import numpy as np
import open3d as o3d
import os

def transform_coordinates(points):
    """
    转换坐标系，使x向前、y向左、z向上
    
    参数:
        points: 原始点云坐标 (numpy数组)
    返回:
        转换后的点云坐标
    """
    # 定义旋转矩阵
    R = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    
    # 应用旋转
    transformed_points = points @ R.T
    
    return transformed_points

def stl_to_pcd(stl_path, output_path, num_points=int(1e5), scale_factor=0.001):
    """
    将STL文件转换为PCD文件
    
    参数:
        stl_path: STL文件路径
        output_path: 输出PCD文件路径
        num_points: 采样的点数
        scale_factor: 缩放因子，默认0.001表示从毫米转换为米
    """
    # 1. 读取STL文件
    print(f"正在读取STL文件: {stl_path}")
    mesh = o3d.io.read_triangle_mesh(stl_path)
    
    # 2. 检查网格是否有效
    if not mesh.has_vertices():
        raise ValueError("STL文件无效或为空")
    
    # 3. 计算网格的法向量
    mesh.compute_vertex_normals()
    
    # 4. 从网格表面均匀采样点云
    print(f"正在采样 {num_points} 个点...")
    point_cloud = mesh.sample_points_uniformly(number_of_points=num_points)
    
    # 5. 获取点云坐标并转换坐标系
    points = np.asarray(point_cloud.points)
    
    # 6. 应用缩放和坐标系变换
    # 先缩放，再变换坐标系
    scaled_points = points * scale_factor
    transformed_points = transform_coordinates(scaled_points)
    # Calculate the centroid (mean position) of all points
    centroid = np.mean(transformed_points, axis=0)
    # Subtract the centroid from all points to center them at origin
    transformed_points = transformed_points - centroid
    # Print the centroid information
    print(f"点云中心位置: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}] m")
    
    # 7. 更新点云坐标
    point_cloud.points = o3d.utility.Vector3dVector(transformed_points)
    
    # 8. 保存为PCD文件
    print(f"正在保存PCD文件: {output_path}")
    o3d.io.write_point_cloud(output_path, point_cloud)
    print("转换完成！")
    print(f"点云范围: x: [{np.min(transformed_points[:, 0]):.3f}, {np.max(transformed_points[:, 0]):.3f}] m")
    print(f"          y: [{np.min(transformed_points[:, 1]):.3f}, {np.max(transformed_points[:, 1]):.3f}] m")
    print(f"          z: [{np.min(transformed_points[:, 2]):.3f}, {np.max(transformed_points[:, 2]):.3f}] m")

if __name__ == "__main__":
    # 当前py文件所在的文件夹路径
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))

    stl_path = os.path.join(BASE_DIR, "./cad_maps/RMUL2026.STL")
    output_path = os.path.join(BASE_DIR, "./cad_maps/RMUL2026.pcd")
     
    num_points = int(1e6)
    
    try:
        stl_to_pcd(stl_path, output_path, num_points)
    except Exception as e:
        print(f"转换过程中出现错误: {str(e)}")