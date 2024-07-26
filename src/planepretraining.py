import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/media/zac/E/project/leju-shenzhen/101/leju_01/bag1_to_pcd/1721394820.887587070.pcd")  


#o3d.visualization.draw_geometries([pcd])  


# 使用RANSAC算法提取平面  
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)  
[a, b, c, d] = plane_model  
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")  
  
# 可视化平面内点  
inlier_cloud = pcd.select_by_index(inliers)  
#inlier_cloud.color = [1, 0, 0]  
#outlier_cloud = pcd.select_by_index(inliers, invert=True)  
#outlier_cloud.color = [0, 0, 1]  
#o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])  

# 计算平面法线  
normal = np.array([a, b, c])  
  
# 假设参考平面是水平面（0,0,1）  
reference_normal = np.array([0, 0, 1])  
  
# 计算两个向量之间的角度  
angle = np.arccos(np.dot(normal, reference_normal) / (np.linalg.norm(normal) * np.linalg.norm(reference_normal)))  
print(angle)
#angle_deg = np.degrees(angle)  
#print(f"Angle between the plane and the horizontal plane: {angle_deg:.2f} degrees")
o3d.visualization.draw_geometries([inlier_cloud])


# 雷达与地面的夹角为0.12