import open3d as o3d
import numpy as np

# Load the PLY file
ply_file = "test.ply"
point_cloud = o3d.io.read_point_cloud(ply_file)

# Access points
points = np.asarray(point_cloud.points)

# Check and access normals
if point_cloud.has_normals():
    normals = np.asarray(point_cloud.normals)
    print("King. Normals exist")
else:
    print("No normals in the PLY file.")
    

point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

point_cloud.normals = o3d.utility.Vector3dVector(
    np.asarray(point_cloud.normals) * 0.005  # Adjust the multiplier as needed
)
# Optional: visualize the point cloud and normals
o3d.visualization.draw_geometries([point_cloud], point_show_normal=True,
                                  width=800,
                                  height=600,
                                  left=50,
                                  top=50)
    
# o3d.visualization.draw_geometries([point_cloud],
#                                   window_name="Point Cloud with Normals",
#                                   width=800,
#                                   height=600,
#                                   left=50,
#                                   top=50)