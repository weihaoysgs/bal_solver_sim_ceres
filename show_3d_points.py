import open3d as o3d
import numpy as np


vis = o3d.visualization.Visualizer()
vis.create_window()

points = []
colors = []    
for i in range(5000):
    # 获取数据至 x,y,z r,g,b ,其中rgb范围为0-1.0
    x = ...
    y = ...
    z = ...
    r = ...
    g = ...
    b = ...
    points.append([x,y,z]) 
    colors.append([r,g,b])


pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)   

  
vis.add_geometry(pcd)
vis.poll_events()
vis.update_renderer()
vis.run() 