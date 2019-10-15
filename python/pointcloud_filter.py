import numpy as np
import open3d as o3d

plotOn = True

if __name__ == "__main__":
    print("Load a pcd point cloud, print it, and render it")
    #pcd = o3d.io.read_point_cloud("../test_pcd.pcd")
    pcd = o3d.io.read_point_cloud("../data/PointCloud2.pcd")
    
    # get the point cloud colors as numpy array
    pcd_colors = np.asarray(pcd.colors)
    # color wanted is (0.025, 0.16, 0.75) which is the tiel bowl so use color distance. 
    # WARNIG this is not illumination or shadow robust.
    pcd_color_dist = np.linalg.norm(np.subtract(pcd_colors, np.array([0.025, 0.16, 0.75])), axis=1)
    pcd_color_dist_sorted = np.argsort(pcd_color_dist)
    # get index of colors that are close enough
    pcd_color_dist_filt = pcd_color_dist < 0.35
    print(pcd_color_dist_filt)

    # filter the original point cloud
    pcd_points_filtered = np.asarray(pcd.points)[pcd_color_dist_filt]
    pcd_color_filtered = np.asarray(pcd.colors)[pcd_color_dist_filt]

    # create new point cloud
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(pcd_points_filtered)
    new_pcd.colors = o3d.utility.Vector3dVector(pcd_color_filtered)
    if plotOn:
      vis = o3d.visualization.VisualizerWithEditing()
      vis.create_window()
      vis.add_geometry(new_pcd)
      vis.run()
      vis.destroy_window()
      #o3d.visualization.draw_geometries([pcd])
