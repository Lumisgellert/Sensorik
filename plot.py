import open3d as o3d

# Punktwolke laden
pcd = o3d.io.read_point_cloud("scan_3d_20250101_120000.xyz", format='xyz')

# Visualisieren
o3d.visualization.draw_geometries([pcd])