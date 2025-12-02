# /// script
# dependencies = ["open3d", "numpy"]
# requires-python = "==3.12"
# ///

import sys
import open3d as o3d
import numpy as np


def main():
    if len(sys.argv) < 2:
        print("Usage: uv run visualize_ply.py <path_to_file.ply>")
        sys.exit(1)

    ply_path = sys.argv[1]
    print(f"📂 Loading point cloud from: {ply_path}")

    # Загружаем облако точек
    pcd = o3d.io.read_point_cloud(ply_path)
    if not pcd.has_points():
        print("❌ Error: файл пустой или не удалось загрузить.")
        sys.exit(1)

    # Информация о файле
    print(pcd)
    print(f"Number of points: {len(pcd.points)}")
    pcd_downsampled = pcd.voxel_down_sample(0.02)

    file_path = "/Users/nikolayborovets/Desktop/for_gif/down/obb_aud_3005_down_0.02.ply"
    o3d.io.write_point_cloud(file_path, pcd_downsampled)

    o3d.visualization.draw_geometries(
        # [pcd, aabb],
        [pcd],
        # window_name="Open3D PLY Viewer",
        # width=1280,
        # height=720,
        # left=50,
        # top=50,
        # point_show_normal=False
    )



if __name__ == "__main__":
    main()
