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

    # # floor with tables
    # center = np.array([-5.0, 3.0, -0.5])
    # extent = np.array([15.0, 15.0, 2.0])
    # theta = np.deg2rad(50)
    # R = np.array([
    #     [np.cos(theta), -np.sin(theta), 0],
    #     [np.sin(theta), np.cos(theta), 0],
    #     [0, 0, 1]
    # ])

    # # only 1 table
    # center = np.array([2.2, 3.3, -0.5])

    # extent = np.array([3.5, 3.5, 2.0])
    # theta = np.deg2rad(50)
    # R = np.array([
    #     [np.cos(theta), -np.sin(theta), 0],
    #     [np.sin(theta), np.cos(theta), 0],
    #     [0, 0, 1]
    # ])

    # # room 3023 moved cut
    # center = np.array([0.0, 0.0, -0.3])

    # extent = np.array([8.0, 8.0, 1.3])
    # theta = np.deg2rad(9)
    # R = np.array([
    #     [np.cos(theta), -np.sin(theta), 0],
    #     [np.sin(theta), np.cos(theta), 0],
    #     [0, 0, 1]
    # ])
    ## R = np.eye(3)

    # # room 3023 del cut
    # center = np.array([0.0, 0.0, -0.3])

    # extent = np.array([8.0, 8.0, 1.3])
    # theta = np.deg2rad(0)
    # R = np.array([
    #     [np.cos(theta), -np.sin(theta), 0],
    #     [np.sin(theta), np.cos(theta), 0],
    #     [0, 0, 1]
    # ])

    # obb = o3d.geometry.OrientedBoundingBox(center, R, extent)

    # cropped_pcd = pcd.crop(obb)
    cropped_pcd = pcd
    print(cropped_pcd)
    print(f"Number of points: {len(cropped_pcd.points)}")

    # file_path = "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/cutted/cutted-3023-del.ply"
    # o3d.io.write_point_cloud(file_path, cropped_pcd)

    o3d.visualization.draw_geometries(
        # [pcd, aabb],
        [cropped_pcd],
        # window_name="Open3D PLY Viewer",
        # width=1280,
        # height=720,
        # left=50,
        # top=50,
        # point_show_normal=False
    )



if __name__ == "__main__":
    main()
