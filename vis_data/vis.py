# /// script
# dependencies = [
#   "numpy>=2.3.5",
#   "open3d>=0.19.0",
# ]
# requires-python = "==3.12"
# ///

import open3d as o3d
import numpy as np
import os

from scipy.spatial.transform import Rotation as R

def read_traj(path):
    with open(path, "r") as f:
        lines = f.readlines()

    poses = []
    for line in lines:
        _, tx, ty, tz, qx, qy, qz, qw = line.split()
        pose = np.eye(4)
        pose[:3, :3] = R.from_quat([float(qx), float(qy), float(qz), float(qw)]).as_matrix()
        pose[:3, 3] = np.array([float(tx), float(ty), float(tz)])
        poses.append(pose)

    return poses

pwd = os.getcwd()

poses = read_traj(f"{pwd}/musicroom/trajectory_leica.txt")

pcd = o3d.io.read_point_cloud(f"{pwd}/musicroom/music_room_filtered.ply")

meshes = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1).transform(pose) for pose in poses]

o3d.visualization.draw_geometries([pcd] + meshes)