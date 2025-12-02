# /// script
# dependencies = ["open3d"]
# requires-python = "==3.12"
# ///

import sys
import open3d as o3d


def clean_ply(input_path: str, output_path: str, nb_neighbors: int = 20, std_ratio: float = 2.0):
    pcd = o3d.io.read_point_cloud(input_path)
    pcd_clean, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )

    # Сохраняем очищенные точки
    o3d.io.write_point_cloud(output_path, pcd_clean)
    print(f"✓ Saved cleaned point cloud to: {output_path}")
    print(f"Removed points: {len(pcd.points) - len(pcd_clean.points)}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: uv run --script clean_ply.py input.ply output.ply [nb_neighbors] [std_ratio]")
        sys.exit(1)

    input_ply = sys.argv[1]
    output_ply = sys.argv[2]
    nb_neighbors = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    std_ratio = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

    clean_ply(input_ply, output_ply, nb_neighbors, std_ratio)
