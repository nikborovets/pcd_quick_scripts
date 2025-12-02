# /// script
# dependencies = [
#   "laspy",
#   "open3d",
# ]
# ///

import sys
import laspy
import open3d as o3d


def convert_las_to_ply(input_path: str, output_path: str):
    """Конвертирует облако точек LAS → PLY"""
    print(f"Чтение {input_path} ...")
    las = laspy.read(input_path)

    # Основные координаты
    points = list(zip(las.x, las.y, las.z))

    # Создаём объект облака точек
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Если есть цвет
    if hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue"):
        colors = [
            (r / 65535, g / 65535, b / 65535)
            for r, g, b in zip(las.red, las.green, las.blue)
        ]
        pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Сохранение в {output_path} ...")
    o3d.io.write_point_cloud(output_path, pcd)
    print("✅ Конвертация завершена.")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Использование: uv run --script convert_las_to_ply.py input.las output.ply")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    convert_las_to_ply(input_file, output_file)
