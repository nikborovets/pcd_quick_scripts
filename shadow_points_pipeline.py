# /// script
# dependencies = [
#   "pye57",
#   "numpy",
#   "open3d",
# ]
# requires-python = "==3.12"
# ///


import numpy as np
import open3d as o3d
import pye57
import json
from typing import Tuple, Optional, List, Dict
from pathlib import Path
import subprocess


def read_scan_with_pose(e57_file: str, scan_index: int, colors: bool = True) -> Tuple[np.ndarray, Optional[np.ndarray], np.ndarray, np.ndarray]:
    """
    Читает один скан из .e57 файла БЕЗ трансформации (в локальных координатах сканера)
    и сохраняет pose данные для последующего применения.

    Args:
        e57_file: путь к .e57 файлу
        scan_index: индекс скана для чтения
        colors: читать ли цвета

    Returns:
        points: массив точек (N, 3) в локальных координатах сканера
        colors_array: массив цветов (N, 3) или None
        rotation: кватернион вращения [w, x, y, z]
        translation: вектор трансляции [x, y, z]
    """
    e57 = pye57.E57(e57_file)

    # Получаем pose данные
    header = e57.get_header(scan_index)
    rotation = header.rotation
    translation = header.translation

    # Читаем точки БЕЗ трансформации (в локальных координатах сканера)
    data = e57.read_scan(scan_index, colors=colors, transform=False, ignore_missing_fields=True)

    # Извлекаем координаты
    points = np.vstack((data['cartesianX'], data['cartesianY'], data['cartesianZ'])).T

    # Извлекаем цвета если есть
    colors_array = None
    if colors and 'colorRed' in data and 'colorGreen' in data and 'colorBlue' in data:
        colors_array = np.vstack((data['colorRed'], data['colorGreen'], data['colorBlue'])).T
        colors_array = colors_array.astype(np.float64) / 255.0  # Нормализация в [0, 1]

    e57.close()
    return points, colors_array, rotation, translation


def save_pose_data(pose_data: List[Tuple[np.ndarray, np.ndarray]], filename: str = "pose_data.json") -> None:
    """
    Сохраняет pose данные (rotation и translation) в JSON файл.

    Args:
        pose_data: список кортежей (rotation, translation) для каждого скана
        filename: имя файла для сохранения
    """
    data_to_save = []
    for i, (rotation, translation) in enumerate(pose_data):
        data_to_save.append({
            "scan_index": i,
            "rotation": rotation.tolist(),
            "translation": translation.tolist()
        })

    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(data_to_save, f, indent=2, ensure_ascii=False)

    print(f"Pose данные сохранены в файл: {filename}")


def load_pose_data(filename: str = "pose_data.json") -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    Загружает pose данные из JSON файла.

    Args:
        filename: имя файла для загрузки

    Returns:
        pose_data: список кортежей (rotation, translation)
    """
    with open(filename, 'r', encoding='utf-8') as f:
        data = json.load(f)

    pose_data = []
    for item in data:
        rotation = np.array(item["rotation"])
        translation = np.array(item["translation"])
        pose_data.append((rotation, translation))

    print(f"Pose данные загружены из файла: {filename}")
    return pose_data


def apply_pose_transformation(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    Применяет pose трансформацию к точкам (из локальных координат в глобальные).

    Args:
        points: массив точек (N, 3) в локальных координатах сканера
        rotation: кватернион вращения [w, x, y, z]
        translation: вектор трансляции [x, y, z]

    Returns:
        transformed_points: массив точек (N, 3) в глобальной системе координат
    """
    # Используем встроенную функцию pye57 для трансформации
    return pye57.E57.to_global(points, rotation, translation)


def save_points_to_ply(points: np.ndarray, colors: Optional[np.ndarray] = None, filename: str = "output.ply") -> None:
    """
    Сохраняет точки в .ply файл.

    Args:
        points: массив точек (N, 3)
        colors: массив цветов (N, 3) или None
        filename: имя выходного файла
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.io.write_point_cloud(filename, pcd)
    print(f"Файл {filename} успешно сохранен")


def load_points_from_ply(filename: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Загружает точки из .ply файла (после обработки в PCL).

    Args:
        filename: имя .ply файла

    Returns:
        points: массив точек (N, 3)
        colors: массив цветов (N, 3) или None
    """
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    return points, colors


def voxel_downsample(points: np.ndarray, colors: Optional[np.ndarray], voxel_size: Optional[float], log_prefix: str = "") -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Обертка для voxel_down_sample с логом; не меняет данные если voxel_size=None."""
    if voxel_size is None:
        return points, colors

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    ds_points = np.asarray(pcd_downsampled.points)
    ds_colors = np.asarray(pcd_downsampled.colors) if pcd_downsampled.has_colors() else None

    print(f"{log_prefix}Downsampling: {len(points)} -> {len(ds_points)} точек")
    return ds_points, ds_colors


def propagate_shadow_removal_to_original(
    original_ply: str,
    removed_points_ply: str,
    output_ply: str,
    search_radius: float,
) -> None:
    """
    Переносит удаление shadow points с downsampled облака на оригинальное облако.

    Args:
        original_ply: путь к оригинальному (не downsampled) облаку
        removed_points_ply: путь к файлу с удаленными shadow points (из C++)
        output_ply: путь для сохранения результата
        search_radius: радиус поиска соседей (рекомендуется voxel_size * 1.5)
    """
    print("Перенос удаления shadow points на оригинальное облако")
    print(f"  Радиус поиска: {search_radius}")

    pcd_original = o3d.io.read_point_cloud(original_ply)
    pcd_removed = o3d.io.read_point_cloud(removed_points_ply)

    original_count = len(pcd_original.points)
    removed_count = len(pcd_removed.points)

    print(f"  Оригинальное облако: {original_count} точек")
    print(f"  Shadow points (downsampled): {removed_count} точек")

    if removed_count == 0:
        print("  Нет shadow points для удаления, сохраняем оригинал")
        o3d.io.write_point_cloud(output_ply, pcd_original)
        return

    tree = o3d.geometry.KDTreeFlann(pcd_original)

    indices_to_remove = set()
    removed_points = np.asarray(pcd_removed.points)

    for point in removed_points:
        k, idx, _ = tree.search_radius_vector_3d(point, search_radius)
        if k > 0:
            indices_to_remove.update(idx)

    if not indices_to_remove:
        print("  Не найдено соответствий в оригинале, сохраняем оригинал")
        o3d.io.write_point_cloud(output_ply, pcd_original)
        return

    all_indices = set(range(original_count))
    indices_to_keep = list(all_indices - indices_to_remove)

    pcd_filtered = pcd_original.select_by_index(indices_to_keep)
    o3d.io.write_point_cloud(output_ply, pcd_filtered)

    print(f"  Удалено точек: {len(indices_to_remove)}")
    print(f"  Осталось точек: {len(pcd_filtered.points)}")
    print(f"  Сохранено в: {output_ply}")


def prepare_scans_for_pcl_multi_file(e57_files: List[str], output_dir: str = "scans_for_pcl", downsample_voxel_size: Optional[float] = None) -> str:
    """
    Подготавливает сканы из нескольких E57 файлов для обработки в PCL.
    Каждый E57 файл может содержать 1+ сканов (станция + опционально iPad tags).
    Берет ВСЕ сканы из каждого файла и нумерует глобально.

    Args:
        e57_files: список путей к E57 файлам (каждый файл = одна станция)
        output_dir: директория для сохранения файлов
        downsample_voxel_size: размер вокселя для downsampling (None - без downsampling)

    Returns:
        pose_file: путь к файлу с pose данными
    """
    # Создаем директорию если не существует
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    print(f"Обработка {len(e57_files)} E57 файлов")
    if downsample_voxel_size is not None:
        print(f"Будет применен downsampling с voxel_size={downsample_voxel_size}")

    pose_data = []
    global_scan_index = 0

    for file_idx, e57_path in enumerate(e57_files):
        print(f"\n[Файл {file_idx + 1}/{len(e57_files)}] {Path(e57_path).name}")
        
        e57 = pye57.E57(e57_path)
        scan_count = e57.scan_count
        print(f"  Найдено сканов в файле: {scan_count}")
        e57.close()

        # Обрабатываем все сканы из текущего файла
        for local_scan_idx in range(scan_count):
            print(f"  Обрабатываем скан {local_scan_idx + 1}/{scan_count} (глобальный индекс: {global_scan_index})")

            # Читаем скан без трансформации
            points, colors, rotation, translation = read_scan_with_pose(e57_path, local_scan_idx)

            points_original = np.copy(points)
            colors_original = np.copy(colors) if colors is not None else None
            original_point_count = len(points_original)

            points, colors = voxel_downsample(points_original, colors_original, downsample_voxel_size, log_prefix="    ")

            original_ply_filename = f"{output_dir}/scan_{global_scan_index:03d}_original.ply"
            save_points_to_ply(points_original, colors_original, original_ply_filename)

            # Имена файлов с глобальной нумерацией для PCL (downsampled или оригинал)
            if downsample_voxel_size is not None:
                ply_filename = f"{output_dir}/scan_{global_scan_index:03d}_ds{downsample_voxel_size}_local.ply"
            else:
                ply_filename = f"{output_dir}/scan_{global_scan_index:03d}_local.ply"
            
            save_points_to_ply(points, colors, ply_filename)

            # Сохраняем pose данные с глобальным индексом
            pose_data.append((rotation, translation))

            print(f"    Точек: {len(points)} (было: {original_point_count})")
            
            global_scan_index += 1

    # Сохраняем pose данные
    pose_file = f"{output_dir}/pose_data.json"
    save_pose_data(pose_data, pose_file)

    voxel_info = f" с downsampling (voxel_size={downsample_voxel_size})" if downsample_voxel_size is not None else ""
    file_pattern = f"scan_XXX_ds{downsample_voxel_size}_local.ply" if downsample_voxel_size is not None else "scan_XXX_local.ply"

    print(f"\nПодготовка завершена{voxel_info}. Всего обработано сканов: {global_scan_index}")
    print(f"Файлы сохранены в директорию: {output_dir}")
    print(f"Теперь можно обработать файлы {file_pattern} в PCL с фильтром ShadowPoints")

    return pose_file


def prepare_scans_for_pcl(e57_file: str, output_dir: str = "scans_for_pcl", downsample_voxel_size: Optional[float] = None) -> str:
    """
    Подготавливает все сканы для обработки в PCL:
    - Читает каждый скан без трансформации
    - Опционально применяет downsampling
    - Сохраняет в локальные .ply файлы
    - Сохраняет pose данные в JSON

    Args:
        e57_file: путь к .e57 файлу
        output_dir: директория для сохранения файлов
        downsample_voxel_size: размер вокселя для downsampling (None - без downsampling)

    Returns:
        pose_file: путь к файлу с pose данными
    """
    # Создаем директорию если не существует
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # Получаем количество сканов
    e57 = pye57.E57(e57_file)
    scan_count = e57.scan_count
    e57.close()

    print(f"Найдено сканов: {scan_count}")
    if downsample_voxel_size is not None:
        print(f"Будет применен downsampling с voxel_size={downsample_voxel_size}")

    pose_data = []

    for i in range(scan_count):
        print(f"Обрабатываем скан {i + 1}/{scan_count}")

        # Читаем скан без трансформации
        points, colors, rotation, translation = read_scan_with_pose(e57_file, i)

        points_original = np.copy(points)
        colors_original = np.copy(colors) if colors is not None else None
        original_point_count = len(points_original)

        points, colors = voxel_downsample(points_original, colors_original, downsample_voxel_size, log_prefix="  ")

        original_ply_filename = f"{output_dir}/scan_{i:03d}_original.ply"
        save_points_to_ply(points_original, colors_original, original_ply_filename)

        # Имена файлов: scan_XXX_ds{voxel}_local.ply или scan_XXX_local.ply
        if downsample_voxel_size is not None:
            ply_filename = f"{output_dir}/scan_{i:03d}_ds{downsample_voxel_size}_local.ply"
        else:
            ply_filename = f"{output_dir}/scan_{i:03d}_local.ply"
        save_points_to_ply(points, colors, ply_filename)

        # Сохраняем pose данные
        pose_data.append((rotation, translation))

        print(f"  Точек: {len(points)} (было: {original_point_count})")

    # Сохраняем pose данные
    pose_file = f"{output_dir}/pose_data.json"
    save_pose_data(pose_data, pose_file)

    voxel_info = f" с downsampling (voxel_size={downsample_voxel_size})" if downsample_voxel_size is not None else ""
    file_pattern = f"scan_XXX_ds{downsample_voxel_size}_local.ply" if downsample_voxel_size is not None else "scan_XXX_local.ply"

    print(f"\nПодготовка завершена{voxel_info}. Файлы сохранены в директорию: {output_dir}")
    print(f"Теперь можно обработать файлы {file_pattern} в PCL с фильтром ShadowPoints")

    return pose_file


def register_processed_scans(processed_dir: str, pose_file: str, output_file: str = "final_registered_scans.ply", voxel_size: float | None = None, pattern: str = "scan_*_filtered.ply") -> None:
    """
    Регистрирует обработанные в PCL сканы в общую систему координат.
    Ожидает файлы вида {pattern}.
    
    Args:
        processed_dir: директория с обработанными .ply файлами
        pose_file: путь к файлу с pose данными
        output_file: имя выходного файла
        voxel_size: размер вокселя для финального downsampling
        pattern: шаблон для поиска обработанных файлов
    """
    # Загружаем pose данные
    pose_data = load_pose_data(pose_file)

    processed_files = sorted(Path(processed_dir).glob(pattern))

    if not processed_files:
        raise FileNotFoundError(f"Не найдены файлы {pattern} в {processed_dir}")

    scan_count = len(processed_files)
    print(f"Найдено обработанных файлов: {scan_count}")

    if len(pose_data) != scan_count:
        raise ValueError(f"Количество pose данных ({len(pose_data)}) не совпадает с количеством файлов ({scan_count})")

    print("Обработанные файлы:")
    for f in processed_files:
        print(f"  {f.name}")

    all_points = []
    all_colors = []

    for i, ply_file in enumerate(processed_files):
        print(f"Обрабатываем файл: {ply_file.name}")

        # Загружаем обработанные точки
        points, colors = load_points_from_ply(str(ply_file))

        # Применяем трансформацию для регистрации в глобальную систему
        rotation, translation = pose_data[i]
        transformed_points = apply_pose_transformation(points, rotation, translation)

        all_points.append(transformed_points)
        if colors is not None:
            all_colors.append(colors)

        print(f"  Точек после трансформации: {len(transformed_points)}")

    # Объединяем все сканы
    combined_points = np.vstack(all_points)
    print(f"Общее количество точек: {len(combined_points)}")

    # Создаем PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(combined_points)

    # Добавляем цвета если они есть во всех сканах
    if all_colors and len(all_colors) == len(all_points):
        combined_colors = np.vstack(all_colors)
        pcd.colors = o3d.utility.Vector3dVector(combined_colors)
        color_status = "с цветовой информацией"
    else:
        color_status = "без цветовой информации"

    # Downsampling
    if voxel_size is not None:
        print(f"Выполняем downsampling с voxel_size={voxel_size}")
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
        # Сохраняем результат
        o3d.io.write_point_cloud(output_file, pcd_downsampled)
        print(f"Файл {output_file} успешно создан {color_status}.")
        print(f"Финальное количество точек: {len(pcd_downsampled.points)}")
    else:
        o3d.io.write_point_cloud(output_file, pcd)
        print(f"Файл {output_file} успешно создан {color_status}.")
        print(f"Финальное количество точек: {len(pcd.points)}")


def export_scans_to_e57(
    processed_dir: str,
    pose_file: str,
    pattern: str,
    output_file: str,
) -> None:
    """
    Экспортирует локальные .ply сканы в один .e57, сохраняя pose (rotation, translation) для каждого скана.
    """
    pose_data = load_pose_data(pose_file)
    processed_files = sorted(Path(processed_dir).glob(pattern))

    if not processed_files:
        raise FileNotFoundError(f"Не найдены файлы {pattern} в {processed_dir}")

    if len(pose_data) != len(processed_files):
        raise ValueError(f"Количество pose данных ({len(pose_data)}) не совпадает с количеством файлов ({len(processed_files)})")

    print(f"Экспорт в E57: найдено {len(processed_files)} файлов по шаблону {pattern}")

    with pye57.E57(output_file, mode="w") as e57_out:
        for i, ply_file in enumerate(processed_files):
            print(f"[E57] {i+1}/{len(processed_files)}: {ply_file.name}")

            points, colors = load_points_from_ply(str(ply_file))

            data = {
                "cartesianX": points[:, 0],
                "cartesianY": points[:, 1],
                "cartesianZ": points[:, 2],
            }

            if colors is not None and len(colors) == len(points):
                # pye57 ожидает uint8
                data["colorRed"] = (colors[:, 0] * 255).astype(np.uint8)
                data["colorGreen"] = (colors[:, 1] * 255).astype(np.uint8)
                data["colorBlue"] = (colors[:, 2] * 255).astype(np.uint8)

            rotation, translation = pose_data[i]
            e57_out.write_scan_raw(
                data,
                rotation=rotation,
                translation=translation,
            )

    print(f"E57 экспорт завершен: {output_file}")


def remove_statistical_outliers(
    input_file: str,
    output_file: str,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
) -> None:
    """
    Удаляет статистические выбросы из облака точек.

    Args:
        input_file: путь к входному .ply файлу
        output_file: путь к выходному .ply файлу
        nb_neighbors: количество соседей для анализа каждой точки
        std_ratio: порог стандартного отклонения (точки с расстоянием > mean + std_ratio * std удаляются)
    """
    print(f"Удаление статистических выбросов из {Path(input_file).name}")
    print(f"  Параметры: nb_neighbors={nb_neighbors}, std_ratio={std_ratio}")

    # Загружаем облако точек
    pcd = o3d.io.read_point_cloud(input_file)
    original_count = len(pcd.points)
    print(f"  Исходное количество точек: {original_count}")

    # Удаляем выбросы
    pcd_clean, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )

    # Сохраняем результат
    o3d.io.write_point_cloud(output_file, pcd_clean)
    
    removed_count = original_count - len(pcd_clean.points)
    print(f"  Удалено выбросов: {removed_count}")
    print(f"  Финальное количество точек: {len(pcd_clean.points)}")
    print(f"  Сохранено в: {output_file}")


def run_pcl_shadow_filter_batch(
    output_dir: str,
    pcl_binary: str,
    radius_search: float,
    threshold: float,
    save_mode: str = "filtered",
) -> None:
    """Запускает внешний C++ ShadowPoints фильтр для всех локальных PLY сканов.

    Ищет файлы:
      - scan_XXX_ds*_local.ply
      - или scan_XXX_local.ply (если downsample не использовался)

    Создает:
      - scan_XXX_ds*_filtered.ply
      - или scan_XXX_filtered.ply
    """
    out_dir = Path(output_dir)
    if not out_dir.is_dir():
        raise FileNotFoundError(f"Директория не найдена: {output_dir}")

    local_files = sorted(out_dir.glob("scan_*_ds*_local.ply"))
    if not local_files:
        local_files = sorted(out_dir.glob("scan_*_local.ply"))

    if not local_files:
        raise FileNotFoundError(f"Не найдены файлы scan_*_ds*_local.ply или scan_*_local.ply в {output_dir}")

    print(f"Найдено {len(local_files)} локальных файлов для фильтрации ShadowPoints")

    for i, input_path in enumerate(local_files):
        stem = input_path.stem  # scan_XXX_ds0.01_local или scan_XXX_local
        if stem.endswith("_local"):
            filtered_stem = stem[:-len("_local")] + "_filtered"  # заменяем _local -> _filtered
        else:
            filtered_stem = stem + "_filtered"
        output_path = out_dir / f"{filtered_stem}.ply"

        cmd = [
            pcl_binary,
            str(radius_search),
            str(threshold),
            str(input_path.resolve()),
            str(output_path.resolve()),
            save_mode,
        ]

        print(f"[{i+1}/{len(local_files)}] Запуск ShadowPoints для {input_path.name}")
        print("Команда:", " ".join(cmd))

        subprocess.run(cmd, check=True)

        if not output_path.is_file():
            raise RuntimeError(f"Ожидаемый выходной файл не найден: {output_path}")

        print(f"  -> OK: {output_path.name}")


def main():
    """Полный автоматический пайплайн:

    1. Читает сканы из E57 без трансформации, опционально даунсемплит и сохраняет scan_XXX_ds*_local.ply
    2. Для каждого локального файла запускает внешний C++ ShadowPoints фильтр (pcd_write_test)
       и создает scan_XXX_ds*_filtered.ply
    3. Регистрирует отфильтрованные сканы в глобальную систему координат по pose_data.json
       и сохраняет финальный объединенный PLY.
    """

    # === НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ ===
    # # e57_file = "/Users/nikolayborovets/Downloads/Bundle_001.e57"  # Путь к .e57 файлу
    # # e57_file = "/Users/nikolayborovets/Desktop/skoltech/blk360_scan_test/data/music_room/music_room_3.e57"  # Путь к .e57 файлу
    # e57_file = "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/3023-moved.e57"  # Путь к .e57 файлу
    # file_name = Path(e57_file).stem
    
    # РЕЖИМ РАБОТЫ
    multi_file_mode = True  # True = несколько E57 файлов, False = один E57 файл
    
    if multi_file_mode:
        # # Список E57 файлов (каждый = одна станция с 1+ сканами)
        e57_files = [
            "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/init/E 3023 A5 init- Setup33.e57",
            "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/init/E 3023 A5 init- Setup34.e57",
            "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/init/E 3023 A5 init- Setup35.e57",
        ]
        file_name = "E_3023_A5_init"  # Название для output директории
        # # Список E57 файлов (каждый = одна станция с 1+ сканами)
        # e57_files = [
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/moved/E 3023 A5 moved- Setup36.e57",
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/moved/E 3023 A5 moved- Setup37.e57",
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/moved/E 3023 A5 moved- Setup38.e57",
        # ]
        # file_name = "E_3023_A5_moved"  # Название для output директории
        # Список E57 файлов (каждый = одна станция с 1+ сканами)
        # e57_files = [
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/del/E 3023 A5 del- Setup39.e57",
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/del/E 3023 A5 del- Setup40.e57",
        #     "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/del/E 3023 A5 del- Setup41.e57",
        # ]
        # file_name = "E_3023_A5_del"  # Название для output директории
    else:
        # Один E57 файл со всеми станциями
        # e57_file = "/Users/nikolayborovets/Downloads/Bundle_001.e57"
        # e57_file = "/Users/nikolayborovets/Desktop/skoltech/blk360_scan_test/data/music_room/music_room_3.e57"
        e57_file = "/Volumes/wBorovetsNV/Skoltech_laba/E-3023-A5-cutted/e57/3023-del.e57"
        file_name = Path(e57_file).stem
    
    # Общие параметры
    # pcl_binary = "/Users/nikolayborovets/Desktop/skoltech/libraries/pcl_using/build/pcd_write_test"  # Абсолютный путь к C++ бинарнику
    pcl_binary = "/Users/nikolayborovets/Desktop/skoltech/pcd_quick_scripts/cpp_pcl_shadowpoints/build/remove_shadowpoints"  # Абсолютный путь к C++ бинарнику
    radius_search = 0.025  # Радиус для NormalEstimationOMP в C++
    threshold = 0.1       # Порог ShadowPoints в C++
    downsample_voxel_size = 0.01  # Воксель для предварительного downsampling (None - без downsampling)
    save_mode = "filtered"  # sphere | highlighted | filtered

    # Параметры для удаления статистических выбросов
    remove_outliers = False  # True = удалять выбросы после регистрации
    nb_neighbors = 20  # Количество соседей для анализа
    std_ratio = 2.0  # Порог стандартного отклонения

    output_dir = f"shadowpoints_output/{file_name}_scans_for_pcl_{radius_search}_{threshold}_voxel_{downsample_voxel_size}_{save_mode}"  # Директория для промежуточных и итоговых файлов (можно сделать абсолютной)

    final_output = f"{output_dir}/final_registered_scans_filtered_full.ply"  # Итоговый объединенный файл после переноса (с propagate)
    final_output_downsampled = f"{output_dir}/final_registered_scans_filtered_downsampled_full.ply"  # Итоговый объединенный файл после переноса downsampled (без propagate)
    final_output_local = f"{output_dir}/final_registered_scans_original.ply"  # Итоговый объединенный файл оригиналов
    final_output_without_outliers = f"{output_dir}/final_registered_scans_filtered_full_shadow_points_{radius_search}_{threshold}_without_outliers_nb_{nb_neighbors}_std_{std_ratio}.ply"  # С удаленными выбросами
    final_output_e57 = f"{output_dir}/final_registered_scans_filtered_full.e57"  # Итоговый e57 с позами
    
    filtered_pattern = "scan_*_filtered_full.ply"
    downsampled_filtered_pattern = "scan_*_ds*_filtered.ply"
    local_pattern = "scan_*_original.ply"

    final_voxel_size = 0.01  # Воксель для финального downsampling (можно None, если не нужен)
    


    # === ЭТАП 1: Подготовка сканов для PCL ===
    print("=== ЭТАП 1: Подготовка сканов для PCL ===")
    
    if multi_file_mode:
        pose_file = prepare_scans_for_pcl_multi_file(
            e57_files=e57_files,
            output_dir=output_dir,
            downsample_voxel_size=downsample_voxel_size,
        )
    else:
        pose_file = prepare_scans_for_pcl(
            e57_file=e57_file,
            output_dir=output_dir,
            downsample_voxel_size=downsample_voxel_size,
        )
    
    print(f"Pose данные сохранены в: {pose_file}")

    # === ЭТАП 2: Запуск внешнего ShadowPoints фильтра (PCL C++) ===
    print("\n=== ЭТАП 2: Фильтрация ShadowPoints (PCL) ===")
    run_pcl_shadow_filter_batch(
        output_dir=output_dir,
        pcl_binary=pcl_binary,
        radius_search=radius_search,
        threshold=threshold,
        save_mode=save_mode,
    )

    # === ЭТАП 2.5: Перенос удаления shadow points на оригинальные облака ===
    print("\n=== ЭТАП 2.5: Перенос удаления на оригинальные облака ===")
    removed_files = sorted(Path(output_dir).glob("scan_*_filtered_removed.ply"))
    if not removed_files:
        removed_files = sorted(Path(output_dir).glob("scan_*_removed.ply"))

    if not removed_files:
        print("  Файлы *_filtered_removed.ply не найдены, этап пропущен")
    else:
        search_radius = (downsample_voxel_size * 1.5) if downsample_voxel_size is not None else radius_search
        for removed_file in removed_files:
            stem_parts = removed_file.stem.split("_")
            if len(stem_parts) < 2:
                print(f"  Пропуск файла {removed_file.name}: не удалось извлечь индекс скана")
                continue
            try:
                scan_idx = int(stem_parts[1])
            except ValueError:
                print(f"  Пропуск файла {removed_file.name}: некорректный индекс скана")
                continue

            original_file = Path(output_dir) / f"scan_{scan_idx:03d}_original.ply"
            output_file = Path(output_dir) / f"scan_{scan_idx:03d}_filtered_full.ply"

            if not original_file.is_file():
                print(f"  Оригинал не найден для scan {scan_idx:03d}: {original_file.name}, пропуск")
                continue

            propagate_shadow_removal_to_original(
                original_ply=str(original_file),
                removed_points_ply=str(removed_file),
                output_ply=str(output_file),
                search_radius=search_radius,
            )

    print("\n=== ЭТАП 3: Регистрация отфильтрованных сканов ===")
    register_processed_scans(
        processed_dir=output_dir,
        pose_file=pose_file,
        output_file=final_output,
        # voxel_size=final_voxel_size,
        pattern=filtered_pattern,
    )
    register_processed_scans(
        processed_dir=output_dir,
        pose_file=pose_file,
        output_file=final_output_downsampled,
        # voxel_size=final_voxel_size,
        pattern=downsampled_filtered_pattern,
    )
    # Регистрация оригинальных сканов
    register_processed_scans(
        processed_dir=output_dir,
        pose_file=pose_file,
        output_file=final_output_local,
        voxel_size=final_voxel_size,
        pattern=local_pattern,
    )

    # Экспорт отфильтрованных (перенесенных) локальных сканов в .e57 с позами
    export_scans_to_e57(
        processed_dir=output_dir,
        pose_file=pose_file,
        pattern=filtered_pattern,
        output_file=final_output_e57,
    )

    print(f"Финальный файл отфильтрованных сканов: {final_output}")

    # # === ЭТАП 4: Удаление статистических выбросов ===
    # if remove_outliers:
    #     print("\n=== ЭТАП 4: Удаление статистических выбросов ===")
        
    #     # Удаление выбросов из отфильтрованного облака
    #     remove_statistical_outliers(
    #         input_file=final_output,
    #         output_file=final_output_without_outliers,
    #         nb_neighbors=nb_neighbors,
    #         std_ratio=std_ratio,
    #     )
        
    #     print(f"\nФинальные файлы после удаления выбросов:")
    #     print(f"  Отфильтрованные: {final_output_without_outliers}")


if __name__ == "__main__":
    main()
