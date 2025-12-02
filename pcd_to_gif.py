# /// script
# dependencies = [
#   "imageio>=2.37.2",
#   "numpy>=2.3.5",
#   "open3d>=0.19.0",
# ]
# requires-python = "==3.12"
# ///


import open3d as o3d
import numpy as np
import imageio


def create_rotation_gif(ply_path, output_gif="rotation_o3d.gif", x_rotation_deg=0.0, center_shift=(0.0, 0.0, 0.0), zoom=0.8):
    # 1. Чтение облака
    pcd = o3d.io.read_point_cloud(ply_path)

    # # Поворот облака вокруг оси X (в градусах)
    # if x_rotation_deg != 0.0:
    #     R = pcd.get_rotation_matrix_from_xyz((np.deg2rad(x_rotation_deg), 0.0, 0.0)) # (x, y, z)
    #     pcd.rotate(R, center=pcd.get_center())

    # 2. Настройка визуализатора
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1920, height=1080, visible=True) # visible=False для headless (может требовать настройки)
    vis.add_geometry(pcd)
    
    # Настройка рендера (белый фон, точки побольше)
    opt = vis.get_render_option()
    # opt.background_color = np.asarray([1, 1, 1])
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2.0

    # Получаем контроллер вида для вращения
    ctr = vis.get_view_control()

    # Первичная настройка ракурса: смотрим чуть сверху вниз на модель
    bbox = pcd.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    # Применяем сдвиг центра (dx, dy, dz)
    center += np.array(center_shift)
    ctr.set_lookat(center)
    # front — вектор ОТ камеры К объекту; берём камеру «спереди и чуть сверху»
    ctr.set_front([1.0, 0.0, 1.0])
    ctr.set_up([0.0, 0.0, 1.0])       # ось Z направлена вверх

    # Zoom
    ctr.set_zoom(zoom)

    images = []
    
    # 3. Цикл вращения
    total_frames = 120
    angle_step = 2 * np.pi / total_frames  # шаг вращения вокруг оси Z
    
    for i in range(total_frames):
        # Вращаем МОДЕЛЬ вокруг оси Z относительно центра
        Rz = pcd.get_rotation_matrix_from_xyz((0.0, 0.0, angle_step))
        pcd.rotate(Rz, center=center)
        vis.update_geometry(pcd)
        
        # Обновляем геометрию и рендер
        vis.poll_events()
        vis.update_renderer()
        
        # Захват кадра
        image = vis.capture_screen_float_buffer(do_render=True)
        # Конвертация в uint8 для imageio
        image = (255 * np.asarray(image)).astype(np.uint8)
        images.append(image)

    vis.destroy_window()

    # 4. Сохранение GIF
    imageio.mimsave(output_gif, images, fps=15, loop=0)
    print(f"GIF сохранен: {output_gif}")

# Запуск
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_1table_blk360_aud_3005_segment_segmented.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_1table_blk360_aud_3005_segment_segmented.gif",
#     center_shift=(0.0, 0.0, -0.8),
#     zoom=0.5,
# )
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/1table_blk360_aud_3005.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/1table_blk360_aud_3005.gif",
#     center_shift=(0.0, 0.0, -0.8),
#     zoom=0.5,
# )
# # ----------------------
create_rotation_gif(
    ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_E_3023_A5_del_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.ply",
    output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_E_3023_A5_del_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.gif",
    center_shift=(0.0, 0.0, -1.5),
    zoom=0.4,
)
create_rotation_gif(
    ply_path="/Users/nikolayborovets/Desktop/for_review/E_3023_A5_del_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.ply",
    output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/E_3023_A5_del_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.gif",
    center_shift=(0.0, 0.0, -1.5),
    zoom=0.4,
)
# # -----------------------------
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_E_3023_A5_init_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_E_3023_A5_init_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/E_3023_A5_init_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/E_3023_A5_init_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# # -----------------------------
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_E_3023_A5_moved_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_E_3023_A5_moved_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0_segment_segmented.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/E_3023_A5_moved_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/E_3023_A5_moved_filtered_shadow_points_0.025_0.1_without_outliers_nb_20_std_2.0.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# -----------------------------
create_rotation_gif(
    ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_music_room_final_registered_scans_filtered_stat_outliers_nb30_std2.0_segment_segmented.ply",
    output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_music_room_final_registered_scans_filtered_stat_outliers_nb30_std2.0_segment_segmented.gif",
    center_shift=(0.0, -0.8, -2.5),
    zoom=0.3,
)
create_rotation_gif(
    ply_path="/Users/nikolayborovets/Desktop/for_review/music_room_final_registered_scans_filtered_stat_outliers_nb30_std2.0.ply",
    output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/music_room_final_registered_scans_filtered_stat_outliers_nb30_std2.0.gif",
    center_shift=(0.0, -0.8, -2.5),
    zoom=0.3,
)
# -----------------------------
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_obb_aud_3005_segment_segmented.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_obb_aud_3005_segment_segmented.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/obb_aud_3005_down_0.02.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/obb_aud_3005_down_0.02.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# -----------------------------
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/semseg/b6d24411_obb_aud_3005_segment_segmented.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/semseg/gifs/b6d24411_obb_aud_3005_segment_segmented.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )
# create_rotation_gif(
#     ply_path="/Users/nikolayborovets/Desktop/for_review/obb_aud_3005_down_0.02.ply",
#     output_gif="/Users/nikolayborovets/Desktop/for_review/gifs/obb_aud_3005_down_0.02.gif",
#     center_shift=(0.0, 0.0, -1.5),
#     zoom=0.4,
# )