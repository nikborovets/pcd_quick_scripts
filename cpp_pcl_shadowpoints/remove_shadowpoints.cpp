#include <iostream>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <chrono>

int main(int argc, char** argv) {
    // Параметры фильтрации по умолчанию
    float radius_search = 0.025f;  // Радиус поиска соседей для вычисления нормалей
    float threshold = 0.25f;       // Порог для фильтра ShadowPoints
    std::string input_path = "/Users/nikolayborovets/Desktop/skoltech/libraries/pcl_using/Setup35_downsampled.ply";
    std::string output_path = ""; // Пусто = сгенерируем имя по параметрам

    // Ожидаемый формат:
    // argv[1] = radius_search
    // argv[2] = threshold
    // argv[3] = input_path
    // argv[4] = output_path
    // argv[5] = save_mode (sphere | highlighted | filtered)

    std::string save_mode = "filtered"; // режим сохранения по умолчанию

    if (argc >= 2)
        radius_search = std::atof(argv[1]);
    if (argc >= 3)
        threshold = std::atof(argv[2]);
    if (argc >= 4)
        input_path = argv[3];
    if (argc >= 5)
        output_path = argv[4];
    if (argc >= 6)
        save_mode = argv[5];

    if (argc > 6) {
        std::cerr << "Usage: " << argv[0]
                  << " [radius_search] [threshold] [input_path] [output_path] [save_mode]\n";
        return -1;
    }

    std::cout << "Using parameters: radius_search=" << radius_search
              << ", threshold=" << threshold << std::endl;
    std::cout << "Input:  " << input_path << std::endl;
    if (!output_path.empty())
        std::cout << "Output: " << output_path << std::endl;
    std::cout << "Save mode: " << save_mode << std::endl;

    // 1. Создаем умные указатели для облаков точек
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_omp(new pcl::PointCloud<pcl::Normal>);

    // 2. Загружаем .ply файл
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read input file.\n");
        return (-1);
    }
    // if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("/Users/nikolayborovets/Desktop/skoltech/libraries/pcl_using/Setup35_downsampled.ply", *cloud) == -1) {
    //     PCL_ERROR("Couldn't read file Setup35_downsampled.ply \n");
    //     return (-1);
    // }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points." << std::endl;

    // 3. Вычисляем нормали с помощью NormalEstimationOMP
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_omp;
    ne_omp.setInputCloud(cloud);
    ne_omp.setSearchMethod(tree);
    ne_omp.setRadiusSearch(radius_search);

    auto start_omp = std::chrono::high_resolution_clock::now();
    ne_omp.compute(*normals_omp);
    auto end_omp = std::chrono::high_resolution_clock::now();
    auto duration_omp = std::chrono::duration_cast<std::chrono::milliseconds>(end_omp - start_omp).count();
    std::cout << "NormalEstimationOMP time: " << duration_omp << " ms" << std::endl;

    // 4. Настраиваем и применяем фильтр ShadowPoints, используя OMP-нормали
    pcl::ShadowPoints<pcl::PointXYZRGB, pcl::Normal> sp_filter(true); // true = extract removed indices
    sp_filter.setInputCloud(cloud);
    sp_filter.setNormals(normals_omp);
    sp_filter.setThreshold(threshold);

    // Применяем фильтр
    sp_filter.filter(*cloud_filtered);
    std::cout << "ShadowPoints filter applied." << std::endl;

    // Сохраняем удаленные точки в отдельный файл
    pcl::IndicesConstPtr removed_indices = sp_filter.getRemovedIndices();
    const bool has_removed = removed_indices && !removed_indices->empty();
    if (has_removed) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, *removed_indices, *removed_cloud);

        std::string removed_filename;
        if (!output_path.empty()) {
            size_t dot_pos = output_path.rfind('.');
            if (dot_pos != std::string::npos) {
                removed_filename = output_path.substr(0, dot_pos) + "_removed" + output_path.substr(dot_pos);
            } else {
                removed_filename = output_path + "_removed.ply";
            }
        } else {
            removed_filename = "removed_shadowpoints.ply";
        }

        pcl::io::savePLYFileASCII(removed_filename, *removed_cloud);
        std::cout << "Saved " << removed_cloud->size() << " removed shadow points to "
                  << removed_filename << std::endl;
    }

    if (has_removed && (save_mode == "highlighted" || save_mode == "sphere")) {
        // 4.5. Подсвечиваем удаленные точки красным цветом в оригинальном облаке
        for (const auto& idx : *removed_indices) {
            (*cloud)[idx].r = 255; // Красный
            (*cloud)[idx].g = 0;
            (*cloud)[idx].b = 0;
        }
        std::cout << "Highlighted " << removed_indices->size() << " removed points in red." << std::endl;
    }

    // 5. Добавляем большой красный шар (радиус 5 см = 0.05 м) в начало координат (0,0,0)
    // Создаем сферу только если выбран режим "sphere"
    if (save_mode == "sphere") {
        const float sphere_radius = 0.05f; // 5 см
        const int num_points_phi = 50; // количество точек по широте
        const int num_points_theta = 100; // количество точек по долготе

        for (int i = 0; i < num_points_phi; ++i) {
            float phi = M_PI * i / (num_points_phi - 1); // от 0 до π
            for (int j = 0; j < num_points_theta; ++j) {
                float theta = 2 * M_PI * j / num_points_theta; // от 0 до 2π

                pcl::PointXYZRGB sphere_point;
                sphere_point.x = sphere_radius * sin(phi) * cos(theta);
                sphere_point.y = sphere_radius * sin(phi) * sin(theta);
                sphere_point.z = sphere_radius * cos(phi);

                sphere_point.r = 255;
                sphere_point.g = 255;
                sphere_point.b = 0;

                // Добавляем точку сферы в облако с подсветкой
                cloud->push_back(sphere_point);
            }
        }

        std::cout << "Added red sphere at origin with "
                  << num_points_phi * num_points_theta << " points." << std::endl;
    }

    // 6. Сохраняем в зависимости от режима
    std::ostringstream filename;
    if (!output_path.empty()) {
        filename << output_path;
    } else {
        if (save_mode == "filtered") {
            filename << "FILTERED_OMP_" << radius_search
                     << "_threshold" << threshold << ".ply";
        } else if (save_mode == "highlighted") {
            filename << "HIGHLIGHTED_OMP_" << radius_search
                     << "_threshold" << threshold << ".ply";
        } else { // sphere или любое другое значение по умолчанию
            filename << "BIG_OMP_" << radius_search
                     << "_threshold" << threshold << "_sphere.ply";
        }
    }

    if (save_mode == "filtered") {
        pcl::io::savePLYFileASCII(filename.str(), *cloud_filtered);
    } else { // highlighted или sphere: сохраняем cloud с подсвеченными точками (и, возможно, сферой)
        pcl::io::savePLYFileASCII(filename.str(), *cloud);
    }

    std::cerr << "Saved "
              << (save_mode == "filtered" ? cloud_filtered->size() : cloud->size())
              << " data points to " << filename.str() << "." << std::endl;

    return (0);
}