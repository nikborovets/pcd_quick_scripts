# pcd_quick_scripts

Набор утилит для обработки облаков точек (PLY, E57, LAS).

## Требования

- Python 3.12
- [uv](https://github.com/astral-sh/uv) — для запуска скриптов

---

## Скрипты

### `viz_ply.py`
Визуализация PLY файла в Open3D.

```bash
uv run --script viz_ply.py <path_to_file.ply>
```

---

### `downsample_ply.py`
Даунсемплинг облака точек (voxel grid).

```bash
uv run --script downsample_ply.py <path_to_file.ply>
```
> Сейчас voxel_size=0.02 и output path захардкожены внутри скрипта — отредактируй под себя.

---

### `las_to_ply.py`
Конвертация LAS → PLY с сохранением цвета.

```bash
uv run --script las_to_ply.py <input.las> <output.ply>
```

---

### `statistic_outliers.py`
Удаление статистических выбросов из PLY.

```bash
uv run --script statistic_outliers.py <input.ply> <output.ply> [nb_neighbors] [std_ratio]
```

| Параметр | По умолчанию | Описание |
|----------|--------------|----------|
| `nb_neighbors` | 20 | Кол-во соседей для анализа |
| `std_ratio` | 2.0 | Порог стандартного отклонения |

---

### `pcd_to_gif.py`
Создание вращающегося GIF из PLY.

```bash
uv run --script pcd_to_gif.py
```
> Параметры (путь к файлу, zoom, center_shift) задаются внутри скрипта — отредактируй вызовы `create_rotation_gif()`.

---

### `shadow_points_pipeline.py`

Полный пайплайн обработки E57 сканов с фильтрацией shadow points (артефакты на границах объектов при лазерном сканировании).

```bash
uv run --script shadow_points_pipeline.py
```

#### Как работает

1. **Этап 1** — Чтение E57: извлекает сканы из E57, сохраняет их в локальных координатах сканера как `scan_XXX_local.ply` + сохраняет pose данные (rotation, translation) в `pose_data.json`
2. **Этап 2** — Фильтрация: для каждого `scan_XXX_local.ply` запускает C++ PCL фильтр ShadowPoints → создаёт `scan_XXX_filtered.ply`
3. **Этап 3** — Регистрация: применяет pose трансформации к отфильтрованным сканам и объединяет их в один PLY в глобальных координатах
4. **Этап 4** — Очистка: удаляет статистические выбросы из итогового облака

#### Параметры (редактируются в `main()`)

| Параметр | Описание |
|----------|----------|
| `multi_file_mode` | `True` = несколько E57 файлов (список), `False` = один E57 файл. **Важно иметь информацию о трансформации между сканами** |
| `e57_files` | Список путей к E57 файлам (для `multi_file_mode=True`) |
| `e57_file` | Путь к одному E57 файлу (для `multi_file_mode=False`) |
| `file_name` | Базовое имя для выходной директории |
| `pcl_binary` | **Абсолютный путь** к скомпилированному `remove_shadowpoints` |
| `radius_search` | Радиус поиска соседей для вычисления нормалей (м). Меньше = точнее, но медленнее. Типично: `0.02–0.05` |
| `threshold` | Порог ShadowPoints. Меньше = агрессивнее фильтрация. Типично: `0.1–0.3` |
| `downsample_voxel_size` | Размер вокселя для даунсемплинга перед фильтрацией (м). `None` = без даунсемплинга |
| `save_mode` | Режим сохранения: `filtered` / `highlighted` / `sphere` |
| `remove_outliers` | `True` = удалять статистические выбросы после регистрации |
| `nb_neighbors` | Кол-во соседей для статистического анализа выбросов |
| `std_ratio` | Порог стандартного отклонения для выбросов |
| `final_voxel_size` | Воксель для финального даунсемплинга объединённого облака |

#### Структура выходной директории

```
{file_name}_scans_for_pcl_{radius}_{threshold}_voxel_{voxel}_{mode}/
├── pose_data.json                     # Позиции сканера для каждого скана
├── scan_000_ds0.01_local.ply          # Локальные координаты (до фильтрации)
├── scan_000_ds0.01_filtered.ply       # После фильтрации ShadowPoints
├── scan_001_ds0.01_local.ply
├── scan_001_ds0.01_filtered.ply
├── ...
├── final_registered_scans_original.ply           # Объединённый без фильтрации
├── final_registered_scans_filtered.ply           # Объединённый с фильтрацией
└── final_registered_scans_filtered_shadow_points_*_without_outliers_*.ply  # Финальный
```

#### Формат pose_data.json

```json
[
  {
    "scan_index": 0,
    "rotation": [w, x, y, z],      // кватернион
    "translation": [x, y, z]       // метры
  },
  ...
]
```

#### Входные E57 файлы

- Каждый E57 может содержать 1+ сканов (станция + iPad tags и т.д.)
- Скрипт автоматически извлекает все сканы из каждого файла
- Pose данные берутся из заголовков E57

---

## C++ ShadowPoints фильтр отдельно от питоновского скрипта

### Установка зависимостей (macOS)

```bash
brew install cmake
brew install pcl
```

### Сборка

```bash
cd cpp_pcl_shadowpoints
mkdir -p build && cd build
cmake ..
make
```

### Запуск

```bash
./remove_shadowpoints <radius_search> <threshold> <input.ply> <output.ply> <save_mode>
```

| Параметр | Описание |
|----------|----------|
| `radius_search` | Радиус поиска соседей для нормалей (напр. `0.025`) |
| `threshold` | Порог ShadowPoints (напр. `0.1`) |
| `input.ply` | Входной PLY файл |
| `output.ply` | Выходной PLY файл |
| `save_mode` | `filtered` / `highlighted` / `sphere` |

**Режимы:**
- `filtered` — только отфильтрованные точки (без shadow points)
- `highlighted` — все точки, shadow points подсвечены красным
- `sphere` — как `highlighted` + жёлтая сфера в начале координат (место сканирования)

**Пример:**
```bash
./remove_shadowpoints 0.025 0.1 scan_000_local.ply scan_000_filtered.ply filtered
```

