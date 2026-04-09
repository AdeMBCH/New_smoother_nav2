# New_smoother_nav2

Prototype complet ROS 2 Kilted + Nav2 avec :

- plugin `Smoother Server` : `nav2_se2_hybrid_smoother::SE2HybridSmoother`
- baselines `no_smoother` / `savitzky_golay`
- bringup TurtleBot3 simulation
- benchmark automatique multi-config avec CSV + plots

## Arborescence

```text
New_smoother_nav2/
├── README.md
├── nav2_se2_hybrid_smoother/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── plugin.xml
│   ├── include/nav2_se2_hybrid_smoother/se2_hybrid_smoother.hpp
│   ├── src/se2_hybrid_smoother.cpp
│   ├── config/
│   │   ├── nav2_params_baseline_savgol.yaml
│   │   ├── nav2_params_no_smoother.yaml
│   │   └── nav2_params_se2_hybrid.yaml
│   ├── bt/
│   │   ├── nav_to_pose_no_smoothing.xml
│   │   ├── nav_to_pose_with_savgol_smoothing.xml
│   │   └── nav_to_pose_with_se2_smoothing.xml
│   └── launch/
│       ├── bringup_no_smoother_tb3.launch.py
│       ├── bringup_savgol_tb3.launch.py
│       └── bringup_se2_hybrid_tb3.launch.py
└── nav2_se2_hybrid_benchmark/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/nav2_se2_hybrid_benchmark
    ├── nav2_se2_hybrid_benchmark/run_benchmark.py
    ├── config/goals_tb3_world.yaml
    └── launch/benchmark_tb3.launch.py
```

## Dépendances système (Ubuntu 24.04 / ROS 2 Kilted)

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-kilted-navigation2 \
  ros-kilted-nav2-bringup \
  ros-kilted-nav2-simple-commander \
  ros-kilted-nav2-minimal-tb4-sim \
  ros-kilted-ros-gz \
  python3-numpy python3-pandas python3-matplotlib python3-yaml
```

Si ton environnement n’a pas déjà la stack TurtleBot3, ajoute:

```bash
sudo apt install -y \
  ros-kilted-turtlebot3 \
  ros-kilted-turtlebot3-msgs \
  ros-kilted-turtlebot3-navigation2 \
  ros-kilted-turtlebot3-simulations
```

`nav2_bringup` + `nav2_minimal_tb4_sim` suffit souvent pour le demo launch inclus ici; les paquets TurtleBot3 ci-dessus sont recommandés pour éviter les manques de modèles/assets selon machines.

## Build

```bash
cd ~/ws_nav2/src
git clone <TON_REPO> New_smoother_nav2
cd ~/ws_nav2
source /opt/ros/kilted/setup.bash
colcon build --symlink-install
```

## Source environnement

```bash
source /opt/ros/kilted/setup.bash
source ~/ws_nav2/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Lancer chaque configuration

### 1) SE2 Hybrid

```bash
ros2 launch nav2_se2_hybrid_smoother bringup_se2_hybrid_tb3.launch.py
```

### 2) Savitzky-Golay baseline

```bash
ros2 launch nav2_se2_hybrid_smoother bringup_savgol_tb3.launch.py
```

### 3) No smoother baseline

```bash
ros2 launch nav2_se2_hybrid_smoother bringup_no_smoother_tb3.launch.py
```

## Benchmark automatique (recommandé)

Le script benchmark :

- lance successivement chaque bringup,
- exécute la même liste de goals,
- récupère path brut + smooth + odom,
- calcule les métriques,
- exporte CSV + PNG.

```bash
source /opt/ros/kilted/setup.bash
source ~/ws_nav2/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 run nav2_se2_hybrid_benchmark run_benchmark \
  --methods no_smoother savgol se2_hybrid \
  --output-dir benchmark_results \
  --startup-wait 25 \
  --nav-timeout 180
```

Avec goals custom :

```bash
ros2 run nav2_se2_hybrid_benchmark run_benchmark \
  --methods no_smoother savgol se2_hybrid \
  --goals-file /chemin/vers/goals.yaml \
  --output-dir benchmark_results_custom
```

## Fichiers de sortie benchmark

Dans `benchmark_results/` :

- `benchmark_raw.csv` (par run/goal)
- `benchmark_summary.csv` (agrégé par méthode)
- `metrics_barplots.png`
- `xy_trajectories.png`
- `paths/<method>/goal_*_{raw,smoothed,trajectory}.csv`

## Paramètres plugin `se2_hybrid`

Dans `nav2_params_se2_hybrid.yaml` :

- `resample_distance`
- `max_iterations`
- `w_data_pos`
- `w_xy_smooth`
- `w_heading_tangent`
- `w_heading_smooth`
- `w_increment_smooth`
- `w_curvature_var`
- `preserve_start_orientation`
- `preserve_goal_orientation`
- `publish_debug_paths`
- `convergence_tol`

Debug topics publiés par le plugin :

- `/debug/se2_hybrid/input_path`
- `/debug/se2_hybrid/output_path`

## Métriques calculées

### Path-level

- `raw_path_length`
- `smoothed_path_length`
- `raw_mean_abs_curvature`
- `smoothed_mean_abs_curvature`
- `raw_mean_abs_curvature_variation`
- `smoothed_mean_abs_curvature_variation`
- `heading_error_raw_vs_smoothed`
- `smoothing_time_ms`

### Nav-level

- `success`
- `navigation_time_s`
- `robot_distance_m`
- `final_goal_error_m`

## Notes API Kilted à vérifier (courtes)

1. Signature exacte de `BasicNavigator.smoothPath(...)` selon patch Nav2 Kilted.
2. Noms d’arguments de `tb3_simulation_launch.py` (`bt_xml_file` / `headless`) selon révision.
3. Plugin planner `nav2_smac_planner/SmacPlannerHybrid` dispo selon installation minimale.
4. Si ton monde TB3 diffère, adapter `config/goals_tb3_world.yaml`.
