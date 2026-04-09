# nav2_se2_hybrid_smoother (ROS 2 Kilted prototype)

Prototype minimal et propre d'un plugin **Nav2 Smoother Server** hybride:

- base de lissage **fortement ancrée en (x, y)**,
- correction locale **SE(2)-aware** sur heading / incréments,
- sans solveur lourd,
- intégration TurtleBot3 + Nav2 pour test rapide.

## Arborescence

```text
New_smoother_nav2/
├── README.md
└── nav2_se2_hybrid_smoother/
    ├── CMakeLists.txt
    ├── package.xml
    ├── plugin.xml
    ├── include/nav2_se2_hybrid_smoother/se2_hybrid_smoother.hpp
    ├── src/se2_hybrid_smoother.cpp
    ├── config/nav2_params_se2_hybrid.yaml
    ├── bt/nav_to_pose_with_se2_smoothing.xml
    ├── launch/se2_hybrid_tb3_demo.launch.py
    └── scripts/benchmark_paths.py
```

## Build

```bash
cd ~/your_ws/src
# copier ce repo dans src, ou symlink
cd ~/your_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select nav2_se2_hybrid_smoother
```

## Source

```bash
source /opt/ros/kilted/setup.bash
source ~/your_ws/install/setup.bash
```

## Lancer la simulation TurtleBot3 + Nav2 avec ce smoother

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_se2_hybrid_smoother se2_hybrid_tb3_demo.launch.py
```

Le launch inclut `nav2_bringup/launch/tb3_simulation_launch.py` et passe:
- le `params_file`: `config/nav2_params_se2_hybrid.yaml`
- le BT XML: `bt/nav_to_pose_with_se2_smoothing.xml`

## Baseline / comparaison

### 1) Sans smoother
Dans le BT, supprimer le noeud `SmoothPath` et passer `ComputePathToPose -> FollowPath` directement.

### 2) Savitzky-Golay
Dans le BT, garder `SmoothPath` mais utiliser `smoother_id="savitzky_golay"`.

### 3) SE2HybridSmoother
Utiliser le BT fourni (id `se2_hybrid`).

## Paramètres exposés (YAML)

Sous `smoother_server.ros__parameters.se2_hybrid`:

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
- `convergence_tol`

Valeurs par défaut incluses:

```yaml
resample_distance: 0.05
max_iterations: 30
w_data_pos: 1.0
w_xy_smooth: 0.25
w_heading_tangent: 0.4
w_heading_smooth: 0.2
w_increment_smooth: 0.15
w_curvature_var: 0.05
preserve_start_orientation: true
preserve_goal_orientation: true
convergence_tol: 1e-4
```

## Benchmark offline minimal

Script: `scripts/benchmark_paths.py`

Entrées CSV attendues:
- colonnes minimales: `x,y`
- optionnelles: `theta`, `smoothing_ms`

Exemple:

```bash
python3 nav2_se2_hybrid_smoother/scripts/benchmark_paths.py \
  --raw raw.csv \
  --variant none:none.csv \
  --variant savgol:savgol.csv \
  --variant se2_hybrid:se2_hybrid.csv \
  --outdir benchmark_out
```

Sorties:
- `benchmark_out/metrics.csv`
- `benchmark_out/paths.png`

Métriques:
- longueur de chemin
- variation moyenne de courbure
- erreur moyenne de heading vs brut
- temps de smoothing moyen (si `smoothing_ms` présent)

## Compatibilité Kilted: points à vérifier

1. **Signature exacte des paramètres de launch** dans `tb3_simulation_launch.py` (`bt_xml_file` peut varier selon révision Nav2).
2. **Disponibilité plugin planner**:
   - config par défaut: `SmacPlannerHybrid`
   - fallback rapide: `SmacPlanner2D` si besoin.
3. **Nom du paramètre BT Navigator**:
   - `default_nav_to_pose_bt_xml` peut dépendre de version/révision.
4. **Noeud BT `SmoothPath`**:
   - ports XML peuvent légèrement varier selon Nav2 Kilted patch-level.

## Limitations

- Prototype orienté faisabilité, pas optimisation globale.
- Correction SE(2) locale volontairement faible (pas reconstruction globale Lie complète).
- Pas de collision checking interne au smoother.
- Le benchmark est offline (CSV), pas encore un pipeline live automatique dans Nav2.
