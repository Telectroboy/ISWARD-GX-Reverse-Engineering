## ISWARD II – Résumé du projet

### Vue d’ensemble
ISWARD II est un workspace ROS (ROS1, Noetic) orienté robotique mobile. Il regroupe des packages pour la perception (capteurs dont caméra Astra), la localisation, la cartographie, la planification/navigation (costmaps, move_base, contrôleur Stanley, planners), le stationnement, la planification de tâches (scheduler) et des pipelines de vision (détection de station/chargeur).

Le lancement principal orchestre ces sous-systèmes via des fichiers `.launch` et propose un enregistrement optionnel de données avec `rosbag`.

### Composants majeurs
- **Capteurs (`sensor-pkg`)**: intégration caméra Astra (`astra_camera`), vérification caméra (`camera_checker`), et lancement capteurs globaux (`launch/sensor.launch`).
- **Châssis (`chassis-pkg`)**: interface et lancement du châssis/locomotion.
- **Localisation (`location-pkg`)**: nœuds et launch pour estimation de pose.
- **Cartographie (`mapping-pkg`)**: actions/messages de mapping, lancement de pipeline de carte.
- **Navigation (`navigation-pkg`)**:
  - `costmap_2d`, `map_server`, `move_base`, `coverage_planner`
  - contrôleur latéral `stanley_controller`
  - conversions capteurs: `pointcloud_to_laserscan`
- **Stationnement (`parking-pkg`)**: logique et lancement du parking autonome.
- **Ordonnancement (`scheduler-pkg`)**: planification/gestion de tâches.
- **Vision (`vision-pkg/charger_pose_pkg`)**: modèles `.onnx`/`.engine` pour détection de chargeur ou cibles de docking.
- **Expérimentations (`exps_pkg`)**: scripts/mode expérimental optionnel.

### Lancement et orchestration
- Lancement principal: `launch/isward_common.launch`
  - Arguments activables: `Dabai`(capteurs), `Chassis`, `Location`, `Scheduler`, `Parking`, `Mapping`, `Nav`, `Iswardvision`, `Exps`, `record_bag`.
  - Inclus: les `*.launch` de chaque package cité ci-dessus.
- Enregistrement rosbag (optionnel): `launch/include/rosbag.launch`
  - Paramètres: `BAG_DIR`, `BAG_NAME`, `BAG_MAX_SIZE` (split avec `--lz4`).
  - Thèmes clés enregistrés: `/Mapping`, `/map`, `/chassis/*`, `/tf`, `/fused_pose/path`, `/gnss_pose/path`.

### Scripts d’exécution
- `scripts/startup_localhost.sh`: configuration locale (`ROS_MASTER_URI=http://localhost:11311`), détection UVC de la caméra et patch dynamique de `uvc_product_id`, sourcing de `/opt/ros/noetic` et `devel/setup.bash`, lancement de `isward_common.launch`.
- `scripts/startup_outdoor.sh`: similaire, pour un hôte réseau `outdoor`.
- `scripts/log_clear.py`: nettoyage/gestion des logs, lancé en arrière-plan par les scripts startup.

### Dépendances et artefacts de build
- Workspace Catkin compilé: répertoires `devel/` et `devel/lib`, includes générés (`devel/include`), messages/actions générés (`devel/share/*`), bindings `gennodejs`, `roseus`, etc.
- Binaires/so partagés pour navigation, capteurs et utilitaires (ex: `libmove_base.so`, `libcostmap_2d.so`, `libstanley_controller.so`, `libmap_server_image_loader.so`, etc.).

### Version et modèle matériel
- `version.yaml`
  - `version`: v1.2.1.250814-0 (release)
  - `model`: G30

### Démarrage rapide
1) Pré-requis système
   - Ubuntu + ROS Noetic (scripts référencent `/opt/ros/noetic`).
   - Drivers caméra Astra et accès `/dev/video0`.

2) Build (si nécessaire)
   - `source /opt/ros/noetic/setup.bash`
   - `catkin_make` (dans la racine du workspace) puis `source devel/setup.bash`

3) Lancement recommandé
   - Local: `bash scripts/startup_localhost.sh`
   - Réseau outdoor: `bash scripts/startup_outdoor.sh`
   - Sans scripts: `roslaunch launch/isward_common.launch`

4) Options utiles
   - Enregistrer un rosbag: `roslaunch launch/isward_common.launch record_bag:=true BAG_DIR:=/chemin BAG_MAX_SIZE:=200`
   - Activer vision (charger_pose): `roslaunch launch/isward_common.launch Iswardvision:=true`

### Arborescence (aperçu)
- `launch/`: `isward_common.launch`, `include/rosbag.launch`
- `scripts/`: `startup_localhost.sh`, `startup_outdoor.sh`, `log_clear.py`
- `src/`:
  - `sensor-pkg/`, `chassis-pkg/`, `location-pkg/`, `mapping-pkg/`, `navigation-pkg/`, `parking-pkg/`, `scheduler-pkg/`, `vision-pkg/`, `exps_pkg/`
- `devel/`: includes, libs, cmake, messages générés, bindings (`gennodejs`, `roseus`)
- `version.yaml`: version logicielle et modèle

### Notes d’intégration
- Les chemins de certains `include` dans `isward_common.launch` sont absolus et pointent vers `/home/isward/isward_ii_ws/...`. Adapter au chemin local si nécessaire.
- Les scripts de startup modifient dynamiquement `astra_camera/launch/dabai_u3.launch` pour le `uvc_product_id` détecté (IDs connus: `0557`, `050e`).

### Support et diagnostics
- Topics à vérifier rapidement: `/tf`, `/map`, `/move_base/*`, `/chassis/*`, `/fused_pose/path`, `/gnss_pose/path`.
- Logs: voir `scripts/log_clear.py` et `rosout` (paramètre `/rosout/omit_topics` activé dans le lancement commun).



### Navigation
- Orchestration: `src/navigation-pkg/launch/navigation.launch` inclut le profil `mower` qui lance:
  - `tf2_ros/static_transform_publisher` (`base_link -> isward_link`)
  - `move_base` (voir `mower/launch/movebase_normal.launch`)
  - `pointcloud_to_laserscan` et `map_server`.
- `move_base`:
  - Global planner: `coverage_planner/CoveragePlanner` (params: `mower/params/coverage_planner_params.yaml`).
  - Local planner: `stanley_controller/StanleyController` (params: `mower/params/stanley_control_params.yaml`).
  - Fréquences et récupérations: `mower/params/move_base_params.yaml` (rotate, move_back).
- Costmaps:
  - Global: `mower/params/global_costmap_params.yaml` (StaticLayer, ObstacleLayer, ContractSensorLayer, InflationLayer).
  - Local: `mower/params/local_costmap_params.yaml` (ObstacleLayer, InflationLayer, fenêtre 6×6m).
  - Communs/sources capteurs: `mower/params/costmap_common_params_burger.yaml` (`/scan`, `/depth/scan`, `PointCloud2`).

### Navigation – détails avancés
- Frames et TF:
  - TF statique `base_link -> isward_link` (offsets 0.35, 0, 0.093) pour aligner les capteurs.
  - Frames principales: `map` (global), `base_link` (robot), `isward_link` (capteur).
- Global planner (`coverage_planner`):
  - Plugin `coverage_planner/CoveragePlanner` ([`coverage_planner.xml`]) et paramètres `coverage_planner_params.yaml` (largeurs de bandes de coûts, chemin de travail).
  - Fonction: génère des chemins de couverture contraints par la `global_costmap`.
- Local planner (Stanley):
  - Plugin `stanley_controller/StanleyController` ([`stanley_control_plugin.xml`]), paramètres `stanley_control_params.yaml`.
  - Fonction: suivi de trajectoire avec contrôle d’erreur latérale, production de `cmd_vel`.
- Costmaps et capteurs:
  - `global_costmap`: `StaticLayer` (depuis `/map`), `ObstacleLayer` (capteurs), `ContractSensorLayer` (collision), `InflationLayer`.
  - `local_costmap`: fenêtre roulante 6×6 m, `ObstacleLayer` + `InflationLayer`.
  - Sources: `/scan`, `/depth/scan`, `PointCloud2` (configurables dans `costmap_common_params_burger.yaml`).
- Recovery et robustesse:
  - Oscillation: distance 0.2 m, timeout 45 s; patiences planificateur/contrôleur configurées.
  - Behaviors: rotation sur place, marche arrière dégagée; relance de planification.

### Cartographie & enregistrement des cartes
- Chargement de carte statique: `navigation-pkg/map_server/launch/map_server.launch` lance `map_server` avec `args="/home/isward/.isward/map/map.yaml"` pour publier `/map`.
- Services Mapping (package `mapping`): services générés dans `devel` indiquent des opérations:
  - `mapping/Mapping` (req `mode`: 1=start mapping, 2=expand mapping)
  - `mapping/loadmap` (charger une carte existante)
  - `mapping/querymap` (vérifier l’existence d’une carte)
- Enregistrement (sauvegarde) d’une carte:
  - Méthode standard ROS: utiliser `map_server` et l’outil `map_saver` pour capturer le topic `/map` en fichiers `.pgm`/`.yaml`.
  - Exemple:
    - `rosrun map_server map_saver -f /home/isward/.isward/map/map`
    - Produit `/home/isward/.isward/map/map.pgm` et `/home/isward/.isward/map/map.yaml` (ce YAML est ensuite utilisé par `map_server.launch`).
  - Si le nœud `mapping` expose des services de sauvegarde personnalisés, le flux est généralement:
    1) Appeler `mapping/Mapping` avec `mode=1` pour démarrer la génération de carte.
    2) Lorsque la carte est stabilisée, appeler un service dédié (ou lancer `map_saver`) pour écrire les fichiers.
    3) Vérifier/charger via `mapping/querymap` et `mapping/loadmap`.
- Emplacement par défaut dans ce projet: `/home/isward/.isward/map/map.yaml` (référencé par `move_base` et `map_server`).

### Planificateur de trajet (global)
- Présent: oui, via le plugin `coverage_planner/CoveragePlanner` (implémente `nav_core::BaseGlobalPlanner`).
- Rôle: calcule un chemin global (souvent de couverture) sur la carte `/map` en tenant compte des coûts (obstacles/inflation).
- Paramètres: `src/navigation-pkg/mower/params/coverage_planner_params.yaml` (ex.: `base_map_cost_width`, `coverage_map_cost_width`, `border_map_cost_width`, chemin de travail `work_path`).
- Chargement: défini dans `mower/launch/movebase_normal.launch` via `base_global_planner`.

### Suivi de trajet (local) et déplacement
- Local planner: `stanley_controller/StanleyController` (plugin `nav_core::BaseLocalPlanner`).
- Rôle: suit le chemin global et publie `cmd_vel` (20 Hz) selon `controller_frequency`.
- Paramètres principaux:
  - `mower/params/stanley_control_params.yaml`: gains latéraux (`lateral_gain_p`), tolérances (`xy_goal_tolerance`), frames (`map_frame`, `cutter_frame`), `robot_radius`, etc.
  - `mower/params/move_base_params.yaml`: `controller_frequency`, tolérances/temporisations, comportements de récupération.
  - Fichier legacy (si bascule vers TrajectoryPlannerROS): `mower/params/base_local_planner_params.yaml` avec `max_vel_x`, `min_vel_x`, `acc_lim_x`, `max_vel_theta`, `acc_lim_theta`, tolérances d’objectif.
- Capteurs/costmaps: les costmaps globale/locale fusionnent couches statiques/obstacles/inflation (sources `/scan`, `/depth/scan`, `PointCloud2`) pour la planification et l’évitement.

### Vitesses et accélérations
- Fréquence de commande: `controller_frequency: 20.0` (dans `move_base_params.yaml`).
- Limites (si `TrajectoryPlannerROS` était utilisé):
  - `max_vel_x: 0.3`, `min_vel_x: 0.08`
  - `max_vel_theta: 0.3`, `min_vel_theta: -0.3`, `min_in_place_vel_theta: 1.0`
  - `acc_lim_x: 0.3`, `acc_lim_y: 0`, `acc_lim_theta: 0.3`
- Avec Stanley: la vitesse linéaire/angulaire est déterminée par le contrôleur selon les paramètres Stanley et les contraintes du robot; ajuster `StanleyController` (et le niveau châssis) pour plafonner/filtrer ces vitesses.

### Détection de blocage et récupérations
- Oscillation et délais:
  - `oscillation_distance: 0.2`, `oscillation_timeout: 45.0`
  - `controller_patience: 3.0`, `planner_patience: 30.0`
- Récupérations activées: `recovery_behavior_enabled: true`.
- Séquence de recovery (configurable):
  - `rotate_recovery/RotateRecovery` (rotation sur place; `max_vel_theta`, `min_in_place_vel_theta`).
  - `move_back_recovery/MoveBackClearRecovery` (recul contrôlé): `distance_backwards`, `predict_distance`, `time_for_backwards`, `frequency`, `lethal_cost`, `linear_vel_x`.
- Logique: si le contrôleur ne parvient pas à produire des commandes pendant `controller_patience` ou si oscillation détectée au-delà des seuils, `move_base` déclenche les comportements de récupération dans l’ordre défini, réessaie la planification selon `planner_frequency`/`max_planning_retries` (ici `1`).

### Auteurs & crédits
- `mapping` (mainteneur): tony-ws1 `<shantizhan@163.com>`.
- `coverage_planner` (mainteneur): Terry `<zhoutaoterry@gmail.com>`.
- `stanley_controller` (mainteneur/auteur): wuyx `<527160730@qq.com>` (licence BSD).
- `location-pkg` (mainteneur): terry `<zhoutaoterry@gmail.com>` (licence BSD).
- `astra_camera` (mainteneur): Mo Cun `<mocun@orbbec.com>` (licence Apache-2.0).
- `camera_checker`, `charger_pose_pkg`, `exps_pkg`: mainteneurs placeholders (emails TODO) – revoir/mettre à jour.
- Librairies tierces: `map_server` (ROS Wiki), `move_base`, `rotate_recovery`, `move_back_recovery`, etc., sous licences open-source indiquées dans leurs `package.xml`.

### Modules – Vue détaillée
- `sensor-pkg`
  - `astra_camera`: drivers/launch caméra Orbbec Astra (`dabai_u3.launch`), publication de nuage de points, depth, IR, color.
  - `camera_checker`: outil de vérification caméra.
  - `launch/sensor.launch`: compose les capteurs.
- `chassis-pkg`
  - Lancements HAL: communication BLE (`hal_ble`), IoT distant/MQTT (`hal_remote`), LTE/USIM (`hal_usim`), GNSS + RTCM radio (`hal_gnss*`).
  - Scripts: gestion batterie (`bat_ctl.py`), nettoyage fichiers (`file_cleaner_node.py`).
- `location-pkg`
  - `launch/location.launch`: pipeline de localisation/pose fusionnée.
- `mapping-pkg`
  - `mapping` (services `Mapping`, `loadmap`, `querymap`) et `launch/mapping.launch`.
- `navigation-pkg`
  - `coverage_planner` (global planner), `stanley_controller` (local planner), `map_server`, `move_base`, costmaps, recoveries, `pointcloud_to_laserscan`.
  - Profil `mower`: assemble TF statique, move_base, map_server, conversion capteur.
- `parking-pkg`
  - `launch/parking.launch`: logique de stationnement autonome.
- `scheduler-pkg`
  - `scheduler.launch` et variantes: orchestration de fonctions (charge `version.yaml`, `functions.yaml`, `velocity.csv`).
  - `functions.yaml`: drapeaux de fonctionnalités (BLE/IOT démarrage tonte, OTA, PIN, Wi‑Fi AP, mapping/charging station, vision locating, etc.). Exemple présent: `VISION_LOCATING:default=true`.
  - `velocity.csv`: profils de consignes (v_lin, v_ang, hauteur lame, intensité lame) pour tests/séquences: 
    - Accélération uniforme (50 pas), décélération (30 pas), marche constante, marche arrière brusque, virage, arrêt.
    - Plage: v_lin ∈ [−0.5, 0.5] m/s; v_ang ∈ [−1.5, 1.5] rad/s; hauteur lame ∈ [30, 60] mm; intensité lame ∈ [0, 4].
- `vision-pkg/charger_pose_pkg`
  - Détection de chargeur/station, modèles `.onnx`/`.engine`, `charger_pose_pkg.launch`.
- `exps_pkg`
  - Scripts et dépendances pour expérimentations (ONNX Runtime, NumPy…).

### Communication & connectivité (App ↔ Robot)
- Surcouche réseau ROS:
  - Variables: `ROS_MASTER_URI` pointent vers `localhost` ou hôte `outdoor` → communication ROS sur LAN/Wi‑Fi/Ethernet.
- IoT distant (application/cloud):
  - `chassis_pkg/hal_remote` (MQTT) se connecte à l’hôte `mqtt.xiaojia-tech.com.cn:8331` et charge des secrets (`secret.yaml`).
  - Usage typique: télémétrie/commandes à distance via MQTT (transporté par Wi‑Fi/Ethernet ou LTE).
- LTE/Cellulaire (GSM/4G):
  - `chassis_pkg/hal_usim`: paramètre un modem LTE via `/dev/tty_LTE` (115200 bauds) → connectivité cellulaire.
- Bluetooth (BLE):
  - `chassis_pkg/hal_ble`: passerelle BLE via `/dev/tty_BLE` (115200 bauds) pour liaison locale courte portée.
- GNSS + corrections (RTCM):
  - `chassis_pkg/hal_rtcm`: radio RTCM via `/dev/tty_RADIO` (IO config 13) pour corrections GNSS.
- Conclusion connectivité: le système peut communiquer via Wi‑Fi/LAN (ROS + MQTT), LTE/Cellulaire (USIM/MQTT), et Bluetooth (BLE). Aucune trace de WebSocket/HTTP directe dans ce workspace; MQTT est la voie primaire vers l’IoT.

### Topics & Services par module (aperçu)
- Navigation
  - `map_server`:
    - Publie: `/map` (nav_msgs/OccupancyGrid)
    - Argument: chemin YAML de carte (`/home/isward/.isward/map/map.yaml`)
  - `move_base`:
    - Paramètres clés: `base_global_planner`, `base_local_planner`, `controller_frequency`, recoveries
    - Interagit avec: `/tf`, costmaps global/local, `cmd_vel` (publication)
  - Costmaps:
    - Sources: `/scan`, `/depth/scan`, `PointCloud2` (cf. `costmap_common_params_burger.yaml`)
  - `pointcloud_to_laserscan`:
    - Convertit: PointCloud2 → LaserScan (topics dépendants du launch)
- Mapping
  - Services (générés): `mapping/Mapping` (mode 1 start, 2 extend), `mapping/loadmap`, `mapping/querymap`
  - Flux de sauvegarde standard: `rosrun map_server map_saver -f /home/isward/.isward/map/map`
- Capteurs
  - `astra_camera`:
    - Paramètres: activation color/depth/IR/point cloud
    - Publie: nuages de points, flux depth/color (noms selon launch `dabai_u3.launch`)
- Châssis
  - `hal_remote` (MQTT): se connecte au broker défini; charge `secret.yaml`
  - `hal_usim`: modem LTE sur `/dev/tty_LTE`
  - `hal_ble`: liaison BLE sur `/dev/tty_BLE`
  - `hal_rtcm`: corrections GNSS via `/dev/tty_RADIO`
- Scheduler
  - Charge `functions.yaml`, `velocity.csv` et `version.yaml` (coordination des comportements)

### Secrets & déploiement
- Fichier `chassis-pkg/launch/secret.yaml`:
  - Champs: `secret` (liste de clés), `model` (modèles pris en charge: `G10X`, `G6`, `G10`, `G10H`, `G20`, `G20H`, `G30`, `G30H`).
  - Utilisation: chargé par `hal_remote` (MQTT) pour authentification/identification.
- Bonnes pratiques:
  - Ne pas versionner des secrets en clair; préférer variables d’environnement ou coffre-fort.
  - Paramétrer `host`, `port` MQTT via variables/launch args lors du déploiement.
- Déploiement typique:
  - LAN/Wi‑Fi: définir `ROS_MASTER_URI` (ex. `http://outdoor:11311`) et lancer `startup_outdoor.sh`.
  - LTE: démarrer `hal_usim` puis `hal_remote` pour liaison MQTT cellulaire.
  - BLE: démarrer `hal_ble` pour commandes locales à courte portée.

### Identification & Numéro de série (SN)
- Éléments présents dans le repo:
  - Caméra Astra: argument/paramètre `serial_number` dans `sensor-pkg/astra_camera/launch/dabai_u3.launch` (peut être renseigné pour cibler un capteur précis).
  - Châssis: service `interface_pkg/ChassisGetInfo` (existe côté artefacts générés) — probable point d’accès aux infos hardware (cartes driver/sensor/power). Le format de réponse n’est pas documenté ici, mais c’est le meilleur candidat pour un SN exposé par firmware.
  - `version.yaml`: contient `version` et `model` seulement (pas de SN). `chassis-pkg/launch/secret.yaml`: clés + modèles supportés (pas de SN).

- Interroger le châssis (exemple):
```bash
rosservice call /ChassisGetInfo "index: 0"     # INDEX_BOARD_DRIVER
rosservice call /ChassisGetInfo "index: 1"     # INDEX_BOARD_SENSOR
rosservice call /ChassisGetInfo "index: 2"     # INDEX_BOARD_POWER
```
Si le firmware expose un numéro de série, il devrait apparaître dans la réponse d’un de ces index (à confirmer sur cible).

- Recommandation de stockage local du SN:
  - Fichier: `~/.isward/device/serial.yaml`
  - Contenu type:
```yaml
serial: "SN-XXXXXXXX"
device_model: "G30"
```
  - Chargement via rosparam dans un launch “site” (exemple):
```xml
<rosparam file="$(env HOME)/.isward/device/serial.yaml" command="load" />
```
  - Accès dans les nœuds: récupérer `~serial` (ou `/serial`) via Param Server.

- Option: propagation par argument launch
```xml
<arg name="robot_serial" default=""/>
<param name="robot_serial" value="$(arg robot_serial)" />
```
Utilisable pour transmettre un SN en ligne de commande:
```bash
roslaunch launch/isward_common.launch robot_serial:=SN-XXXXXXXX
```

### Tableau récapitulatif par module

| Module | Lancements clés | Paramètres notables | IO (topics/services) | Notes |
|---|---|---|---|---|
| sensor-pkg / astra_camera | `sensor-pkg/launch/sensor.launch`, `astra_camera/launch/dabai_u3.launch` | `enable_point_cloud`, `enable_color`, `enable_depth`, `enable_ir` | Nuage de points, flux depth/color (selon launch) | Les scripts `startup_*.sh` patchent `uvc_product_id` dynamiquement |
| chassis-pkg (BLE) | `chassis-pkg/launch/hal_comm.hal_ble.launch` | `port=/dev/tty_BLE`, `baudrate=115200` | Liaison BLE (série) | Connexion locale courte portée |
| chassis-pkg (MQTT/remote) | `chassis-pkg/launch/hal_comm.hal_remote.launch` | `host=mqtt.xiaojia-tech.com.cn`, `port=8331`, `secret.yaml` | MQTT (télémétrie/commandes) | Transport via Wi‑Fi/LAN ou LTE |
| chassis-pkg (LTE) | `chassis-pkg/launch/hal_usim.launch` | `lte_port=/dev/tty_LTE`, `lte_baudrate=115200` | Modem cellulaire | Apporte connectivité GSM/4G/LTE |
| chassis-pkg (GNSS/RTCM) | `chassis-pkg/launch/hal_gnss*.launch` | `/dev/tty_RADIO`, `config_io_index=13` | Corrections RTCM GNSS | Pour précision de position |
| location-pkg | `location-pkg/launch/location.launch` | — | Poses fusionnées (p.ex. `/fused_pose/path`, `/gnss_pose/path`) | Noms exacts à vérifier à l’exécution |
| mapping-pkg / mapping | `mapping-pkg/launch/mapping.launch`, `mapping/launch/mapping.launch` | Services: `Mapping`, `loadmap`, `querymap` | `/map` via `map_server`; services `mapping/*` | Sauvegarde via `map_saver` → `.pgm` + `.yaml` |
| navigation-pkg / mower | `navigation-pkg/launch/navigation.launch`, `mower/launch/navigation.launch` | TF statique, inclus `move_base`, `map_server`, conversion PC2→Laser | Utilise `/map`, `/scan`, `/depth/scan`, `PointCloud2` | Profil complet de nav |
| move_base | `mower/launch/movebase_normal.launch` | `base_global_planner=coverage_planner`, `base_local_planner=stanley_controller`, `controller_frequency=20` | `cmd_vel` (sortie contrôle), costmaps G/L | Récupérations: rotate, move_back |
| costmap_2d | — (chargé par `move_base`) | `global_costmap_params.yaml`, `local_costmap_params.yaml`, `costmap_common_params_burger.yaml` | Consomme `/scan`, `/depth/scan`, `PointCloud2` | Plugins: Static/Obstacle/Inflation/ContractSensor |
| coverage_planner (global) | `coverage_planner.xml` (plugin) | `coverage_planner_params.yaml` | Chemin global (nav_core) | Largeurs de coûts; `work_path` |
| stanley_controller (local) | `stanley_control_plugin.xml` (plugin) | `stanley_control_params.yaml`, `StanleyController.cfg` | `cmd_vel` (via move_base) | Vitesse max dynamique: `max_linear_x`, `max_angular_z` (dyn reconf) |
| pointcloud_to_laserscan | `mower/launch/pointcloud2laserscan.launch` | — | LaserScan dérivé | Alimente `ObstacleLayer` |
| map_server | `map_server/launch/map_server.launch` | Arg: `/home/isward/.isward/map/map.yaml` | Publie `/map` | Utilisable avec `map_saver` |
| parking-pkg | `parking-pkg/launch/parking.launch` | — | — | Stationnement autonome |
| scheduler-pkg | `scheduler-pkg/launch/scheduler.launch` (+ `*_normal/gdb`) | `functions.yaml`, `velocity.csv`, `version.yaml` | — | Orchestration haute-niveau |
| vision-pkg / charger_pose_pkg | `vision-pkg/charger_pose_pkg/launch/charger_pose_pkg.launch` | `config/camera_and_charger_info.yaml` | Topics vision (selon node) | Détection de chargeur/docking |
| exps_pkg | `exps_pkg/launch/exps_pkg.launch` | — | — | Essais/benchmarks |

### Topics ROS – description
- `/Mapping`: service/action bus pour le package `mapping` (démarrage/extension du mapping). Utilisé pour piloter la construction de carte.
- `/chassis/batteries`: télémétrie batteries (tension, SOC) publiée par `chassis_pkg`.
- `/chassis/fault`: état/événements de fautes châssis (codes erreurs côté bas niveau).
- `/chassis/gnss`: pose/état GNSS filtré (prêt pour fusion localisation).
- `/chassis/gnss_raw`: données GNSS brutes (avant fusion/filtrage).
- `/chassis/imu`: IMU du châssis (accélération, gyros) pour estimation de mouvement.
- `/chassis/odometer`: odométrie instantanée (vitesse/compteurs) du châssis.
- `/chassis/odometer_total`: odomètre cumulatif (distance totale).
- `/chassis/rtcm_stream`: flux RTCM (corrections GNSS) via radio/USIM.
- `/chassis/sensor`: agrégat capteurs châssis (états divers).
- `/chassis/set_velocity`: consigne vitesse de bas niveau (peut être dérivée de `cmd_vel` ou utilisée pour bypass).
- `/chassis/trigger`: événements déclencheurs châssis (ex. E-stop, triggers physiques).
- `/cmd_vel`: commande vitesse géométrique (Twist) issue de `move_base`/local planner (Stanley).
- `/collision`: signal/capteur collision (alimente `ContractSensorLayer` des costmaps globales).
- `/depth/scan`: scan lidar synthétique depuis caméra profondeur (node `pointcloud_to_laserscan`). Consommé par `ObstacleLayer`.
- `/error_code`: bus d’erreurs génériques applicatives (consolidation de fautes/retours).
- `/exps/obstacle_signal`: signalisation obstacle pour modes expérimentaux (`exps_pkg`).
- `/host_ctrl/dev_to_host` et `/host_ctrl/host_to_dev`: canal de contrôle hôte-app locale (IPC/bridge) côté host controller.
- `/keyboard_ctrl/dev_to_keyboard` et `/keyboard_ctrl/keyboard_to_dev`: téléop clavier (aller/retour) pour tests en local.
- `/last_valid_plan`: dernier chemin global valide (diagnostic `move_base`/planner).
- `/location`: pose/localisation fusionnée (sortie pipeline `location-pkg`).
- `/location_state`: état interne du module de localisation (qualité, mode).
- `/map`: carte occupancy grid publiée par `map_server`.
- `/map_metadata`: métadonnées de la carte (résolution, taille, origine).
- `/maping_status` (sic): statut du processus de cartographie (en cours/terminé/erreurs).
- `/move_base/recovery_status`: état courant des comportements de récupération (rotate/move_back).
- `/nav_node_init`: signal d’initialisation du nœud de navigation (prêt à recevoir des objectifs).
- `/navi/cmd/{goal, cancel, status, feedback, result}`: canaux actionlib pour commande de navigation (interface `NaviCtrlAction`), provenant de `interface_pkg`/scheduler.
- `/navi/plan_path`: requêtes/résultats de planification de chemin (niveau supérieur au planner global standard, côté interface/scheduler).
- `/parking/yaw`: angle/tête du véhicule pendant le stationnement (pilotage parking).
- `/parking_vision_init`: état/trigger d’initialisation vision parking (charger_pose).
- `/path_fusion`: trajectoire fusionnée (combinaison de sources planifiées/sensorielles).
- `/path_signal`: signalisation d’étapes de trajet (waypoints, segments, events).
- `/remote/iot`: messages IoT génériques (pont MQTT `hal_remote`).
- `/remote_ctrl/dev_to_remote` et `/remote_ctrl/remote_to_dev`: commandes/télémétrie côté télécommande distante (via MQTT/bridge).
- `/robot_walk_path`: chemin suivi (trace temps réel) pour debug/visualisation.
- `/rosout` et `/rosout_agg`: logs ROS standard.
- `/scheduler/safe_with`: état sécurité scheduler (conditions safe/unsafe).
- `/scheduler/state`: état machine d’états du scheduler (mode courant, transitions).
- `/switch_location`: trigger de bascule de source de localisation (GNSS ↔ fusion, etc.).
- `/test/rslt`: résultats de tests (canal de test/validation).
- `/tf` et `/tf_static`: transformations de frames (runtime et statiques). `tf_static` inclut `base_link -> isward_link`.
- `/touch_stop`: arrêt tactile/arrêt d’urgence logiciel (E-stop soft).
- `/usim/rssi`: niveau RSSI cellulaire LTE (mesure qualité radio de `hal_usim`).

### Gestion des logs & bag (log_clear.py)
- Script `scripts/log_clear.py` lancé en arrière-plan par les scripts startup.
- Rôle: maintenance des répertoires de logs et bag.
  - Prépare `~/.isward/bag` si absent.
  - Seuils dépendants de `version.yaml`: release (0.1 GiB) vs test (1 GiB bag, 0.3 GiB logs).
  - Supprime périodiquement (toutes 30 s) les contenus excédentaires en conservant `latest` quand présent.
- Répertoires concernés: `/home/isward/.isward/bag`, `/var/log`, `/root/.ros/log`, `/home/isward/.ros/log`, `/home/isward/.isward/log`.

### Lancement capteurs (sensor.launch)
- `sensor-pkg/launch/sensor.launch` inclut:
  - `astra_camera/launch/dabai_u3.launch` (flux color/depth/IR/pointcloud, param `serial_number` possible).
  - `camera_checker/launch/camera_checker.launch` (sanity check caméra).

### Données runtime et artefacts (~/.isward)
- Emplacement: `~/.isward` (vous avez ajouté ce répertoire; il contient les données runtime essentielles).
- Sous-dossiers/fichiers importants:
  - `~/.isward/map/`
    - `map.yaml`, `map.pgm`: carte courante utilisée par `map_server` et `move_base`.
    - `gen_path/`: génération de cartes/chemins (ex: `gen_path/map.yaml`, `gen_path/map.pgm`).
    - `draft_map.info`, `work_map.info`, `record.info`: méta-infos de workflow de carte (brouillon, travail, enregistrement).
  - `~/.isward/coverage_planner/`
    - `global/map.png`, `sub_*/map.png`, `sub_*/coverage`, `sub_*/planner`: artefacts de plan de couverture (par sous-zones), `zones/map.png`, `zones/zone`.
    - `map_list`, `plan_info`: indexation et méta-infos de plans.
  - `~/.isward/nav/isward.urdf.xacro`: description robot (URDF xacro) utilisée par navigation/visualisation.
  - `~/.isward/vision/charger_obj/*.engine`, `charger_opt/*.engine`: modèles TensorRT pour vision (détection station/chargeur).
  - `~/.isward/bag/`: stockage des rosbags (contrôlé par `log_clear.py`).
  - `*.png` de diagnostic (ex: `error_recovery_behaviors.png`, `can_not_reach_*`, `robot_sliped.png`): captures d’échec de planning/récupérations.
  - `*.info` (ex: `cfg.info`, `csys.info`, `maintenance.info`, `task.info`): méta/config système, maintenance, tâches (formats internes).
  - `gnss_record.log`: traces GNSS.

### Vision – objets détectés (TensorRT/ONNX)
- Modèles et cibles:
  - `detect_charger_station.onnx`: détecte la station/borne de charge (objet chargeur).
  - `detect_point.onnx`: détecte un point de repère (marker) associé au chargeur.
- Emplacements modèles:
  - Source ONNX: `src/vision-pkg/charger_pose_pkg/data/weights/` (référencés par le launch).
  - Engines TensorRT générés: `~/.isward/vision/charger_obj/*.engine`, `~/.isward/vision/charger_opt/*.engine`.
- Lancement Vision:
  - `vision-pkg/charger_pose_pkg/launch/charger_pose_pkg.launch`
    - `enable_station:=true`
    - `charger_object_model_path`: chemin ONNX station
    - `point_model_path`: chemin ONNX point
    - `camera_and_charger_info.yaml`: intrinsics/ROI
    - `save_image_dir`: `~/.isward/raw_charger` (captures)
    - `name_server_enable_transform`: `/enable_charger_location` (commande/trigger)
- Calibration/ROI (`camera_and_charger_info.yaml`):
  - `camera_matrix` et `distortion_coefficients` (RGB)
  - `camera_matrix_ir` et `distortion_coefficients_ir` (IR)
  - `charger_side_length`: 0.06 m
  - `label_point`: polygone image [x,y,...] délimitant la zone d’intérêt
- Entrées/Sorties attendues:
  - Entrée: flux caméra (RGB/IR selon config) avec intrinsics calibrés
  - Sorties: détections bbox + estimation pose locale de la station/point (consommées pour localisation de stationnement/retour base). Le nœud publie aussi des images dans `save_image_dir` si activé.
- Intégration nav/parking:
  - Le signal `/parking_vision_init` (déjà documenté) et la commande `/enable_charger_location` permettent d’activer la recherche de chargeur pour le docking.

### Détection d’obstacles et d’animaux
- État actuel (présent dans le projet):
  - Évitement d’obstacles via costmaps (`costmap_2d::ObstacleLayer` + `InflationLayer`).
    - Sources: `/depth/scan` (depuis `pointcloud_to_laserscan`), éventuellement `PointCloud2`/`/scan` selon config.
  - Capteurs châssis et contact/collision:
    - Topic `/collision` alimentant `ContractSensorLayer` (couche collision) dans la costmap globale.
  - Recoveries (`move_base`): rotation, reculer/nettoyer si blocage.
- Absences:
  - Aucun modèle dédié “animaux/personnes/objets spécifiques” n’est inclus (pas de YOLO/SSD/TF model générique dans ce repo).
  - Seules les détections vision livrées concernent la station de charge (charger_pose).
- Options d’intégration (recommandations):
  - Vision objets/animaux: intégrer un nœud de détection (ex. `ros-yolov5`/`ros-openvino`/`tensorRT-yolov5`) publiant des bboxes et zones interdites → convertir en couche costmap (plugin custom ou `costmap_converter`).
  - Lidar/Depth: ajuster `obstacle_range`, `raytrace_range`, `inflation_radius` pour sécurité accrue autour d’obstacles potentiels.
  - Sécurité: ajouter une règle de “stop” si classe sensible (animal/enfant) détectée devant, via un gate sur `cmd_vel`.

### Système – dossier opt (Jetson/NVIDIA & ROS Noetic)
- `root/opt/ros/noetic`: outils/paquets ROS préinstallés (rqt, rviz, rostopic, roslaunch…).
- `root/opt/nvidia`: utilitaires Jetson (jetson-io, VPI, nsight, USB gadget…).
- `root/opt/ota_package`: payloads de mise à jour bootloader.
- Conseils: ne modifiez pas ces dossiers; utilisez-les (rqt/rviz) côté runtime.

Notes:
- Les chemins de lancement actuels pointent déjà sur `~/.isward/map/map.yaml` pour `move_base`/`map_server`. Vous pouvez basculer sur `gen_path/map.yaml` pour tester une carte générée.
- Les PNG de diagnostic sont utiles pour le post‑mortem; conservez-les lors des tickets.

### Procédure complète (pas-à-pas, détaillée)
1) Préparation système
   - Installer ROS Noetic et dépendances (`move_base`, `map_server`, `costmap_2d`, capteurs Astra).
   - Créer les répertoires: `~/.isward/{map,bag,log,device}`.
   - Placer carte initiale si disponible: `~/.isward/map/map.yaml` + `map.pgm`.
   - (Option) `~/.isward/device/serial.yaml` pour SN (voir section Identification).

2) Configuration capteur caméra
   - Vérifier `/dev/video0` et patch auto `uvc_product_id` (scripts `startup_*.sh`).
   - (Option) Renseigner `serial_number` dans `astra_camera/launch/dabai_u3.launch` si multi‑caméras.

3) Lancement système
   - Local: `bash scripts/startup_localhost.sh` (définit ROS env, lance `isward_common.launch`).
   - Outdoor/LAN: `bash scripts/startup_outdoor.sh`.
   - Vérifier topics clés: `/tf`, `/map`, `/cmd_vel`, `/chassis/*`.

4) Cartographie (si nécessaire)
   - Démarrer le service `mapping` via `/Mapping` (mode 1=start, 2=extend) ou lancer le mapping selon vos procédures.
   - Sauvegarder la carte: `rosrun map_server map_saver -f $(env HOME)/.isward/map/map`.
   - Confirmer: `~/.isward/map/map.yaml` et `.pgm` mis à jour.

5) Navigation et couverture
   - `move_base` utilise CoveragePlanner (global) + Stanley (local); vérifier que `map_server` publie `/map`.
   - Surveiller `last_valid_plan`, `/move_base/recovery_status`, `robot_walk_path`.
   - Artefacts de couverture générés sous `~/.isward/coverage_planner/` (global et `sub_*`).

6) Tonte (lame et zones)
   - Vérifier le champ `mow` dans `NaviCtrlFeedback` (si la coupe est requise).
   - Activer la lame via service `ChassisCtrlBlade` (voir section Tonte). Surveiller `ChassisSensor.blade_*`.

7) Connectivité & IoT
   - MQTT: `hal_remote` → `mqtt.xiaojia-tech.com.cn:8331`, secrets depuis `chassis-pkg/launch/secret.yaml`.
   - LTE: `hal_usim` (RSSI sur `/usim/rssi`). BLE: `hal_ble`.

8) Visualisation à distance
   - `web_video_server` (HTTP) ou `rosbridge_server` (WebSocket), voir section dédiée.
   - RViz/rqt sur LAN/VPN.

9) Logs & bag
   - `scripts/log_clear.py` purge automatiquement; vérifier seuils (release/test) et espace disque.
   - Rosbags sous `~/.isward/bag` (option rosbag dans `isward_common.launch`).

10) Diagnostics & recovery
   - Oscillation/timeout → récupérations (rotate, move_back). Voir `move_base_params.yaml`.
   - PNG d’erreur sous `~/.isward` (`error_*`, `can_not_reach_*`) pour analyse.

### Téléop clavier (commands clavier)
- Topics:
  - `/keyboard_ctrl/keyboard_to_dev`: commandes émises depuis le clavier vers le dispositif (robot). Typiquement, messages de téléop manuelle (avancer/reculer/gauche/droite/stop) via un nœud de téléop clavier.
  - `/keyboard_ctrl/dev_to_keyboard`: retours d’état du dispositif vers l’interface clavier (acks, états).
- Remarques:
  - Les mappings exacts touches→actions ne sont pas exposés dans ce dépôt. Ils dépendent du nœud qui publie sur `/keyboard_ctrl/keyboard_to_dev` (non fourni ici). Par défaut, la téléop peut aussi agir via `/cmd_vel` si un node `teleop_twist_keyboard` est utilisé.

### Tonte: zones, couverture et lame
- Zones de tonte / couverture:
  - Global planner `coverage_planner`: génère des trajectoires de couverture sur `/map`.
  - Message `interface_pkg/MowMapData`: expose `mow_area` (surface couverte) et `mow_rate` (taux), utile pour télémetrie/rapports de tonte.
  - Paramètres de couverture: `mower/params/coverage_planner_params.yaml` (`base_map_cost_width`, `coverage_map_cost_width`, `border_map_cost_width`) et `work_path` pour fichiers de travail.
  - Indicateurs dans Stanley (dyn. reconfigure): paramètres liés au "border plan" (`border_plan_rotate_yaw_diff`, `border_lookforward_distance`) et seuil `last_record_area_points_num_thresh` pour remow d’une zone si obstacles ont disparu.
- Démarrage/contrôle de la lame (moteur de coupe):
  - Service `interface_pkg/ChassisCtrlBlade` (requête/réponse) pour piloter la lame au niveau châssis.
  - Télémétrie lame dans `interface_pkg/ChassisSensor`: `blade_height` (hauteur en mm), `blade_speed` (tr/min réel).
  - Intégration nav/scheduler: le champ `mow` (bool) dans `NaviCtrlFeedback` indique si la découpe est requise sur le segment courant.
- Flux opérationnel type:
  1) Navigation vers/à l’intérieur d’une zone (CoveragePlanner) avec suivi Stanley.
  2) Activation de la lame via `ChassisCtrlBlade` en fonction du plan/statut `mow`.
  3) Publication de la progression de coupe via `MowMapData` (surface/taux), mise à jour de l’état jusqu’à fin de zone.

### Schéma des nœuds et topics (Mermaid)
```mermaid
flowchart LR
  subgraph Perception & Capteurs
    AC[astra_camera]
    PCL2LASER[pointcloud_to_laserscan]
    AC -->|PointCloud2| PCL2LASER
    PCL2LASER -->|/depth/scan| COSTMAP_LOCAL
  end

  subgraph Navigation
    MB[move_base]
    COSTMAP_GLOBAL[global_costmap]
    COSTMAP_LOCAL[local_costmap]
    CP[coverage_planner]
    STANLEY[stanley_controller]
    MB <--> CP
    MB <--> STANLEY
    MB <--> COSTMAP_GLOBAL
    MB <--> COSTMAP_LOCAL
    MB -->|/cmd_vel| CHASSIS
  end

  subgraph Cartographie
    MAPS[map_server]
    MAPPING[mapping]
    MAPPING <--> |/Mapping| IFACE[interface_pkg]
    MAPS -->|/map| COSTMAP_GLOBAL
    MAPS -->|/map| MB
  end

  subgraph Localisation
    LOC[location]
    LOC -->|/location| MB
  end

  subgraph Châssis & IO
    CHASSIS[chassis_pkg]
    BLE[hal_ble]
    USIM[hal_usim]
    REM[hal_remote (MQTT)]
    RTCM[hal_rtcm]
    REM <--> |/remote/*, /host_ctrl/*| CHASSIS
    USIM -->|/usim/rssi| CHASSIS
    RTCM -->|/chassis/rtcm_stream| CHASSIS
  end

  subgraph Vision
    VISION[charger_pose_pkg]
    VISION -->|/parking_vision_init| MB
  end

  TF[tf2 static publisher]
  TF -->|/tf_static (base_link→isward_link)| MB
  CHASSIS -->|/chassis/*, /collision| COSTMAP_GLOBAL
  CHASSIS -->|/chassis/*| MB

  ROSOUT[rosout]
  MB -->|/rosout| ROSOUT
  CHASSIS -->|/rosout| ROSOUT
  MAPS -->|/rosout| ROSOUT
```

### Visualisation à distance (options)
- Sur LAN/VPN (ROS natif)
  - `rqt_image_view` ou RViz sur le même `ROS_MASTER_URI`.
  - Conseillé: `image_transport` en `compressed`/`compressedDepth` pour réduire le débit.
  - Commandes:
```bash
sudo apt install ros-noetic-rqt-image-view
ROS_MASTER_URI=http://<robot>:11311 rqt
```

- Web (HTTP MJPEG) sans dev applicatif
  - Installer `web_video_server` puis exposer un port HTTP.
  - Commandes:
```bash
sudo apt install ros-noetic-web-video-server
rosrun web_video_server web_video_server _port:=8080
# Naviguer: http://<robot>:8080/  (sélectionner un topic ex: /camera/color/image_raw)
```

- API WebSocket (intégration web/app)
  - Ajouter `rosbridge_server` pour clients JS (Foxglove/Webviz, applis custom).
  - Commandes:
```bash
sudo apt install ros-noetic-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch
# Par défaut ws://<robot>:9090
```

- Snapshots (captures ponctuelles)
  - Sauvegarder des images sur disque (puis transfert via SCP/MQTT/HTTP).
  - Commandes:
```bash
sudo apt install ros-noetic-image-view
rosrun image_view image_saver image:=/camera/color/image_raw \
  _filename_format:=/tmp/frame-%04d.jpg
```

- Flux compressés (WAN)
  - Utiliser `image_transport` et consommer `/.../image_raw/compressed` ou `/compressedDepth`.
  - Réduire résolution/fréquence côté source si nécessaire.

- Sécurité & réseau
  - Ouvrir les ports nécessaires (ex: 8080 pour web_video_server, 9090 pour rosbridge) sur le pare-feu.
  - Préférer VPN/SSH tunnel sur WAN; limiter l’accès par IP.

