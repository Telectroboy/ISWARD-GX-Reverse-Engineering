## ISWARD II – Recommandations, options et évolutions

### Chemins et portabilité
- Remplacer les chemins absolus `/home/isward/...` par:
  - `$(env HOME)/.isward/map/map.yaml` pour la carte
  - `$(find <pkg>)/launch/...` pour tous les includes
- Ajouter un `arg map_yaml` aux launch pour changer de carte sans éditer les fichiers.

### Identification & SN
- Interroger le châssis via `ChassisGetInfo` (INDEX_BOARD_*), parser la réponse pour le SN.
- Stocker le SN dans `~/.isward/device/serial.yaml` et charger via `<rosparam ...>`.
- Propager un `arg robot_serial` dans les launch si besoin d’override.

### Visualisation à distance
- HTTP: `web_video_server` (port 8080) pour MJPEG.
- WebSocket: `rosbridge_server` (port 9090) pour intégration front (WebViz/Foxglove).
- Snapshots: `image_view/image_saver` pour captures ponctuelles.
- Sécurité: restreindre par VPN/SSH tunnel et firewall.

### Navigation et sécurité
- Ajuster `obstacle_range`, `raytrace_range`, `inflation_radius` pour marges de sécurité.
- Augmenter `controller_patience`, peaufiner `oscillation_*` selon l’environnement.
- Ajouter un “cmd_vel gate” (stop) si conditions dangereuses (p.ex. vision détecte animal/enfant).

### Détection d’objets/animaux (extension)
- Intégrer un détecteur (YOLO/TensorRT) publiant des bboxes/classes.
- Convertir détections en polygones costmap (plugin custom ou `costmap_converter`).
- Règles: ralentir, contourner, ou stop selon classe/distance.

### Cartographie
- Workflow recommandé: `Mapping` (mode 1) → vérification → `map_saver` → pointer `map_server` sur la nouvelle carte.
- Conserver les cartes (historique) sous `~/.isward/map/` (versionnez les YAML/PGM clés).

### IoT et secrets
- Éviter de versionner des secrets: utiliser variables d’environnement ou coffre-fort.
- Paramétrer `hal_remote` (host/port) par arguments ou env.

### Logs & rosbags
- Vérifier les seuils dans `log_clear.py` (release/test) selon l’espace disponible.
- Archiver automatiquement les PNG diagnostics de `~/.isward` avec timestamp.

### Déploiement & QA
- Check-list avant déploiement:
  - ROS Noetic et dépendances installés
  - carte présente (YAML/PGM)
  - secrets/`secret.yaml` validés
  - MQTT accessible (firewall/ports)
  - capteurs OK (`sensor.launch`)
  - test `move_base` sur parcours court; vérifier recoveries
  - vision chargeur opérationnelle (`charger_pose_pkg.launch`)


