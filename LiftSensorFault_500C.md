# Diagnostic : défaut 500C / capteur de levée (CLIFF_FAULT)

**Produit :** tondeuse robot Isward II (ROS Noetic)  
**Symptôme app :** `500C` — *lift sensor fault*  
**Cause racine identifiée :** **aimant cassé / manquant** sur la roue folle **gauche** (capteur magnétique Hall / reed)  
**Date du diagnostic : 17 mai 2026  

---

## Résumé exécutif

La tondeuse affichait un défaut intermittent **500C** (*lift sensor fault*), parfois dès le démarrage, parfois après quelques minutes de tonte, **sans modification du logiciel ROS**.

L'investigation a montré que :

1. Le défaut logiciel correspond à **`CLIFF_FAULT` (256)** — détection de **roue folle avant** non posée, et non à la hauteur de lame (`LIFT_FAULT`).
2. Les capteurs concernés sont des **capteurs magnétiques** sous les roues folles (`cliff[]` dans `/chassis/sensor`).
3. Le côté **gauche** (`cliff[1]`) restait à une valeur basse (~2–52) au repos, alors que le **droit** (`cliff[0]`) était correct (~96).
4. La rotation manuelle de la roue gauche faisait chuter puis verrouiller `cliff[1]`, déclenchant **`scram: True`** (arrêt d'urgence).
5. **Cause finale :** petit **aimant cassé** côté gauche — l'aimant ne passe plus correctement devant le capteur Hall au repos.

---

## Architecture logicielle et matérielle

### Cartes et liaisons série

Le nœud `hal_chassis` (`chassis_pkg`) dialogue avec trois cartes via un hub USB CH9344 :

| Port ROS / udev | Carte | Baud | Rôle |
|-----------------|-------|------|------|
| `/dev/tty_CHASSIS` | Carte moteur (driver) | 460800 | Roues motrices, odometrie, lame |
| `/dev/tty_SENSOR` | **Carte capteurs** | 460800 | `cliff`, `putup`, `crash`, pluie, température |
| `/dev/tty_POWER` | Carte alimentation | 115200 | Batterie |

Configuration typique : `hal_chassis_normal.launch` (package `chassis_pkg`).

### Topics ROS utiles

| Topic | Type | Description |
|-------|------|-------------|
| `/chassis/sensor` | `interface_pkg/ChassisSensor` | Valeurs capteurs (traitées 0–100) |
| `/chassis/fault` | `interface_pkg/ChassisFault` | Masque de défauts (`fault_info`) |
| `/error_code` | `std_msgs/Int32` | Code erreur exposé à l'app / BLE (ex. `10010`) |

### Familles de capteurs (message `ChassisSensor`)

```text
uint8[] crash   # Collision pare-chocs (0–100)
uint8[] putup   # Soulèvement châssis (0–100) — souvent vide sur ce modèle
uint8[] cliff   # Suspension / sol sous roues folles (0–100)
bool  scram     # Arrêt d'urgence actif (True = sécurité enclenchée)
```

### Codes de défaut (`ChassisFault`)

| Constante | Valeur | Signification |
|-----------|--------|----------------|
| `CLIFF_FAULT` | **256** | **Roue folle / anti-chute** (万向轮抬起检测) — **défaut observé** |
| `LIFT_FAULT` | 8 | Hauteur de lame (moteur de coupe) — **non impliqué ici** |
| `CRASH_FAULT` | 2 | Collision / pare-chocs |
| `BOARD_SENSOR_FAULT` | 131072 | Carte capteurs |

> **Note :** l'app affiche souvent *« lift sensor fault »* / **500C** pour toute la famille « levée / soulèvement », alors que ROS signale en réalité **`CLIFF_FAULT`**.

### Correspondance index `cliff[]` ↔ côté (validée sur machine)

| Index ROS | Côté | Rôle |
|-----------|------|------|
| **`cliff[0]`** | **Droit** | Capteur magnétique sous roue folle droite |
| **`cliff[1]`** | **Gauche** | Capteur magnétique sous roue folle gauche |

**La direction de navigation** ne vient **pas** de ces capteurs : GNSS (`/chassis/gnss`), IMU (`/chassis/imu`), odometrie (`/chassis/odometer`).

---

## Symptômes observés

### Côté utilisateur / app

- Code **500C** — *lift sensor fault*
- Comportement **intermittent** : parfois ~5 min de tonte, parfois défaut immédiat
- Pas de panne évidente au premier regard (nettoyage, essais d'échange matériel)

### Côté ROS (état défaillant typique)

```yaml
cliff: [96, 52]   # puis [96, 2] après rotation / défaut
scram: False      # puis True après seuil franchi
fault_info: 256   # CLIFF_FAULT
error_code: 10010
putup: []         # vide en permanence sur cette machine
crash: [100, 99]  # pare-chocs OK
```

### Tableau des tests décisifs

| Test | Résultat | Conclusion |
|------|----------|------------|
| Roues au sol vs levées | `cliff` inchangé `[96, 52]` | Aimant absent / signal figé au repos |
| Rotation roue **gauche** | `cliff[1]` : 21 → 3 → 2, `scram: True` | Capteur vivant, **aimant HS ou mal aligné** |
| Capteurs débranchés + **reboot total** | Plus de défaut à l'app | Panne dans la **chaîne capteurs** |
| Capteurs débranchés, sans reboot | Valeurs **identiques** | État **latched** (pas lecture live) |
| Gauche seul / droit seul / les deux | `[44,34]` → `[96,34]` → `[96,42]` | Panne **gauche** (`cliff[1]`) |
| Carte capteurs + roue gauche (2ᵉ tondeuse) | `[96, 96]` stable | Gauche réparée |
| Droit oublié débranché | `[55, 96]` | Entrée ouverte = valeur **basse** côté débranché |

---

## Principe de fonctionnement (capteurs magnétiques)

Chaque roue folle avant :

- **capteur Hall** fixe sur le châssis ;
- **aimant** sur la tige de la roulette.

La carte capteurs renvoie une valeur **0–100** :

| Valeur `cliff[i]` | Interprétation |
|-------------------|----------------|
| **~90–100** | Aimant aligné — roue **OK** |
| **~40–55** | Zone limite — risque de défaut |
| **< ~30** | Aimant éloigné — roue « levée » → **CLIFF_FAULT** |

En **rotation lente**, les valeurs **doivent osciller**. Au **repos**, elles doivent rester **hautes** (~96) en position de tonte normale.

---

## Procédure de diagnostic

### Prérequis

```bash
source /opt/ros/noetic/setup.bash
source /home/isward/isward_ii_ws/devel/setup.bash
```

### Étape 1 — Lecture rapide

```bash
rostopic echo /chassis/sensor -n 1
rostopic echo /chassis/fault -n 1
rostopic echo /error_code -n 1
```

### Étape 2 — Interprétation au repos (roues au sol, arrêt)

| `cliff` | Diagnostic |
|---------|------------|
| `[96, 96]` | **OK** |
| `[96, <55]` | Problème **gauche** (`cliff[1]`) |
| `[<55, 96]` | Problème **droit** (`cliff[0]`) |
| `[bas, bas]` | Deux côtés, carte, ou câblage |
| Identique avec capteurs débranchés | **Latched** → coupure batterie + reset |

### Étape 3 — Un capteur à la fois

1. Brancher **gauche seul** → noter `cliff`
2. Brancher **droit seul**
3. Brancher **les deux**

L'entrée qui monte vers **~96** quand branchée identifie le bon index.

### Étape 4 — Rotation lente d'une roue

```bash
watch -n 0.3 'source /home/isward/isward_ii_ws/devel/setup.bash; rostopic echo /chassis/sensor -n 1 | grep cliff'
```

- Oscillation puis retour haut → capteur OK, vérifier **aimant au repos**
- Reste à 2–3 → aimant **cassé / absent**
- `scram: True` → reset app + coupure alimentation

### Étape 5 — Carte capteurs

```bash
rosservice call /chassis/get_board_version "index: 1"
rosservice call /chassis/get_board_uuid "index: 1"
```

(`index: 0` = moteur, `1` = capteurs, `2` = alimentation)

### Étape 6 — Effacement du défaut

1. Corriger la cause mécanique (aimant, capteur, nappe)
2. **Coupure batterie** 30–60 s
3. Reset app (`REMOTE_ERROR_RESOLVE` si disponible)
4. Vérifier `scram: False` et `cliff: [96, 96]`

---

## Champ `scram` (arrêt d'urgence)

| Valeur | Signification |
|--------|----------------|
| `scram: True` | Arrêt d'urgence **actif** |
| `scram: False` | Arrêt levé — **ne garantit pas** l'absence de défaut cliff |

Le reset app peut remettre `scram` à `False` alors que `cliff[1]` reste bas.

---

## Pièges et faux diagnostics

### 1. Valeurs ROS figées après défaut

Après incident, `rostopic` peut afficher `[96, 2]` **même capteurs débranchés** tant que le défaut n'est pas réinitialisé. **Toujours** couper l'alimentation complète avant de conclure.

### 2. Confusion LIFT vs CLIFF vs app

| Terme | Réalité |
|-------|---------|
| App « lift sensor » / 500C | Souvent **`CLIFF_FAULT`** (roues folles) |
| `LIFT_FAULT` (8) | Hauteur de **lame** |
| `putup[]` vide | ≠ `cliff` — ne pas mélanger |

### 3. Côté débranché = valeur basse

Un capteur **non branché** peut lire ~34–55, pas zéro. Tester **les deux branchés** avant validation.

### 4. Échange tondeuse sans test gauche/droit

Le test **un côté à la fois** + **rotation** reste indispensable.

---

## Résolution (cas documenté)

| Action | Résultat |
|--------|----------|
| Diagnostic `cliff[1]` bas au repos | Côté **gauche** |
| Rotation → chute vers 2 + `scram` | Aimant / alignement |
| **Aimant cassé gauche** réparé/remplacé | Cause racine |
| Remplacement temporaire carte + roue gauche (tondeuse 2) | `cliff: [96, 96]` |
| Validation finale | `scram: False`, `cliff: [96, 96]` stable |

### Critères « OK pour tondre »

```bash
rostopic echo /chassis/sensor -n 1
# cliff: [96, 96]  (ou les deux > ~80)
# scram: False

rostopic echo /chassis/fault -n 1
# fault_info: 0
```

Rotation lente des deux roues : valeurs **varient** puis **reviennent hautes** en position normale.

---

## Référence des commandes

```bash
source /opt/ros/noetic/setup.bash
source /home/isward/isward_ii_ws/devel/setup.bash

rostopic echo /chassis/sensor -n 1
rostopic echo /chassis/fault -n 1
rostopic echo /error_code -n 1

rosservice call /chassis/get_board_version "index: 1"
rosservice call /chassis/get_board_uuid "index: 1"

watch -n 0.5 'rostopic echo /chassis/sensor -n 1 | grep -E "cliff|scram"'
```

### Sensibilité app (`~/.isward/cfg.info`)

```json
"security": {
    "crash": 1,
    "putup": 0
}
```

`putup: 0` = sensibilité max sur la chaîne « soulèvement châssis » (si câblée). Le défaut documenté ici concerne surtout **`cliff`**.

---

## Chronologie du diagnostic

1. Défaut **500C** — `fault_info: 256`, `cliff: [96, 52]`
2. `putup: []` permanent
3. Tests sol / levage / rotation → `cliff` figé puis réactif à la rotation gauche
4. Débranchement + reboot → plus de défaut
5. Tests gauche / droit / les deux → **`cliff[1]` = gauche**
6. Remplacement carte + roue gauche → `[96, 96]`
7. **Aimant cassé gauche** identifié

---

## Recommandations maintenance

1. Inspecter les **aimants** des deux roues folles (collage, fissure).
2. Nettoyer capteurs Hall et vérifier l'**entrefer**.
3. Après choc : `rostopic echo /chassis/sensor -n 1` au repos.
4. Ne pas tondre si un `cliff[i] < 55` au repos.
5. Avant de changer la carte capteurs : test **un côté à la fois**.

---

<img width="600" height="800" alt="image" src="https://github.com/user-attachments/assets/828871d2-930d-4f54-b9fa-1c3d9e67b080" />
<img width="3024" height="4032" alt="IMG_2489" src="https://github.com/user-attachments/assets/610c7c55-941f-4cdf-a597-8e0f118f4a55" />


*Document de diagnostic terrain — Isward II, ROS Noetic, `hal_chassis` / carte capteurs via `/dev/tty_SENSOR`.*
