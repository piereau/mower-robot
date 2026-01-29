---
stepsCompleted: [1, 2, 3]
inputDocuments:
  - /Users/pierrot/Downloads/AI_Powered_Robotic_Lawn_Mower_with_Auton.pdf
session_topic: "Architecture technique robot tondeuse autonome pour vignes"
session_goals: "Solution pratique + tradeoffs clairs"
selected_approach: "ai-recommended"
techniques_used:
  - "Constraint Mapping"
  - "Morphological Analysis"
date: 2026-01-28
author: Pierrot
---

# Brainstorming Session: Architecture Robot Tondeuse Vigne

**Facilitator:** Pierrot  
**Date:** 2026-01-28  
**Durée:** ~45 minutes

---

## Session Overview

**Topic:** Architecture technique pour un robot tondeuse autonome destiné à tondre les mauvaises herbes entre les rangs de vigne.

**Goals:**
- Solution pratique et réaliste pour un prototype
- Tradeoffs clairs entre différentes approches
- Base solide pour évolution académique → entreprise

**Constraints identifiées:**
- Low latency (temps réel)
- Low power (embarqué RPi)
- Maintainability (architecture simple et évolutive)

---

## Prototype Actuel

| Composant | Description |
|-----------|-------------|
| **Compute** | Raspberry Pi 4B |
| **Caméra** | RPICam 2.1 (connectée CSI) |
| **Moteurs** | 2 moteurs DC (chenilles) + pont en H |
| **Microcontrôleur** | Arduino Nano (contrôle moteurs) |
| **Capteur distance** | HC-SR04 (ultrason) |
| **LiDAR** | Type LD19 (pas encore intégré) |
| **Locomotion** | Chenilles (pas roues) |

---

## Décisions d'Architecture

### 1. Architecture Générale: ROS 2 + Arduino

**Décision:** Architecture hybride avec séparation claire des responsabilités.

```
┌─────────────────────────────────────────┐
│  RPi 4B (ROS 2) - Haut niveau           │
│  - Perception (SLAM, vision)            │
│  - Localisation (EKF fusion)            │
│  - Navigation (Nav2)                    │
│  - Mission management                   │
│  - UI / Télémétrie                      │
└─────────────────────────────────────────┘
              │ Série 50-100 Hz
              ▼
┌─────────────────────────────────────────┐
│  Arduino Nano - Temps réel              │
│  - PID vitesse moteurs                  │
│  - Lecture encodeurs                    │
│  - Watchdog / E-stop                    │
│  - Sécurité bas niveau                  │
└─────────────────────────────────────────┘
```

**Rationale:**
- Arduino garantit la sécurité même si RPi plante
- Séparation claire: cerveau (RPi) vs réflexes (Arduino)
- Latence contrôlée pour les moteurs

### 2. MQTT: Où l'utiliser (et où non)

**Décision:**
- **OUI** pour: télémétrie, UI, logs, cloud, analytics
- **NON** pour: commande moteur bas niveau, boucle de sécurité

**Rationale:**
- MQTT n'est pas temps réel strict
- Boucle critique doit rester série directe avec watchdog
- MQTT excellent pour découplage services haut niveau

### 3. Cartographie: SLAM + RTK-GPS (hybride)

**Décision:** Utiliser les deux approches en parallèle.

| Approche | Usage | Avantages | Limites |
|----------|-------|-----------|---------|
| **RTK-GPS** | Position globale (parcelle) | Réutilisable, géofencing, traçabilité | Dégradé sous feuillage |
| **SLAM LiDAR** | Position locale (précision) | Robuste au glissement, évitement | Dérive long terme |

**Fusion:** EKF (`robot_localization`) combine:
- Encodeurs (effort)
- IMU (rotation)
- LiDAR SLAM (correction monde)
- RTK-GPS (ancrage global, quand disponible)

### 4. Capteurs Prioritaires

**Indispensables (à ajouter):**
1. **Encodeurs roues** - PID vitesse + odométrie
2. **IMU (9 axes)** - Compensation glissement chenilles, stabilité

**Déjà présents:**
3. **HC-SR04** - Proximité simple
4. **RPICam** - Vision (optionnel pour guidage rang)

**À intégrer:**
5. **LD19 LiDAR** - SLAM 2D + évitement

**Plus tard:**
6. **RTK-GPS** - Localisation globale

### 5. Pourquoi IMU est critique avec chenilles

Les chenilles **glissent** beaucoup plus que des roues:
- Sol meuble, pente, ornières en vigne
- L'odométrie seule est très imprécise

L'IMU apporte:
- **Gyro** → vitesse de rotation instantanée
- **Accéléro** → détection inclinaison, basculement
- **Fusion** → continuité entre fixes GPS

### 6. Demi-tours en bout de rang

**Décision:** Nav2 gère les manœuvres serrées.

**Prérequis:**
- Footprint robot défini (dimensions chenilles)
- Local planner adapté (rotations sur place)
- Zones "headland" définies dans la carte

**Mission type:**
```
rang_i → waypoint_fin → demi_tour → alignement → rang_i+1
```

---

## Stacks Logicielles Recommandées

### ROS 2 (RPi)
- **SLAM 2D:** `slam_toolbox`
- **Navigation:** Nav2
- **Fusion capteurs:** `robot_localization` (EKF)
- **LiDAR driver:** `ldlidar_ros2` ou `ldrobot-lidar-ros2`

### Arduino
- PID vitesse (bibliothèque PID ou custom)
- Protocole série binaire avec CRC
- Watchdog 200ms timeout

---

## Contrat d'Interfaces

### Série RPi ↔ Arduino

**Format trame:**
```
SOF(2)=0xAA55 | VER(1) | MSG_TYPE(1) | SEQ(1) | LEN(1) | PAYLOAD(N) | CRC16(2)
```

**Messages:**
- `CMD_VEL` (0x01): v_mm_s, w_mrad_s, flags
- `CMD_STOP` (0x02): reason_code
- `TEL_ODOM` (0x81): ticks, vitesses, status
- `TEL_SAFETY` (0x82): safety_bits, fault_code

**Fréquences:**
- RPi → Arduino: 50 Hz
- Arduino → RPi: 50 Hz
- Watchdog timeout: 200 ms

### Topics ROS 2 Principaux

| Topic | Type | Fréquence | Source |
|-------|------|-----------|--------|
| `/scan` | LaserScan | 8-15 Hz | LD19 driver |
| `/imu/data` | Imu | 50-200 Hz | IMU node |
| `/wheel/odometry` | Odometry | 30-100 Hz | serial_bridge |
| `/odometry/filtered` | Odometry | 30-100 Hz | robot_localization |
| `/cmd_vel` | Twist | 20-50 Hz | Nav2 / teleop |
| `/map` | OccupancyGrid | ~1 Hz | slam_toolbox |

---

## Ressources Externes Référencées

### Papers
- "AI-Powered Robotic Lawn Mower with Autonomous Navigation" (IRJET 2025)

### GitHub Repos
- [bgewehr/RPiMower](https://github.com/bgewehr/RPiMower) - Architecture MQTT sensors
- [acredsfan/autonomous_mower](https://github.com/acredsfan/autonomous_mower) - YOLOv8, Coral TPU, Nav

### Documentation
- [slam_toolbox](https://docs.ros.org/en/humble/p/slam_toolbox/)
- [Nav2](https://docs.nav2.org/)
- [ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2)

---

## Questions Ouvertes (à clarifier)

1. **Type exact des moteurs** - DC brushed confirmé (pont en H), mais specs (tension, courant) à vérifier
2. **Encodeurs** - À acheter et intégrer
3. **IMU** - À choisir (ICM-20948 recommandé)
4. **Vitesse max** - À définir pour dimensionner latence/freinage
5. **Dimensions robot** - Pour footprint Nav2

---

## Prochaines Étapes (BMAD)

1. ✅ Brainstorming (terminé)
2. 🔄 Product Brief (en cours)
3. ⏳ PRD
4. ⏳ Architecture Document
5. ⏳ Epics & Stories
6. ⏳ Implementation
