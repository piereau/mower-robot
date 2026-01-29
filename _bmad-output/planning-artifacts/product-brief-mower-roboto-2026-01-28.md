---
stepsCompleted: [1, 2, 3, 4, 5, 6]
status: complete
inputDocuments:
  - _bmad-output/brainstorming/brainstorming-session-2026-01-28.md
  - /Users/pierrot/Downloads/AI_Powered_Robotic_Lawn_Mower_with_Auton.pdf
date: 2026-01-28
author: Pierrot
---

# Product Brief: mower-roboto

## Executive Summary

**mower-roboto** est un robot tondeuse autonome conçu pour entretenir les inter-rangs de vigne. Il combine navigation autonome (SLAM + RTK-GPS), détection d'obstacles et pilotage précis sur chenilles pour opérer dans un environnement agricole exigeant.

Le projet démarre comme prototype académique avec l'ambition d'évoluer vers une solution commerciale.

---

## Core Vision

### Problem Statement

L'entretien des inter-rangs de vigne (désherbage mécanique, tonte) est une tâche répétitive, chronophage et pénible. Les solutions actuelles sont soit manuelles (coûteuses en main d'œuvre), soit basées sur des herbicides (impact environnemental), soit des robots commerciaux très coûteux et peu adaptés au terrain viticole.

### Problem Impact

- Coût main d'œuvre élevé pour les viticulteurs
- Pénibilité du travail (chaleur, terrain accidenté)
- Pression réglementaire pour réduire les herbicides
- Manque de solutions autonomes abordables adaptées à la vigne

### Why Existing Solutions Fall Short

- **Robots commerciaux** (Husqvarna, etc.): conçus pour pelouses plates, pas pour terrain agricole accidenté
- **Tracteurs inter-rangs**: nécessitent un opérateur, coût élevé
- **Solutions DIY**: manquent de robustesse et d'autonomie réelle

### Proposed Solution

Un robot à chenilles autonome capable de:
- Naviguer entre les rangs de vigne sans intervention humaine
- Cartographier le domaine (SLAM + RTK-GPS)
- Éviter les obstacles (piquets, fils, végétation)
- Effectuer des demi-tours en bout de rang
- Être supervisé à distance via interface web/mobile

### Key Differentiators

- **Chenilles**: meilleure traction sur terrain meuble/pentu
- **Architecture hybride ROS 2 + Arduino**: robustesse industrielle + sécurité temps réel
- **Double localisation (SLAM + RTK)**: fiabilité même sous feuillage
- **Coût maîtrisé**: composants accessibles (RPi, Arduino, LiDAR LD19)
- **Évolutivité**: architecture modulaire prête pour l'industrialisation

---

## Target Users

### Primary Users

**Viticulteur indépendant**
- Exploite 5-30 ha de vigne
- Manque de main d'œuvre saisonnière
- Cherche à automatiser les tâches répétitives
- Sensible au coût et à la simplicité d'utilisation

**Chef de culture (grande exploitation)**
- Gère 50+ ha, supervise des équipes
- Objectifs d'optimisation des coûts et de la qualité
- Besoin de supervision multi-robots à terme

### Secondary Users

**Prestataire de services viticoles**
- Propose des services d'entretien aux exploitations
- Cherche des outils différenciants pour son offre
- Volume d'utilisation plus élevé

### User Journey

1. **Discovery**: Salon agricole, bouche-à-oreille, démo terrain
2. **Onboarding**: Cartographie initiale du domaine, configuration zones
3. **Core Usage**: Lancement quotidien/hebdo, supervision via app
4. **Success Moment**: Premier passage autonome complet sans intervention
5. **Long-term**: Intégration dans le planning d'exploitation
