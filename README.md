# ABS Digital Twin — SIL Co-Simulation Platform

[![CI](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/workflows/ci.yml)

> Software-in-the-Loop platform simulating an automotive ABS ECU (C) coupled to a vehicle dynamics model (Python) via a custom binary protocol over TCP at 100 Hz.

**Status:** 🚧 Work in progress — **Semaine 1 terminée** (jour 7 / 21). Plant Python validé, protocole binaire spec+impl bilingue (C/Py), milestone réseau passé (1000 trames, 0 erreur, latence p99 < 0.5 ms en loopback WSL).

---

## 🎯 Objectif

Construire une plateforme de co-simulation entre :

- **Plant model** (Python) — modèle dynamique de véhicule (Newton + Pacejka) avec capteurs virtuels et injection de pannes.
- **ECU simulé** (C) — algorithme ABS bang-bang avec architecture HAL/Drivers/Application, machine à états, module diagnostic FMEA.

Les deux processus communiquent par socket TCP avec un protocole binaire encadré CRC16, à 100 Hz, avec watchdog et plausibility checks.

## 🏗️ Architecture

```
┌──────────────────────────────────┐                  ┌──────────────────────────────────┐
│   PLANT MODEL (Python)           │   TCP loopback   │   ECU SIMULÉ (C)                 │
│                                  │  ─── sensor ──►  │                                  │
│  • Dynamique véhicule            │                  │  • HAL virtuelle                 │
│  • Pacejka tire model            │                  │  • State machine ABS             │
│  • Capteurs virtuels + bruit     │  ◄── actuator ── │  • Diagnostic (FMEA)             │
│  • Fault Injector CLI            │                  │  • Watchdog                      │
└──────────────────────────────────┘                  └──────────────────────────────────┘
```

Détails : [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)

## 📚 Documentation

| Fichier | Contenu | Statut |
|---|---|---|
| [`docs/PHYSICS.md`](docs/PHYSICS.md) | Équations véhicule, slip ratio, Pacejka | ✅ |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Choix de design, structure logicielle | ✅ |
| [`docs/PROTOCOL.md`](docs/PROTOCOL.md) | Spec trame binaire, framing, CRC | ✅ |
| `docs/FMEA.md` | Analyse modes de défaillance | ⏳ jour 15 |
| `docs/RESULTS.md` | Benchmarks, courbes, distance d'arrêt | ⏳ jour 20-21 |

## 🚀 Lancement (état actuel — fin de semaine 1)

**Validation du plant offline** (sans socket) :
```bash
python3 -m plant.simulate                  # 2 scénarios + plot dans results/
python3 -m plant.simulate --no-plot        # juste les chiffres
```
Sortie attendue : `no_abs ≈ 60 m, oracle ≈ 47 m, gain ≈ 20 %, [OK] within envelope`.

**Tests unitaires protocole** (CRC, framing, layout) :
```bash
python3 -m unittest tests.test_protocol -v        # 13/13 OK
```

**Build du contrôleur C** (cible Linux/WSL — POSIX sockets) :
```bash
cd controller && make all
./build/test_crc                                  # KAT CRC-CCITT-FALSE
```

**Milestone semaine 1** (1000 trames, vérif round-trip & CRC) — deux terminaux :
```bash
# T1 — serveur Python
python3 -m scripts.milestone_w1 --frames 1000 --rate 1000

# T2 — client C (échange brake_command = v_vehicle en écho)
./controller/build/ping_client 127.0.0.1 9000 1000
```
Attendu : `frames 1000/1000, 0 echo mismatches, mean latency < 1 ms, 0 CRC errors`.

## 🧪 Scénarios de test prévus

| Scénario | Attendu |
|---|---|
| Freinage nominal sec | distance ≈ 45 m, slip ∈ [0.1, 0.2] |
| Plaque de verglas | ABS module, voiture contrôlable |
| Capteur roue figé | détection <100 ms, mode dégradé |
| Perte communication 200 ms | watchdog déclenche, brake = last_safe |
| CRC corrompu (1% trames) | trames ignorées, système stable |
| Bruit capteur +/- 5 rad/s | filtre absorbe, pas d'oscillation |

## 🎓 Compétences mises en œuvre

- **Embedded C** : architecture en couches (HAL/Drivers/Application), allocation statique, MISRA-aware
- **Sûreté de fonctionnement** : FMEA, plausibility checks, watchdog, DTC, fail-safe
- **IPC / Communication** : sockets TCP, protocole binaire encadré, CRC16, framing, gestion désynchronisation
- **Temps réel** : `clock_nanosleep(TIMER_ABSTIME)`, mesure de jitter, période 10 ms stable
- **Tests** : tests unitaires C, scénarios d'injection de pannes scriptés
- **Modélisation physique** : équations Newton, modèle de pneu Pacejka, intégration Euler

## 📅 Planning

Projet sur 3 semaines en `full focus` (mai-juin 2026).

- **Semaine 1** ✅ Fondations : physique, plant Python, protocole et sockets.
- **Semaine 2** ⏳ ECU : architecture modulaire C, machine à états, algo bang-bang, boucle temps réel.
- **Semaine 3** ⏳ Sûreté : FMEA, module diagnostic, fault injector, scénarios de validation, polish.

---

*Projet personnel pour montée en compétences en logiciel embarqué automobile.*
