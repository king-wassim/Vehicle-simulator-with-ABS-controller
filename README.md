# ABS Digital Twin — SIL Co-Simulation Platform

> Software-in-the-Loop platform simulating an automotive ABS ECU (C) coupled to a vehicle dynamics model (Python) via a custom binary protocol over TCP at 100 Hz.

**Status:** 🚧 Work in progress — phase initiale (jour 1-2 / 21).

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
| `docs/PROTOCOL.md` | Spec trame binaire, framing, CRC | ⏳ jour 5-7 |
| `docs/FMEA.md` | Analyse modes de défaillance | ⏳ jour 15 |
| `docs/RESULTS.md` | Benchmarks, courbes, distance d'arrêt | ⏳ jour 20-21 |

## 🚀 Lancement (à venir)

```bash
# Terminal 1 — ECU (C)
cd controller && make && ./build/abs_ecu

# Terminal 2 — Plant (Python)
cd plant && python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 simulate.py --scenario nominal
```

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

- **Semaine 1** — Fondations : physique, plant Python, protocole et sockets.
- **Semaine 2** — ECU : architecture modulaire C, machine à états, algo bang-bang, boucle temps réel.
- **Semaine 3** — Sûreté : FMEA, module diagnostic, fault injector, scénarios de validation, polish.

---

*Projet personnel pour montée en compétences en logiciel embarqué automobile.*
