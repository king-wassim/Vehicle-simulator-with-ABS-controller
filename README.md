# ABS Digital Twin — SIL Co-Simulation Platform

[![CI](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/workflows/ci.yml)

> Software-in-the-Loop platform simulating an automotive ABS ECU (C) coupled to a vehicle dynamics model (Python) via a custom binary protocol over TCP at 100 Hz.

**Status:** ✅ **Sprint 3-semaines complet (jour 21/21)**. Plant Python validé, protocole binaire spec+impl bilingue (C/Py), ECU C complet (HAL + state machine + bang-bang + super-loop 100 Hz), module diagnostic FMEA, fault injector Python, **6 scénarios d'acceptance verts**, RESULTS.md avec benchmarks. CI GitHub Actions verte. Voiture freine de 100→0 km/h en ~45 m sous contrôle C, jitter σ < 100 µs, détection de panne capteur en < 110 ms.

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
| [`docs/JOURNAL.md`](docs/JOURNAL.md) | Journal jour-par-jour : fait / appris / décisions | ✅ |
| [`docs/SKILLS.md`](docs/SKILLS.md) | Matrice compétences ↔ livrables (KPIT/Vitesco) | ✅ |
| [`docs/INTERVIEW_QA.md`](docs/INTERVIEW_QA.md) | 30 questions d'entretien anticipées + réponses | ✅ |
| [`CHANGELOG.md`](CHANGELOG.md) | Releases par semaine, format Keep-a-Changelog | ✅ |
| [`docs/FMEA.md`](docs/FMEA.md) | Analyse modes de défaillance (10 modes, cartographie DTC, pyramide defense-in-depth) | ✅ |
| [`docs/RESULTS.md`](docs/RESULTS.md) | Benchmarks complets : Python oracle vs C SIL, 6 scénarios, plots | ✅ |

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

**Tests unitaires C** (30 assertions : CRC + state machine + bang-bang) :
```bash
make -C controller test
```

**Milestone semaine 2 — SIL end-to-end** (vraie co-simulation : voiture freine sous contrôle C) :
```bash
# T1 — plant Python (sim physique + serveur protocole)
python3 -m scripts.sil_brake_test --surface dry_asphalt --duration 6

# T2 — ECU C (state machine + bang-bang + super-loop 100 Hz)
./controller/build/abs_ecu 127.0.0.1 9000 6 100
```
Attendu : `stopping distance ∈ [35, 55] m, ECU_ACTIVE atteint, 0 DTC, jitter σ < 1 ms`.

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
- **Sûreté de fonctionnement** : FMEA, plausibility checks, watchdog, DTC, fail-operational
- **IPC / Communication** : sockets TCP, protocole binaire encadré, CRC16, framing, gestion désynchronisation
- **Temps réel** : `clock_nanosleep(TIMER_ABSTIME)`, mesure de jitter (σ < 1 ms), période 10 ms stable
- **Tests** : 30 tests unitaires C + 13 Python + 2 milestones d'intégration, CI verte à chaque commit
- **Modélisation physique** : équations Newton, modèle de pneu Pacejka, intégration Euler

→ Mapping détaillé compétences ↔ fichiers : [`docs/SKILLS.md`](docs/SKILLS.md).

## 🎯 Ce que ce projet prouve

| Question recruteur | Preuve dans le repo |
|---|---|
| "Tu sais structurer du C embarqué ?" | Séparation `drivers/` / `hal/` / `app/`, 30 tests unitaires, build `-Werror -Wpedantic` propre |
| "Tu connais AUTOSAR ?" | Pattern MCAL/HAL/SWC reproduit ; `hal_sensors.h` réutilisable sur STM32 sans toucher au métier |
| "Tu sais concevoir un protocole binaire ?" | `docs/PROTOCOL.md` + impl bilingue C/Python + CRC validé byte-pour-byte |
| "Tu maîtrises le temps réel ?" | Super-loop `clock_nanosleep(TIMER_ABSTIME)` 100 Hz, jitter σ = 211 µs mesuré |
| "Tu penses sûreté ?" | State machine fault-aware, latch après N défauts, fail-operational, FMEA formel (S3) |
| "Tu testes ce que tu écris ?" | 43 tests unitaires + 2 milestones end-to-end + CI GitHub Actions verte |

## ⚠️ Limitations assumées (honnêteté technique)

- **Pas certifié ISO 26262 / ASIL** — c'est un projet pédagogique, pas un livrable production. Patterns appliqués, pas process complet.
- **Modèle quart-véhicule** uniquement — pas de répartition longitudinale/latérale, pas de transfert de charge, pas de 4-roues indépendantes.
- **Pneu Pacejka simplifié à 3 paramètres** — pas de dépendance en charge / angle de carrossage / température. Suffisant pour valider l'algo ABS.
- **Pas de freinage régénératif** (utile EV) — la HAL est prête à recevoir une seconde source de couple, ce n'est pas implémenté.
- **Float au lieu de fixed-point** — choix de simplicité. En vrai safety-critical on durcirait en `q15` ou équivalent pour reproductibilité bit-à-bit.
- **TCP loopback au lieu de CAN** — choix SIL. Le passage CAN-FD ne change que la HAL (par design).

## 📅 Planning

Projet sur 3 semaines en `full focus` (mai-juin 2026).

- **Semaine 1** ✅ Fondations : physique, plant Python, protocole et sockets.
- **Semaine 2** ✅ ECU : architecture modulaire C, machine à états, algo bang-bang, boucle temps réel.
- **Semaine 3** ✅ Sûreté : FMEA, module diagnostic, fault injector, **6 scénarios d'acceptance verts**, RESULTS.md.

## 📦 Releases

| Tag | Date | Périmètre | CI |
|---|---|---|---|
| `v0.1-week1` | 2026-05-26 | Plant Python, protocole binaire, sockets, milestone 1000 trames | ✅ [run #26496884457](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26496884457) |
| `v0.2-week2` | 2026-05-27 | ECU C complet (HAL, FSM, bang-bang, super-loop), milestone SIL freinage | ✅ [run #26503946600](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26503946600) |
| `v0.3-week3` | 2026-05-27 | FMEA, diagnostic (44 tests C), fault injector, 6 scénarios, RESULTS.md | ✅ [run #26521054772](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26521054772) |

Détail complet : [`CHANGELOG.md`](CHANGELOG.md).

---

*Projet personnel pour montée en compétences en logiciel embarqué automobile.*
