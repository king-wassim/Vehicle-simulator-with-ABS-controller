# SKILLS — Mapping livrables ↔ compétences (offres KPIT / Vitesco / Bosch)

> Tableau de correspondance entre les mots-clés qu'on lit dans les offres
> automotive embedded et les fichiers du projet qui les *prouvent*. À garder
> sous la main pour rédiger CV / LinkedIn et pour pointer un recruteur vers
> un fichier précis pendant l'entretien.

---

## 1. C embarqué (mot-clé : "Embedded C", "Bare-metal C", "ECU SW")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Standard C11, strict warnings | `controller/Makefile`, `.github/workflows/ci.yml` | `-std=c11 -Wall -Wextra -Wpedantic -Werror`. CI échoue sur le moindre warning. |
| Pas de `malloc`, allocation statique | `controller/src/hal/*.c`, `controller/src/app/*.c` | Toutes les structs sont owned par le caller ou statiques globales. Conforme MISRA-C Rule 21.3. |
| Header guards + `static` pour encapsulation | tous les `.c` du contrôleur | Variables internes module en `static`, headers avec `#ifndef ABS_X_H`. |
| Structs `__attribute__((packed))` pour wire format | `controller/src/drivers/protocol.h` | Évite le padding compilateur, taille exacte sur le fil. Vérifiée par `_Static_assert`. |
| Bit-shifting, masques, bitfields | `controller/src/drivers/crc.c`, `protocol.h` (DTC), `socket_drv.c` (length parsing) | CRC-16/CCITT-FALSE écrit à la main, DTC en bitfield, length encodé little-endian byte à byte. |
| Gestion d'erreurs propre (retour codé, errno) | `socket_drv.c`, `hal_*.c` | Pattern `if (rc < 0) { perror; return -1; }`, `errno` préservé entre wrappers. |

---

## 2. Architecture logicielle (mot-clé : "AUTOSAR", "Layered SW Architecture", "MCAL/RTE/SWC")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Séparation MCAL / HAL / SWC | `controller/src/{drivers,hal,app}` | App code (`abs_controller`, `abs_state`) ne `#include` jamais `socket.h` — il passe par `hal_sensors_read()`. Une réécriture STM32 ne touche que `drivers/` et `hal/`. |
| Interface stable, implémentation interchangeable | `hal_sensors.h` vs `hal_sensors.c` | L'API métier (sensors_data_t, hal_sensors_read) survit à un changement de transport. |
| Pas de fuite d'abstraction | `hal_internal.h` | Le partage de socket entre les deux HAL est marqué *internal* — non exposé aux apps. |
| Single-source-of-truth pour les contrats wire | `protocol.h` ↔ `plant/protocol.py` | Tailles `_Static_assert`'ées des deux côtés (18 B sensor, 12 B actuator). |

---

## 3. Communication & protocoles (mot-clé : "CAN", "Bus Protocol", "Diagnostic over UDS")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Conception d'un protocole binaire framé | `docs/PROTOCOL.md` | Spec de 8 sections : envelope, framing, endianness, resync, sizes, versioning. |
| Endianness explicite (little-endian) | `protocol.py`, `socket_drv.c`, `protocol.h` | Format `<I fff H` côté Python, lecture byte-à-byte côté C. |
| CRC-16/CCITT-FALSE (AUTOSAR E2E) | `crc.c` (C), `crc16_ccitt` (Python), `test_crc.c`, `tests/test_protocol.py` | Implémentation bit-shift identique des deux côtés. KAT `0x29B1` sur `"123456789"`. |
| Resynchronisation sur perte d'octets | `socket_drv.c::try_extract_frame`, `protocol.py::FrameDecoder` | Magic `0xAA55` recherché byte par byte, length validée, CRC validé — sinon glissement et retry. |
| `TCP_NODELAY` pour latence sub-ms | `socket_drv.c::set_tcp_nodelay` | Désactive l'algo de Nagle. Mesure : RTT moyen 0.225 ms sur loopback WSL. |
| Versioning du protocole | `docs/PROTOCOL.md` §8 | Convention : nouveau magic en cas de breaking change, garantit qu'un récepteur v1 n'interprète pas une trame v2. |

---

## 4. Sûreté de fonctionnement (mot-clé : "Functional Safety", "ISO 26262", "FuSa", "ASIL")

> *Caveat à dire en entretien* : "Je n'ai pas certifié ASIL — c'est un
> projet personnel. Mais j'ai appliqué les patterns qu'on retrouve dans
> les codes safety-critical : FMEA, plausibility, watchdog, fail-operational."

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Analyse FMEA (à venir Semaine 3) | `docs/FMEA.md` (J15) | Tableau formel : ≥ 8 modes de défaillance, sévérité, détection, mitigation. |
| Plausibility checks | `abs_state.c::sensors_usable`, `main.c::diagnostic_quick` | Range check redondant (`v ∈ [-1, 120] m/s`, `ω ∈ [-1, 500] rad/s`) à plusieurs étages. |
| Watchdog software | `main.c` (boucle), `protocol.h::DTC_COMM_TIMEOUT` | Si pas de trame en > 50 ms, DTC levé et bascule en FAULT_DEGRADED. |
| DTC bitfield | `protocol.h::DTC_*` | 6 codes, alignés avec les modes FMEA, transmis dans chaque actuator frame. |
| Fail-operational | `abs_controller.c::abs_controller_compute` (branche FAULT_*) | En faute, le contrôleur passe la pédale brute — le conducteur peut *quand même* freiner. |
| Fault latching avec hystérésis temporelle | `abs_state.c::abs_state_update` | `ABS_FAULT_LATCH_THRESHOLD = 10` cycles consécutifs avec DTC → FAULT_LATCHED sticky. |
| State machine défensive | `abs_state.c` | "Fault check first" : tant que DTC ou capteurs invalides, pas de transition normale. |

---

## 5. Temps réel (mot-clé : "Real-Time", "Scheduling", "Hard RT")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Boucle périodique sans drift | `main.c` (super-loop), `time_utils.h` | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` avec timestamp absolu cumulé. |
| Mesure de jitter cycle-par-cycle | `main.c::jitter_*` | Welford streaming (mean, σ, min, max) — O(1) mémoire, numériquement stable. |
| Validation sub-ms en environnement non-RT | Milestone W2 dans `JOURNAL.md` | σ = 211 µs sur 410 cycles sous WSL2 (avec hyperviseur). Cible σ < 1 ms ✓. |
| Compréhension `usleep` vs `nanosleep_abstime` | commentaires `main.c` + `JOURNAL.md` J13-14 | Sais expliquer pourquoi `usleep` drifte et `TIMER_ABSTIME` non. |

---

## 6. Modélisation physique (mot-clé : "Vehicle Dynamics", "Plant Model", "Simulink-like")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Équations Newton + intégration Euler | `plant/vehicle.py`, `docs/PHYSICS.md` §4 | `dv/dt = −μg`, `Iω̇ = μmgR − T_b`, Euler explicite à `dt = 1 ms`. |
| Modèle de pneu Pacejka simplifié | `plant/tire.py`, `docs/PHYSICS.md` §3 | `μ = D·sin(C·atan(B·s))`, presets sec/mouillé/neige/verglas. |
| Slip ratio + protection numérique | `plant/tire.py::compute_slip`, `abs_controller.c::abs_controller_slip_protected` | Guard à `v < v_min` (sinon division par zéro). |
| Validation par ordres de grandeur | `plant/simulate.py` (assertions d'envelope) | "Sans ABS ~60 m, avec ABS ~45 m" : c'est dans la littérature, mon modèle reproduit. |

---

## 7. Tests & qualité (mot-clé : "Unit Testing", "Test Coverage", "CI/CD")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| Tests unitaires C | `controller/tests/test_*.c` (30 assertions) | KAT CRC, transitions state machine, math slip, bang-bang, hystérésis, fail-operational. |
| Tests unitaires Python | `tests/test_protocol.py` (13 tests) | KAT CRC, layout structs, roundtrip pack/unpack, framing, resync, chunked feed, CRC mismatch, oversize. |
| Cross-language consistency | `test_crc.c` vs `tests/test_protocol.py` | Mêmes vecteurs CRC, même trame de référence (`0xBC19`). |
| Integration tests end-to-end | `scripts/milestone_w1.py`, `scripts/sil_brake_test.py` | "1000 frames @ 1 kHz + 0 erreur", "voiture s'arrête en 35-55 m sous ABS C". |
| CI verte à chaque push | `.github/workflows/ci.yml`, Actions URL | Build C strict + unit tests + envelope check + milestones d'intégration. |
| Régression automatique | assertions d'envelope dans `simulate.py` et `sil_brake_test.py` | Si la physique drifte ou le contrôleur régresse, la CI casse. |

---

## 8. Outillage & workflow (mot-clé : "Git", "Make", "Linux", "Bash")

| Compétence revendiquée | Fichier / preuve | Détail technique |
|---|---|---|
| `make` clean (targets, dependencies, `.PHONY`) | `controller/Makefile` | Targets séparés : `abs_ecu`, `test_*`, `ping_client`, `test`, `tests`, `clean`. |
| Linux user (WSL Ubuntu) | toute la stack C | POSIX sockets (`sys/socket.h`), `clock_nanosleep`, `clock_gettime(CLOCK_MONOTONIC)`. |
| Git commits descriptifs, messages structurés | `git log` | Préfixe `feat(week-N):`, body décrit pourquoi + quoi, co-author. |
| Cross-platform conscient | Python tourne sur Windows + WSL, C sur WSL | Choix documenté dans memory + ARCHITECTURE.md. |
| Documentation parallèle au code | `docs/` (7 fichiers à terme) | PHYSICS, ARCHITECTURE, PROTOCOL, FMEA, RESULTS, JOURNAL, SKILLS, INTERVIEW_QA. |

---

## 9. Soft skills implicites

| Signal | Preuve dans le repo |
|---|---|
| Capacité à **prioriser** | Plan en 3 semaines avec jalons explicites, deliverables tracking. Pas de feature creep. |
| Capacité à **expliquer** | `JOURNAL.md`, `PHYSICS.md` rédigés "comme un cours" : si je peux l'expliquer, j'ai compris. |
| Capacité à **se corriger** | `JOURNAL.md` documente les bugs (C=1.9→1.65, V_STATIC_FREEZE, LONG_MAX) et leur fix. Pas de honte du process. |
| **Honnêteté technique** | Section "Limitations" du README, caveats explicites ("je n'ai pas certifié ASIL"). Pas de bullshit. |
| **Réflexe industrie** | Wording AUTOSAR / MISRA / E2E / FuSa partout où c'est légitime, pas plaqué. |

---

## 10. Pitch CV — 4 lignes prêtes à coller

> **ABS Digital Twin (projet personnel, 2026)** — Plateforme SIL de
> co-simulation entre un ECU C bare-metal (HAL, machine à états, bang-bang,
> super-loop 100 Hz avec `clock_nanosleep`) et un modèle de véhicule Python
> (Pacejka, Newton/Euler), communicant via un protocole binaire CRC-16
> custom over TCP. 30 tests unitaires C + 13 Python, CI GitHub Actions verte,
> milestone SIL end-to-end : voiture freinée de 100 → 0 km/h en 54 m sous
> contrôle C, jitter σ < 1 ms.
