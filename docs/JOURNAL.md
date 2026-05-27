# JOURNAL — Journal de bord, jour par jour

> Document destiné à **se souvenir** de ce qui a été construit et à **savoir
> le défendre en entretien**. Pour chaque jour : ce que j'ai fait, ce que
> j'ai appris, les décisions techniques avec leur justification, les pièges
> rencontrés.

---

## Semaine 1 — Fondations, plant Python, protocole binaire et sockets

### Jours 1-2 — Comprendre l'ABS *avant* de coder

**Fait**
- Lu Lesics "How does ABS work", articles Wikipedia (slip ratio, Pacejka), survol du chapitre 8 de Rajamani.
- Rédigé `docs/PHYSICS.md` (8 sections, 290 lignes) : intuition, slip ratio, courbe μ(s), équations du mouvement, validation par ordres de grandeur.
- Rédigé `docs/ARCHITECTURE.md` : pourquoi SIL, pourquoi TCP, pourquoi HAL/SWC, pourquoi pas de RTOS.

**Appris**
- L'intuition contre-intuitive de l'ABS : "écraser plus fort = freiner moins". À cause de la courbe μ(s) qui chute après le pic.
- La formule de Pacejka simplifiée à 3 paramètres `μ = D·sin(C·atan(B·s))` — fameuse "Magic Formula".
- Le piège numérique : `slip = (v − ωR)/v` diverge à v=0. Toujours protéger.
- La hiérarchie SIL → HIL → VIL dans l'industrie.

**Décisions clés**
- **Quart-véhicule** plutôt que voiture complète : standard pour la simulation ABS, divise la masse par 4, suffisant pour le pédagogique.
- **Pacejka simplifié à 3 paramètres** plutôt que la formule complète à 10 : on garde l'essentiel (pic + chute) sans la complexité d'identification expérimentale.
- **Documenter en premier** : écrire la physique en clair avant de coder évite de débugger des fantômes mathématiques.

**À défendre en entretien**
- *"Pourquoi pas un PID ?"* → ABS réel est bang-bang/hystérésis car le pneu est très non linéaire ; un PID linéarisé autour d'un point d'opération sortirait du domaine valide en une fraction de seconde.
- *"Et le freinage régénératif des EV ?"* → Hors-scope ici (modèle Newton pur), mais la HAL serait modifiée pour additionner couple hydraulique + couple machine électrique.

---

### Jours 3-4 — Plant Python "offline" (sans socket)

**Fait**
- `plant/tire.py` — Pacejka simplifié + presets (sec / mouillé / neige / verglas).
- `plant/vehicle.py` — Quart-véhicule + intégration Euler explicite à 1 ms.
- `plant/oracle_abs.py` — Contrôleur bang-bang de référence (servira aussi de cible à reproduire en C).
- `plant/simulate.py` — Deux scénarios (no_abs / oracle), plot matplotlib, assertions d'envelope sur la distance d'arrêt.

**Appris**
- Le coefficient de forme `C` du Pacejka longitudinal vaut **≈ 1.65**, pas 1.9. Avec 1.9 le pic est trop pointu et μ(s=1) tombe à 0.34 (au lieu de ~0.65 attendu). C'est le genre d'erreur qui ne se voit qu'en simulant.
- `dt = 1 ms` est nécessaire pour la stabilité numérique d'Euler explicite sur les phases de blocage roue où ω chute en quelques ms. À `dt = 10 ms`, ω passe en négatif en un seul pas.
- À v très faible (< 1 m/s), il faut **basculer kinetic → static friction** sinon le modèle "coast forever" à v = 0.6 m/s (friction = 0 quand slip = 0, mais slip non défini à v → 0).

**Décisions clés**
- **Step du plant à 1 ms, communication ECU à 10 ms** : ratio 10×, garde une intégration physique propre tout en simulant un vrai ECU à 100 Hz.
- **Oracle Python séparé du C** : le `oracle_abs.py` est la *référence* qu'on cherchera à reproduire en C. Si le SIL avec contrôleur C donne ~la même distance, c'est que le C est correct.
- **`assert` d'envelope dans `simulate.py`** : la simulation échoue si le numéro n'est pas dans l'intervalle attendu. Régression automatique.

**Pièges rencontrés**
- Premier run : vehicle ne s'arrêtait jamais (stopping_time = 6.00 s = SIM_DURATION). Diagnostic : à v < 1 m/s, μ devenait 0, vehicle coasted. Fix : snap-to-stop avec `V_STATIC_FREEZE = 1.0 m/s`.
- Première courbe : pic Pacejka pas au bon endroit. Diagnostic : C = 1.9 lu dans Wikipedia est en réalité pour le slip *latéral*, pas longitudinal. Fix : C = 1.65.

**Validation**
- `no_abs` → 59.6 m en 4.15 s (cible ~60 m ✓)
- `oracle` → 47.5 m en 3.56 s (cible ~45 m ✓)
- Gain ABS → 20.3 % de distance gagnée

**À défendre en entretien**
- *"Tu as validé ta physique comment ?"* → Par ordre de grandeur : sans ABS la roue bloque immédiatement (slip = 1, μ tombe à 0.65), avec ABS l'oracle maintient slip ≈ 0.15 (pic μ ≈ 1.0). Différence de distance ≈ 25 %, conforme à la littérature.

---

### Jours 5-7 — Protocole binaire, CRC, sockets

**Fait**
- `docs/PROTOCOL.md` — Spec formelle (240 lignes) : envelope, framing, CRC, endianness, resync, tailles, versioning.
- `plant/protocol.py` (Python) + `controller/src/drivers/protocol.h` (C) — Définitions miroirs des structs `sensor_payload_t` (18 B) et `actuator_payload_t` (12 B).
- `controller/src/drivers/crc.[ch]` (C) + `crc16_ccitt` dans `plant/protocol.py` (Python) — CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) identique au bit près.
- `plant/protocol.py:FrameDecoder` — Décodeur avec resync sur magic, gestion CRC mismatch, oversize, chunked-feed.
- `controller/src/drivers/socket_drv.[ch]` (C) — Driver POSIX TCP_NODELAY avec `send_all` et `recv` bufferisé.
- `plant/socket_server.py` — Serveur TCP single-client côté plant.
- `controller/tests/test_crc.c` — Tests KAT C (0x29B1 sur "123456789").
- `tests/test_protocol.py` — 13 tests Python (KAT, layout, roundtrip, framing, resync, chunked, CRC mismatch, oversize length).
- `controller/tests/ping_client.c` + `scripts/milestone_w1.py` — Milestone end-to-end W1.

**Appris**
- `__attribute__((packed))` empêche le compilateur d'ajouter du padding entre les champs ; sans ça, la taille de la struct sur le wire dépendrait de l'alignement du compilateur.
- Le **little-endian** est universel sur x86 et la plupart des ARM Cortex-M — on évite des swaps en restant LE.
- CRC-16/CCITT-FALSE est le standard AUTOSAR pour la protection end-to-end sur CAN.
- `recv()` peut retourner **moins d'octets que demandé** : il faut toujours boucler (`recv_exact`).
- Sans `TCP_NODELAY`, l'algorithme de Nagle agrège les petits paquets et ajoute ~40 ms de latence — inacceptable pour du temps réel.
- `SIGPIPE` est levé quand on écrit sur un socket fermé. À ignorer (`signal(SIGPIPE, SIG_IGN)`) sinon le process meurt silencieusement.

**Décisions clés**
- **Header magique `0xAA55` + length** : permet la resynchronisation après corruption. Trouve-magic / valide-length / valide-CRC / consomme — sinon glisse d'un octet et recommence.
- **CRC sur (type + length + payload)** : magic exclus par design (s'il est corrompu, la resync gère).
- **TCP loopback** plutôt qu'UDP : on veut décider *nous-mêmes* quand simuler une perte (via fault injector en S3), pas la subir.
- **CRC implémenté à la main (bit-shift, pas table)** : c'est une question d'entretien classique, et le code fait 8 lignes — pas un drame en perf à 100 Hz.

**Validation cross-langage**
- KAT CRC : `crc16_ccitt("123456789")` = `0x29B1` côté C ET côté Python.
- KAT trame de référence (type + length + payload constant) : CRC = `0xBC19` côté C ET côté Python.
- Dump hex de la trame complète : `55aa0112007b0000000000c8410000a04200004842000019bc` — analysable champ par champ.

**Milestone W1**
- 1000 trames Python → C → Python à 1 kHz, **mean RTT = 0.225 ms** (cible < 1 ms), 0 erreur CRC, 0 byte dropped, 0 mismatch echo. Tests Python `13/13`, KAT C `5/5`.

**À défendre en entretien**
- *"Pourquoi pas JSON ?"* → JSON suppose un parser dynamique, des allocations heap, gestion d'erreur de tokenisation. Sur ECU on n'a souvent ni heap ni parser. Le binaire packed colle au modèle CAN/SPI réel.
- *"Comment tu détectes une désynchronisation ?"* → Magic `0xAA55` introuvable → scanner byte par byte jusqu'à le retrouver. Length > MAX_PAYLOAD → idem. CRC fail → idem. Compteur d'erreurs incrémenté à chaque cas pour les DTC.

---

## Semaine 2 — Architecture ECU, machine à états, bang-bang, temps réel

### Jours 8-10 — Couche HAL et machine à états

**Fait**
- `controller/src/hal/hal_sensors.{c,h}` — Lecture capteur abstraite (timeouts, statistiques internes, flag `valid`).
- `controller/src/hal/hal_actuators.{c,h}` — Écriture commande frein abstraite.
- `controller/src/hal/hal_link.c` + `hal_internal.h` — Tiny shim partagé entre les deux HAL pour qu'elles cohabitent sur un même socket en SIL (sur un vrai ECU elles auraient deux drivers différents).
- `controller/src/app/abs_state.{c,h}` — Machine à états INIT → STANDBY → MONITOR → ACTIVE + FAULT_DEGRADED / FAULT_LATCHED.
- `controller/tests/test_abs_state.c` — 11 assertions couvrant toutes les transitions documentées + fault latching + recovery.

**Appris**
- **AUTOSAR-style** : séparer la couche basse (MCAL / drivers, ici socket) de la couche métier (SWC, ici abs_controller) via une HAL/RTE intermédiaire. Sur la cible réelle on remplace MCAL → l'app code ne change pas.
- Une machine à états **doit avoir une logique de priorité claire** : ici la priorité absolue est "fault check first" — tant qu'un DTC est levé ou que les capteurs sont invalides, on ne fait pas de transition de mode normal. Évite les bugs où un fault rare fait sortir d'un état à risque.
- Le **latch** (FAULT_LATCHED) après N fautes consécutives évite d'osciller entre FAULT_DEGRADED et MONITOR si une panne intermittente se résorbe partiellement. C'est explicitement demandé par ISO 26262 sur les fonctions safety-critical.

**Décisions clés**
- **Pas de `malloc` dans la HAL** : la struct globale est statique, l'appelant fournit son contexte (`abs_state_ctx_t`). Conforme MISRA-C Rule 21.3 ("dynamic memory allocation shall not be used").
- **`hal_internal.h` non exposé aux apps** : le partage de socket entre `hal_sensors` et `hal_actuators` est une particularité SIL ; sur un vrai ECU il n'existerait pas. On le cache pour ne pas polluer l'API métier.
- **Sensors range check redondant** dans `abs_state.c` ET dans le futur `diagnostic.c` : "defense in depth". Si le diagnostic plante, la state machine refuse quand même les valeurs aberrantes.

**Tests** : 11/11 transitions vérifiées.

**À défendre en entretien**
- *"C'est quoi AUTOSAR ?"* → Architecture standard automobile : sépare le matériel (MCAL) de la logique (SWC) via la couche RTE. Permet de porter un SWC d'un MCU à un autre sans toucher au code métier. Ici je n'ai pas l'outillage Vector DaVinci mais je reproduis le pattern.
- *"Pourquoi un FAULT_LATCHED ?"* → Pour éviter une oscillation entre dégradé et nominal sur une panne intermittente. Une fois latché, seul un cycle de boot (= visite garage) peut le réinitialiser. C'est le comportement attendu sur un voyant ABS persistant au tableau de bord.

---

### Jours 11-12 — Algorithme bang-bang + tests unitaires

**Fait**
- `controller/src/app/abs_controller.{c,h}` — Slip protégé, bang-bang avec hystérésis, comportement fail-operational (passe la pédale en fault), bypass low-speed.
- `controller/tests/test_abs_controller.c` — 14 assertions sur la math du slip (free / locked / clamp / v→0), la logique bang-bang (release / re-apply / hold), et le pass-through en FAULT_*.

**Appris**
- **Fail-operational** ≠ **fail-safe** : en automotive, un ABS qui tombe en panne doit *quand même laisser le conducteur freiner* (= passer la pédale brute). Mieux vaut "freinage sans ABS" que "pas de freinage". Le code reflète ça : en FAULT_*, on retourne `driver_request` sans modulation.
- **Hystérésis** : si on ne sortait du release qu'en repassant exactement sous 0.20, le contrôleur chattererait. Avec deux seuils (UPPER = 0.20 pour release, LOWER = 0.10 pour re-apply), on a une zone morte qui filtre le bruit.
- **C twin du Python oracle** : les deux ont exactement les mêmes seuils et la même logique. C'est explicite (`ABS_SLIP_UPPER = 0.20f` côté C, `SLIP_UPPER_THRESHOLD = 0.20` côté Python). Si le C SIL donne une distance proche du Python oracle (~45 m), le port est correct.

**Décisions clés**
- **Slip math exposée séparément** (`abs_controller_slip`, `abs_controller_slip_protected`) pour pouvoir la tester sans simuler tout le contrôleur.
- **Le contexte est caller-owned** (`abs_controller_ctx_t`) : pas de variable globale dans le module, le caller alloue. Permet d'avoir plusieurs instances (utile en HIL multi-roues).

**Tests** : 14/14 (incluant les cas vicieux : slip = 1 quand ω = 0, clamp [0, 1] sur slip négatif ou aberrant, division-by-zero protégée).

**À défendre en entretien**
- *"Pourquoi bang-bang et pas PID ?"* → Voir J1-2.
- *"Comment tu testes une boucle de contrôle ?"* → On extrait la math en fonctions pures (`abs_controller_slip`), on injecte des `sensors_data_t` synthétiques, on vérifie la sortie attendue. Le contexte est explicite donc déterministe.
- *"Tu fais quoi si le capteur lâche ?"* → La HAL retourne `valid = false`. La state machine bascule en FAULT_DEGRADED. Le contrôleur passe la pédale brute. Le superviseur incrémente un compteur ; à 10 fautes consécutives → FAULT_LATCHED, voyant tableau de bord allumé jusqu'à reset.

---

### Jours 13-14 — Boucle temps réel et milestone SIL

**Fait**
- `controller/src/utils/time_utils.h` — Helpers timespec (add_ns, diff_us, now_ms_mono).
- `controller/src/main.c` — Super-loop 100 Hz avec `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`, jitter stats Welford (mean, stddev, min, max), CLI args, log toutes les 0.5 s, signaux SIGINT/SIGTERM/SIGPIPE.
- `scripts/sil_brake_test.py` — Vrai test SIL : plant Python ↔ C abs_ecu freine la voiture, assertions sur distance / état / DTC.

**Appris**
- `clock_nanosleep(TIMER_ABSTIME)` vs `usleep(10000)` : ABSTIME réveille à un instant **absolu** calculé à l'avance, donc ne drifte pas si une itération prend 2 ms ; `usleep` dort *en plus*, donc le cycle suivant arrive à 12 ms, puis 22 ms, etc.
- **Welford** pour calculer mean et stddev en streaming sans stocker tous les échantillons (numériquement stable, O(1) mémoire).
- WSL2 ajoute ~400 µs de latence base au-dessus de Linux bare metal (à cause de l'hyperviseur). La stddev reste sub-ms quand même.
- **Watchdog software** : si pas de trame depuis > 50 ms, on lève `DTC_COMM_TIMEOUT` et on bascule en FAULT_DEGRADED. Le ratio 50 ms / 10 ms = 5 → on tolère 5 trames perdues avant de paniquer.

**Décisions clés**
- **Boucle simple-thread** (super-loop) au lieu d'un RTOS multi-tâche : on connaît déjà FreeRTOS, l'intérêt pédagogique ici est ailleurs (sûreté). Un seul thread → pas de race condition à débugger.
- **Quick diagnostic dans `main.c`** pour le moment (watchdog + range check) ; le module FMEA complet vient en Semaine 3 dans `app/diagnostic.[ch]`.
- **CLI explicite** (`host port duration driver_pedal_bar`) plutôt qu'un fichier de config — pour scripter facilement les scénarios.

**Bug rencontré**
- `LONG_MAX_LIKE` utilisé dans `jitter_init` avant son `#define`. Le préprocesseur ne le remplaçait pas → erreur de compilation à `-Werror`. Fix : `#include <limits.h>` et utiliser `LONG_MAX`/`LONG_MIN`. Bonne leçon sur l'ordre macro/usage.

**Milestone W2** (vrai SIL co-simulation)
- **Distance d'arrêt** : 54.3 m sur dry asphalt (envelope [35, 55] ✓, comparable à l'oracle Python 47.5 m).
- **Temps d'arrêt** : 4.09 s.
- **États visités** : STANDBY → MONITOR → ACTIVE (transitions complètes).
- **DTC** : 0x0000 (aucune faute).
- **Jitter** : mean 407 µs, **σ = 211 µs** (cible < 1 ms ✓), max 1.48 ms sur 410 cycles.
- **ECU stats** : 410 cycles, 409 rx_ok, 1 timeout (le dernier, après que le plant ait fermé).

**À défendre en entretien**
- *"Comment tu garantis le temps réel ?"* → `clock_nanosleep(TIMER_ABSTIME)` sur `CLOCK_MONOTONIC` (immune aux changements d'heure système), période recalculée en cumulatif (pas relatif). Mesure du jitter cycle par cycle avec Welford. Σ < 1 ms validé sur 60 s.
- *"Et si une itération dépasse les 10 ms ?"* → `clock_nanosleep` se réveille immédiatement, le cycle suivant est compressé. Si ça arrive souvent on a un problème de scheduling — on inspecte les priorités process (SCHED_FIFO en prod) et les contentions CPU.
- *"Tu mesures comment la latence sur CAN réel ?"* → Sur HIL on capture les frames avec un analyseur CAN (CANalyzer, Vehicle Spy) et on calcule round-trip à partir des timestamps. Ici en SIL j'utilise la même technique : timestamp dans le payload, soustraction côté serveur.

---

## Semaine 3 — Sûreté de fonctionnement (à venir, jours 15-21)

À documenter ici au fur et à mesure :
- J15 : FMEA formel
- J16-17 : Module diagnostic (plausibility, stuck-sensor, noise)
- J18-19 : Fault injector Python + 6 scénarios d'acceptance
- J20-21 : RESULTS.md, polish, démo

---

## Pratiques transverses (à mentionner en entretien)

- **CI GitHub Actions** dès le J7 : build Python + C `-Werror`, tests unitaires, milestone end-to-end. Verte à chaque commit majeur (W1 : `0526e91`, W2 : `a519c1b`).
- **Tests à chaque module** : 13 Python + 30 C = 43 tests unitaires + 2 milestones d'intégration. Pas une démo "ça compile" mais "ça compile, ça passe les tests, et ça fait le job demandé".
- **Documentation en parallèle du code** : PHYSICS, ARCHITECTURE, PROTOCOL, FMEA, RESULTS, JOURNAL, SKILLS, INTERVIEW_QA. La maintenance future (par moi ou un coéquipier) ne dépend pas de mes souvenirs.
- **Commits descriptifs** : préfixe `feat(week-N):`, corps qui décrit *pourquoi* + *quoi*, pas juste un changelog mécanique.
- **Cross-platform conscient** : Windows host + WSL Ubuntu target, code Python portable, code C POSIX (mais HAL prête à recevoir un MCAL Windows / STM32 / Aurix).
