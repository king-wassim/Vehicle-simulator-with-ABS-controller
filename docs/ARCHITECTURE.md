# ARCHITECTURE.md — Conception du système

> **Objectif.** Documenter les choix de design *avant* de coder. Pourquoi deux processus ? Pourquoi TCP ? Pourquoi cette structure de dossiers ? Le code peut changer, les raisons ne doivent pas s'oublier.

---

## 1. Vue d'ensemble — Software-in-the-Loop (SIL)

### 1.1 Le concept SIL

Dans l'industrie automobile, on développe un ECU (Electronic Control Unit) en plusieurs étapes :

| Étape | Quoi | Hardware |
|---|---|---|
| **MIL** (Model-in-the-Loop) | Algo en Simulink, simulé entièrement | Aucun |
| **SIL** (Software-in-the-Loop) | Algo compilé en C, simulé contre un modèle plant | Aucun ⭐ **notre cas** |
| **HIL** (Hardware-in-the-Loop) | Vrai ECU branché à un simulateur temps réel | ECU physique + bench |
| **VIL** (Vehicle-in-the-Loop) | ECU dans une vraie voiture, sur banc | Voiture complète |

On fait du **SIL** : le code C qui tournera *un jour* sur une vraie carte STM32 / Infineon Aurix est aujourd'hui compilé pour Linux et discute avec un plant Python qui joue le rôle de la voiture.

### 1.2 Architecture deux-processus

```
┌──────────────────────────────────┐                  ┌──────────────────────────────────┐
│   PLANT MODEL (Python)           │                  │   ECU SIMULÉ (C)                 │
│   "Le monde physique"            │                  │   "Le logiciel embarqué"         │
│   Process A                      │                  │   Process B                      │
│                                  │                  │                                  │
│  • Dynamique véhicule (Newton)   │  TCP loopback    │  • HAL virtuelle                 │
│  • Modèle de pneu (Pacejka)      │  ─── sensor ──►  │  • State machine ABS             │
│  • Capteurs virtuels + bruit     │                  │  • Diagnostic (FMEA)             │
│  • Fault Injector (CLI)          │  ◄── actuator ── │  • Watchdog                      │
│  • Logger CSV + plot             │                  │  • Logger                        │
└──────────────────────────────────┘                  └──────────────────────────────────┘
```

**Pourquoi deux processus séparés et pas un seul script ?**

- **Isolation** : si l'ECU plante (segfault, deadlock), le plant continue à tourner et on peut analyser.
- **Réalisme** : un vrai ECU ne partage pas la mémoire de la voiture. Il reçoit des trames par bus CAN. On reproduit cette contrainte.
- **Langages séparés** : Python est rapide à écrire pour la physique et le tracé (matplotlib). C est obligatoire pour l'ECU si on veut un CV crédible.
- **Démontre la maîtrise IPC** : c'est explicitement un de tes 3 axes prioritaires.

---

## 2. Choix de la couche transport — TCP vs UDP vs autre

### 2.1 Les candidats

| Transport | Pour | Contre |
|---|---|---|
| **TCP** loopback (127.0.0.1) | Fiabilité (pas de pertes), framing facile | Latence variable, Nagle, blocking |
| **UDP** loopback | Latence basse, plus proche du CAN | Pertes possibles → faut le gérer (mais c'est exactement le scénario `COMM_LOSS` du FMEA) |
| **Unix domain socket** | Le plus rapide | Linux only — on veut tester le protocole binaire |
| **Pipe nommé** | Simple | Pas vraiment réseau |
| **Shared memory** | Le plus rapide | Pas du tout réaliste pour simuler CAN |

### 2.2 Décision : TCP loopback avec `TCP_NODELAY`

**Raisons :**
1. **Fiabilité contrôlée** : on veut décider *nous-mêmes* quand simuler une perte (via le fault injector), pas la subir.
2. **Framing facile** : header magique + length + CRC, comme un vrai protocole bus série.
3. **Disable Nagle** (`TCP_NODELAY=1`) supprime la latence d'agrégation → on est <1ms en loopback.
4. **Portabilité** : tourne sur n'importe quel OS.

On pourra ajouter une variante UDP en bonus si le temps le permet (semaine 3).

---

## 3. Le protocole applicatif

Spécifié en détail dans [PROTOCOL.md](./PROTOCOL.md) (jour 5-7). Résumé :

```c
// Trame Plant → ECU (capteurs)
typedef struct __attribute__((packed)) {
    uint16_t magic;          // 0xAA55
    uint16_t length;         // taille payload
    uint32_t timestamp_ms;
    float    v_vehicle;      // m/s
    float    omega_wheel;    // rad/s
    float    brake_pressure; // bar (feedback boucle hydraulique)
    uint16_t fault_flags;    // bitfield injection de pannes côté plant
    uint16_t crc16;
} sensor_frame_t;            // 24 bytes

// Trame ECU → Plant (actionneurs)
typedef struct __attribute__((packed)) {
    uint16_t magic;          // 0xAA55
    uint16_t length;
    uint32_t timestamp_ms;
    float    brake_command;  // bar — commande pression
    uint16_t ecu_status;     // état machine ABS
    uint16_t dtc_code;       // Diagnostic Trouble Code (bitfield)
    uint16_t crc16;
} actuator_frame_t;          // 20 bytes
```

**Propriétés** :
- `__attribute__((packed))` : pas de padding aligné, taille exacte sur le fil.
- **Little-endian** : convention de l'architecture x86 et la plupart des ARM Cortex-M.
- **CRC16-CCITT** : standard automobile, polynôme `0x1021`.
- **Header magique** : permet la resynchronisation si on perd des octets.

---

## 4. Architecture interne du contrôleur C

### 4.1 Structure de dossiers

```
controller/
├── src/
│   ├── main.c                  # super-loop temps réel
│   ├── hal/                    # Hardware Abstraction Layer
│   │   ├── hal_sensors.{c,h}   # API: hal_sensors_read() — cache l'origine (socket ici, ADC sur cible)
│   │   └── hal_actuators.{c,h} # API: hal_actuators_write()
│   ├── app/                    # Logique métier
│   │   ├── abs_controller.{c,h}  # algo bang-bang
│   │   ├── abs_state.{c,h}       # state machine
│   │   └── diagnostic.{c,h}      # plausibility checks + DTC
│   ├── drivers/                # "drivers" simulant le périphérique CAN
│   │   ├── socket_drv.{c,h}    # encapsule send/recv TCP
│   │   └── crc.{c,h}           # CRC16-CCITT
│   └── utils/
│       ├── logger.{c,h}        # log csv vers stdout / fichier
│       ├── ring_buffer.{c,h}   # buffer circulaire (réutilisable, déjà connu via FreeRTOS)
│       └── time_utils.{c,h}    # clock_nanosleep helpers
├── tests/                      # tests unitaires (Unity ou asserts)
├── CMakeLists.txt
└── Makefile                    # alternative simple
```

### 4.2 Le pattern HAL — pourquoi c'est crucial

```
┌─────────────────────────────────────┐
│  app/abs_controller.c               │
│  (logique métier — agnostique HW)   │
└───────────────┬─────────────────────┘
                │ appelle hal_sensors_read()
                ▼
┌─────────────────────────────────────┐
│  hal/hal_sensors.h  (interface)     │
└───────────────┬─────────────────────┘
                │ implémenté par
        ┌───────┴────────┐
        ▼                ▼
  hal_sensors_         hal_sensors_
   socket.c             stm32.c    (futur)
   (PC, SIL)           (vraie cible)
```

L'algo ABS ne sait pas si les données viennent d'un socket ou d'un ADC : il appelle juste `hal_sensors_read()`. Ça permet :
- De **porter** vers une vraie carte en réécrivant uniquement la HAL.
- De **tester unitairement** l'algo en mockant `hal_sensors_read()`.
- De respecter le principe AUTOSAR de séparation des couches (RTE / BSW / SWC).

### 4.3 Choix : super-loop, pas RTOS

Pour ce projet on fait une **super-loop temps réel** (pas FreeRTOS embarqué) :

```c
while (running) {
    wait_until_next_period_10ms();
    sensors = hal_sensors_read();
    dtc     = diagnostic_check(&sensors, &prev);
    state   = abs_state_update(state, &sensors, dtc);
    actuator = abs_compute(state, &sensors);
    hal_actuators_write(&actuator);
    logger_step(&sensors, &actuator, state, dtc);
}
```

Pourquoi pas un RTOS ?
- **Pédagogie** : tu as déjà fait du FreeRTOS, on ne réapprend pas. On creuse plutôt la sûreté.
- **Déterminisme** : un seul thread = pas de race condition à débugger.
- **Suffisant** : à 100 Hz pour 1 boucle de contrôle, on est très loin de saturer un CPU moderne.

Une variante 2-threads (un thread RX socket, un thread contrôle) serait plus réaliste — on la fera **en bonus** si on a du temps en fin de semaine 2.

---

## 5. Timing et temps réel

### 5.1 Période de boucle : 10 ms (100 Hz)

C'est l'ordre de grandeur d'un vrai ECU ABS automobile (les Bosch tournent typiquement à 50–200 Hz selon les modèles). Justifications :
- Temps caractéristique d'une roue qui bloque ≈ 50 ms → on échantillonne 5× plus vite (Nyquist OK).
- Pression hydraulique modulable en ~5 ms → on peut commander une transition par cycle.
- Largement faisable même sur un ARM Cortex-M3 à 72 MHz.

### 5.2 Mécanisme retenu : `clock_nanosleep(TIMER_ABSTIME)`

```c
struct timespec next = now();
while (running) {
    next.tv_nsec += 10 * 1000 * 1000;   // +10ms
    normalize(&next);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    do_one_cycle();
}
```

Pourquoi pas `usleep(10000)` ?
- `usleep` **drifte** : si une itération prend 2 ms, la prochaine sera à 12 ms, puis 22 ms… le système s'accumule du retard.
- `clock_nanosleep(TIMER_ABSTIME)` réveille à un instant **absolu** — pas de drift.

### 5.3 Mesure du jitter

Chaque cycle on log :
```
jitter_us = (real_wakeup_time - scheduled_wakeup_time)
```
**Objectif : écart-type < 1 ms sur 60 s.** Si on n'y arrive pas, on enquête (priorités process, contention CPU, etc.).

---

## 6. Hiérarchie des décisions de sûreté

Le contrôleur doit toujours faire **le moins risqué quand il a un doute** :

```
   Trame valide ──► algo normal
        │
        ├─ CRC bad ──► ignore, incrémente compteur d'erreurs
        │
        ├─ Timeout > 50ms ──► passe en FAULT, brake = "last known safe"
        │
        ├─ Sensor stuck ──► FAULT, freinage dégradé (pas d'ABS, pression conducteur passe direct)
        │
        ├─ Plausibility violée ──► FAULT, DTC levé, log
        │
        └─ N erreurs consécutives ──► verrouillage en FAULT jusqu'à reset
```

C'est la philosophie **fail-operational** : si le système ABS lui-même est en défaut, le conducteur doit *quand même* pouvoir freiner (= la commande brute du conducteur passe sans modulation). Mieux vaut « freinage sans ABS » que « pas de freinage du tout ».

Détaillé dans [FMEA.md](./FMEA.md) (semaine 3).

---

## 7. Liste des livrables documentaires

| Fichier | Quand | Contenu |
|---|---|---|
| `PHYSICS.md` | ✅ jour 1-2 | Équations véhicule + Pacejka |
| `ARCHITECTURE.md` | ✅ ce document | Choix de design |
| `PROTOCOL.md` | jour 5-7 | Spec trame binaire, CRC, framing |
| `FMEA.md` | jour 15 | Tableau d'analyse modes de défaillance |
| `RESULTS.md` | jour 20-21 | Benchmarks, courbes, distance d'arrêt |

---

## 8. Décisions ouvertes (à trancher en cours de route)

- [ ] **Build system** : CMake (plus pro, plus complexe) vs Makefile (simple, suffit). Décision : **commencer en Makefile**, basculer CMake si besoin de tests Unity en S2.
- [ ] **Logging** : CSV plat (`ts,v,omega,slip,mu,brake,state,dtc`) ou binaire compact ? **CSV** pour visualisation facile dans le notebook Python.
- [ ] **Format des résultats** : matplotlib PNG seuls ou bonus GIF animé pour le README ? **PNG d'abord, GIF si reste du temps S3**.
