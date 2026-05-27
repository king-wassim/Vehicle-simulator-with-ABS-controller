# RESULTS — Benchmarks, mesures, validation expérimentale

> Tous les chiffres de ce document sont reproductibles : `make -C controller all`
> puis `bash scripts/scenarios/run_all.sh` puis `python3 -m scripts.plot_results`.
> Les plots PNG sont générés depuis les CSV produits par les scénarios.

---

## 1. Performance du contrôleur — Python oracle vs C SIL

**Question** : le port C reproduit-il fidèlement le comportement de la référence Python ?

**Méthode** : même algorithme bang-bang avec hystérésis (slip ∈ [0.10, 0.20]),
exécuté à 100 Hz sur le même plant (Vehicle + Pacejka simplifié, dry asphalt,
v₀ = 100 km/h, pédale conducteur = 100 bar). Le Python oracle calcule en local,
le C SIL communique via le protocole binaire à travers un socket TCP loopback.

![oracle vs SIL](../results/oracle_vs_c_sil.png)

| Métrique | Oracle Python | C SIL (end-to-end) | Δ |
|---|---|---|---|
| Distance d'arrêt | ~47 m | **~45 m** | < 5 % |
| Temps d'arrêt | ~3.6 s | ~3.5 s | < 5 % |
| Slip pendant ABS actif | oscille [0.05, 0.30] | oscille [0.05, 0.30] | identique |
| Comportement pédale | bang-bang | bang-bang | identique |

**Conclusion** : le contrôleur C reproduit l'oracle Python à ± 5 % près. La
faible différence vient du protocole (latence de cycle + quantification float32
sur le wire). C'est le critère de portage validé.

---

## 2. Performance de communication — milestone Semaine 1

**Question** : le protocole binaire est-il viable pour du temps réel ?

**Méthode** : 1000 trames @ 1 kHz, plant → ECU → plant en écho.

| Métrique | Valeur mesurée (loopback WSL) | Cible | Verdict |
|---|---|---|---|
| Frames échangées | 1000 / 1000 | 1000 | ✅ |
| CRC errors | 0 | 0 | ✅ |
| Bytes droppés | 0 | 0 | ✅ |
| Echo mismatches | 0 | 0 | ✅ |
| **RTT mean** | **0.225 ms** | < 1 ms | ✅ |
| RTT p50 | 0.216 ms | — | |
| RTT p95 | 0.332 ms | — | |
| RTT p99 | 0.435 ms | — | |

CI GitHub Actions (ubuntu-22.04, runners partagés) — seuil relâché à 5 ms
pour absorber le bruit ; toujours green.

---

## 3. Performance temps réel — milestone Semaine 2

**Question** : la super-loop tient-elle la grille 100 Hz sans drift ?

**Méthode** : `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` à 10 ms.
Mesure du jitter (real_wakeup − scheduled_wakeup) à chaque cycle via
Welford streaming (mean / σ / min / max sur ~400 cycles).

| Métrique | Valeur (WSL2 Ubuntu) | Cible | Verdict |
|---|---|---|---|
| Cycles exécutés sur 4 s | 410 | 400 ± 10 | ✅ |
| Jitter mean | 145-180 µs (selon run) | sub-ms | ✅ |
| **Jitter σ** | **40-55 µs** | < 1 ms | ✅ |
| Jitter max observé | ~500 µs | < 2 ms | ✅ |

Note : sur Linux bare-metal le mean tomberait à ~30 µs. WSL2 ajoute la
latence Hyper-V mais σ reste très bonne, ce qui est ce qui compte pour
le déterminisme.

---

## 4. Sûreté de fonctionnement — 6 scénarios d'acceptance (Semaine 3)

**Question** : le système réagit-il correctement aux pannes du FMEA ?

**Méthode** : un fault injector Python (`plant/fault_injector.py`) injecte
des pannes spécifiques pendant qu'on roule. Chaque scénario vérifie que
les DTC attendus sont levés et que le comportement de mitigation est
correct.

![distances par scénario](../results/scenarios_summary.png)

| Scénario | FMEA | Distance | DTC raised | Reaction state | PASS ? |
|---|---|---|---|---|---|
| **s1 nominal** | — | 45.4 m | aucun | MONITOR/ACTIVE | ✅ |
| **s2 ice patch** | F10 | 200 m | aucun (env, pas faute) | MONITOR/ACTIVE | ✅ |
| **s3 stuck sensor** | F01 | 56.1 m | STUCK + PLAUSIBILITY | FAULT_DEGRADED → LATCHED | ✅ |
| **s4 comm loss 200 ms** | F03, F08 | 50.5 m | COMM_TIMEOUT + STUCK | LATCHED (20 cycles > 10) | ✅ |
| **s5 CRC corruption 1 %** | F04 | 45.6 m | aucun (taux < seuil) | MONITOR/ACTIVE/STANDBY | ✅ |
| **s6 noisy sensor σ=5** | F02 | 46.7 m | SENSOR_RANGE (pics > seuil) | FAULT_DEGRADED transitoire | ✅ |

**Mesure clé — temps de réaction au défaut** (s3) :
- Fault injectée à `t = 0.5 s`
- Premier `DTC_SENSOR_STUCK` levé à `t = 0.61 s`
- **Détection latence = 110 ms** (cible FMEA < 250 ms) ✅

### Évolution des DTC dans le temps

![timeline DTC](../results/dtc_timeline.png)

Lecture : pour chaque scénario, les bandes colorées indiquent les
cycles où le bit DTC correspondant est levé. Les scénarios fault-free
(s1, s2, s5) sont vides ; s3 montre la levée rapide de STUCK+PLAUSIBILITY ;
s4 montre la latched persistante de COMM_TIMEOUT ; s6 a quelques pics
transitoires de SENSOR_RANGE quand le bruit pousse omega hors-plage.

---

## 5. Vérification continue — CI GitHub Actions

À chaque commit sur `main`, la pipeline ubuntu-22.04 enchaîne :

1. `unittest tests.test_protocol` — 13 tests Python (CRC, framing, layout).
2. `python -m plant.simulate` — envelope check + génération du PNG plant.
3. `make -C controller all CFLAGS="… -Werror"` — build C strict.
4. `make -C controller test` — **44 tests unitaires C** (5 CRC + 11 state +
   14 controller + 14 diagnostic).
5. `scripts/milestone_w1.py` — milestone W1 ping (latence relâchée à 5 ms).
6. `scripts/sil_brake_test.py` — milestone W2 SIL (voiture freine sous C).

Historique : verte aux commits W1 (`0526e91`), W2 (`a519c1b`), W3 (`0a0e171`).
- W1 : [run #26496884457](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26496884457)
- W2 : [run #26503946600](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26503946600)
- W3 : [run #26521054772](https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26521054772) ✅

---

## 6. Limites mesurables — ce qui ne marche PAS encore

À être explicite — section honnêteté technique :

- **Le détecteur F02 (noise)** est désactivé dans l'orchestrateur. Les
  patterns de sign-flipping d'un bang-bang naturel chevauchent ceux d'un
  bruit gaussien σ=5. Une solution propre nécessiterait soit d'isoler la
  contribution du contrôleur (résidu d'un filtre passe-bas), soit de gater
  la détection sur l'état ECU. Le code reste, testé en isolation
  (`test_diagnostic.c`), juste pas câblé. Voir `JOURNAL.md` W3 J16-17.
- **F10 (ice patch)** : pas de détection directe — on s'appuie sur la
  réactivité du bang-bang pour gérer la perte d'adhérence. Une vraie
  détection demanderait un estimateur de μ en ligne (Kalman, etc.).
- **Pas de durée de validation > 60 s** : tous les runs durent 6-15 s.
  Une CI nightly avec un run d'1 h permettrait de pister le jitter sur
  durée longue.
- **WSL2 ajoute ~400 µs de latence base** par rapport à Linux bare-metal.
  À refaire sur un vrai ARM Cortex-M pour des chiffres "publiables".

---

## 7. Reproductibilité — comment regénérer ces chiffres

```bash
# Tests unitaires
python3 -m unittest tests.test_protocol -v
make -C controller test

# Milestones d'intégration (2 terminaux)
python3 -m scripts.milestone_w1 --frames 1000 --rate 1000 &
./controller/build/ping_client 127.0.0.1 9000 1000

python3 -m scripts.sil_brake_test --duration 6 &
./controller/build/abs_ecu 127.0.0.1 9000 6 100

# Tous les scénarios + plots
bash scripts/scenarios/run_all.sh
python3 -m scripts.plot_results
```

Tout tourne sur WSL Ubuntu 22.04 + Python 3.10 + gcc 11.4.
