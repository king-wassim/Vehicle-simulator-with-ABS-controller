# INTERVIEW_QA — Questions probables en entretien, réponses préparées

> Trente questions qu'un recruteur ou un ingé technique chez
> KPIT / Vitesco / Bosch / Continental peut poser en regardant ce projet.
> Pour chacune : ce que la question vraiment cherche, une réponse concise
> à donner à l'oral, et un endroit dans le repo où aller chercher la preuve
> si on demande à voir.

---

## A. Questions sur le projet lui-même

### Q1 — "Présente-moi ton projet en 2 minutes."

**Cherche** : capacité à synthétiser, à parler vrai.

**Réponse**
> J'ai construit une plateforme Software-in-the-Loop pour un ECU d'ABS,
> en 21 jours. Côté physique, un modèle de quart-véhicule Python avec
> dynamique Newton et pneu Pacejka. Côté logiciel, un ECU en C bare-metal
> avec une couche HAL séparée du métier (comme AUTOSAR), une machine à
> états INIT-STANDBY-MONITOR-ACTIVE-FAULT, un algo bang-bang à hystérésis
> sur le slip ratio, et une super-loop temps réel à 100 Hz via
> `clock_nanosleep TIMER_ABSTIME`. Les deux processes parlent un protocole
> binaire que j'ai conçu, avec un CRC-16 CCITT-FALSE. J'ai mesuré : la
> voiture freine de 100 à 0 km/h en 54 mètres sous le contrôle du C, jitter
> sigma 211 microsecondes, zéro erreur CRC sur 1000 trames. Tout est
> testé — 30 tests unitaires C, 13 Python — et la CI GitHub Actions est
> verte. Mon but était d'apprendre l'embarqué automobile en profondeur :
> AUTOSAR, sûreté de fonctionnement, real-time, communication bus.

### Q2 — "Pourquoi un ABS, et pas un autre exemple ?"

**Cherche** : capacité à justifier ses choix.

**Réponse**
> Trois raisons. Premièrement, l'ABS est *le* cas d'école du contrôle
> embarqué automobile — il combine dynamique non-linéaire (courbe μ-slip),
> contraintes temps réel (100 Hz), et exigences safety-critical
> (fail-operational). Deuxièmement, c'est suffisamment compact pour
> qu'une seule personne livre une démo complète en trois semaines.
> Troisièmement, le vocabulaire (slip ratio, Pacejka, DTC) est exactement
> celui des offres KPIT et Vitesco — un recruteur reconnaît immédiatement
> les patterns.

### Q3 — "Qu'est-ce qui te différencie d'un script Python qui tracerait des courbes ?"

**Cherche** : tu sais ce qui rend ça "embarqué" vraiment.

**Réponse**
> Trois choses concrètes. **Un** : le contrôleur est en C bare-metal,
> compilé `-Werror -Wpedantic`, sans malloc, avec allocation statique —
> portable demain sur STM32 sans réécriture du métier. **Deux** : le
> protocole est binaire packé, little-endian, CRC, framing — comme du CAN
> tunnelé. **Trois** : la boucle est cadencée à 100 Hz avec mesure de
> jitter, pas une boucle Python qui dort `sleep(0.01)` et drifte. La
> séparation HAL/SWC est faite à dessein : si demain je remplace le socket
> par un driver ADC, le `abs_controller.c` ne change pas une ligne.

---

## B. Questions techniques — physique et contrôle

### Q4 — "C'est quoi un slip ratio ?"

**Réponse**
> Le rapport `s = (v − ωR)/v`, où `v` est la vitesse de la voiture et
> `ωR` la vitesse linéaire équivalente de la roue. À slip = 0, la roue
> roule librement. À slip = 1, la roue est complètement bloquée. Le truc,
> c'est que la force de freinage transmise au sol n'est pas monotone en
> slip : elle augmente d'abord, atteint un pic vers `s = 0.15`, puis
> chute. C'est pour ça qu'une roue bloquée freine *moins bien* qu'une
> roue qui tourne un peu plus lentement que la voiture.

**Preuve** : `docs/PHYSICS.md` §2, `plant/tire.py:compute_slip`.

### Q5 — "Pourquoi un bang-bang et pas un PID ?"

**Réponse**
> La courbe `μ(s)` est très non-linéaire et change avec la surface
> (sec, mouillé, verglas). Un PID supposerait une linéarisation autour
> d'un point d'opération — mais ce point bouge en permanence. Un
> bang-bang à hystérésis exploite directement la forme physique : "tant
> que le slip est dans la zone optimale, je tiens ; s'il dépasse, je
> relâche ; s'il retombe, je réapplique." C'est aussi la philosophie des
> vrais ABS Bosch, qui modulent en ouvert/fermé sur la valve hydraulique,
> à 5-20 Hz par roue.

**Preuve** : `controller/src/app/abs_controller.c`,
`plant/oracle_abs.py`.

### Q6 — "C'est quoi Pacejka ?"

**Réponse**
> La Magic Formula de Hans Pacejka — un modèle empirique de pneu où la
> force longitudinale est `μ(s) = D·sin(C·atan(B·s))`. Trois paramètres
> physiques : `D` = pic d'adhérence (1.0 sur sec, 0.1 sur verglas), `C` ≈
> 1.65 = facteur de forme, `B` ≈ 10 = raideur initiale. La version
> complète a 10+ coefficients pour modéliser la dépendance en charge,
> angle de glissement, etc. Pour un projet pédagogique, la version 3
> paramètres suffit largement.

**Preuve** : `plant/tire.py`, `docs/PHYSICS.md` §3.

### Q7 — "Comment tu valides ton modèle physique ?"

**Réponse**
> Par ordres de grandeur depuis la littérature. Sans ABS, sur dry asphalt
> à 100 km/h, une voiture s'arrête en ~60 mètres. Avec un ABS qui
> maintient le slip à ~0.15, ~45 mètres. Mon plant donne 59.6 m et
> 47.5 m respectivement, soit un gain de 20 % — conforme aux ~25 %
> publiés par Bosch dans leurs whitepapers. Si mon plant était faux, je
> débuggerais le C en chassant des fantômes. C'est explicite dans le
> code : `simulate.py` échoue si les distances sortent de l'envelope.

**Preuve** : `plant/simulate.py:main` (assertions), `docs/PHYSICS.md` §5.

---

## C. Questions techniques — C et architecture

### Q8 — "Pourquoi `__attribute__((packed))` ?"

**Réponse**
> Pour interdire au compilateur d'ajouter du padding entre les champs.
> Sans `packed`, un `uint16_t` après un `uint32_t` peut être aligné sur
> 4 octets selon le compilateur, et la taille de la struct sur le wire
> dépend de l'ABI. C'est inacceptable quand on échange des bytes entre
> deux processus / deux MCU. `packed` garantit la layout exacte. Je
> vérifie aussi avec `_Static_assert(sizeof(...) == 18)` pour qu'un
> upgrade compilo qui ignorerait le pragma casse le build.

**Preuve** : `controller/src/drivers/protocol.h`.

### Q9 — "Différence MCAL / RTE / SWC dans AUTOSAR ?"

**Réponse**
> Trois couches. **MCAL** (Microcontroller Abstraction Layer) : drivers
> bas-niveau, dépend du microcontrôleur — ADC, SPI, CAN driver. **SWC**
> (Software Component) : la logique métier, ne sait rien du matériel.
> **RTE** (Runtime Environment) : la couche de glue entre les deux,
> générée par l'outil (Vector DaVinci, EB tresos), qui route les
> appels et gère les interfaces. Ici je n'ai pas l'outil mais je
> reproduis le pattern : `drivers/` = MCAL, `hal/` = ECU Abstraction +
> RTE simplifié, `app/` = SWC. Si je porte sur STM32, je réécris
> `drivers/` et la moitié de `hal/`, le `app/` ne bouge pas.

**Preuve** : `controller/src/{drivers,hal,app}/`, `docs/ARCHITECTURE.md`
§4.2.

### Q10 — "Tu connais MISRA-C ?"

**Réponse**
> Oui. C'est une norme de codage automotive qui interdit les constructions
> C dangereuses (`goto` non-structuré, malloc dans le runtime, casts
> entre pointeurs et entiers, etc.). Je ne suis pas certifié sur ce
> projet, mais j'ai appliqué les règles principales : **21.3** pas de
> `malloc` ; **5.x** : pas d'identifier qui shadow ; **15.5** : un seul
> point d'exit par fonction quand possible ; headers guards.
> J'ai aussi mis `-Werror -Wpedantic` qui chope une partie de ce que
> MISRA chope.

**Preuve** : `controller/Makefile` (CFLAGS), code itself.

### Q11 — "Pourquoi pas de RTOS comme FreeRTOS ?"

**Réponse**
> Volontaire. **Un** : j'ai déjà fait du FreeRTOS dans d'autres projets,
> je voulais creuser autre chose ici (la sûreté). **Deux** : super-loop
> mono-thread = pas de race conditions à débugger, déterminisme parfait.
> **Trois** : à 100 Hz sur une boucle de contrôle, on est très loin de
> saturer un Cortex-M moderne — un RTOS apporterait peu. Si je voulais
> ajouter une seconde boucle (par exemple un thread RX socket dédié pour
> protéger le contrôle d'un blocage I/O), je passerais sur 2 threads
> pthreads, ou sur FreeRTOS pour la cible réelle.

**Preuve** : `controller/src/main.c`, `docs/ARCHITECTURE.md` §4.3.

### Q12 — "Tu utiliserais quoi à la place d'un socket sur un vrai ECU ?"

**Réponse**
> Selon la cible : **CAN** (Controller Area Network) pour la com
> intra-véhicule, **CAN-FD** si je dois passer >8 octets de payload (mon
> sensor_payload fait 18 B, donc CAN-FD obligatoire) ou **SPI/I2C** pour
> les capteurs locaux directs. Le driver MCAL est différent, mais mon
> API `hal_sensors_read(sensors_data_t*)` reste identique — c'est tout
> le point de la HAL.

**Preuve** : `controller/src/hal/hal_sensors.h` (commentaires).

---

## D. Questions techniques — communication

### Q13 — "Comment marche ton CRC-16 ?"

**Réponse**
> CRC-16/CCITT-FALSE, polynôme `0x1021`, init `0xFFFF`, pas de réflexion,
> XOR-out à 0. C'est la variante spécifiée par AUTOSAR E2E pour la
> protection bout-en-bout sur CAN. L'implémentation est un shift-register
> bit-à-bit : pour chaque byte, XOR avec le high-byte du CRC, puis 8
> shifts à gauche en XORant avec le polynôme si le top bit était à 1.
> Sur Cortex-M moderne ou avec une table 256 entrées on peut accélérer,
> mais à 100 Hz le bit-shift est largement suffisant.

**Preuve** : `controller/src/drivers/crc.c`.

### Q14 — "Comment tu détectes qu'une trame est corrompue *en route* ?"

**Réponse**
> Deux mécanismes en série. **D'abord la length** : je vérifie qu'elle
> est ≤ 64 B (notre MAX_PAYLOAD). Si elle est aberrante (par exemple
> `0xFFFF`), je rejette. **Ensuite le CRC** : je recalcule sur
> `type + length + payload` et je compare aux deux derniers bytes. Si
> mismatch, je drop la trame et j'incrémente `errors_crc`. Le décodeur
> resync ensuite en scannant le prochain magic `0xAA55`. Tout ça sans
> jamais arrêter la boucle.

**Preuve** : `controller/src/drivers/socket_drv.c::try_extract_frame`,
`plant/protocol.py::FrameDecoder`.

### Q15 — "Pourquoi tu fais ton propre protocole au lieu de gRPC / ZeroMQ / MQTT ?"

**Réponse**
> Parce qu'aucun de ces frameworks ne tourne sur un Cortex-M sans heap,
> sans OS, et sans 100 ko de RAM. Le protocole que j'ai conçu fait 18 +
> 12 = 30 bytes utiles par cycle, 9 bytes d'envelope, et le parser tient
> en 200 lignes de C. Il **mime ce qu'on a sur CAN** : header magique
> (≈ ID CAN), length, payload, CRC. Apprendre à concevoir ça, c'est
> apprendre à dialoguer avec n'importe quel bus série série propriétaire
> du monde réel.

**Preuve** : `docs/PROTOCOL.md` §1.

---

## E. Questions techniques — temps réel

### Q16 — "C'est quoi `clock_nanosleep TIMER_ABSTIME` ?"

**Réponse**
> Un appel POSIX qui dort jusqu'à un instant **absolu** spécifié dans
> un `struct timespec`. Différence avec `usleep(10000)` : `usleep` dort
> *en plus*, donc si ton itération met 2 ms, la prochaine sera à `t +
> 12 ms`, puis à `t + 22 ms`, etc. — drift cumulatif. `TIMER_ABSTIME`
> avec un timestamp recalculé en absolu (`next += 10ms` à chaque tour)
> garde la grille fixe. Pas de drift, même si une itération dépasse.
> J'utilise `CLOCK_MONOTONIC` pour être immune aux changements d'heure
> système.

**Preuve** : `controller/src/main.c::main` (boucle while).

### Q17 — "Comment tu mesures le jitter ?"

**Réponse**
> À chaque réveil, je note `real_wake = clock_gettime(MONOTONIC)` puis
> `jitter_us = real_wake − scheduled_wake`. J'accumule avec
> Welford — mean et variance en streaming, O(1) mémoire, numériquement
> stable même sur des millions d'échantillons. À la fin je sors mean,
> stddev, min, max. Cible : σ < 1 ms sur 60 s. Mesuré : σ = 211 µs en
> WSL2 (qui ajoute ~400 µs de latence base par-dessus Linux bare-metal).

**Preuve** : `controller/src/main.c::jitter_*`.

### Q18 — "Et si une itération dépasse les 10 ms ?"

**Réponse**
> `clock_nanosleep` se réveille immédiatement parce que la cible est
> déjà dans le passé. Le cycle suivant est compressé — il essaiera de
> rattraper. Si ça arrive *souvent*, c'est qu'on a un problème de
> scheduling : on inspecte les priorités process (en prod on met
> SCHED_FIFO avec priorité élevée pour bypass le scheduler standard),
> et les contentions CPU (autres process gourmands). Le jitter max
> mesuré (max us) sert d'alerte : si on voit du > 5 ms régulièrement
> on enquête.

**Preuve** : commentaire dans `controller/src/main.c`,
`docs/ARCHITECTURE.md` §5.

---

## F. Questions techniques — sûreté

### Q19 — "C'est quoi un FMEA ?"

**Réponse**
> Failure Mode and Effects Analysis. Un tableau formel où, pour chaque
> composant, on liste : ses modes de défaillance possibles, leurs effets
> locaux et système, leur sévérité (1-10), la stratégie de détection,
> et la mitigation. C'est la première étape de toute analyse de sûreté
> automotive. Pour mon ABS : capteur figé, capteur bruité, capteur hors
> range, comm timeout, CRC corrompu, slip aberrant, etc. Chaque mode
> a un DTC et une réaction.

**Preuve** : `docs/FMEA.md` (à venir Semaine 3, J15).

### Q20 — "Fail-safe ou fail-operational ?"

**Réponse**
> Pour un ABS : **fail-operational** est la bonne réponse. Quand
> l'électronique tombe, le système hydraulique doit *quand même*
> permettre au conducteur de freiner — sans la modulation ABS, mais
> avec la pression. Dans mon code, en `ECU_FAULT_DEGRADED` ou
> `ECU_FAULT_LATCHED`, le contrôleur retourne `driver_request` brut.
> Au pire la roue bloque, mais le conducteur garde le contrôle.
> Fail-safe (= couper l'actionneur) serait catastrophique sur un frein.

**Preuve** : `controller/src/app/abs_controller.c::abs_controller_compute`
(branches FAULT_*).

### Q21 — "C'est quoi ISO 26262 / ASIL ?"

**Réponse**
> ISO 26262 = norme de sûreté fonctionnelle automotive (équivalent
> "qualité safety" du logiciel). Elle définit des niveaux ASIL
> (Automotive Safety Integrity Level) de A à D : QM (quality managed,
> pas safety) → A (faible) → D (le plus strict, ex : airbag, freinage).
> Un ABS typique est **ASIL B/C** sur le freinage, **ASIL D** sur
> certaines fonctions critiques (anti-roll-over). Sur ce projet je n'ai
> pas certifié — c'est un projet perso — mais j'ai appliqué les patterns
> attendus à ces niveaux : redondance, plausibility, watchdog, FMEA,
> fail-operational.

**Preuve** : à mentionner à l'oral, supportée par `docs/FMEA.md`
(à venir).

### Q22 — "Que se passe-t-il si le capteur de roue se grippe (valeur figée) ?"

**Réponse**
> Trois étages de protection. **Un** : la HAL retourne la valeur, le
> module diagnostic compte les échantillons identiques consécutifs ;
> si ça dépasse un seuil (10 par exemple), `DTC_SENSOR_STUCK` est levé.
> **Deux** : la state machine bascule en `ECU_FAULT_DEGRADED`. **Trois** :
> le contrôleur passe la pédale conducteur sans modulation. Et si la
> panne persiste 10 cycles, on latche en `FAULT_LATCHED` — voyant ABS
> au tableau de bord jusqu'à reset garage. Détection promise dans la
> FMEA : < 100 ms.

**Preuve** : `controller/src/app/diagnostic.c` (à venir S3),
`abs_state.c::abs_state_update`.

---

## G. Questions plus larges

### Q23 — "Tu as utilisé une IA pour ce projet ?"

**Réponse**
> Oui — explicitement Claude (Anthropic) pour la rédaction de
> documentation, le pair-programming sur les parties pénibles
> (boilerplate, tests unitaires), et la relecture. Les décisions
> techniques (architecture, choix de protocole, calibration Pacejka,
> seuils ABS) sont les miennes — j'ai dû savoir quoi demander et
> juger les réponses. C'est documenté dans les co-authors des commits
> Git. Je vois l'IA comme un junior toujours disponible qui me fait
> gagner du temps sur le typage, mais qui ne remplace pas la
> compréhension.

### Q24 — "Pourquoi en français pour les docs et en anglais pour le code ?"

**Réponse**
> Convention pro classique. Le code et les identifiants en anglais
> parce que c'est le standard de l'industrie, portable, lisible par
> n'importe quelle équipe internationale. Les docs en français parce
> que c'est ma langue de réflexion — j'explique mieux les concepts dans
> ma langue native, et le projet est destiné en partie à des recruteurs
> francophones (KPIT France, Vitesco France). En entretien je peux
> switcher à l'anglais sans problème.

### Q25 — "Et tu déploierais ça en prod ?"

**Réponse**
> Non, **pas en l'état**, et c'est important de le dire. C'est un projet
> pédagogique. Pour aller en prod il faudrait : certification ISO 26262
> ASIL (process complet de plusieurs mois), tests sur HIL avec vrai
> calculateur, validation à -40 °C / +85 °C, EMC, fuzzing du protocole,
> reviews de code par équipe sûreté, formal verification sur les états
> critiques, traçabilité complète des requirements aux tests… Le projet
> prouve que je *comprends* les patterns. Pas qu'il est *prêt* pour une
> voiture.

### Q26 — "Quelle a été ta plus grosse galère sur ce projet ?"

**Réponse**
> Le calibrage du modèle Pacejka. J'avais pris `C = 1.9` (valeur que
> Wikipedia donne pour le slip *latéral*), mais c'est `C = 1.65` pour
> le longitudinal. Résultat : ma voiture sans ABS s'arrêtait en 105 m
> au lieu de 60. Pendant une heure j'ai cherché un bug dans
> l'intégration Euler, dans mes équations, dans le slip ratio. C'est
> en traçant la courbe `μ(s)` que j'ai vu que `μ(s=1)` valait 0.34 au
> lieu de 0.65 attendu — d'où la divergence. Leçon : valider chaque
> brique en isolation avant d'intégrer. C'est documenté dans
> `docs/JOURNAL.md` J3-4.

### Q27 — "Tu ferais quoi différemment si tu recommençais ?"

**Réponse**
> Trois choses. **Un** : commencer par la CI dès le J1, pas le J7. Ça
> aurait évité que des régressions silencieuses passent à travers.
> **Deux** : écrire `abs_controller.c` *avant* la HAL et le main.c — la
> logique métier d'abord, le plomberie après. Quand on commence par la
> plomberie on ne sait pas si elle sert. **Trois** : mettre en place
> un `cmake` dès le départ au lieu de `make`. Ça scale mieux quand on
> ajoute Unity, des coverage tools, ou de la cross-compilation. Mais
> globalement le plan en 3 semaines a tenu, donc l'essentiel est OK.

---

## H. Questions piège / "stress test"

### Q28 — "Explique-moi pourquoi un float n'est pas un bon choix pour des données safety-critical."

**Réponse**
> Trois problèmes. **Un** : les NaN et les Inf — une seule division par
> zéro silencieuse contamine tous les calculs en aval. **Deux** : la
> précision relative ; à grandes valeurs, le pas minimum entre deux
> floats consécutifs peut dépasser la tolérance attendue (ex : à
> 1e6, l'ULP d'un f32 vaut 0.06 — inadapté pour du µs). **Trois** :
> la portabilité entre MCU (FPU vs émulation soft) peut changer le
> résultat bit à bit. En automotive safety-critical on utilise plutôt
> du **fixed-point** (entier 32 bits + scale factor) qui est déterministe
> et reproductible. Sur ce projet j'ai utilisé float par simplicité, ce
> serait à durcir pour ASIL D.

### Q29 — "Comment tu garantis que ton CRC fonctionne ?"

**Réponse**
> Trois niveaux. **Un** : KAT (known-answer test) sur le vecteur
> canonique `"123456789"` qui doit donner `0x29B1` — c'est le test
> reconnu pour CRC-16/CCITT-FALSE. **Deux** : KAT croisée C ↔ Python
> sur une trame de référence qui doit donner le même CRC dans les deux
> langages (`0xBC19`). **Trois** : test négatif — je flippe un bit du
> CRC, le décodeur doit le rejeter ET compter une erreur. Ces trois
> tests sont dans `test_crc.c` et `tests/test_protocol.py`, joués à
> chaque CI.

### Q30 — "Si je te donne un STM32F4 demain, combien de temps pour porter ?"

**Réponse**
> Pour un MVP (sortie sur UART d'un brake_command fake plutôt que sur
> CAN, capteurs simulés en table) : **un week-end**. Je remplace
> `socket_drv.c` par `uart_drv.c` (utilise HAL CubeMX), je remplace
> `clock_nanosleep` par un timer hardware avec interrupt à 10 ms, je
> garde tout le code `app/` tel quel. Pour une version complète sur CAN
> avec vrais capteurs : **deux à trois semaines** — il faut configurer
> CAN, écrire la couche FreeRTOS si on veut multi-tâche, valider sur
> banc. C'est exactement le scénario qui montre la valeur du pattern
> HAL : 80 % du code ne bouge pas.

---

## Méta — comment utiliser ce document

- **Avant un entretien** : relire les Q1, Q2, Q4-7, Q9-12, Q19-22 — c'est ~80 % de ce qu'on te posera.
- **Pendant l'entretien** : si on te questionne sur un point précis, dire "regarde dans `docs/X.md`" et expliquer à voix haute. Ça montre que la doc existe et que tu la connais.
- **À mettre à jour** : si on te pose une question pas dans cette liste, l'ajouter ici après l'entretien. C'est une asset cumulatif.
