# FMEA — Analyse des modes de défaillance (Failure Mode and Effects Analysis)

> Document de référence pour la sûreté de fonctionnement du système ABS.
> Pour chaque composant identifié, on liste les modes de défaillance
> possibles, leurs effets (locaux et système), leur sévérité (échelle 1-10),
> la méthode de détection et la mitigation appliquée.
>
> Inspiré du processus FMEA tel qu'utilisé en automotive sous ISO 26262.
> Ce projet n'est pas certifié, mais applique la même démarche analytique.

---

## 1. Principe et lecture du tableau

### 1.1 Colonnes

| Colonne | Signification |
|---|---|
| **ID** | Identifiant unique (`F01`, `F02`...) référencé partout dans le code |
| **Composant** | Élément matériel ou logiciel concerné |
| **Mode de défaillance** | Comment le composant peut échouer |
| **Effet local** | Conséquence immédiate sur le module |
| **Effet système** | Conséquence sur la fonction ABS / sur le freinage / sur le conducteur |
| **Sévérité (S)** | 1-10 (10 = catastrophe absolue) |
| **Occurrence (O)** | 1-10 (10 = quasi-certain) |
| **Détection (D)** | 1-10 (10 = invisible) — d'autant plus haut que c'est dur à détecter |
| **RPN** | Risk Priority Number = S × O × D (priorité d'attention) |
| **Détection (méthode)** | Mécanisme qu'on a mis en place pour détecter |
| **Mitigation** | Réaction du système quand détecté |
| **DTC associé** | Code (bitfield) levé dans `actuator_payload_t.dtc_code` |
| **État cible** | État ECU vers lequel on bascule |

### 1.2 Convention de sévérité (adaptée de J1739 / AIAG-VDA)

| Score | Effet |
|---|---|
| 10 | Conducteur en danger immédiat (perte de contrôle total) |
| 9 | Perte de la fonction ABS sans avertissement |
| 8 | Perte de la fonction ABS avec avertissement (voyant tableau de bord) |
| 7 | Fonction ABS dégradée, conducteur garde la maîtrise |
| 5-6 | Comportement non optimal mais sûr (oscillations, sous-modulation) |
| 1-4 | Inconvénient cosmétique (logs sales, voyant flash transitoire) |

---

## 2. Tableau FMEA — 10 modes documentés

| ID | Composant | Mode de défaillance | Effet local | Effet système | S | O | D | RPN | Détection (méthode) | Mitigation | DTC | État cible |
|----|-----------|---------------------|-------------|---------------|---|---|---|-----|---------------------|------------|-----|------------|
| **F01** | Capteur ω_roue | **Valeur figée (stuck)** — le capteur retourne toujours la même valeur (encrassement, court-circuit dans la chaîne d'acquisition) | Slip calculé à partir d'une vitesse roue obsolète | ABS prend de mauvaises décisions, peut maintenir le frein actif alors que la roue est bloquée → perte d'efficacité du freinage | 9 | 3 | 6 | 162 | Compteur d'échantillons identiques consécutifs > `STUCK_THRESHOLD` (10 cycles = 100 ms) | Passage en `FAULT_DEGRADED`, pédale conducteur transmise directe (fail-operational) | `DTC_SENSOR_STUCK` (bit 0) | `FAULT_DEGRADED` |
| **F02** | Capteur ω_roue | **Bruit excessif** — capteur valide mais variance anormale (EMI, mauvais shielding, vibrations) | Slip oscille → contrôleur bang-bang chatter | Modulation du frein erratique, freinage haché, distance d'arrêt augmentée de 5-15 % | 6 | 5 | 5 | 150 | Variance glissante sur fenêtre de 20 échantillons > `NOISE_VARIANCE_THRESHOLD` (≈ 4.0 rad/s²) | Filtre médian temporaire + DTC logué (compte les occurrences pour DTC permanent) | `DTC_SENSOR_NOISE` (bit 1) | `FAULT_DEGRADED` (transitoire) |
| **F03** | Lien de communication (CAN/socket) | **Trame perdue / pas de données** — peer crashed, câble débranché, bus saturé | Pas de mise à jour des capteurs | Watchdog se déclenche, ECU ne sait plus ce qui se passe → impossible de moduler intelligemment | 8 | 4 | 3 | 96 | Watchdog logiciel : pas de trame valide reçue depuis > `WATCHDOG_TIMEOUT_MS` (50 ms) | `brake_command = last_known_safe`, passe pédale conducteur, voyant ABS | `DTC_COMM_TIMEOUT` (bit 2) | `FAULT_DEGRADED` |
| **F04** | Lien de communication | **CRC erroné** — corruption en cours de transmission (parasite EMI, bit flip) | Trame reçue avec données fausses | Décision prise sur valeurs aléatoires → comportement chaotique, slip aberrant | 9 | 6 | 2 | 108 | Vérification CRC-16/CCITT-FALSE sur chaque trame, mismatch → drop + compteur | Trame ignorée, attente de la suivante, compteur d'erreurs incrémenté ; si > seuil → DTC | `DTC_COMM_CRC` (bit 3) | (cumul) `FAULT_DEGRADED` |
| **F05** | Capteur ω_roue ou v_véhicule | **Valeur hors plage physique** — ex : ω négatif, v > 200 km/h, ω > 500 rad/s | Le slip calculé est aberrant | Slip > 1 ou < 0 → mu hors domaine → contrôleur prend des décisions absurdes | 7 | 3 | 2 | 42 | Range check : `v ∈ [-1, 120] m/s` et `ω ∈ [-1, 500] rad/s` | Saturation à la borne + DTC logué | `DTC_SENSOR_RANGE` (bit 4) | `FAULT_DEGRADED` |
| **F06** | Calcul slip | **Division par zéro** à v_véhicule = 0 | Crash potentiel ou NaN propagé | Comportement indéfini, peut bloquer toute la boucle | 10 | 2 | 7 | 140 | Test `if (v < V_MIN_FOR_SLIP)` AVANT division | Retourne slip = 0 + bascule en `STANDBY` si v < 5 km/h | `DTC_PLAUSIBILITY` (bit 5) | `STANDBY` (normal) |
| **F07** | Capteur ω_roue vs v_véhicule | **Plausibilité physique violée** — ω_roue × R > v_véhicule en freinage (la roue tourne plus vite que la voiture) | Slip négatif (impossible en pur freinage) | Si non détecté : contrôleur croit qu'il faut freiner plus alors que la voiture est en perte d'adhérence | 7 | 2 | 4 | 56 | Cross-check : si `ω × R > v + ε` pendant > 5 cycles consécutifs → implausible | DTC levé + bascule en `FAULT_DEGRADED` | `DTC_PLAUSIBILITY` (bit 5) | `FAULT_DEGRADED` |
| **F08** | Logique applicative | **Fautes consécutives multiples** — combinaison de F01-F07 répétée | Le système n'arrive plus à recouvrer | Modulation ABS définitivement compromise sur ce cycle de conduite | 8 | 2 | 1 | 16 | Compteur `fault_count` ; si ≥ `ABS_FAULT_LATCH_THRESHOLD` (10) cycles consécutifs avec DTC | Latch `FAULT_LATCHED` permanent jusqu'au reset (cycle de boot) | (cumul) | `FAULT_LATCHED` |
| **F09** | Actionneur hydraulique (modélisé) | **Saturation pression frein** — pression demandée > capacité actionneur | Pression effective bornée | Bang-bang continue mais à pression sub-optimale ; distance d'arrêt légèrement allongée | 4 | 6 | 9 | 216 | Saturation logicielle dans `plant/vehicle.py::step` à `max_brake_pressure` | Comportement physique correct (pas un bug, c'est attendu) | — | inchangé |
| **F10** | Surface route (env) | **Changement brutal d'adhérence** — passage sec → verglas, μ_max chute de 1.0 à 0.1 | Le contrôleur, calibré pour μ ≈ 1.0, surestime largement la capacité de freinage | Distance d'arrêt explose ; sans détection, slip part facilement à 0.5+ | 6 | 4 | 8 | 192 | Pas de détection directe (le pneu ne sait pas) — mitigation via algo robuste qui détecte rapidement le slip excessif | Bang-bang réagit en relâchant rapidement (slip > UPPER → cmd = 0) | — | reste `ACTIVE` (par design) |

---

## 3. Cartographie DTC ↔ FMEA

Le `dtc_code` transmis dans chaque `actuator_payload_t` est un bitfield qui
permet de cumuler plusieurs fautes simultanées.

| Bit | DTC | Modes FMEA couverts |
|-----|------------------------|---------------------|
| 0 | `DTC_SENSOR_STUCK` | F01 |
| 1 | `DTC_SENSOR_NOISE` | F02 |
| 2 | `DTC_COMM_TIMEOUT` | F03 |
| 3 | `DTC_COMM_CRC` | F04 |
| 4 | `DTC_SENSOR_RANGE` | F05 |
| 5 | `DTC_PLAUSIBILITY` | F06, F07 |
| 6-15 | _reserved_ | extensions futures |

Le bitfield permet par exemple de signaler simultanément
`DTC_SENSOR_NOISE | DTC_COMM_CRC` (= `0x0A`) si le lien est à la fois
bruité et corrompu.

---

## 4. Pyramide de défense ("defense in depth")

Le système n'a pas un seul niveau de protection — chaque couche peut
attraper une faute que la précédente aurait ratée :

```
┌─────────────────────────────────────────────────┐
│ NIVEAU 4 — Fail-operational                      │
│  abs_controller passe driver_request brut        │
│  → Conducteur peut TOUJOURS freiner              │
└─────────────────────────────────────────────────┘
                       ▲
┌─────────────────────────────────────────────────┐
│ NIVEAU 3 — Latch après fautes répétées           │
│  abs_state → FAULT_LATCHED après N=10 cycles     │
│  → Voyant ABS persistant, visite garage          │
└─────────────────────────────────────────────────┘
                       ▲
┌─────────────────────────────────────────────────┐
│ NIVEAU 2 — État dégradé sur DTC                  │
│  abs_state → FAULT_DEGRADED, ABS désactivé       │
│  → Freinage sans modulation, conducteur OK       │
└─────────────────────────────────────────────────┘
                       ▲
┌─────────────────────────────────────────────────┐
│ NIVEAU 1 — Détection module diagnostic            │
│  diagnostic.c : range, stuck, noise, plausibility│
│  → DTC levé dans actuator_payload_t              │
└─────────────────────────────────────────────────┘
                       ▲
┌─────────────────────────────────────────────────┐
│ NIVEAU 0 — Validation à la source                │
│  HAL : flag `valid`, watchdog, range basique     │
│  → Données invalides marquées dès l'entrée       │
└─────────────────────────────────────────────────┘
```

Si la HAL rate une valeur aberrante (niveau 0), le diagnostic la chope
(niveau 1). Si le diagnostic la rate, la state machine la chope via
son range check redondant (niveau 2). Si la state ne bascule pas, le
controller en mode FAULT passe quand même la pédale (niveau 4). Une
faute doit franchir **plusieurs barrières** indépendantes pour causer
un dommage — c'est exactement la philosophie ISO 26262.

---

## 5. Vérification — comment on prouve que chaque mitigation marche

Chaque mode de défaillance a un scénario d'acceptance dans
`scripts/scenarios/` qui injecte la faute et vérifie la réaction
attendue.

| ID | Scénario d'acceptance | Critère de PASS |
|----|----------------------|-----------------|
| F01 | `stuck_wheel_sensor.py` | DTC_SENSOR_STUCK levé en < 200 ms, état → FAULT_DEGRADED |
| F02 | `noisy_sensor.py` | DTC_SENSOR_NOISE levé sur bruit > seuil, distance d'arrêt < +20 % |
| F03 | `comm_loss_200ms.py` | DTC_COMM_TIMEOUT levé après 50 ms, brake = last_safe |
| F04 | `crc_corruption_1pct.py` | Trames corrompues ignorées, système continue, DTC compteur incrémenté |
| F05 | (couvert par range tests unitaires + scenarios stuck/noise) | Saturation aux bornes, pas de comportement aberrant |
| F06 | (couvert par tests unitaires `test_abs_controller.c`) | `compute_slip(v=0)` retourne 0, pas de NaN |
| F07 | (à intégrer dans `noisy_sensor.py` extended) | Implausibilité détectée si ω × R dépasse v |
| F08 | `comm_loss_persistent.py` | FAULT_LATCHED atteint après 10 cycles continus |
| F09 | (test physique nominal) | Pression saturée à 150 bar même si commande = 200 |
| F10 | `ice_patch.py` | Distance d'arrêt cohérente avec μ=0.1 (~200 m+), voiture reste contrôlable |

---

## 6. Limites de l'analyse

À être honnête en entretien :

- **Pas une vraie FMEA d'industrie** — un vrai FMEA automotive est piloté
  par une équipe pluridisciplinaire (sécurité, élec, méca, logiciel) sur
  des semaines, avec un outil dédié (APIS IQ, Plato FMEA, etc.) et des
  reviews formelles. Ici c'est un FMEA *logiciel* fait à une personne en
  une journée — l'exercice pédagogique compte plus que l'exhaustivité.
- **Pas d'analyse hardware** — les modes liés à des défauts pièces
  (rupture câble capteur, fissure circuit imprimé, dérive composant en
  température) sont hors-scope. Sur le vrai projet, ils ajouteraient
  ~30 modes supplémentaires.
- **Pas de calcul de probabilité quantitatif** — les colonnes O / D sont
  des estimations qualitatives. Un vrai dossier ASIL impose des taux de
  défaillance issus de bases (Siemens SN29500, MIL-HDBK-217).
- **Pas de Fault Tree Analysis (FTA) en miroir** — FMEA est bottom-up,
  FTA est top-down ; les deux se complètent en ISO 26262. Hors-scope ici.

---

## 7. Pour aller plus loin (références)

- **ISO 26262 part 9** : Automotive Safety Integrity Level (ASIL)-oriented
  and safety-oriented analyses (FMEA, FTA, FMEDA).
- **SAE J1739** : Potential Failure Mode and Effects Analysis in Design.
- **AIAG-VDA FMEA Handbook (2019)** : la référence industrie automobile
  EU/US conjointe.
- **Bosch ECU Description Handbook** : voir les fiches DTC standardisées
  pour le freinage électronique (P0500-series).
