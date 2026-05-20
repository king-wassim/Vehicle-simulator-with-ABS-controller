# PHYSICS.md — Le modèle physique de l'ABS

> **Objectif de ce document.** Avant d'écrire la moindre ligne de code, on doit comprendre *pourquoi* l'ABS existe et *quelles équations* on va intégrer numériquement. Ce document est rédigé comme un cours : si tu peux le réexpliquer à voix haute, tu as compris.

---

## 1. Pourquoi l'ABS existe — l'intuition physique

### 1.1 Le paradoxe contre-intuitif

Question piège qu'on pose souvent en entretien :

> « Quelle est la pression de freinage qui minimise la distance d'arrêt ? »

Réponse naïve : la pression maximale. **Faux.**

Si tu écrases complètement la pédale, les roues se **bloquent** (elles arrêtent de tourner). Une roue bloquée glisse sur la route. Et — surprise — **une roue qui glisse freine moins bien qu'une roue qui roule un peu plus vite que prévu**. En plus, une roue bloquée perd toute capacité de direction : tu peux tourner le volant, la voiture continue tout droit.

L'ABS résout exactement ce problème : **garder la roue dans la zone optimale de glissement**, juste avant le blocage.

### 1.2 Les trois régimes d'une roue qui freine

| Régime | Slip ratio | Comportement | Adhérence |
|---|---|---|---|
| **Rouleur libre** | `s ≈ 0` | La roue tourne à la vitesse de la voiture | Faible (peu de force de freinage transmise) |
| **Optimal** | `s ∈ [0.10, 0.20]` | La roue tourne *un peu plus lentement* que la voiture | **Maximale** — pic de la courbe μ(s) |
| **Bloquée** | `s = 1` | La roue ne tourne plus, elle glisse | Faible et chaotique |

L'ABS = un système qui **module la pression** pour maintenir `s` dans la zone optimale, même quand le conducteur appuie comme un fou sur la pédale.

---

## 2. Le slip ratio (taux de glissement)

### 2.1 Définition formelle

Le slip ratio compare la vitesse linéaire de la voiture (`v`) à la vitesse linéaire équivalente de la roue (`ω × R`, où `ω` est la vitesse angulaire en rad/s et `R` le rayon de la roue) :

```
       v_voiture − ω_roue × R
  s  = ──────────────────────────
              v_voiture
```

Notations dans le code :
- `v_vehicle` = `v` (m/s)
- `omega_wheel` = `ω` (rad/s)
- `wheel_radius` = `R` (m)

### 2.2 Cas limites — à mémoriser

| Situation | `ω × R` | `s` |
|---|---|---|
| Roue libre (la voiture roule, on ne freine pas) | `= v` | `0` |
| Roue qui tourne *moins vite* que v (on freine modérément) | `< v` | `∈ (0, 1)` |
| Roue **complètement bloquée** | `= 0` | `1` |
| Roue qui *patine* en accélération (`ω × R > v`) | `> v` | `< 0` (parfois redéfini en `(ωR−v)/ωR`) |

Dans notre projet on ne traite que **le freinage** → `s ∈ [0, 1]`.

### 2.3 Piège numérique à anticiper

Quand `v_vehicle → 0` (voiture quasi-arrêtée), on divise par presque zéro. **Toujours** protéger :

```c
if (v_vehicle < V_MIN_FOR_SLIP) {     // ex: 1.0 m/s
    return 0.0f;                       // slip non défini → on retourne 0
}
float slip = (v_vehicle - omega_wheel * R) / v_vehicle;
```

C'est exactement le **mode de défaillance F06** dans `docs/FMEA.md` (à venir).

---

## 3. La courbe magique : μ(slip) selon Pacejka

### 3.1 Qu'est-ce que μ ?

`μ` = **coefficient d'adhérence** entre le pneu et la route. C'est ce qui détermine combien de force longitudinale (freinage) le pneu peut transmettre. La force de freinage au sol vaut :

```
F_frein_sol = μ(s) × m × g / 4    (par roue, sur 4 roues identiques)
```

`μ` n'est **pas une constante** — il dépend du slip ratio. C'est le cœur de toute la physique des pneus.

### 3.2 Forme typique de la courbe

```
   μ
   │
1.0│             ★ pic (slip ≈ 0.15)
   │           ╱   ╲
0.8│         ╱       ╲
   │        ╱          ─────────  zone glissement (s → 1)
0.6│       ╱
   │     ╱
0.4│   ╱
   │ ╱
0.2│╱
   │
   └──────────────────────────────► slip ratio s
   0   0.1  0.2  0.4  0.6  0.8  1.0
```

**Lecture de la courbe :**
- Au début (`s` petit), `μ` monte presque linéairement → plus on freine, plus on freine (logique).
- Vers `s ≈ 0.15` : **pic** d'adhérence — c'est la zone qu'on vise.
- Au-delà : `μ` **redescend** → on freine *moins bien* en bloquant plus. C'est le piège.
- À `s = 1` (roue bloquée) : `μ` ≈ 0.5–0.7 × pic, sur sec. Sur verglas, beaucoup moins.

### 3.3 Formule de Pacejka simplifiée — la « Magic Formula »

La formule complète a 10+ coefficients empiriques. Pour notre projet on prend la version simplifiée à 3 paramètres :

```
μ(s) = D × sin( C × atan( B × s ) )
```

avec :
- `B` ≈ 10  (stiffness factor — pente initiale)
- `C` ≈ 1.9 (shape factor)
- `D` ≈ 1.0 (peak factor — pic d'adhérence sur sec, ≈ 0.1 sur verglas)

**Vérification rapide** (à faire en Python, jour 3) :
```python
import numpy as np, matplotlib.pyplot as plt
s = np.linspace(0, 1, 500)
mu = 1.0 * np.sin(1.9 * np.arctan(10 * s))
plt.plot(s, mu); plt.xlabel("slip ratio"); plt.ylabel("μ"); plt.grid()
```
Tu dois voir un pic vers `s ≈ 0.15`, μ_max ≈ 1.0, puis chute vers ≈ 0.65 à s=1.

### 3.4 Les surfaces

Pour les scénarios de test, on changera `D` :

| Surface | D (μ_max) |
|---|---|
| Asphalte sec | 1.0 |
| Asphalte mouillé | 0.7 |
| Neige tassée | 0.3 |
| Verglas | 0.1 |

Le **fault injector** (semaine 3) basculera `D` en plein freinage → scénario « plaque de verglas ».

---

## 4. Les équations du mouvement — modèle bicycle simplifié

On modélise un **quart de véhicule** (une roue, 1/4 de la masse). C'est l'approximation standard en simulation ABS et c'est largement suffisant pour notre cas.

### 4.1 Variables d'état

| Symbole | Nom | Unité | Notation code |
|---|---|---|---|
| `v` | Vitesse longitudinale véhicule | m/s | `v_vehicle` |
| `ω` | Vitesse angulaire roue | rad/s | `omega_wheel` |
| `T_b` | Couple de freinage appliqué | N·m | `brake_torque` |

### 4.2 Constantes

| Symbole | Valeur typique | Unité | Notation code |
|---|---|---|---|
| `m` | 400 (quart) ou 1500 (total) | kg | `mass` |
| `R` | 0.30 | m | `wheel_radius` |
| `I` | 1.2 | kg·m² | `wheel_inertia` |
| `g` | 9.81 | m/s² | `G` |

### 4.3 Équation 1 — dynamique longitudinale du véhicule (Newton)

Le sol pousse la voiture *vers l'arrière* via la force de friction longitudinale. Cette force vaut `μ(s) × m × g / N_roues`, mais pour un modèle quart-véhicule on simplifie en :

```
        dv         μ(s) × m × g
  m × ──── = − ─────────────────
        dt              1
```

Soit, par roue :

```
  dv/dt = − μ(s) × g
```

**Note :** on néglige la résistance aérodynamique (`½ρCxAv²`) et la résistance au roulement (`Crr × m × g`). À 100 km/h, l'aéro représente ~10% du freinage max — acceptable pour un projet pédagogique. Si tu veux le rajouter c'est trivial (3 lignes), c'est bonus.

### 4.4 Équation 2 — dynamique de rotation de la roue

Deux couples agissent sur la roue :
- Le **couple moteur depuis la route** (positif quand la roue freine) : `+ μ(s) × m × g × R`
- Le **couple de freinage** (commandé par l'ECU via la pression hydraulique, négatif) : `− T_b`

D'où :

```
        dω
  I × ──── = μ(s) × m × g × R − T_b
        dt
```

### 4.5 Lien pression de frein ↔ couple

Approximation linéaire (suffisante) :

```
  T_b = K_b × P
```

avec `K_b` ≈ 1.0 N·m/bar (à calibrer pour que la pression max ~150 bar produise un couple cohérent qui bloque effectivement la roue).

### 4.6 Le système couplé — ce qu'on intégrera

À chaque pas `dt = 0.01 s` :

```
1. Calculer s    = (v − ω·R) / v             [protégé contre v→0]
2. Calculer μ    = D · sin(C · atan(B·s))
3. Calculer F    = μ · m · g
4. Calculer T_b  = K_b · P_brake             [P_brake vient de l'ECU]
5. dv/dt         = − F / m                   [= −μ·g]
6. dω/dt         = (F·R − T_b) / I
7. v_new         = v   + dv/dt · dt
8. ω_new         = ω   + dω/dt · dt          [clamp à >= 0]
```

C'est l'**intégration d'Euler explicite**. Simple, ça marche pour `dt ≤ 10 ms`. Pour `dt` plus grand on prendrait Runge-Kutta 4, hors scope ici.

---

## 5. Validation — ordres de grandeur à vérifier

Avant de connecter le contrôleur C, le plant Python doit produire **ces résultats** :

### Scénario A — freinage maximum sans ABS (pédale écrasée)

| Mesure | Valeur attendue |
|---|---|
| Vitesse initiale | 100 km/h = 27.78 m/s |
| Pression frein | constante = 100 bar |
| Temps de blocage de la roue | ~0.2 s après le début du freinage |
| Distance d'arrêt | ~60 m |
| Slip ratio en fin de freinage | `= 1` (roue bloquée) |

### Scénario B — freinage avec « ABS oracle » bang-bang

Un contrôleur trivial qui maintient `s ∈ [0.10, 0.20]` manuellement.

| Mesure | Valeur attendue |
|---|---|
| Distance d'arrêt | ~45 m |
| Slip ratio moyen | ~0.15 |
| Gain vs sans ABS | ~25% de distance gagnée |

**Si ces deux ordres de grandeur ne sortent pas, c'est qu'il y a un bug dans le plant** — surtout, on le corrige avant de toucher au C.

---

## 6. Vocabulaire à connaître par cœur (entretien)

| Terme FR | Terme EN | Définition courte |
|---|---|---|
| Taux de glissement | Slip ratio | `(v − ωR)/v` |
| Adhérence | Grip / friction coefficient | `μ`, fonction du slip et de la surface |
| Bang-bang | Bang-bang / hysteresis control | Commande binaire avec deux seuils |
| Pic d'adhérence | Peak friction | Maximum de μ(s), vers s≈0.15 |
| Quart-véhicule | Quarter-car model | Simplification à une roue / m/4 |
| Magic Formula | Pacejka formula | Modèle empirique de pneu |
| Friction longitudinale | Longitudinal friction | Force de freinage le long de l'axe de la voiture |

---

## 7. Pour aller plus loin (optionnel)

- **Rajesh Rajamani** — *Vehicle Dynamics and Control* (Springer, 2nd ed.) → chapitre 8 sur l'ABS, chapitre 13 sur le modèle de pneu. C'est *la* référence académique.
- **Hans Pacejka** — *Tire and Vehicle Dynamics* (3rd ed., Butterworth-Heinemann) → la bible de la modélisation pneu, plus avancé.
- **YouTube : Lesics — "How does ABS work"** → 15 min, animations claires, vulgarisation très propre.
- **Wikipedia EN** : *Anti-lock braking system*, *Slip ratio*, *Hans B. Pacejka*.

---

## 8. Ce qu'il faut retenir pour la suite

1. L'ABS sert à garder `s ∈ [0.10, 0.20]` pour rester sur le **pic** de la courbe μ.
2. Le slip ratio se calcule à partir de `v` et `ω·R`. **Protéger v→0**.
3. La courbe Pacejka simplifiée : `μ = D·sin(C·atan(B·s))`, `B=10, C=1.9, D=1.0`.
4. Deux équations différentielles couplées : `dv/dt = −μg`, `dω/dt = (μgmR − T_b)/I`.
5. Intégration Euler explicite à `dt = 10 ms`.
6. Validation : sans ABS ~60 m, avec ABS bang-bang ~45 m.

Le plant Python du jour 3-4 implémente exactement ces 6 points. Pas plus.
