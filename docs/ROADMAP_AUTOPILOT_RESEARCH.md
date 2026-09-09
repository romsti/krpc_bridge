# Audit de la roadmap Gemini et feuille de route « autopilote maximal »

**Projet :** `krpc_bridge`  
**Date de l'audit :** 9 septembre 2026  
**Périmètre :** KSP 1.12.5, kRPC 0.6.0 et les mods détectés dans l'installation locale  
**Statut du document :** recherche et proposition d'architecture, pas spécification d'API figée

## Résumé exécutif

La roadmap Gemini était globalement bonne sur l'architecture : séparer un cœur robuste de plugins optionnels, isoler les erreurs de compatibilité au build, déplacer les calculs lourds hors du thread Unity et fournir un mécanisme de découverte par réflexion. Une part importante de ce socle a effectivement été réalisée dans `krpc_bridge`.

En revanche, ses priorités fonctionnelles doivent être profondément révisées :

1. Elle raisonnait à partir de kRPC 0.5.4. Or kRPC 0.6.0 expose déjà les ranges de physique, le delta-v par étage, le tenseur d'inertie, les couples disponibles par famille d'actionneurs, les commandes directes des moteurs/gimbals/RCS/surfaces et surtout la simulation de la force **et du couple** aérodynamiques à un état arbitraire. Plusieurs anciens « plugins prioritaires » seraient donc aujourd'hui des doublons.
2. Elle sous-estimait le vrai problème d'un autopilote externe : la cohérence temporelle. Lire 50 propriétés puis envoyer 20 RPC séparés ne garantit ni un échantillon issu du même tick physique, ni l'application simultanée des commandes.
3. Elle ne décrivait pas suffisamment la dynamique des actionneurs : montée et descente de poussée, vitesse de gimbal, limites asymétriques, délais/allumages, saturation, débit réel, géométrie par tuyère et autorité instantanée.
4. Elle proposait des briques de données mais pas la matrice qui les relie aux six degrés de liberté. Un autopilote avancé a besoin d'une **matrice d'efficacité 6×N**, de bornes, de vitesses de variation et d'un solveur d'allocation de contrôle.
5. Elle n'avait pas identifié Throttle Controlled Avionics (TCA), pourtant installé localement. TCA contient déjà un allocateur moteur/RCS, des modèles de moteurs lents, un radar et plusieurs contrôleurs utiles comme référence ou backend optionnel.

La priorité recommandée n'est donc pas « ajouter le plus de propriétés possible ». Elle est de construire un chemin déterministe :

```text
KSP + mods
    │  un instant physique cohérent
    ▼
Snapshot atomique, horodaté, versionné
    │
    ├── estimation d'état et identification
    ├── prédiction aéro / terrain / propulsion
    └── guidage et allocation 6-DoF
                │  une trame de commande cohérente
                ▼
       Application atomique au FixedUpdate
                │
                └── retour de commande réalisée, saturation et défauts
```

Les trois lots à plus forte valeur sont donc :

- **P0 — `Actuators v2` :** snapshot et commande atomiques, géométrie par effecteur, dynamique moteur/gimbal, accusé d'application et watchdog.
- **P1 — autorité 6-DoF :** matrice d'efficacité, enveloppe réalisable, allocation contrainte et retour de force/couple réellement obtenu.
- **P2 — prédiction et sécurité :** oracle aérodynamique kRPC 0.6.0, dérivées locales, terrain multi-rayons, prédicteur propulsé, enregistreur/rejeu et superviseur de sécurité.

## 1. Méthode, preuves et limites

L'analyse croise quatre types de preuves :

- le dépôt actuel et son historique Git ;
- le document Gemini retrouvé comme objet Git non référencé ;
- les DLL réellement installées dans KSP, inspectées par métadonnées et décompilation en lecture seule ;
- les documentations et dépôts officiels de kRPC, KSP/mods et quelques références NASA de GNC.

Le plan original a été retrouvé dans le blob Git :

```text
80b93b2d1f8703a77a00aa4eb63b02be5db3f453
docs/REFACTOR.md
Titre : krpc_bridge : architecture Core & Plugins, et catalogue de fonctionnalités
Date interne : 27 juillet 2026
```

Il indiquait lui-même n'avoir vérifié en ligne que kRPC 0.5.4 et ne pas avoir eu accès aux DLL de mods. Cette réserve explique une grande partie des écarts constatés.

Versions vérifiées dans l'installation locale :

| Composant | Version |
|---|---:|
| kRPC SpaceCenter | 0.6.0.0 |
| MechJeb | 2.15.3.0 |
| Trajectories | 2.4.5.4 |
| SCANsat | 21.1 |
| TCA | 3.8.0.1 |
| Kerbal Engineer Redux | 1.1.9.5 |

Les détails internes de KSP et des mods ne sont pas des contrats stables. Toute nouvelle intégration par réflexion doit donc être : détectée par capacité, versionnée, testée au chargement, et désactivée proprement si un membre manque.

## 2. Ce que proposait réellement la roadmap Gemini

### 2.1 Architecture

Le plan proposait :

- un `Core` sans dépendance optionnelle et un assembly par plugin ;
- quatre mécanismes distincts : `Wait`, `Jobs`, `MainThread` et `EventBus` ;
- une isolation build-time, car une signature RPC invalide peut empêcher le service kRPC de démarrer ;
- des RPC groupés et plats, peu d'allocations et aucune réflexion répétée dans les boucles chaudes ;
- un utilitaire `DescribeType` pour remplacer les suppositions de compatibilité par de l'introspection réelle.

Cette direction était pertinente et reste valable. Le dépôt actuel possède le cœur, l'EventBus/les jobs et le service `Discovery`, dont les fonctions vont déjà au-delà du `DescribeType` imaginé : recherche d'assemblies et de types, description des membres, hiérarchie, recherche de `PartModule` et empreinte de type à l'exécution.

### 2.2 Catalogue fonctionnel d'origine

Le catalogue était large : ranges de physique, Krakensbane/origine flottante, terrain groupé, joints et structure, drag cubes/FAR, thermique, rendu, caméra, delta-v stock, enregistreur 60 Hz, vaisseaux déchargés, temps/packing, éditeur, R&D et Parallax ; puis Trajectories, MechJeb, RealFuels, SCANsat, Kerbalism, StageRecovery, FMRS/OCISLY, Principia, SystemHeat, kOS, Kerbal Konstructs et KER ; enfin télémétrie overlay, rejeu déterministe et accès générique typé aux `PartModule`.

Son ordre de priorité était :

1. `DescribeType` ;
2. delta-v stock ;
3. ranges de physique ;
4. enregistreur 60 Hz ;
5. Trajectories ;
6. champs génériques typés ;
7. terrain groupé ;
8. rendu polyligne ;
9. mathématiques MechJeb ;
10. Krakensbane/origine ;
11. caméra ;
12. StageRecovery ;
13. aéro stock/FAR ;
14. génération de craft dans l'éditeur ;
15. Principia.

### 2.3 État actuel de chaque proposition

Légende : **fait** = présent dans le dépôt ; **amont** = mieux fourni par kRPC 0.6.0 ; **partiel** = une partie existe ; **à faire** = valeur encore nette ; **optionnel** = faible effet direct sur le pilotage.

| Élément Gemini | État 2026 | Décision recommandée |
|---|---|---|
| Architecture Core + plugins | **Fait** | Conserver |
| Wait / Jobs / MainThread / EventBus | **Fait** | Conserver, instrumenter |
| Isolation et scanner de build | **Fait** | En faire une barrière CI obligatoire |
| `DescribeType` | **Fait**, devenu `Discovery` | Ajouter des profils de capacités, pas plus de réflexion brute |
| Identifiants stables | **Fait** via `Ident` | Ajouter génération de topologie et identité de transform |
| Delta-v stock | **Amont** | Ne pas dupliquer : `Vessel.stages`, `delta_v`, `burn_time` |
| Ranges de physique | **Amont** | Ne pas dupliquer : `Vessel.physics_range` |
| Trajectories | **Fait** | Garder comme témoin balistique/aéro non propulsé |
| MechJeb | **Fait**, très étendu | Garder comme oracle/backend, éviter les conflits de contrôle |
| FMRS et OCISLY | **Fait** | Hors boucle GNC principale |
| Échantillons moteur/gimbal | **Partiel**, ajout post-roadmap | Refaire en `Actuators v2` atomique et par effecteur |
| Drag cubes / aéro stock | **Partiellement amont** | Utiliser d'abord `simulate_aerodynamic_wrench_at`; exporter les cubes pour audit/offline |
| FAR état arbitraire | **Amont** via le même appel kRPC | Ajouter seulement les diagnostics/limitations FAR manquants |
| Couples, inertie | **Amont** au niveau vaisseau/partie | Ajouter les données dynamiques et le mapping par actionneur |
| Enregistreur 60 Hz | **Partiel** pour l'AutoPilot kRPC | Créer un recorder général multi-service |
| Terrain groupé/collider | **À faire** | Très haute valeur pour atterrissage et rover |
| Krakensbane/origine flottante | **À faire** | Exposer surtout les événements et repères, pas les détails sans usage |
| Joints/structure | **À faire** | Ajouter charges, marges et modes flexibles observés |
| Champs génériques `PartModule` | **À faire** | Utile comme secours, mais derrière des adaptateurs typés |
| SCANsat | **À faire** | Valeur mission/site et cartes, pas vérité locale de collision |
| Télémétrie overlay | **À faire** | Après snapshot/recorder ; réutiliser le même schéma |
| Rejeu déterministe | **À faire** | Priorité élevée pour régression, même si KSP n'est pas parfaitement déterministe |
| Thermique | **À faire** | Exposer marges et prédiction à court terme |
| Vaisseaux déchargés/save | **À faire** | Utile pour flotte, peu utile à la boucle rapide |
| Rendu/caméra/éditeur/R&D | **Optionnel** | Ne pas les mélanger à la roadmap autopilote |
| RealFuels/Kerbalism/Principia/etc. | **Optionnel par installation** | Plugins séparés, activés par capacités |

## 3. Ce que kRPC 0.6.0 rend déjà disponible

La documentation actuelle de kRPC doit être considérée comme la base minimale avant toute extension. `krpc_bridge` ne devrait pas refléter une seconde fois ce qui y existe déjà.

### 3.1 État mécanique et autorité

kRPC expose désormais : masse, centre de masse, moment principal, tenseur d'inertie 3×3, couple disponible et accélération angulaire disponible. Les contributions peuvent être séparées entre roues de réaction, RCS, gimbals, surfaces de contrôle et autres actionneurs. La même API expose les étages, leur delta-v et leur durée de combustion. [Documentation officielle `Vessel`](https://krpc.github.io/krpc/latest/python/api/space-center/vessel.html)

Conséquence : un plugin « inertie/couple total » ou « delta-v » n'a plus de valeur. La vraie valeur ajoutée est de relier chaque actionneur individuel à une variation de force/couple, avec ses bornes et sa dynamique.

### 3.2 Aérodynamique à un état arbitraire

`Flight.simulate_aerodynamic_wrench_at` prend un corps, une position, une vitesse, une rotation, une vitesse angulaire et un UT, puis renvoie force et couple autour du centre de masse. Le calcul stock utilise les parties, drag cubes et états de surfaces actuels. Avec FAR, certaines entrées sont ignorées : cette différence doit être signalée explicitement au client. [Documentation officielle `Flight`](https://krpc.github.io/krpc/latest/python/api/space-center/flight.html#SpaceCenter.Flight.simulate_aerodynamic_wrench_at)

Conséquence : recréer tout le modèle stock en Python n'est plus la priorité. Le bridge doit plutôt fournir :

- une évaluation **batch** pour amortir les RPC ;
- des dérivées locales et une mesure de confiance ;
- un export brut des drag cubes pour analyse, rejeu ou approximation offline ;
- la provenance du modèle (`stock`, `FAR`, autre) et les dimensions d'état réellement prises en compte.

### 3.3 Actionneurs déjà pilotables

kRPC permet déjà le throttle indépendant par moteur, l'override gimbal en trois axes, les overrides RCS et de surfaces, ainsi que les couples/forces disponibles correspondants. Les overrides directs sont relâchés à la déconnexion du client ou au changement de vaisseau. [Documentation officielle `Parts`](https://krpc.github.io/krpc/latest/python/api/space-center/parts.html)

Les leases courts de `krpc_bridge` restent utiles comme watchdog plus strict. Mais les RPC actuels du bridge ne doivent pas devenir une API concurrente moins riche. Ils devraient évoluer vers des commandes atomiques, dotées d'un propriétaire, d'une échéance, d'une génération et d'un accusé d'application.

### 3.4 AutoPilot kRPC devenu beaucoup plus complet

L'AutoPilot kRPC 0.6.0 offre cibles de rotation/direction/roll, lissage et soft-start, limites de vitesse angulaire, autotuning, réglages temporels/PID, détection de modes oscillatoires, filtres notch/passe-bas, réductions de bande passante/feed-forward/sortie et un journal complet par tick, limité à 3 000 échantillons. [Documentation officielle `AutoPilot`](https://krpc.github.io/krpc/latest/python/api/space-center/auto-pilot.html)

Conséquence : avant d'écrire un nouveau contrôleur d'attitude, il faut instrumenter et benchmarker celui-ci. Un contrôleur externe se justifie surtout pour :

- le couplage translation–rotation ;
- les allocations asymétriques ou sous-actionnées ;
- le contrôle moteur par moteur ;
- le MPC/contraintes trajectoire ;
- les pannes, limites thermiques et ressources ;
- les véhicules atypiques et multi-corps.

## 4. Lacunes du service `Actuators` actuel

Le service actuel a apporté une capacité essentielle : échantillons groupés moteur/gimbal, throttle indépendant sous lease, gimbal direct sous lease et géométrie de poussée. C'est le bon domaine, mais pas encore un contrat suffisant pour un autopilote de haute précision.

### 4.1 Perte de géométrie

`ThrustDirectionSample` et `ThrustPositionSample` résument séparément plusieurs transforms de poussée. Pour un moteur multi-tuyères, cette réduction perd la correspondance position–direction–fraction de poussée. Elle empêche de reconstruire exactement :

\[
F = \sum_i f_i d_i, \qquad
\tau = \sum_i (r_i-r_{CoM}) \times (f_i d_i)
\]

Le contrat v2 doit publier une ligne **par transform de poussée**, avec un même identifiant d'effecteur, sa position, sa direction, son multiplicateur et le module moteur parent.

### 4.2 Dynamique moteur absente

La DLL KSP 1.12.5 vérifiée contient notamment `requestedThrottle`, `currentThrottle`, `finalThrust`, `requestedMassFlow`, `propellantReqMet`, `realIsp`, `useEngineResponseTime`, `engineAccelerationSpeed` et `engineDecelerationSpeed`. Le mode alternatif ajoute un `throttleResponseRate` et plusieurs multiplicateurs de démarrage/arrêt.

Le modèle stock standard interpole la commande avec une vitesse différente à la montée et à la descente. Un autopilote qui ne connaît que la poussée maximale suppose donc à tort que la poussée est instantanée. Il faut exposer à la fois :

- commande demandée, throttle interne et poussée effectivement réalisée ;
- paramètres de montée/descente et modèle de réponse détecté ;
- temps dérivés `rise_time_90` et `fall_time_10`, mesurés si la sémantique du module n'est pas connue ;
- poussée minimale, limite de poussée, délai d'allumage, possibilité d'arrêt/redémarrage ;
- débit demandé/réalisé, satisfaction en ergols et cause de flameout ;
- mode moteur, inverseur, vecteurs de poussée et dépendance pression/vitesse.

Pour RealFuels/SolverEngines, ajouter allumages restants, stabilité d'ullage, pression/alimentation, spool et contraintes propres au moteur, sans prétendre qu'un champ générique équivaut à un contrat typé. RealFuels documente explicitement l'ullage, les allumages limités et le throttle response. [Dépôt officiel RealFuels](https://github.com/KSP-RO/RealFuels) · [source SolverEngines](https://github.com/KSP-RO/SolverEngines/blob/master/SolverEngines/EngineModule.cs)

### 4.3 Dynamique gimbal absente

`ModuleGimbal` contient des limites directionnelles `XP/YP/XN/YN`, des axes activables, des multiplicateurs, `useGimbalResponseSpeed`, `gimbalResponseSpeed`, les rotations initiales et les actuations cible/réalisée. Sa mise à jour applique une interpolation par tick quand la réponse finie est active.

Le bridge doit exposer :

- limites positives/négatives par axe et par transform ;
- axes réellement disponibles, inversions et mapping local→vaisseau ;
- commande cible, position réalisée et vitesse maximale ;
- marge avant saturation et dérivée force/couple par unité de commande ;
- trois composantes normalisées, pas seulement deux ;
- état lock/limiter/override et motif d'indisponibilité.

### 4.4 Absence d'atomicité

Chaque RPC actuel peut être traité sur un tick différent. C'est dangereux pour une commande multi-moteurs : une demi-trame peut créer un couple transitoire très supérieur à la commande voulue.

Il faut deux primitives centrales :

```text
read_control_snapshot(vessel_id, schema_version)
apply_control_frame(vessel_id, generation, sequence, valid_until_tick, values...)
```

Le snapshot doit contenir `physics_tick`, `UT`, `fixed_delta_time`, `topology_generation`, tous les états demandés et le numéro de la dernière commande appliquée. La trame doit être validée en entier, mise en attente, puis appliquée en une fois au début d'un `FixedUpdate`. Si elle est invalide, périmée ou liée à une ancienne topologie, rien ne doit être appliqué.

## 5. Architecture cible

### 5.1 Ce qui doit vivre dans le bridge

Le bridge est le bon endroit pour :

- lire un état cohérent du monde Unity/KSP ;
- accéder aux APIs internes ou aux mods non exposés par kRPC ;
- appliquer des commandes atomiques dans le thread physique ;
- imposer watchdogs, propriétaires et politiques de sécurité locales ;
- effectuer des calculs batch courts qui évitent des milliers de RPC ;
- enregistrer les données brutes au rythme physique.

Il ne devrait pas contenir par défaut le guidage de mission, l'optimisation de trajectoire longue durée ou le planificateur stratégique. Ces algorithmes évoluent plus vite et se testent mieux côté client Python/C++.

Exception : un allocateur ou une boucle intérieure qui doit impérativement agir chaque `FixedUpdate` peut vivre en C#, à condition d'accepter des consignes de haut niveau et de publier tous ses diagnostics.

### 5.2 Contrats de données

Chaque contrat à haut débit devrait respecter les règles suivantes :

- **version explicite** du schéma et taille/stride annoncés ;
- tableaux plats ou `byte[]` versionné, pas une forêt de `KRPCClass` ;
- `NaN` seulement avec un masque de validité et un code de raison ;
- repère, unités et convention d'axes dans le contrat ;
- tick/UT communs, génération de topologie et empreinte de capacités ;
- identifiants stables pour partie, module et transform ;
- aucune réflexion, recherche de composant ou allocation importante dans la boucle chaude ;
- mesure du coût CPU et du temps de sérialisation.

### 5.3 Propriété et arbitrage

Plusieurs pilotes peuvent se battre : kRPC AutoPilot, MechJeb, TCA, SAS stock et client externe. Le bridge doit rendre ce conflit impossible ou au moins visible.

Proposition : un `ControlAuthorityService` attribue des domaines (`attitude`, `translation`, `engines`, `gimbals`, `RCS`, `surfaces`, `staging`) à un propriétaire avec priorité, génération et échéance. Une acquisition refusée indique le propriétaire actuel. Une préemption explicite génère un événement et apparaît dans le recorder.

## 6. Catalogue complet des capacités à ajouter

### 6.1 Capteurs et état commun

| Capacité | Utilité GNC | Lieu recommandé | Priorité |
|---|---|---|---:|
| Snapshot vaisseau atomique | Élimine le skew entre position, attitude, masse et actionneurs | Core/FlightFrame | P0 |
| Tick, UT, `fixedDeltaTime`, warp et pause | Synchronisation/intégration | Core | P0 |
| Position/vitesse/attitude/vitesse angulaire/accélération dans un repère fixé | État 6-DoF non ambigu | Snapshot | P0 |
| Masse, CoM, tenseur d'inertie complet et dérivées | Contrôle et feed-forward | Snapshot, à partir de kRPC | P0 |
| Topologie, arbre de parties, staging, docking et événements de destruction | Invalidations propres | Ident/Topology | P0 |
| Force/couple externes réalisés par famille | Identification et contrôle robuste | Snapshot | P1 |
| Contact sol par roue/jambe/partie, normale, glissement, compression | Touchdown, rover, hop | Terrain/Contact | P1 |
| Pression dynamique, Mach, AoA, sideslip, force/couple aéro | Contraintes et estimation | kRPC + batch | P1 |
| Températures, flux, marges de rupture/ablation | Protection thermique | Thermal | P2 |
| Électricité, ressources accessibles, débit et horizon d'épuisement | Faisabilité | Resources | P2 |
| Signal/comms/délai et statut de contrôle | Missions avec RemoteTech/CommNet | Comms | P3 |
| État Krakensbane/origine flottante et événements de shift | Dérivées/rejeu sans sauts | Frames | P2 |
| Charges de joints, force/couple de rupture, contraintes | Protection structurelle | Structure | P2 |

### 6.2 Actionneurs

#### Moteurs

- état complet par moteur et par transform de poussée ;
- commande indépendante, limite de poussée, activate/shutdown, mode et inverseur ;
- throttle demandé/interne/réalisé et force réelle ;
- rampes montée/descente, retard, allumages, ullage, spool et débit ;
- courbes poussée/Isp selon pression, Mach et mode, évaluables en batch ;
- panne/dégradation, disponibilité, ressource limitante et temps avant extinction ;
- dérivée locale du wrench par rapport au throttle et au gimbal.

#### Gimbals

- géométrie et état par transform ;
- limites directionnelles asymétriques, axes actifs et vitesse de réponse ;
- cible, réalisation, saturation et commande directe 3D ;
- lease avec propriétaire/échéance et retour d'application.

#### RCS

- ligne par tuyère : position, direction, force, ressource, ISP, état et module parent ;
- distinction rotation/translation, précision, limite de poussée et disponibilité ;
- overrides atomiques, soit par module, soit par groupe d'effecteurs ;
- autorité 6-DoF réelle tenant compte des réservoirs, de la CoM et des blocages ;
- modes « translation sans couple », « couple sans translation » et minimum de consommation.

#### Roues de réaction

- couple positif/négatif, authority limiter, saturation logique et puissance requise ;
- estimation du couple réellement appliqué ;
- contribution à la matrice d'autorité ;
- commande par groupe seulement si KSP permet un contrat stable ; sinon l'autorité globale kRPC reste la primitive.

#### Surfaces aérodynamiques

- position, axe de charnière, aire, plage et rôle pitch/yaw/roll ;
- override, angle cible/réalisé, `actuatorSpeed`, loi exponentielle et saturation ;
- dérivée locale du wrench via perturbations de `simulate_aerodynamic_wrench_at` ;
- efficacité variable avec Mach/q/AoA et marge au décrochage.

#### Autres effecteurs

- aérofreins/spoilers et surfaces déployables ;
- parachutes : état, risque, pression/altitude de déploiement, temps d'ouverture ;
- jambes, roues, direction, freinage, friction et moteurs de rover ;
- ports d'amarrage, aimants/grappins et acquisition ;
- action groups et staging atomique conditionnel ;
- robotics/servos, hélices et rotors si Breaking Ground est présent ;
- ballast, pompes et transfert de ressources pour véhicules exotiques.

### 6.3 Matrice d'efficacité et allocation de contrôle

Pour N variables de commande, publier une matrice locale :

\[
B = \frac{\partial [F_x,F_y,F_z,\tau_x,\tau_y,\tau_z]}{\partial u}
\quad \in \mathbb{R}^{6\times N}
\]

avec, pour chaque colonne : bornes, vitesse de variation, zone morte, coût d'ergol/électricité, disponibilité, groupe et confiance. Le service doit aussi publier rang, conditionnement, directions non contrôlables et enveloppe réalisable approximative.

Deux niveaux sont utiles :

1. **lecture seule** : le client reçoit `B`, résout son propre QP/WLS puis envoie une trame brute ;
2. **allocateur embarqué** : le client demande un wrench et des poids, le bridge applique une solution au même tick.

La fonction objectif minimale est :

\[
\min_u \|W(Bu-w_{cmd})\|^2 + \lambda\|R(u-u_{prev})\|^2
\]

sous bornes, slew rates, groupes exclusifs, allumages et ressources. Il faut renvoyer le wrench demandé, le wrench prévu, le résidu, les saturations et le statut du solveur. Le contrôle allocation du SLS utilise lui aussi les positions moteurs, propriétés de masse et profils de poussée, avec une allocation pondérée adaptée en temps réel. [NASA, *Space Launch System Ascent Flight Control Design*](https://ntrs.nasa.gov/api/citations/20140008731/downloads/20140008731.pdf)

### 6.4 Aérodynamique et drag cubes

Trois niveaux complémentaires :

1. **Oracle live kRPC** : batch de wrenchs hypothétiques, source de vérité prioritaire.
2. **Dérivées/stabilité** : différences centrales autour de l'état pour produire `dF/dα`, `dM/dα`, `dM/dq`, dérivées de commandes, point de trim, marge statique, pression dynamique limite et confiance numérique.
3. **Export drag cubes** : par partie, publier `Area[6]`, `Drag[6]`, `Depth[6]`, centre, taille, poids/blend, modificateurs, occlusion et état procedural. Ce niveau sert à l'audit, aux surrogates offline et au rejeu ; il ne doit pas devenir une réimplémentation fragile du modèle stock.

Le batch devrait accepter un tableau de `(position, velocity, rotation, angular_velocity, UT, control_state)` et renvoyer wrench, modèle utilisé et flags. Pour FAR, indiquer que la vitesse angulaire/UT ne participent pas de la même manière selon l'API kRPC actuelle. Le dépôt FAR reste la source de compatibilité primaire. [Dépôt officiel FAR](https://github.com/ferram4/Ferram-Aerospace-Research)

### 6.5 Terrain, contact et choix de site

Un seul `surface_height(lat, lon)` ne suffit pas pour un atterrissage autonome. Ajouter :

- raycasts batch depuis un cône/empreinte configurable ;
- point, distance, normale de collider, partie/scatter touché et matériau ;
- grille locale orientée, plan ajusté, pente, rugosité, courbure et marches ;
- clearance sous moteurs, jambes et volume englobant ;
- vitesse/altitude terrain-relative et temps au contact par point bas ;
- recherche de zones sûres avec rayon minimal, pente/roughness maximales et coût de divert ;
- cache multi-résolution et invalidation lors d'un changement de corps/scene/origine.

SCANsat peut fournir cartes, couverture, anomalies, biomes, ressources et topographie à l'échelle mission. Il ne doit pas remplacer les raycasts/colliders de la phase terminale. [Dépôt officiel SCANsat](https://github.com/KSPModStewards/SCANsat)

Cette séparation suit une architecture éprouvée : navigation relative au terrain, carte d'élévation locale, détection des dangers, sélection d'un site sûr puis guidage de divert. [NASA/JPL, ALHAT](https://robotics.jpl.nasa.gov/what-we-do/research-tasks/alhat-autonomous-landing-and-hazard-avoidance-technology/) · [NASA, SPLICE](https://www.nasa.gov/safe-and-precise-landing-integrated-capabilities-evolution-splice/)

### 6.6 Trajectoire et guidage

Trajectories est utile mais son propre dépôt précise que les futurs étages et les parachutes ne sont pas simulés et que la prédiction vise le vaisseau courant. [Dépôt officiel Trajectories](https://github.com/neuoy/KSPTrajectories)

Il faut conserver son impact comme **witness** indépendant, puis ajouter un prédicteur propulsé qui accepte :

- un calendrier throttle/gimbal/attitude ou wrench désiré ;
- réponse moteur et gimbal finie ;
- masse/CoM/inertie et consommation évolutives ;
- événements : staging, extinction, déploiement, limite thermique/q/charge ;
- atmosphère et wrench aéro arbitraire ;
- terrain ellipsoïde + topographie locale ;
- propagation d'incertitude ou au minimum scénarios min/nominal/max.

Le guidage de descente propulsée peut ensuite résoudre une trajectoire contrainte minimisant carburant ou erreur d'atterrissage. La convexification « lossless » est une référence solide pour gérer les bornes non convexes de throttle et fournir une solution minimum-erreur si la cible exacte est infaisable. [NASA NTRS, Acikmese & Blackmore](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20120001230.pdf)

Capacités de guidage à bâtir côté client au-dessus du bridge :

- montée : pitch program adaptatif, limites max-q/AoA/charge/thermique, cutoff précis ;
- insertion : PEG ou optimisation directe, gestion du throttle et des étages ;
- boostback/entry : ciblage d'impact, modulation de portance et contraintes thermiques ;
- landing burn : suicide burn robuste, convex/MPC, divert terrain ;
- rendez-vous/docking : CW/Lambert/non-linéaire, corridor et évitement collision ;
- attitude : feed-forward inertiel, contraintes gimbal/propellant slosh approximées ;
- rover/VTOL : suivi de terrain, vitesse sûre, anti-basculement.

### 6.7 Estimation d'état et délais

Même si KSP fournit un état « vrai », un autopilote réseau subit délai, jitter, ticks manqués et changements de repère. Ajouter au protocole :

- horloge/tick de capture et d'application ;
- estimation aller-retour et histogramme de jitter ;
- compteur de trames perdues/périmées ;
- prédiction courte à l'instant d'application ;
- possibilité d'injecter bruit, biais, retard et pertes pour les tests ;
- état estimé et covariance côté client, enregistrés avec l'état vrai.

La navigation d'atterrissage réelle combine altimètre, vitesse relative au sol, TRN et détection de dangers dans un filtre ; cette séparation état vrai/mesures/estimateur est utile même dans KSP pour tester un autopilote robuste. [NASA NTRS, estimateur ALHAT](https://ntrs.nasa.gov/citations/20170009197)

### 6.8 Structure, oscillations et sécurité d'enveloppe

Ajouter :

- inventaire des joints et corps rigides ;
- force/couple courants, break force/torque, marge et taux de marge ;
- flexion relative entre sections instrumentées ;
- fréquences dominantes et qualité de l'estimation ;
- limites configurables max-q, g, AoA, température, flux, joint et vitesse d'impact ;
- événements imminents et actions de repli.

Ne pas doubler la suppression d'oscillation déjà offerte par kRPC AutoPilot. Le recorder général doit toutefois enregistrer son détecteur, ses filtres et sorties pour distinguer oscillation structurelle, saturation et mauvais réglage.

### 6.9 Ressources, thermique et santé

Pour chaque ressource/actionneur :

- quantité/capacité accessible selon les règles de flux ;
- débit instantané demandé/réalisé ;
- horizon d'épuisement et ressource limitante ;
- consommation électrique et marge de production ;
- température, flux interne/convection/rayonnement, maxTemp et skinMaxTemp ;
- état nominal/dégradé/en panne, raison et temps depuis changement ;
- prédiction courte aux commandes prévues.

Un `SafetySupervisor` local doit pouvoir appliquer une politique simple si le client disparaît : expiration de la trame, throttle sûr, maintien ou libération d'attitude, sortie du warp, et éventuellement abort préconfiguré. La politique doit être explicite par mission, jamais implicite.

### 6.10 Enregistrement, rejeu et validation

Créer un recorder général, distinct du diagnostic AutoPilot :

- ring buffer en mémoire au rythme physique ;
- canaux définis par schéma, préalloués ;
- snapshots, commandes demandées/appliquées, événements et diagnostics solveur ;
- export binaire versionné + métadonnées JSON ;
- marqueurs de scene, staging, docking, reload, origin shift et topology generation ;
- mode trigger : N secondes avant et après anomalie ;
- streaming asynchrone hors thread Unity.

Le « rejeu déterministe » strict n'est pas garanti par Unity/KSP. Il faut viser trois niveaux :

1. rejeu de données pour visualisation et analyse ;
2. rejeu du contrôleur contre les snapshots enregistrés ;
3. ré-exécution KSP avec mêmes entrées et comparaison par tolérances/statistiques.

Tests indispensables : crafts canoniques mono/multi-moteurs, moteurs lents, gimbals asymétriques, RCS désaxé, véhicule flexible, staging en commande, panne d'ergol, chute réseau, FAR/TCA présents ou absents. Puis campagnes Monte-Carlo sur masse, aérodynamique, délai et perturbations ; NASA souligne aussi le rôle des simulations haute fidélité et Monte-Carlo pour valider la sélection de site et le GNC d'atterrissage. [NASA Science, Terrain Relative Navigation](https://science.nasa.gov/science-research/science-enabling-technology/technology-highlights/terrain-relative-navigation-landing-between-the-hazards/)

## 7. Intégrations de mods recommandées

### 7.1 TCA — priorité haute, absent du plan Gemini

L'assembly TCA 3.8.0.1 installé expose notamment `EngineWrapper`, `EngineOptimizer`, `RCSOptimizer`, `EnginesProps`, `Radar` et `LandingTrajectory`. Les membres inspectés couvrent géométrie de poussée, couple spécifique, throttle, accélération/décélération, temps 90/10 %, optimisation moteur/RCS et radar.

Le dépôt officiel décrit TCA comme un système de contrôle temps réel des limiteurs moteur/RCS pour obtenir poussée et couple, y compris avec moteurs lents, VTOL, hover et landing. [Dépôt et manuel TCA](https://github.com/allista/ThrottleControlledAvionics)

Plan d'intégration :

1. plugin read-only : disponibilité, modules, autorité, diagnostics et solutions TCA ;
2. backend optionnel d'allocation : consigne wrench/translation/attitude, résultat et résidu ;
3. contrôles TCA haut niveau seulement si un propriétaire unique est garanti ;
4. suite comparative : allocateur maison contre TCA sur les mêmes crafts.

TCA ne doit pas devenir une dépendance du Core. S'il est absent, `Actuators v2` et l'allocateur natif continuent de fonctionner.

### 7.2 MechJeb — oracle et backend de mission

Le plugin actuel expose déjà la montée, le landing predictor/autopilot, les nœuds, SmartASS, les opérations de manœuvre et une couche générique de modules/réglages. Les prochains ajouts utiles ne sont pas « encore plus de réflexion », mais :

- snapshots atomiques des sorties du prédicteur ;
- source/provenance et âge de la solution ;
- état de convergence, warnings et hypothèses ;
- arbitrage d'autorité lors d'un engage/disengage ;
- comparaison systématique MechJeb/Trajectories/prédicteur maison.

[Dépôt officiel MechJeb](https://github.com/MuMech/MechJeb2)

### 7.3 SCANsat — planification et cible

Exposer couverture par type de scan, cartes altitude/pente/biome/ressources, anomalies, waypoints et fraîcheur/résolution des données. Le sélecteur de site doit pouvoir exiger que l'information soit réellement scannée. En phase terminale, repasser aux raycasts locaux.

### 7.4 RealFuels / SolverEngines / TestFlight — propulsion réaliste

Créer un descripteur optionnel d'engine health : allumages, ullage, pression, fiabilité, panne, maintenance, throttle/spool, limites transitoires. Une commande impossible doit être rejetée avec raison et incluse dans l'enveloppe de contrôle.

### 7.5 FAR — aérodynamique

S'appuyer d'abord sur l'intégration FAR de kRPC. Ajouter les coefficients, conditions de validité, diagnostics de voxelisation et batchs seulement si l'API officielle réellement installée les permet. Éviter toute réflexion silencieuse dont l'échec renverrait des zéros plausibles.

### 7.6 Principia — mécanique orbitale

Plugin séparé fournissant repères, trajectoires, manœuvres et prédictions Principia avec détection stricte de version. Très forte valeur pour le guidage orbital complexe, mais coût de maintenance élevé et aucune raison de le placer avant la chaîne actionneur/snapshot.

### 7.7 Kerbalism / SystemHeat

Exposer les contraintes qui rendent une trajectoire ou un burn infaisable : puissance, refroidissement, radiateurs, ressources process, température et durée soutenable. Priorité mission/longue durée, pas boucle intérieure.

### 7.8 Parallax, Kerbal Konstructs, StageRecovery, kOS et KER

- **Parallax :** colliders/scatters uniquement dans la détection de dangers, avec API facultative.
- **Kerbal Konstructs :** pistes, pads, statiques et zones opérationnelles pour ciblage.
- **StageRecovery/FMRS/OCISLY :** coût/récupération et orchestration multi-véhicules, hors GNC temps réel.
- **kOS :** éventuellement transporter des consignes/événements, mais éviter deux boucles concurrentes.
- **KER :** source d'affichage/validation ; kRPC couvre déjà beaucoup de données mécaniques.

## 8. API proposée, par lots

Les noms sont illustratifs ; le format exact doit être prototypé et benchmarké.

### Lot P0 — cohérence et sécurité de commande

```text
Capabilities.describe_vessel(vessel_id) -> schema/version/topology/effectors
FlightFrame.read(vessel_id, channel_mask) -> one atomic snapshot
Control.acquire(vessel_id, domains, ttl, priority) -> owner_token/generation
Control.apply_frame(owner_token, sequence, apply_tick, valid_until_tick, values)
Control.last_result(owner_token) -> applied/rejected/reason/achieved
```

Critères d'acceptation : aucune demi-trame, rejet d'une ancienne topologie, expiration testée, libération sûre sur changement de vaisseau/scene, et coût p99 mesuré.

### Lot P1 — actionneurs et autorité 6-DoF

```text
Effectors.snapshot_v2(...) -> per-transform geometry/state/dynamics/bounds
Authority.matrix(...) -> B, bounds, slew, costs, rank, condition
Allocator.solve(...) -> controls/predicted_wrench/residual/status
Allocator.command_wrench(...) -> atomic scheduled application
```

Inclure moteur, gimbal, RCS, roues et surfaces. Commencer par moteur/gimbal, car c'est le besoin booster/landing le plus direct.

### Lot P2 — environnement et prédiction

```text
Aero.evaluate_batch(states, controls) -> wrench/model/validity
Aero.linearize(state, controls, steps) -> Jacobians/confidence
Terrain.raycast_batch(rays) -> hits/normals/materials
Terrain.evaluate_footprint(pose, geometry) -> slope/roughness/clearance/safe
Trajectory.propagate(initial_state, schedule, events, uncertainty)
```

### Lot P3 — observabilité et robustesse

```text
Recorder.configure(schema, pretrigger, posttrigger)
Recorder.start/mark/stop/export
Health.snapshot/events
Safety.configure/arm/disarm/status
```

### Lot P4 — adaptateurs

TCA, SCANsat, RealFuels/SolverEngines, FAR avancé, Principia, Kerbalism/SystemHeat, Parallax/Kerbal Konstructs. Chaque DLL optionnelle reste dans son propre assembly.

## 9. Roadmap recommandée

### Phase 0 — deux semaines : baseline et anti-doublons

- fixer officiellement kRPC 0.6.0 comme baseline ou détecter précisément chaque capacité ;
- cartographier toutes les propriétés natives utilisées ;
- documenter ce que `Actuators` ajoute réellement aux overrides kRPC ;
- définir unités, axes, IDs, erreurs, générations et règles de propriété ;
- ajouter benchmarks RPC/tick et crafts de test.

**Sortie :** ADR + schéma v2 + tests de compatibilité, sans nouvelle logique de vol.

### Phase 1 — trois à cinq semaines : `Actuators v2`

- snapshot atomique ;
- transform de poussée indivisible position+direction+poids ;
- dynamique moteur et gimbal ;
- trame atomique et planifiée ;
- token propriétaire, TTL, séquence et ack ;
- événements de topologie.

**Sortie :** contrôle moteur/gimbal sûr, mesurable et sans skew.

### Phase 2 — quatre à six semaines : autorité et allocation

- matrice 6×N analytique moteur/gimbal/RCS ;
- ajout roues/surfaces ;
- bornes, slew, rang et conditionnement ;
- WLS/QP, fallback déterministe et diagnostics ;
- adapter TCA read-only, puis backend expérimental ;
- tests de panne et comparaison TCA.

**Sortie :** consigne de wrench réalisable et commande coordonnée.

### Phase 3 — trois à cinq semaines : aéro et terrain

- batch de l'oracle kRPC 0.6.0 ;
- linéarisation et trim ;
- export drag cubes de diagnostic ;
- raycasts/grilles/normales/roughness/clearance ;
- SCANsat pour site global.

**Sortie :** état aérodynamique prédictif et landing-site score fiable.

### Phase 4 — quatre à huit semaines : prédiction propulsée

- intégrateur 3/6-DoF avec événements ;
- moteur/gimbal lents, consommation et staging ;
- contraintes q/thermique/structure/terrain ;
- scénarios d'incertitude ;
- comparaison Trajectories + MechJeb.

**Sortie :** prédicteur utilisable par boostback, entry et landing burn.

### Phase 5 — en parallèle puis obligatoire avant « production »

- recorder général et replay contrôleur ;
- superviseur de sécurité ;
- campagne Monte-Carlo ;
- fault injection réseau/actionneur/capteur ;
- budgets CPU, mémoire, bande passante et latence.

### Phase 6 — adaptateurs d'écosystème

Ajouter uniquement les plugins présents dans les profils de mission visés. TCA et SCANsat sont prioritaires dans l'installation actuelle ; RealFuels/FAR/Principia ne doivent être développés qu'avec leurs DLL et crafts de test disponibles.

## 10. Priorisation finale par valeur

| Rang | Capacité | Impact autopilote | Effort | Risque |
|---:|---|---:|---:|---:|
| 1 | Snapshot + commande atomiques | Très fort | Moyen | Moyen |
| 2 | Dynamique moteur/gimbal + géométrie par transform | Très fort | Moyen | Moyen |
| 3 | Propriété/TTL/génération/ack | Très fort | Faible–moyen | Faible |
| 4 | Matrice d'efficacité 6×N | Très fort | Élevé | Moyen |
| 5 | Allocation contrainte + achieved wrench | Très fort | Élevé | Élevé |
| 6 | Recorder/replay/fault injection | Très fort | Moyen | Faible |
| 7 | Batch aéro + dérivées locales | Fort | Moyen | Moyen |
| 8 | Terrain batch/normales/footprint | Fort | Moyen | Moyen |
| 9 | Prédicteur propulsé événementiel | Très fort | Très élevé | Élevé |
| 10 | Safety supervisor | Fort | Moyen | Moyen |
| 11 | Structure/charges/modes flexibles | Fort | Élevé | Élevé |
| 12 | TCA adapter | Fort | Moyen | Élevé compatibilité |
| 13 | Thermique/ressources prédictifs | Moyen–fort | Moyen | Moyen |
| 14 | SCANsat/site global | Moyen | Faible–moyen | Faible |
| 15 | Drag cubes bruts | Moyen | Moyen | Élevé compatibilité |
| 16 | Champs `PartModule` génériques | Moyen | Moyen | Élevé sécurité/type |
| 17 | Krakensbane/origin events | Moyen | Faible | Moyen |
| 18 | Principia | Fort mais spécialisé | Très élevé | Très élevé |
| 19 | Overlay/rendu | Faible pour GNC | Moyen | Faible |
| 20 | Caméra/éditeur/R&D | Hors cœur GNC | Élevé | Moyen |

## 11. Ce qu'il ne faut pas faire

- Réimplémenter le delta-v, les ranges de physique, l'inertie ou l'aéro arbitraire déjà présents dans kRPC 0.6.0.
- Ajouter des centaines de getters sans timestamp commun.
- Exposer seulement une position moyenne et une direction moyenne pour un moteur multi-transform.
- Envoyer les actionneurs un par un sans application atomique.
- Retourner zéro quand un mod ou membre réfléchi est absent ; retourner `unsupported` avec raison.
- Faire tourner optimisation, réflexion ou allocations massives dans le thread Unity sans budget.
- Laisser MechJeb, TCA, SAS, AutoPilot kRPC et le client externe posséder simultanément les mêmes axes.
- Construire un prédicteur sophistiqué avant d'avoir recorder, vérité réalisée et jeux de régression.
- Promettre un rejeu bit-à-bit de KSP ; utiliser tolérances, invariants et statistiques.

## 12. Définition de « perfectionner l'autopilote »

Le succès ne se mesure pas au nombre de procédures RPC. Il se mesure à ces invariants :

- toutes les entrées d'une décision viennent du même tick ou portent leur âge ;
- toutes les sorties coordonnées s'appliquent ensemble ou pas du tout ;
- le contrôleur connaît l'autorité, les délais, les saturations et les pannes ;
- force et couple demandés sont comparés à force et couple obtenus ;
- les changements de craft/stage/CoM invalident proprement les anciens modèles ;
- toute prédiction annonce son modèle, son âge, sa convergence et son incertitude ;
- toute mission peut être rejouée, expliquée et testée sous perturbations ;
- la disparition du client conduit à un état sûr configuré ;
- les plugins optionnels enrichissent le système sans pouvoir casser le Core.

Atteindre ces invariants donnera davantage de précision et de fiabilité que l'ajout isolé de n'importe quel champ — y compris les drag cubes ou la vitesse de gimbal. Ces champs deviennent réellement utiles lorsqu'ils participent à une chaîne temporellement cohérente, observable et testable.

## Sources primaires principales

- [kRPC — dépôt officiel](https://github.com/krpc/krpc)
- [kRPC 0.6.0 — `Vessel`](https://krpc.github.io/krpc/latest/python/api/space-center/vessel.html)
- [kRPC 0.6.0 — `Flight`](https://krpc.github.io/krpc/latest/python/api/space-center/flight.html)
- [kRPC 0.6.0 — `Parts`](https://krpc.github.io/krpc/latest/python/api/space-center/parts.html)
- [kRPC 0.6.0 — `AutoPilot`](https://krpc.github.io/krpc/latest/python/api/space-center/auto-pilot.html)
- [Trajectories — dépôt officiel](https://github.com/neuoy/KSPTrajectories)
- [MechJeb — dépôt officiel](https://github.com/MuMech/MechJeb2)
- [Throttle Controlled Avionics — dépôt officiel](https://github.com/allista/ThrottleControlledAvionics)
- [SCANsat — dépôt officiel](https://github.com/KSPModStewards/SCANsat)
- [Ferram Aerospace Research — dépôt officiel](https://github.com/ferram4/Ferram-Aerospace-Research)
- [RealFuels — dépôt officiel](https://github.com/KSP-RO/RealFuels)
- [SolverEngines — dépôt officiel](https://github.com/KSP-RO/SolverEngines)
- [NASA — SLS Ascent Flight Control Design](https://ntrs.nasa.gov/api/citations/20140008731/downloads/20140008731.pdf)
- [NASA/JPL — Minimum Landing Error Powered-Descent Guidance](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20120001230.pdf)
- [NASA/JPL — ALHAT](https://robotics.jpl.nasa.gov/what-we-do/research-tasks/alhat-autonomous-landing-and-hazard-avoidance-technology/)
- [NASA — SPLICE](https://www.nasa.gov/safe-and-precise-landing-integrated-capabilities-evolution-splice/)
- [NASA — Terrain Relative Navigation](https://science.nasa.gov/science-research/science-enabling-technology/technology-highlights/terrain-relative-navigation-landing-between-the-hazards/)
- [NASA NTRS — estimateur d'état ALHAT](https://ntrs.nasa.gov/citations/20170009197)

