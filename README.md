# lodcells

LOD discrets par maillage, cellules spatiales et sélection par erreur projetée en pixels, en Rust pur, pour les mondes ouverts dans Godot 4 (GDExtension + addon GDScript).

Inspiré (idées, pas code) de Godotite (coupe en erreur-pixel, sélection incrémentale à « slack », validateur), nanite-webgpu (erreur additive monotone, sphère partagée, impostors) et des mécanismes natifs de Godot (MultiMesh, visibility ranges). Né pour un projet hôte.

Licence MIT.

## Structure

| Crate / dossier | Rôle |
|---|---|
| `crates/lodcells-core` | Cœur sans dépendance moteur : soudure, chaîne de LOD (`meshopt-rs`, port Rust pur, pas de C++) à bords verrouillés et coutures respectées, erreur de surface additive mesurée, sélection à slack + hystérésis, validateur (appliqué en debug à chaque chaîne construite). |
| `crates/lodcells-godot` | GDExtension (godot-rust 0.5, API 4.7) : `LodBuilder`, `LodSelector`. |
| `godot/addons/lodcells` | Addon : nœud `lod_cells.gd` (MultiMesh par maillage × niveau × cellule). |
| `godot/tests/test_lodcells.gd` | Test headless de l'ensemble. |
| `tools/build.sh` | Compile l'extension et l'installe dans l'addon. |

```sh
cargo test --workspace            # 25 tests Rust (+ 1 banc ignoré)
cargo clippy --workspace --all-targets -- -D warnings
cargo test --release -p lodcells-core -- --ignored bench --nocapture   # banc 16 k / 64 k sommets
sh tools/build.sh                 # → godot/addons/lodcells/bin/liblodcells_godot.so
godot --headless --path godot --import
godot --headless --path godot --script res://tests/test_lodcells.gd
```

## API

### Rust (`lodcells-core`)

- `LodChain::build(&IndexedMesh, &LodParams) -> Result<LodChain, BuildError>` : niveau 0 = maillage d'origine (moins les triangles dégénérés par la soudure), puis niveaux plus grossiers à erreur strictement croissante ; `source_indices` de chaque niveau pointe dans le tampon de sommets **d'origine** : la simplification travaille en topologie entièrement soudée (laisser ses « wedges » à meshoptimizer fige tout sommet à ≥ 3 faces dures — mesuré : 270 cellules sur 332 sans niveau dans un projet hôte), puis chaque coin reçoit, parmi les sommets d'origine de sa position, celui dont la normale de fan (calculée sur les triangles d'origine, positions seules) est la plus proche en *angle* (cosinus, pas produit scalaire brut : la pondération par l'aire ne favorise pas les grandes faces — `wedge_choice_ignores_fan_area`) de la normale du triangle — normales/UV de face conservées, niveau 0 = indices d'origine. `LodParams` : `ratio` (0,5), `max_levels` (4), `min_gain` (0,1), `target_error` (1,0 = pas de limite pratique ; borne par effondrement, relative au rayon, convertie en unité meshoptimizer — l'ancienne version la passait × rayon, donc sans effet, et 0,05 réellement appliqué laisse 270 cellules sur 332 sans niveau grossier dans un projet hôte), `weld_tolerance` (1e-4), `lock_border` (true : positions de bord identiques à tous les niveaux). Refuse sans paniquer : `ratio` hors ]0, 1[, paramètre négatif/NaN, maillage vide, indices non triangulaires ou hors plage.
- Verrouillage des bords sans C++ : meshopt-rs 0.1.2 (dernière version sur crates.io, vérifiée dans sa source) n'a ni `LockBorder` ni verrou par sommet ; une pré-passe ajoute à chaque sommet de bord (arête à un seul triangle, en topologie soudée) une « ailette » vers deux sommets fictifs partagés, ce qui le classe `Locked` chez meshoptimizer (plus de deux demi-arêtes ouvertes) ; les ailettes sont retirées de la sortie. Trous intérieurs compris, coutures exclues.
- Erreur d'une étape : distance maximale d'une position retirée à la **surface** simplifiée (triangle le plus proche, grille de triangles), nulle sur un plan.
- `select::select_level_with(chain, center, radius, cam, threshold_px, prev, hysteresis) -> Selection` : niveau le plus grossier dont l'erreur projetée ≤ seuil, avec hystérésis relative autour du niveau précédent, et le slack (distance que la caméra peut parcourir sans changer la décision).
- `Selector` : sélection incrémentale par exemplaire (`new(n)`, `push()`, `begin_frame(&cam)`, `select(i, …)`, `evaluated`, `hysteresis`, défaut 0,15). Déplacement cumulé en `f64`.
- `validate::validate(&chain) -> Vec<Issue>` : invariants d'une chaîne (erreur croissante, niveaux plus grossiers, indices en plage, pas de triangle dégénéré) ; appelé en `debug_assert!` par `LodChain::build`. (`CellGrid`/`Instance`, jamais branchés — le nœud Godot groupe lui-même par clé — ont été retirés.)

### GDExtension

`LodBuilder` (RefCounted) — propriétés `ratio` (0,5), `max_levels` (4), `target_error` (1,0, relatif au rayon), `weld_tolerance` (1e-4), `lock_border` (true) ; `defaults()` ; `build(positions: PackedVector3Array, indices: PackedInt32Array) -> Dictionary` `{levels: [PackedInt32Array…], errors, triangles, center, radius}`. Entrée invalide : `godot_error` + dictionnaire **vide**.

`LodSelector` (RefCounted) — propriété `hysteresis` (0,15) ; `add_chain(errors: PackedFloat32Array) -> int` (erreurs finies, strictement croissantes, sinon −1) ; `add_cell(chain: int, center: Vector3, radius: float) -> int` (incrémental : n'invalide pas les autres cellules ; chaîne inconnue → −1 ; chaîne < 0 = toujours niveau 0 ; réemploie l'emplacement d'une cellule retirée) ; `remove_cell(id) -> bool` (la cellule n'est plus évaluée, −1 dans `select_changed` ; id inconnu ou déjà retiré → false) ; `begin_frame(cam_pos, fov_y_deg, viewport_height)` ; `select_changed(threshold_px) -> PackedInt32Array` (niveau par cellule, −1 = inchangé) ; `evaluated()` ; `cell_count()` (cellules vivantes).

La GDExtension est compilée avec `experimental-threads` : `LodBuilder` est instancié et appelé depuis des fils `WorkerThreadPool` (un builder par tâche, jamais partagé).

### Nœud `lod_cells.gd` (Node3D)

Exports : `cell_size` (409,6 m), `visibility_end` (3 000 m), `threshold_px` (2), `max_levels`, `ratio`, `target_error` (1,0), `weld_tolerance`, `lock_border` (true), `hysteresis`, `cache_dir` (`user://lodcache`, "" = pas de cache). Les cinq paramètres de construction sont poussés vers `LodBuilder` **avant chaque construction** et font partie de la clé du cache disque (`CACHE_VERSION` 4).

- `add_cell(key, scene: Node3D, xforms: Array[Transform3D], vis_end := -1.0) -> int` : exemplaires d'une scène sous une clé (Vector2i de cellule de streaming, int, String…), plusieurs scènes par clé ; chaque `MultiMeshInstance3D` est ancrée au centroïde de sa tuile et ses exemplaires sont relatifs à cette ancre (petits nombres côté GPU loin de l'origine). `add_building(scene, xforms, vis_end)` = `add_cell` sous une clé automatique (entiers négatifs).
- `clear_cell(key) -> bool` : libère les MMI de la clé ; `cell_count`, `instance_count`, `multimesh_count` décroissent, les emplacements du sélecteur sont réemployés. `has_cell(key)`, `cell_keys()`.
- `prepare_scene(scene, budget_usec := 0) -> bool` / `prepare_mesh(mesh)` : construction des niveaux en `WorkerThreadPool` (tableaux de surface lus sur le fil principal — `surface_get_arrays` est un appel synchronisé du RenderingServer —, `LodBuilder` local à la tâche, `ArrayMesh` assemblés sur le fil principal par `reap`) ; `reap(budget_usec := 0)` récolte les tâches finies (nombre restant) et, avec un budget, n'assemble les `ArrayMesh` que tant qu'il n'est pas dépassé (le reste attend l'appel suivant : avant, 66 ms d'un coup sur des tuiles de terrain à 6 niveaux) ; `unassembled_count()`, `pending_count()`, `wait_all()`. Sans préparation, `add_cell` construit en synchrone.
- `forget_mesh(mesh) -> bool` / `forget_scene(scene) -> int` : oublient les niveaux construits d'un maillage (ArrayMesh des niveaux > 0 et entrée de chaîne rendus ; le maillage d'origine appartient à l'appelant) — à réserver aux maillages qu'aucune cellule vivante n'utilise ; faux si inconnu ou encore en préparation. L'emplacement de chaîne du sélecteur (≤ `max_levels` floats) reste alloué côté Rust.
- Cache disque : `user://lodcache/<md5(positions, indices, paramètres)>.bin` = `"LODC"` + u32 `CACHE_VERSION` + `var_to_bytes({levels, errors, center, radius})`, relu identique octet à octet, ignoré (et réécrit) si la version diffère ; compteurs `cache_hits` / `cache_misses` ; `encode_cache` / `decode_cache` / `cache_key` statiques.
- Diagnostics `evaluated_last_frame()`, `levels()` (cellules vivantes), compteurs `instance_count`, `multimesh_count`, `lod_mesh_count`, `cell_count`. Sans l'extension compilée : un seul niveau, pas de sélection (avertissement).

## Mesures (tests Rust, `--nocapture`)

| Cas | Résultat |
|---|---|
| Grille bosselée 32×32, bords verrouillés | 1 922 → 122 triangles en 4 niveaux, **124 → 124** sommets de bord, positions de bord identiques à tous les niveaux (`border_positions_are_identical_at_all_levels`) |
| Même grille, `lock_border = false` | 124 → 14 sommets de bord (`without_lock_border_the_outline_is_simplified`) |
| Grille 24×24 avec trou 4×4 et une couture dupliquée | bords = contour + trou (108), couture exclue, identiques à tous les niveaux |
| Plan plat 32×32 | erreurs 2e-6 / 4e-6 / 7e-6 / 9e-6 (plancher 1e-7·rayon par niveau ; l'ancienne mesure « sommet le plus proche » donnait 1 / 4 / 11 / 26) |
| Toit à deux pans (arête dupliquée, normales différentes), `target_error` 0,02 | 512 → 92 triangles en 3 niveaux, chaque coin de chaque triangle porte la normale de sa face (dot > 0,9) ; cube 24 sommets : niveau 0 exact (l'ancien « premier sommet de la position » donnait la normale d'une autre face), 12 → 6 → 2 triangles avec, à chaque coin, le meilleur sommet disponible |
| **A/B sur un projet hôte** (copie de HEAD, cache dédié, `screenshot.gd` + variante de `measure_split.gd` attendant la fin du streaming, RX 9070 XT 1280×720) | Vue lointaine (une ville dense + 900 m) : **814 k → 441 k primitives (−46 %)**, part lod_cells 727 k → 388 k ; niveaux des 332 cellules {0: 100, 1: 48, 2: 34, 3: 32, 4: 118} → {0: 38, 1: 42, 2: 65, 3: 100, 4: 87} ; vue ville haut : 3 034 k → 2 809 k (−7 %), part lod_cells 2 268 k → 2 057 k (−9 %) ; draw calls inchangés (243 / 1 014) ; captures `ville_jour`, `ville_haut`, `monde_loin` indiscernables à l'œil. Avec `lock_border = false` (objets isolés, pas de tuile voisine) : 411 k (−50 %) et 2 679 k (−12 %), niveaux {0: 14, 1: 19, 2: 40, 3: 70, 4: 189}. |
| Banc release, grille bosselée 128² = 16 384 sommets | **63 ms** la chaîne complète (32 258 → 2 016 triangles, bords 508 à tous les niveaux) |
| Banc release, 256² = 65 536 sommets | **388 ms** (130 050 → 8 128 triangles, bords 1 020) |

## Limites réelles

- Trois sommets de bord alignés peuvent former un triangle d'aire nulle (la métrique quadrique est nulle dans le plan ; 17 sur les 988 triangles du toit de test) : invisible, conservé pour garder le sommet de bord dans le niveau.
- Le choix du sommet d'origine par coin se fait sur la normale de fan : deux coutures UV dans une même face plane (même normale) sont indiscernables côté cœur, le coin peut prendre les UV de l'autre îlot aux niveaux grossiers (jamais au niveau 0). Passer des attributs au cœur (`simplify_with_attributes`) lèverait la limite.
- `lock_border = true` par défaut (tuiles jointives). Pour des objets isolés (bâtiments), le verrou freine la simplification des faces ouvertes : −46 % contre −50 % de primitives en vue lointaine dans un projet hôte ; à désactiver sur le nœud des bâtiments.
- `target_error` borne chaque effondrement (métrique quadrique de meshoptimizer), pas la déviation cumulée du niveau : c'est `error` (mesurée) qui sert à la sélection.
- Backend meshopt C++ (`LockBorder`, `result_error` natifs) : aucun compilateur C++ sur la machine de développement (`g++`/`clang++` absents, seul `cc` est présent) ; chemin prévu si l'un apparaît : feature cargo `cxx` sur `lodcells-core` activant le crate `meshopt` (bindings C++) derrière la même fonction `simplify_step`, les tests ci-dessus servant de contrat commun.
- La sphère de cellule est calculée à l'ajout ; le nœud peut être déplacé (décalage d'origine) mais pas ses exemplaires.
- Pas de culling par cellule ni d'impostors (prochaines étapes).

## Historique

- 2026-10-29 — S11 : bords verrouillés par pré-passe d'ailettes (meshopt-rs 0.1.2 n'a pas de `LockBorder` ; tuile 128² : 508 sommets de bord à tous les niveaux au lieu de 508 → 72), coutures d'attributs respectées (sommet d'origine choisi par coin sur la normale de fan, plus de « premier sommet d'origine de la position » ; vérification : comparaison en cosinus et non en produit scalaire brut, une grande face inclinée à 60° l'emportait sur une petite face exacte, `CACHE_VERSION` 4), erreur d'étape = distance à la surface simplifiée (plan plat ≈ 0 au lieu de 1/4/11/26), `target_error` converti en unité meshoptimizer (était passé × rayon, donc sans effet ; défaut 1,0 = ancien comportement effectif), `lock_border` exposé (cœur, GDExtension, nœud, clé de cache), `CellGrid` retiré, `validate` branché en debug, banc 16 k / 64 k en CI, `CACHE_VERSION` 3. A/B sur un projet hôte : −46 % de primitives en vue lointaine à visuel égal (tableau ci-dessus).
- 2026-10-29 — S06 : API incrémentale `add_cell` / `clear_cell` (MMI libérés, `cell_count` décroît, emplacements du sélecteur réemployés via `LodSelector.remove_cell`), construction des niveaux hors fil principal (`prepare_scene` + `reap`, `experimental-threads`), cache disque versionné des niveaux (relu identique octet à octet), MMI ancrées au centroïde de tuile ; mesure headless : `add_cell` après préparation 0,06 ms contre 5,26 ms en synchrone (grille 40×40) ; 47 vérifications dans `test_lodcells.gd`, 20 tests Rust.
- 2026-10-29 — S04 : exports du nœud réellement appliqués (`max_levels` = 1 donne 4 MultiMesh contre 10), validation des entrées côté Rust et GDExtension (plus de panique sur `ratio` ≥ 1, indices hors plage, maillage vide), hystérésis 15 % (0 bascule sur 1 000 images d'oscillation contre > 900 sans), `add_cell` incrémental (1 évaluation au lieu de toutes), déplacement cumulé en `f64` (100 000 images à 0,5 m : écart de slack max 1,2e-5, contre 1e-3 dès l'image 1 789 en `f32`), clippy en CI.
- 2026-10-28 — erreurs à l'échelle de l'exemplaire, sphère par centre de maillage, `step_error` en grille ; CI ; GDExtension + addon ; cœur.
