# lodcells

LOD discrets par maillage, cellules spatiales et sélection par erreur projetée en pixels, en Rust pur, pour les mondes ouverts dans Godot 4 (GDExtension + addon GDScript).

Inspiré (idées, pas code) de Godotite (coupe en erreur-pixel, sélection incrémentale à « slack », validateur), nanite-webgpu (erreur additive monotone, sphère partagée, impostors) et des mécanismes natifs de Godot (MultiMesh, visibility ranges). Né pour un projet hôte.

Licence MIT.

## Structure

| Crate / dossier | Rôle |
|---|---|
| `crates/lodcells-core` | Cœur sans dépendance moteur : soudure, chaîne de LOD (`meshopt-rs`, port Rust pur, pas de C++), erreur additive mesurée, cellules, sélection à slack + hystérésis, validateur. |
| `crates/lodcells-godot` | GDExtension (godot-rust 0.5, API 4.7) : `LodBuilder`, `LodSelector`. |
| `godot/addons/lodcells` | Addon : nœud `lod_cells.gd` (MultiMesh par maillage × niveau × cellule). |
| `godot/tests/test_lodcells.gd` | Test headless de l'ensemble. |
| `tools/build.sh` | Compile l'extension et l'installe dans l'addon. |

```sh
cargo test --workspace            # 19 tests Rust
cargo clippy --workspace --all-targets -- -D warnings
sh tools/build.sh                 # → godot/addons/lodcells/bin/liblodcells_godot.so
godot --headless --path godot --import
godot --headless --path godot --script res://tests/test_lodcells.gd
```

## API

### Rust (`lodcells-core`)

- `LodChain::build(&IndexedMesh, &LodParams) -> Result<LodChain, BuildError>` : niveau 0 = maillage soudé, puis niveaux plus grossiers à erreur strictement croissante ; `source_indices` de chaque niveau pointe dans le tampon de sommets **d'origine** (attributs conservés côté moteur). Refuse sans paniquer : `ratio` hors ]0, 1[, paramètre négatif/NaN, maillage vide, indices non triangulaires ou hors plage.
- `select::select_level_with(chain, center, radius, cam, threshold_px, prev, hysteresis) -> Selection` : niveau le plus grossier dont l'erreur projetée ≤ seuil, avec hystérésis relative autour du niveau précédent, et le slack (distance que la caméra peut parcourir sans changer la décision).
- `Selector` : sélection incrémentale par exemplaire (`new(n)`, `push()`, `begin_frame(&cam)`, `select(i, …)`, `evaluated`, `hysteresis`, défaut 0,15). Déplacement cumulé en `f64`.
- `CellGrid`, `Instance`, `validate::validate(&chain) -> Vec<Issue>`.

### GDExtension

`LodBuilder` (RefCounted) — propriétés `ratio` (0,5), `max_levels` (4), `target_error` (0,05, relatif au rayon), `weld_tolerance` (1e-4) ; `defaults()` ; `build(positions: PackedVector3Array, indices: PackedInt32Array) -> Dictionary` `{levels: [PackedInt32Array…], errors, triangles, center, radius}`. Entrée invalide : `godot_error` + dictionnaire **vide**.

`LodSelector` (RefCounted) — propriété `hysteresis` (0,15) ; `add_chain(errors: PackedFloat32Array) -> int` (erreurs finies, strictement croissantes, sinon −1) ; `add_cell(chain: int, center: Vector3, radius: float) -> int` (incrémental : n'invalide pas les autres cellules ; chaîne inconnue → −1 ; chaîne < 0 = toujours niveau 0) ; `begin_frame(cam_pos, fov_y_deg, viewport_height)` ; `select_changed(threshold_px) -> PackedInt32Array` (niveau par cellule, −1 = inchangé) ; `evaluated()` ; `cell_count()`.

### Nœud `lod_cells.gd` (Node3D)

Exports : `cell_size` (409,6 m), `visibility_end` (3 000 m), `threshold_px` (2), `max_levels`, `ratio`, `target_error`, `weld_tolerance`, `hysteresis`. Les quatre paramètres de construction sont poussés vers `LodBuilder` **avant chaque construction** (un changement après coup ne touche que les maillages pas encore construits). `add_building(scene: Node3D, xforms: Array[Transform3D], vis_end := -1.0)` ; diagnostics `evaluated_last_frame()`, `levels()`, compteurs `instance_count`, `multimesh_count`, `lod_mesh_count`, `cell_count`. Sans l'extension compilée : un seul niveau, pas de sélection (avertissement).

## Limites réelles (mesurées)

- **Les bords ne sont PAS verrouillés.** `meshopt-rs` 0.1 n'a pas d'option de verrouillage (`simplify_options` de meshoptimizer n'est pas porté) : les sommets de bord ne sont que pondérés. Mesures : grille 16×16, 60 → 25 sommets de bord au niveau 4 (test `border_vertices_are_not_locked_with_meshopt_rs_0_1`) ; tuile de terrain un projet hôte, 508 → 72 (audit S04). Les coins et l'enveloppe survivent (`border_outline_is_preserved`), mais deux tuiles voisines à des niveaux différents peuvent présenter des fissures. À traiter par un verrouillage explicite (poids infini des arêtes ouvertes, ou meshopt C++) ou par des jupes.
- L'erreur par étape est une borne simple (distance max d'un sommet retiré au sommet conservé le plus proche), pas l'erreur de surface de meshoptimizer (0.1 ne la renvoie pas).
- La sphère de cellule est calculée à l'ajout ; le nœud peut être déplacé (décalage d'origine) mais pas ses exemplaires.
- Pas de culling par cellule ni d'impostors (prochaines étapes).

## Historique

- 2026-10-29 — S04 : exports du nœud réellement appliqués (`max_levels` = 1 donne 4 MultiMesh contre 10), validation des entrées côté Rust et GDExtension (plus de panique sur `ratio` ≥ 1, indices hors plage, maillage vide), hystérésis 15 % (0 bascule sur 1 000 images d'oscillation contre > 900 sans), `add_cell` incrémental (1 évaluation au lieu de toutes), déplacement cumulé en `f64` (100 000 images à 0,5 m : écart de slack max 1,2e-5, contre 1e-3 dès l'image 1 789 en `f32`), clippy en CI.
- 2026-10-28 — erreurs à l'échelle de l'exemplaire, sphère par centre de maillage, `step_error` en grille ; CI ; GDExtension + addon ; cœur.
