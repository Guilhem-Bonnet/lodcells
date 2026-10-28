# lodcells

LOD discrets par maillage, cellules spatiales et sélection par erreur projetée en pixels, en Rust pur, pour les mondes ouverts dans Godot (couche gdext à venir).

Inspiré (idées, pas code) de Godotite (coupe en erreur-pixel, sélection incrémentale à « slack », validateur), nanite-webgpu (erreur additive monotone, sphère partagée, impostors) et des mécanismes natifs de Godot (MultiMesh, visibility ranges). Né pour un projet hôte.

Licence MIT.

## État (2026-10-28)

`lodcells-core` : soudure par position, chaîne de LOD (meshopt-rs, port Rust pur — pas de compilateur C++ requis), erreur additive monotone mesurée, cellules spatiales, sélection par erreur-pixel avec slack incrémental, validateur. 11 tests. Prochaine couche : `lodcells-godot` (gdext, MultiMesh par maillage × niveau × cellule).
