//! lodcells — LOD discrets et sélection par erreur écran.
//!
//! Pipeline : [`mesh::IndexedMesh`] → [`lod::LodChain`] (niveaux simplifiés
//! par meshopt-rs, bords verrouillés par pré-passe, coutures d'attributs
//! respectées, erreur de surface mesurée et monotone, chaîne validée par
//! [`validate`] en debug) → [`select`] (niveau par exemplaire d'après l'erreur
//! projetée en pixels, avec « slack » : distance que la caméra peut parcourir
//! avant que la décision change). Le groupement des exemplaires par cellule
//! est du ressort du moteur (nœud `lod_cells.gd` côté Godot).
//! Aucune dépendance moteur : la couche Godot vit dans un autre crate.

pub mod lod;
pub mod mesh;
pub mod select;
pub mod validate;

pub use lod::{BuildError, LodChain, LodLevel, LodParams};
pub use mesh::IndexedMesh;
pub use select::{Camera, Selection, Selector};
pub use validate::{validate, Issue};
