//! lodcells — LOD discrets, cellules spatiales et sélection par erreur écran.
//!
//! Pipeline : [`mesh::IndexedMesh`] (soudé par position) → [`lod::LodChain`]
//! (niveaux simplifiés par meshoptimizer, bords verrouillés, erreur monotone)
//! → [`cells::CellGrid`] (exemplaires groupés par cellule) → [`select`]
//! (niveau par exemplaire d'après l'erreur projetée en pixels, avec « slack »
//! : distance que la caméra peut parcourir avant que la décision change).
//! Aucune dépendance moteur : la couche Godot vit dans un autre crate.

pub mod cells;
pub mod lod;
pub mod mesh;
pub mod select;
pub mod validate;

pub use cells::{CellGrid, CellKey, Instance};
pub use lod::{BuildError, LodChain, LodLevel, LodParams};
pub use mesh::IndexedMesh;
pub use select::{Camera, Selection, Selector};
