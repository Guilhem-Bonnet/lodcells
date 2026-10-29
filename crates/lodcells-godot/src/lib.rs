//! GDExtension lodcells : deux classes RefCounted.
//!
//! - `LodBuilder` : à partir des positions/indices d'une surface, renvoie les
//!   niveaux de LOD sous forme d'index arrays dans le tampon de sommets
//!   d'origine (le GDScript reconstruit des ArrayMesh avec tous les attributs).
//! - `LodSelector` : cellules enregistrées (centre, rayon, erreurs du maillage),
//!   niveau par cellule d'après l'erreur projetée en pixels, sélection
//!   incrémentale à « slack ».

use godot::prelude::*;
use lodcells_core::lod::{LodChain, LodLevel, LodParams};
use lodcells_core::mesh::IndexedMesh;
use lodcells_core::select::{Camera, Selector};

struct LodCellsExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LodCellsExtension {}

#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct LodBuilder {
    /// Fraction de triangles visée à chaque niveau.
    #[var]
    ratio: f32,
    #[var]
    max_levels: i64,
    /// Erreur maximale par étape, relative au rayon du maillage.
    #[var]
    target_error: f32,
    #[var]
    weld_tolerance: f32,
}

#[godot_api]
impl LodBuilder {
    #[func]
    fn defaults(&mut self) {
        let p = LodParams::default();
        self.ratio = p.ratio;
        self.max_levels = p.max_levels as i64;
        self.target_error = p.target_error;
        self.weld_tolerance = p.weld_tolerance;
    }

    /// Niveaux de LOD d'une surface. Renvoie {"levels": [PackedInt32Array…]
    /// (indices dans les sommets d'origine, niveau 0 inclus), "errors":
    /// PackedFloat32Array, "triangles": PackedInt32Array, "center": Vector3,
    /// "radius": float}. Un tableau d'indices vide = liste triangulaire 0..n.
    #[func]
    fn build(&self, positions: PackedVector3Array, indices: PackedInt32Array) -> VarDictionary {
        let positions: Vec<[f32; 3]> = positions.to_vec().iter().map(|v| [v.x, v.y, v.z]).collect();
        let indices: Vec<u32> = if indices.is_empty() {
            (0..positions.len() as u32).collect()
        } else {
            indices.to_vec().iter().map(|&i| i.max(0) as u32).collect()
        };
        let params = LodParams {
            ratio: if self.ratio > 0.0 { self.ratio } else { 0.5 },
            max_levels: if self.max_levels > 0 { self.max_levels as usize } else { 4 },
            target_error: if self.target_error > 0.0 { self.target_error } else { 0.05 },
            weld_tolerance: self.weld_tolerance.max(0.0),
            ..LodParams::default()
        };
        let chain = LodChain::build(&IndexedMesh::new(positions, indices), &params);
        let mut levels: Array<PackedInt32Array> = Array::new();
        let mut errors = PackedFloat32Array::new();
        let mut triangles = PackedInt32Array::new();
        for l in &chain.levels {
            let idx: PackedInt32Array = l.source_indices.iter().map(|&i| i as i32).collect();
            levels.push(&idx);
            errors.push(l.error);
            triangles.push(l.mesh.triangle_count() as i32);
        }
        let mut d = VarDictionary::new();
        d.set("levels", &levels);
        d.set("errors", &errors);
        d.set("triangles", &triangles);
        d.set("center", Vector3::new(chain.center[0], chain.center[1], chain.center[2]));
        d.set("radius", chain.radius);
        d
    }
}

struct Cell {
    /// `None` : maillage à un seul niveau, jamais évalué (reste au niveau 0).
    chain: Option<usize>,
    center: [f32; 3],
    radius: f32,
}

#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct LodSelector {
    chains: Vec<LodChain>,
    cells: Vec<Cell>,
    selector: Option<Selector>,
    camera: Option<Camera>,
    last: Vec<i32>,
}

#[godot_api]
impl LodSelector {
    /// Déclare une chaîne par ses erreurs cumulées (niveau 0 = 0) ; renvoie son id.
    #[func]
    fn add_chain(&mut self, errors: PackedFloat32Array) -> i64 {
        let levels = errors
            .to_vec()
            .into_iter()
            .map(|e| LodLevel { mesh: IndexedMesh::default(), error: e, source_indices: Vec::new() })
            .collect();
        self.chains.push(LodChain { levels, center: [0.0; 3], radius: 0.0 });
        self.selector = None;
        (self.chains.len() - 1) as i64
    }

    /// Enregistre une cellule (sphère englobante monde) d'une chaîne ; renvoie son id.
    #[func]
    fn add_cell(&mut self, chain: i64, center: Vector3, radius: f32) -> i64 {
        let chain = usize::try_from(chain).ok();
        self.cells.push(Cell { chain, center: [center.x, center.y, center.z], radius });
        self.selector = None;
        (self.cells.len() - 1) as i64
    }

    #[func]
    fn cell_count(&self) -> i64 {
        self.cells.len() as i64
    }

    /// À appeler chaque image avant `select_all`.
    #[func]
    fn begin_frame(&mut self, cam_pos: Vector3, fov_y_deg: f32, viewport_height: f32) {
        let cam = Camera { position: [cam_pos.x, cam_pos.y, cam_pos.z], fov_y: fov_y_deg.to_radians(), viewport_height };
        if self.selector.is_none() {
            self.selector = Some(Selector::new(self.cells.len()));
            self.last = vec![-1; self.cells.len()];
        }
        self.selector.as_mut().unwrap().begin_frame(&cam);
        self.camera = Some(cam);
    }

    /// Niveau par cellule (−1 = inchangé depuis le dernier appel) ; les
    /// cellules sans chaîne valide restent au niveau 0.
    #[func]
    fn select_changed(&mut self, threshold_px: f32) -> PackedInt32Array {
        let (Some(sel), Some(cam)) = (self.selector.as_mut(), self.camera.as_ref()) else {
            return PackedInt32Array::new();
        };
        let mut out = PackedInt32Array::new();
        out.resize(self.cells.len());
        for (i, c) in self.cells.iter().enumerate() {
            let level = match c.chain.and_then(|k| self.chains.get(k)) {
                Some(chain) if chain.len() > 1 => sel.select(i, chain, c.center, c.radius, cam, threshold_px).level as i32,
                _ => 0,
            };
            out[i] = if level != self.last[i] { level } else { -1 };
            self.last[i] = level;
        }
        out
    }

    /// Nombre de cellules réévaluées lors du dernier `select_changed`.
    #[func]
    fn evaluated(&self) -> i64 {
        self.selector.as_ref().map_or(0, |s| s.evaluated as i64)
    }
}
