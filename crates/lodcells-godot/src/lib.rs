//! GDExtension lodcells : deux classes RefCounted.
//!
//! - `LodBuilder` : à partir des positions/indices d'une surface, renvoie les
//!   niveaux de LOD sous forme d'index arrays dans le tampon de sommets
//!   d'origine (le GDScript reconstruit des ArrayMesh avec tous les attributs).
//! - `LodSelector` : cellules enregistrées (centre, rayon, erreurs du maillage),
//!   niveau par cellule d'après l'erreur projetée en pixels, sélection
//!   incrémentale à « slack » et hystérésis.
//!
//! Toute entrée invalide est signalée par `godot_error!` et un résultat neutre
//! (dictionnaire vide, −1, tableau vide) : jamais de panique côté moteur.

use godot::prelude::*;
use lodcells_core::lod::{LodChain, LodLevel, LodParams};
use lodcells_core::mesh::IndexedMesh;
use lodcells_core::select::{Camera, Selector, DEFAULT_HYSTERESIS};

struct LodCellsExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LodCellsExtension {}

/// `tool` : utilisable dans l'éditeur Godot (aperçu des cellules dans la vue 3D).
#[derive(GodotClass)]
#[class(tool, base = RefCounted, init)]
pub struct LodBuilder {
    /// Fraction de triangles visée à chaque niveau, dans ]0, 1[.
    #[var]
    #[init(val = 0.5)]
    ratio: f32,
    /// Nombre maximal de niveaux au-delà de l'origine (0 = un seul niveau).
    #[var]
    #[init(val = 4)]
    max_levels: i64,
    /// Erreur maximale par effondrement, relative au rayon du maillage
    /// (1,0 = pas de limite pratique, cf. lodcells-core).
    #[var]
    #[init(val = 1.0)]
    target_error: f32,
    /// Soudure par position avant simplification (0 = désactivée).
    #[var]
    #[init(val = 1e-4)]
    weld_tolerance: f32,
    /// Verrouille les sommets de bord (positions de bord identiques à tous les
    /// niveaux : pas de fissure entre tuiles voisines à des niveaux différents).
    #[var]
    #[init(val = true)]
    lock_border: bool,
}

#[godot_api]
impl LodBuilder {
    /// Remet les paramètres par défaut de lodcells-core.
    #[func]
    fn defaults(&mut self) {
        let p = LodParams::default();
        self.ratio = p.ratio;
        self.max_levels = p.max_levels as i64;
        self.target_error = p.target_error;
        self.weld_tolerance = p.weld_tolerance;
        self.lock_border = p.lock_border;
    }

    /// Niveaux de LOD d'une surface. Renvoie {"levels": [PackedInt32Array…]
    /// (indices dans les sommets d'origine, niveau 0 inclus), "errors":
    /// PackedFloat32Array, "triangles": PackedInt32Array, "center": Vector3,
    /// "radius": float}. Un tableau d'indices vide = liste triangulaire 0..n.
    /// Entrée invalide (ratio hors ]0,1[, indice hors plage ou négatif,
    /// maillage vide…) : erreur Godot et dictionnaire vide.
    #[func]
    fn build(&self, positions: PackedVector3Array, indices: PackedInt32Array) -> VarDictionary {
        let positions: Vec<[f32; 3]> = positions.to_vec().iter().map(|v| [v.x, v.y, v.z]).collect();
        let raw = indices.to_vec();
        if let Some(&neg) = raw.iter().find(|&&i| i < 0) {
            godot_error!("LodBuilder.build : indice négatif {neg}");
            return VarDictionary::new();
        }
        let indices: Vec<u32> = if raw.is_empty() { (0..positions.len() as u32).collect() } else { raw.iter().map(|&i| i as u32).collect() };
        if self.max_levels < 0 {
            godot_error!("LodBuilder.build : max_levels {} négatif", self.max_levels);
            return VarDictionary::new();
        }
        let params = LodParams {
            ratio: self.ratio,
            max_levels: self.max_levels as usize,
            target_error: self.target_error,
            weld_tolerance: self.weld_tolerance,
            lock_border: self.lock_border,
            ..LodParams::default()
        };
        let chain = match LodChain::build(&IndexedMesh::new(positions, indices), &params) {
            Ok(c) => c,
            Err(e) => {
                godot_error!("LodBuilder.build : {e}");
                return VarDictionary::new();
            }
        };
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
    /// Emplacement libéré par `remove_cell` (réemployé par le prochain `add_cell`).
    removed: bool,
}

/// `tool` : la sélection des niveaux tourne aussi dans l'éditeur.
#[derive(GodotClass)]
#[class(tool, base = RefCounted, init)]
pub struct LodSelector {
    chains: Vec<LodChain>,
    cells: Vec<Cell>,
    /// Emplacements libérés, réemployés en pile (les ids restent stables côté
    /// moteur : un id libéré ne redevient valide qu'après réattribution).
    free: Vec<usize>,
    selector: Selector,
    camera: Option<Camera>,
    last: Vec<i32>,
    /// Marge d'hystérésis relative sur l'erreur projetée (0 = aucune).
    #[var]
    #[init(val = DEFAULT_HYSTERESIS)]
    hysteresis: f32,
}

#[godot_api]
impl LodSelector {
    /// Déclare une chaîne par ses erreurs cumulées (niveau 0 = 0) ; renvoie son
    /// id, ou −1 si les erreurs ne sont pas croissantes / finies.
    #[func]
    fn add_chain(&mut self, errors: PackedFloat32Array) -> i64 {
        let errors = errors.to_vec();
        if errors.is_empty() || errors.iter().any(|e| !e.is_finite() || *e < 0.0) || errors.windows(2).any(|w| w[1] <= w[0]) {
            godot_error!("LodSelector.add_chain : erreurs attendues finies, ≥ 0 et strictement croissantes ({errors:?})");
            return -1;
        }
        let levels = errors.into_iter().map(|e| LodLevel { mesh: IndexedMesh::default(), error: e, source_indices: Vec::new() }).collect();
        self.chains.push(LodChain { levels, center: [0.0; 3], radius: 0.0 });
        (self.chains.len() - 1) as i64
    }

    /// Enregistre une cellule (sphère englobante monde) d'une chaîne ; renvoie
    /// son id. `chain` < 0 = pas de chaîne (toujours niveau 0) ; un id de
    /// chaîne inconnu est refusé (−1). L'ajout est incrémental : les décisions
    /// des autres cellules ne sont pas recalculées.
    #[func]
    fn add_cell(&mut self, chain: i64, center: Vector3, radius: f32) -> i64 {
        let chain = if chain < 0 {
            None
        } else if (chain as usize) < self.chains.len() {
            Some(chain as usize)
        } else {
            godot_error!("LodSelector.add_cell : chaîne {chain} inconnue ({} déclarées)", self.chains.len());
            return -1;
        };
        if !radius.is_finite() || radius < 0.0 {
            godot_error!("LodSelector.add_cell : rayon {radius} invalide");
            return -1;
        }
        let cell = Cell { chain, center: [center.x, center.y, center.z], radius, removed: false };
        if let Some(i) = self.free.pop() {
            // réemploi d'un emplacement libéré : décision en cache oubliée, niveau
            // « dernier servi » remis à inconnu pour que select_changed le renvoie
            self.cells[i] = cell;
            self.selector.reset(i);
            self.last[i] = -1;
            return i as i64;
        }
        self.cells.push(cell);
        self.selector.push();
        self.last.push(-1);
        (self.cells.len() - 1) as i64
    }

    /// Retire une cellule : son id n'est plus évalué (−1 dans `select_changed`)
    /// et sera réattribué par un `add_cell` ultérieur. Renvoie false pour un id
    /// inconnu ou déjà retiré (erreur Godot, pas de panique).
    #[func]
    fn remove_cell(&mut self, id: i64) -> bool {
        let Some(c) = usize::try_from(id).ok().and_then(|i| self.cells.get_mut(i)) else {
            godot_error!("LodSelector.remove_cell : cellule {id} inconnue ({} emplacements)", self.cells.len());
            return false;
        };
        if c.removed {
            godot_error!("LodSelector.remove_cell : cellule {id} déjà retirée");
            return false;
        }
        c.removed = true;
        c.chain = None;
        self.selector.reset(id as usize);
        self.free.push(id as usize);
        true
    }

    /// Nombre de cellules vivantes (emplacements libérés exclus).
    #[func]
    fn cell_count(&self) -> i64 {
        (self.cells.len() - self.free.len()) as i64
    }

    /// À appeler chaque image avant `select_changed`. Caméra invalide (fov hors
    /// ]0°, 180°[, hauteur ≤ 0) : erreur et image ignorée.
    #[func]
    fn begin_frame(&mut self, cam_pos: Vector3, fov_y_deg: f32, viewport_height: f32) {
        if fov_y_deg.is_nan() || fov_y_deg <= 0.0 || fov_y_deg >= 180.0 || viewport_height.is_nan() || viewport_height <= 0.0 || !cam_pos.is_finite() {
            godot_error!("LodSelector.begin_frame : caméra invalide (fov {fov_y_deg}°, hauteur {viewport_height}, position {cam_pos})");
            return;
        }
        let cam = Camera { position: [cam_pos.x, cam_pos.y, cam_pos.z], fov_y: fov_y_deg.to_radians(), viewport_height };
        self.selector.hysteresis = self.hysteresis;
        self.selector.begin_frame(&cam);
        self.camera = Some(cam);
    }

    /// Niveau par cellule (−1 = inchangé depuis le dernier appel) ; les
    /// cellules sans chaîne restent au niveau 0. Tableau vide sans
    /// `begin_frame` préalable ou si le seuil n'est pas > 0.
    #[func]
    fn select_changed(&mut self, threshold_px: f32) -> PackedInt32Array {
        if threshold_px.is_nan() || threshold_px <= 0.0 {
            godot_error!("LodSelector.select_changed : seuil {threshold_px} px invalide (attendu > 0)");
            return PackedInt32Array::new();
        }
        let Some(cam) = self.camera.as_ref() else {
            return PackedInt32Array::new();
        };
        let sel = &mut self.selector;
        let mut out = PackedInt32Array::new();
        out.resize(self.cells.len());
        for (i, c) in self.cells.iter().enumerate() {
            if c.removed {
                out[i] = -1;
                continue;
            }
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
        self.selector.evaluated as i64
    }
}
