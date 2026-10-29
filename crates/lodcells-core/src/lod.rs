//! Chaîne de LOD discrets d'un maillage.
//!
//! Chaque niveau est une simplification du précédent par `meshopt-rs` (port
//! Rust pur de meshoptimizer : effondrements d'arêtes à métrique quadrique).
//! Trois choix propres à lodcells, tous prouvés par les tests de ce module :
//!
//! - **coutures d'attributs** : la simplification travaille en topologie
//!   entièrement soudée par position (`weld_tolerance`) — aussi agressive que
//!   possible : laisser à meshoptimizer ses « wedges » classe `Locked` tout
//!   sommet à trois faces dures ou plus et fige les bâtiments à faces plates
//!   (mesuré sur un projet hôte : 270 cellules sur 332 sans aucun niveau). Les
//!   indices de chaque niveau sont ensuite ramenés au tampon d'origine coin par
//!   coin : parmi les sommets d'origine de la position (wedges), celui dont la
//!   normale de fan — calculée sur les triangles d'origine, positions seules —
//!   est la plus proche de la normale géométrique du triangle. Un triangle
//!   garde ainsi les normales/UV de sa face (`seams_keep_face_normals`) ; le
//!   niveau 0 reprend les indices d'origine tels quels.
//! - **bords verrouillés** : meshopt-rs 0.1.2 n'expose ni `LockBorder` ni
//!   verrou par sommet (vérifié dans sa source : `simplify` n'a pas d'options).
//!   On s'appuie sur sa classification : un sommet à plus de deux demi-arêtes
//!   ouvertes est `Locked` et n'est jamais déplacé. Une pré-passe ajoute, par
//!   sommet de bord (arête portée par un seul triangle, en topologie soudée),
//!   une « ailette » : un triangle vers deux sommets fictifs partagés par
//!   toutes les ailettes. Le sommet de bord gagne deux demi-arêtes ouvertes,
//!   les fictifs en ont autant que d'ailettes : tous `Locked`, les ailettes ne
//!   s'effondrent jamais et sont retirées après simplification. Les positions
//!   de bord sont alors identiques à tous les niveaux
//!   (`border_positions_are_identical_at_all_levels`).
//! - **erreur mesurée** : l'erreur propre d'une étape est la distance maximale
//!   d'une position retirée à la *surface* simplifiée (triangle le plus
//!   proche, via une grille de triangles), nulle sur un plan
//!   (`flat_plane_has_zero_error`). L'erreur d'un niveau est additive donc
//!   strictement croissante, ce qui rend la sélection monotone en distance
//!   (idée reprise de nanite-webgpu / Godotite).

use std::collections::HashSet;

use meshopt_rs::simplify::simplify;
use meshopt_rs::vertex::Position;

use crate::mesh::{border_flags, IndexedMesh};

#[derive(Debug, Clone, Copy)]
struct P([f32; 3]);

impl Position for P {
    fn pos(&self) -> [f32; 3] {
        self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LodLevel {
    /// Géométrie du niveau en positions soudées, compactée (sert aux mesures :
    /// bords, erreur, validation).
    pub mesh: IndexedMesh,
    /// Erreur géométrique cumulée, en unités du maillage (rayon de déviation).
    pub error: f32,
    /// Mêmes triangles que `mesh.indices`, mais indexés dans le tampon de
    /// sommets **d'origine** : un moteur construit le niveau avec les attributs
    /// d'origine (normales, UV, matériau) sans toucher à ses sommets. Chaque
    /// coin garde le sommet d'origine de sa face (coutures respectées).
    pub source_indices: Vec<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LodChain {
    /// Niveau 0 = maillage d'origine (erreur 0), puis de plus en plus grossier.
    pub levels: Vec<LodLevel>,
    /// Sphère englobante du niveau 0 (centre, rayon), partagée par tous les niveaux.
    pub center: [f32; 3],
    pub radius: f32,
}

#[derive(Debug, Clone)]
pub struct LodParams {
    /// Fraction de triangles visée à chaque niveau (0,5 = moitié).
    pub ratio: f32,
    /// Nombre maximal de niveaux au-delà de l'origine.
    pub max_levels: usize,
    /// Arrêt si un niveau retire moins de cette fraction des triangles.
    pub min_gain: f32,
    /// Erreur maximale tolérée par effondrement d'arête (métrique quadrique de
    /// meshoptimizer), relative au rayon de la sphère englobante, convertie en
    /// unité meshoptimizer (relative à la plus grande dimension de la boîte).
    /// 1,0 (défaut) = pas de limite pratique : le nombre de triangles visé est
    /// atteint et c'est l'erreur *mesurée* du niveau qui pilote la sélection
    /// (mesuré sur un projet hôte : 0,05 laisse 270 cellules sur 332 sans aucun
    /// niveau grossier). Valeurs utiles pour borner chaque pas : 0,01 à 0,1.
    pub target_error: f32,
    /// Soudure par position avant simplification (0 = désactivée).
    pub weld_tolerance: f32,
    /// Verrouille les sommets de bord (arêtes ouvertes) : positions de bord
    /// identiques à tous les niveaux, donc pas de fissure entre tuiles
    /// voisines à des niveaux différents.
    pub lock_border: bool,
}

impl Default for LodParams {
    fn default() -> Self {
        LodParams { ratio: 0.5, max_levels: 4, min_gain: 0.1, target_error: 1.0, weld_tolerance: 1e-4, lock_border: true }
    }
}

/// Entrée refusée par [`LodChain::build`]. Ces cas faisaient paniquer
/// meshopt-rs (assertion `target_index_count <= indices.len()` pour un ratio
/// ≥ 1, indexation hors tampon) ou produisaient une chaîne vide.
#[derive(Debug, Clone, PartialEq)]
pub enum BuildError {
    /// Aucun sommet, aucun triangle, ou plus aucun triangle après soudure.
    EmptyMesh,
    /// Le nombre d'indices n'est pas un multiple de 3.
    IndexCountNotTriangles { count: usize },
    /// Un indice dépasse le tampon de sommets.
    IndexOutOfRange { index: u32, vertices: usize },
    /// `ratio` doit être dans ]0, 1[.
    InvalidRatio(f32),
    /// `target_error`, `min_gain` ou `weld_tolerance` négatif ou NaN.
    InvalidParam(&'static str),
}

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BuildError::EmptyMesh => write!(f, "maillage vide (aucun sommet ou aucun triangle)"),
            BuildError::IndexCountNotTriangles { count } => write!(f, "{count} indices : pas un multiple de 3"),
            BuildError::IndexOutOfRange { index, vertices } => write!(f, "indice {index} hors du tampon de {vertices} sommets"),
            BuildError::InvalidRatio(r) => write!(f, "ratio {r} hors de ]0, 1["),
            BuildError::InvalidParam(p) => write!(f, "paramètre {p} négatif ou NaN"),
        }
    }
}

impl std::error::Error for BuildError {}

impl LodParams {
    /// Vérifie les paramètres sans construire.
    pub fn validate(&self) -> Result<(), BuildError> {
        if self.ratio.is_nan() || self.ratio <= 0.0 || self.ratio >= 1.0 {
            return Err(BuildError::InvalidRatio(self.ratio));
        }
        for (name, v) in [("target_error", self.target_error), ("min_gain", self.min_gain), ("weld_tolerance", self.weld_tolerance)] {
            if v.is_nan() || v < 0.0 {
                return Err(BuildError::InvalidParam(name));
            }
        }
        Ok(())
    }
}

/// Entrée du simplificateur, fixe pour toute la chaîne : positions soudées,
/// « wedges » (sommets d'origine de chaque position soudée) et leur normale
/// de fan, sommets fictifs des ailettes de verrouillage.
struct Workset {
    /// Positions soudées, indexées par sommet soudé.
    welded: Vec<[f32; 3]>,
    /// Sommets d'origine de chaque sommet soudé (CSR : `wedges[wedge_start[w]..wedge_start[w+1]]`).
    wedge_start: Vec<u32>,
    wedges: Vec<u32>,
    /// Normale de fan (aire pondérée) de chaque sommet d'origine, d'après les
    /// triangles d'origine : un sommet d'une face dure ne voit que sa face.
    normals: Vec<[f32; 3]>,
    /// Positions des sommets fictifs (dans la boîte, distinctes de tout sommet).
    dummies: Option<[[f32; 3]; 2]>,
    /// Erreur par effondrement en unité meshoptimizer.
    meshopt_error: f32,
}

impl Workset {
    /// Indices d'origine d'un niveau donné en indices soudés : pour chaque coin
    /// d'un triangle, le sommet d'origine de la position dont la normale de fan
    /// est la plus proche de la normale géométrique du triangle (les coutures
    /// d'attributs — faces dures, îlots UV — gardent ainsi le sommet de leur
    /// face ; une seule couture par position : choix direct).
    fn source_indices(&self, welded_indices: &[u32]) -> Vec<u32> {
        let mut out = Vec::with_capacity(welded_indices.len());
        for t in welded_indices.as_chunks::<3>().0 {
            let (a, b, c) = (self.welded[t[0] as usize], self.welded[t[1] as usize], self.welded[t[2] as usize]);
            let n = cross(sub(b, a), sub(c, a));
            for &w in t {
                let (s, e) = (self.wedge_start[w as usize] as usize, self.wedge_start[w as usize + 1] as usize);
                let cands = &self.wedges[s..e];
                let best = if cands.len() == 1 {
                    cands[0]
                } else {
                    // cosinus de l'angle, pas le produit scalaire brut : les
                    // normales de fan sont pondérées par l'aire, une grande
                    // face inclinée l'emporterait sinon sur une petite face
                    // exacte (`wedge_choice_ignores_fan_area`)
                    let cos = |x: u32| {
                        let v = self.normals[x as usize];
                        dot(n, v) / len2(v).sqrt().max(f32::MIN_POSITIVE)
                    };
                    *cands.iter().max_by(|&&x, &&y| cos(x).total_cmp(&cos(y))).unwrap()
                };
                out.push(best);
            }
        }
        out
    }
}

impl LodChain {
    /// Construit la chaîne. Le niveau 0 est le maillage d'origine, moins les
    /// triangles dégénérés par la soudure. Refuse (sans paniquer) un maillage
    /// vide, des indices hors plage ou non triangulaires, et des paramètres
    /// hors domaine.
    pub fn build(source: &IndexedMesh, params: &LodParams) -> Result<LodChain, BuildError> {
        params.validate()?;
        if source.positions.is_empty() || source.indices.is_empty() {
            return Err(BuildError::EmptyMesh);
        }
        if !source.indices.len().is_multiple_of(3) {
            return Err(BuildError::IndexCountNotTriangles { count: source.indices.len() });
        }
        if let Some(&bad) = source.indices.iter().find(|&&i| i as usize >= source.positions.len()) {
            return Err(BuildError::IndexOutOfRange { index: bad, vertices: source.positions.len() });
        }
        // soudure : positions canoniques + table sommet d'origine → sommet soudé
        let (welded, remap) = if params.weld_tolerance > 0.0 {
            source.weld(params.weld_tolerance)
        } else {
            (source.clone(), (0..source.positions.len() as u32).collect())
        };
        // niveau 0 : triangles d'origine non dégénérés après soudure ; normales
        // de fan des sommets d'origine (aire pondérée, non normalisées)
        let mut base = Vec::with_capacity(source.indices.len());
        let mut normals = vec![[0.0f32; 3]; source.positions.len()];
        for t in source.indices.as_chunks::<3>().0 {
            let (a, b, c) = (remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]);
            if a != b && b != c && a != c {
                base.extend_from_slice(t);
                let (pa, pb, pc) = (welded.positions[a as usize], welded.positions[b as usize], welded.positions[c as usize]);
                let n = cross(sub(pb, pa), sub(pc, pa));
                for &i in t {
                    let v = &mut normals[i as usize];
                    *v = [v[0] + n[0], v[1] + n[1], v[2] + n[2]];
                }
            }
        }
        if base.is_empty() {
            return Err(BuildError::EmptyMesh);
        }
        // wedges : sommets d'origine référencés, groupés par sommet soudé
        let mut referenced = vec![false; source.positions.len()];
        for &i in &base {
            referenced[i as usize] = true;
        }
        let mut wedge_start = vec![0u32; welded.positions.len() + 1];
        for (i, &w) in remap.iter().enumerate() {
            if referenced[i] {
                wedge_start[w as usize + 1] += 1;
            }
        }
        for w in 0..welded.positions.len() {
            wedge_start[w + 1] += wedge_start[w];
        }
        let mut fill = wedge_start.clone();
        let mut wedges = vec![0u32; wedge_start[welded.positions.len()] as usize];
        for (i, &w) in remap.iter().enumerate() {
            if referenced[i] {
                wedges[fill[w as usize] as usize] = i as u32;
                fill[w as usize] += 1;
            }
        }
        let (center, radius) = welded.bounding_sphere();
        let (lo, hi) = welded.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
        let extent = (0..3).map(|a| hi[a] - lo[a]).fold(0.0f32, f32::max).max(1e-6);
        let ws = Workset {
            dummies: if params.lock_border { dummy_positions(&welded.positions, lo, hi) } else { None },
            welded: welded.positions,
            wedge_start,
            wedges,
            normals,
            // meshoptimizer mesure l'erreur dans la boîte remise à l'échelle
            // [0, 1] par sa plus grande dimension ; notre paramètre est relatif
            // au rayon. (L'ancienne version passait target_error × rayon, soit
            // une limite effectivement infinie, d'où le défaut 1,0 qui reproduit
            // ce comportement.)
            meshopt_error: params.target_error * radius.max(1e-6) / extent,
        };
        let floor = 1e-7 * radius.max(1.0);
        let mut prev_w: Vec<u32> = base.iter().map(|&i| remap[i as usize]).collect();
        let mut levels = vec![LodLevel { mesh: compact(&ws.welded, &prev_w), error: 0.0, source_indices: base }];
        for _ in 0..params.max_levels {
            let prev = levels.last().unwrap();
            let want = (prev_w.len() as f32 * params.ratio) as usize / 3 * 3;
            if want < 3 || prev.mesh.triangle_count() < 2 {
                break;
            }
            let next_w = simplify_step(&ws, &prev_w, want);
            let removed = prev_w.len().saturating_sub(next_w.len()) as f32 / prev_w.len() as f32;
            if next_w.len() < 3 || removed < params.min_gain {
                break;
            }
            let step = step_error(&ws.welded, &prev_w, &next_w);
            let error = prev.error + step.max(floor);
            levels.push(LodLevel { mesh: compact(&ws.welded, &next_w), error, source_indices: ws.source_indices(&next_w) });
            prev_w = next_w;
        }
        let chain = LodChain { levels, center, radius };
        debug_assert!(crate::validate::validate(&chain).is_empty(), "{:?}", crate::validate::validate(&chain));
        Ok(chain)
    }

    pub fn len(&self) -> usize {
        self.levels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.levels.is_empty()
    }
}

/// Deux positions dans la boîte `[lo, hi]` qui ne coïncident avec aucun
/// sommet (meshoptimizer regroupe les sommets de position *identique* :
/// un fictif confondu avec un vrai sommet en changerait la classification).
/// `None` si l'on n'en trouve pas (boîte dégénérée en un point).
fn dummy_positions(positions: &[[f32; 3]], lo: [f32; 3], hi: [f32; 3]) -> Option<[[f32; 3]; 2]> {
    let bits = |p: &[f32; 3]| [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
    let taken: HashSet<[u32; 3]> = positions.iter().map(bits).collect();
    let mut found: Vec<[f32; 3]> = Vec::new();
    for k in 0..64u32 {
        let p = [0, 1, 2].map(|a| {
            let f = (0.37 + k as f32 * 0.2371 + a as f32 * 0.3113).fract();
            lo[a] + (hi[a] - lo[a]) * f
        });
        if !taken.contains(&bits(&p)) && found.iter().all(|q| bits(q) != bits(&p)) {
            found.push(p);
            if found.len() == 2 {
                return Some([found[0], found[1]]);
            }
        }
    }
    None
}

/// Une étape de simplification en topologie soudée (indices soudés en entrée
/// et en sortie). Les sommets de bord reçoivent une ailette vers les deux
/// sommets fictifs : classés `Locked` par meshoptimizer, ils ne bougent pas ;
/// les ailettes (qui ne s'effondrent jamais, tous leurs sommets étant
/// `Locked`) comptent dans la cible puis sont retirées.
fn simplify_step(ws: &Workset, indices: &[u32], want: usize) -> Vec<u32> {
    let n = ws.welded.len() as u32;
    let mut verts: Vec<P> = ws.welded.iter().map(|&p| P(p)).collect();
    let mut idx: Vec<u32> = indices.to_vec();
    let mut fins = 0usize;
    if let Some([d1, d2]) = ws.dummies {
        let border = border_flags(indices, ws.welded.len());
        let mut done = vec![false; ws.welded.len()];
        for &w in indices {
            if border[w as usize] && !done[w as usize] {
                done[w as usize] = true;
                idx.extend_from_slice(&[w, n, n + 1]);
                fins += 1;
            }
        }
        if fins > 0 {
            verts.push(P(d1));
            verts.push(P(d2));
        }
    }
    let target = (want + 3 * fins).min(idx.len());
    let mut out = vec![0u32; idx.len()];
    let k = simplify(&mut out, &idx, &verts, target, ws.meshopt_error);
    out.truncate(k);
    let mut res = Vec::with_capacity(out.len());
    for t in out.as_chunks::<3>().0 {
        if t.iter().all(|&i| i < n) {
            res.extend_from_slice(t);
        }
    }
    res
}

/// Erreur géométrique d'une étape : distance maximale d'une position retirée
/// (référencée par `prev`, plus par `next`) à la surface de `next` — triangle
/// le plus proche, recherche en grille. Indices dans l'espace soudé.
fn step_error(pos: &[[f32; 3]], prev: &[u32], next: &[u32]) -> f32 {
    let mut in_next = vec![false; pos.len()];
    for &i in next {
        in_next[i as usize] = true;
    }
    let mut seen = vec![false; pos.len()];
    let mut removed: Vec<[f32; 3]> = Vec::new();
    for &i in prev {
        if !in_next[i as usize] && !seen[i as usize] {
            seen[i as usize] = true;
            removed.push(pos[i as usize]);
        }
    }
    if removed.is_empty() || next.is_empty() {
        return 0.0;
    }
    let mut grid = TriGrid::new(pos, next);
    removed.iter().map(|p| grid.nearest_d2(*p)).fold(0.0f32, f32::max).sqrt()
}

/// Grille uniforme de triangles pour les requêtes « surface la plus proche »
/// de `step_error` : chaque triangle est rangé dans toutes les cellules que
/// couvre sa boîte ; la requête parcourt des anneaux de cellules de rayon
/// croissant et s'arrête dès que le meilleur candidat est plus près que tout
/// ce qu'un anneau suivant peut contenir.
struct TriGrid<'a> {
    pos: &'a [[f32; 3]],
    tris: &'a [u32],
    min: [f32; 3],
    h: f32,
    dims: [usize; 3],
    cells: Vec<Vec<u32>>,
    /// Dernier numéro de requête ayant testé chaque triangle (pas de double test).
    stamp: Vec<u32>,
    epoch: u32,
}

impl<'a> TriGrid<'a> {
    const MAX_DIM: usize = 64;

    fn new(pos: &'a [[f32; 3]], tris: &'a [u32]) -> Self {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for &i in tris {
            let p = pos[i as usize];
            for a in 0..3 {
                min[a] = min[a].min(p[a]);
                max[a] = max[a].max(p[a]);
            }
        }
        let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
        let longest = extent.iter().cloned().fold(0.0f32, f32::max).max(1e-6);
        let n_tris = (tris.len() / 3).max(1);
        // ~2 triangles par cellule occupée, sans dépasser MAX_DIM cellules par axe
        let volume = extent.iter().map(|e| e.max(longest * 1e-3)).product::<f32>();
        let h = (volume / (n_tris as f32 / 2.0)).cbrt().max(longest / Self::MAX_DIM as f32);
        let dims = [0, 1, 2].map(|a| ((extent[a] / h).floor() as usize + 1).min(Self::MAX_DIM));
        let mut g = TriGrid { pos, tris, min, h, dims, cells: vec![Vec::new(); dims[0] * dims[1] * dims[2]], stamp: vec![0; n_tris], epoch: 0 };
        for (t, tri) in tris.as_chunks::<3>().0.iter().enumerate() {
            let (a, b, c) = (pos[tri[0] as usize], pos[tri[1] as usize], pos[tri[2] as usize]);
            let lo = g.cell_of([0, 1, 2].map(|k| a[k].min(b[k]).min(c[k])));
            let hi = g.cell_of([0, 1, 2].map(|k| a[k].max(b[k]).max(c[k])));
            for z in lo[2]..=hi[2] {
                for y in lo[1]..=hi[1] {
                    for x in lo[0]..=hi[0] {
                        let k = g.flat([x, y, z]);
                        g.cells[k].push(t as u32);
                    }
                }
            }
        }
        g
    }

    fn cell_of(&self, p: [f32; 3]) -> [usize; 3] {
        [0, 1, 2].map(|a| (((p[a] - self.min[a]) / self.h).floor().max(0.0) as usize).min(self.dims[a] - 1))
    }

    fn flat(&self, c: [usize; 3]) -> usize {
        (c[2] * self.dims[1] + c[1]) * self.dims[0] + c[0]
    }

    /// Carré de la distance de `p` au triangle le plus proche. Tout triangle
    /// non encore rencontré à l'anneau r est entièrement dans des cellules
    /// d'anneau > r, donc à au moins r·h de p (y compris p hors de la boîte).
    fn nearest_d2(&mut self, p: [f32; 3]) -> f32 {
        self.epoch += 1;
        let c = self.cell_of(p);
        let max_r = *self.dims.iter().max().unwrap();
        let mut best = f32::INFINITY;
        for r in 0..=max_r {
            let lo = [0, 1, 2].map(|a| c[a].saturating_sub(r));
            let hi = [0, 1, 2].map(|a| (c[a] + r).min(self.dims[a] - 1));
            for z in lo[2]..=hi[2] {
                for y in lo[1]..=hi[1] {
                    for x in lo[0]..=hi[0] {
                        let on_ring = [x, y, z].iter().zip(&c).any(|(&v, &cv)| v.abs_diff(cv) == r);
                        if !on_ring {
                            continue;
                        }
                        let k = self.flat([x, y, z]);
                        for ci in 0..self.cells[k].len() {
                            let t = self.cells[k][ci] as usize;
                            if self.stamp[t] == self.epoch {
                                continue;
                            }
                            self.stamp[t] = self.epoch;
                            let tri = &self.tris[3 * t..3 * t + 3];
                            let d2 = point_triangle_d2(p, self.pos[tri[0] as usize], self.pos[tri[1] as usize], self.pos[tri[2] as usize]);
                            best = best.min(d2);
                        }
                    }
                }
            }
            if best <= (r as f32 * self.h).powi(2) {
                break;
            }
        }
        best
    }
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn len2(a: [f32; 3]) -> f32 {
    dot(a, a)
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Carré de la distance d'un point au triangle `abc` (régions de Voronoï du
/// triangle, Ericson, *Real-Time Collision Detection* §5.1.5). Triangle
/// dégénéré : distance au plus proche de ses sommets.
pub(crate) fn point_triangle_d2(p: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f32 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return len2(ap);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return len2(bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return len2(sub(ap, [ab[0] * v, ab[1] * v, ab[2] * v]));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return len2(cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return len2(sub(ap, [ac[0] * w, ac[1] * w, ac[2] * w]));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let bc = sub(c, b);
        return len2(sub(bp, [bc[0] * w, bc[1] * w, bc[2] * w]));
    }
    let denom = va + vb + vc;
    if denom.abs() <= f32::MIN_POSITIVE || !denom.is_finite() {
        return len2(ap).min(len2(bp)).min(len2(cp));
    }
    let v = vb / denom;
    let w = vc / denom;
    let q = [ab[0] * v + ac[0] * w, ab[1] * v + ac[1] * w, ab[2] * v + ac[2] * w];
    len2(sub(ap, q))
}

/// Maillage compact d'indices soudés : garde seulement les sommets
/// référencés, renumérote.
fn compact(welded: &[[f32; 3]], indices: &[u32]) -> IndexedMesh {
    let mut remap = vec![u32::MAX; welded.len()];
    let mut positions = Vec::new();
    let indices = indices
        .iter()
        .map(|&i| {
            let slot = &mut remap[i as usize];
            if *slot == u32::MAX {
                positions.push(welded[i as usize]);
                *slot = (positions.len() - 1) as u32;
            }
            *slot
        })
        .collect();
    IndexedMesh { positions, indices }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::validate::validate;

    /// Grille N×N (2·(N−1)² triangles) de hauteur `height(x, z)`.
    pub(crate) fn grid_with(n: usize, height: impl Fn(f32, f32) -> f32) -> IndexedMesh {
        let mut positions = Vec::new();
        for z in 0..n {
            for x in 0..n {
                positions.push([x as f32, height(x as f32, z as f32), z as f32]);
            }
        }
        let mut indices = Vec::new();
        for z in 0..n - 1 {
            for x in 0..n - 1 {
                let a = (z * n + x) as u32;
                let b = a + 1;
                let c = a + n as u32;
                let d = c + 1;
                indices.extend_from_slice(&[a, c, b, b, c, d]);
            }
        }
        IndexedMesh::new(positions, indices)
    }

    /// Grille plane N×N : coplanaire, donc fortement simplifiable sans erreur
    /// de surface.
    pub(crate) fn grid(n: usize) -> IndexedMesh {
        grid_with(n, |_, _| 0.0)
    }

    /// Grille bosselée (amplitude ~1 pour un pas de 1) : les bords ont de
    /// vraies raisons d'être simplifiés si rien ne les verrouille.
    pub(crate) fn bumpy(n: usize) -> IndexedMesh {
        grid_with(n, |x, z| (x * 0.7).sin() * (z * 0.5).cos() * 1.5 + (x * 0.13 + z * 0.21).sin())
    }

    fn loose() -> LodParams {
        LodParams { target_error: 1.0, ..Default::default() }
    }

    #[test]
    fn chain_is_monotonic_and_shrinks() {
        let chain = LodChain::build(&grid(32), &loose()).unwrap();
        assert!(chain.len() >= 3, "niveaux : {}", chain.len());
        assert_eq!(chain.levels[0].error, 0.0);
        for w in chain.levels.windows(2) {
            assert!(w[1].mesh.triangle_count() < w[0].mesh.triangle_count(), "chaque niveau est plus grossier");
            assert!(w[1].error > w[0].error, "erreur strictement croissante");
            let max = *w[1].mesh.indices.iter().max().unwrap() as usize;
            assert!(max < w[1].mesh.positions.len(), "indices renumérotés");
        }
        assert!(chain.radius > 0.0);
        assert!(validate(&chain).is_empty());
    }

    /// Verrouillage des bords : les positions de bord (arêtes ouvertes) sont
    /// exactement les mêmes à tous les niveaux, sur une grille bosselée où le
    /// simplificateur voudrait les réduire.
    #[test]
    fn border_positions_are_identical_at_all_levels() {
        let src = bumpy(32);
        let chain = LodChain::build(&src, &loose()).unwrap();
        assert!(chain.len() >= 3, "niveaux : {}", chain.len());
        let base = chain.levels[0].mesh.border_positions();
        assert_eq!(base.len(), 4 * 31, "grille 32×32 : 4·31 sommets de bord");
        for (i, l) in chain.levels.iter().enumerate() {
            assert_eq!(l.mesh.border_positions(), base, "niveau {i} : bords différents");
            assert_eq!(l.mesh.border_vertex_count(), 124, "niveau {i}");
        }
        let last = chain.levels.last().unwrap().mesh.triangle_count();
        assert!(last < chain.levels[0].mesh.triangle_count() / 4, "l'intérieur se simplifie quand même ({last} triangles)");
        eprintln!("bosselée 32×32 : {} → {} triangles en {} niveaux, bords 124 → 124", chain.levels[0].mesh.triangle_count(), last, chain.len() - 1);
        assert!(validate(&chain).is_empty());
    }

    /// Sans verrouillage, meshopt-rs 0.1.2 réduit bien les bords (il ne fait
    /// que les pondérer) : c'est la pré-passe qui les tient.
    #[test]
    fn without_lock_border_the_outline_is_simplified() {
        let src = bumpy(32);
        let chain = LodChain::build(&src, &LodParams { lock_border: false, ..loose() }).unwrap();
        let first = chain.levels[0].mesh.border_vertex_count();
        let last = chain.levels.last().unwrap().mesh.border_vertex_count();
        assert_eq!(first, 124);
        assert!(last < first, "bords non verrouillés : {first} → {last}");
        eprintln!("bosselée 32×32 sans verrou : bords {first} → {last}");
    }

    /// Le verrouillage s'applique aussi aux trous intérieurs (eux aussi des
    /// arêtes ouvertes) et après soudure (coutures ≠ bords).
    #[test]
    fn holes_are_borders_too_and_seams_are_not() {
        // grille 24×24 avec un trou 4×4 au centre
        let mut g = bumpy(24);
        let mut kept = Vec::new();
        for t in g.indices.as_chunks::<3>().0 {
            let hole = t.iter().all(|&i| {
                let (x, z) = ((i % 24) as i32, (i / 24) as i32);
                (10..=14).contains(&x) && (10..=14).contains(&z)
            });
            if !hole {
                kept.extend_from_slice(t);
            }
        }
        g.indices = kept;
        // couture : on duplique la colonne x = 6 (sommets 6 + 24·z) pour les
        // triangles à sa droite → deux sommets par position, mais pas un bord
        let n = g.positions.len() as u32;
        let mut dup = std::collections::HashMap::new();
        for z in 0..24u32 {
            dup.insert(6 + 24 * z, n + z);
            g.positions.push(g.positions[(6 + 24 * z) as usize]);
        }
        for t in g.indices.as_chunks_mut::<3>().0 {
            let right = t.iter().any(|&i| i % 24 > 6 && i % 24 < 12);
            if right {
                for i in t.iter_mut() {
                    if let Some(&d) = dup.get(i) {
                        *i = d;
                    }
                }
            }
        }
        let chain = LodChain::build(&g, &loose()).unwrap();
        assert!(chain.len() >= 3, "niveaux : {}", chain.len());
        let base = chain.levels[0].mesh.border_positions();
        // 4·23 sommets du contour + 16 du trou ; la couture n'en est pas
        assert_eq!(base.len(), 4 * 23 + 16, "bords = contour + trou, pas la couture");
        for l in &chain.levels {
            assert_eq!(l.mesh.border_positions(), base);
        }
        assert!(validate(&chain).is_empty());
    }

    /// Erreur juste : un plan parfaitement plat se simplifie sans erreur de
    /// surface (l'ancienne mesure « sommet conservé le plus proche » donnait
    /// 1/4/11/26 sur un plan).
    #[test]
    fn flat_plane_has_zero_error() {
        let chain = LodChain::build(&grid(32), &loose()).unwrap();
        assert!(chain.len() >= 4, "niveaux : {}", chain.len());
        for l in &chain.levels[1..] {
            assert!(l.error < 1e-4 * chain.radius, "erreur {} sur un plan (rayon {})", l.error, chain.radius);
        }
        eprintln!("plan 32×32 : erreurs {:?}", chain.levels.iter().map(|l| l.error).collect::<Vec<_>>());
    }

    /// L'erreur mesurée borne bien la déviation : sur la grille bosselée, les
    /// erreurs sont strictement croissantes, et inférieures à l'amplitude des
    /// bosses (2,5) : une erreur à la surface, pas au sommet le plus proche.
    #[test]
    fn bumpy_error_is_a_surface_distance() {
        let chain = LodChain::build(&bumpy(32), &loose()).unwrap();
        let errs: Vec<f32> = chain.levels.iter().map(|l| l.error).collect();
        eprintln!("bosselée 32×32 : erreurs {errs:?}");
        for w in errs.windows(2) {
            assert!(w[1] > w[0]);
        }
        // chaque pas est borné par le creux-à-crête des bosses (5), l'erreur
        // cumulée par max_levels fois cette valeur
        let steps: Vec<f32> = errs.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(steps.iter().all(|&s| s < 5.0), "pas {steps:?}");
        let last = *errs.last().unwrap();
        assert!(last > 0.05 && last < 4.0 * 5.0, "erreur finale {last}");
        // l'erreur « au sommet le plus proche » de l'ancienne version serait ≥ 1
        // dès le premier niveau (le pas de grille) : ici le premier niveau est
        // bien en dessous
        assert!(errs[1] < 1.0, "niveau 1 : {}", errs[1]);
    }

    #[test]
    fn point_triangle_distance_regions() {
        let (a, b, c) = ([0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]);
        let d = |p| point_triangle_d2(p, a, b, c).sqrt();
        assert!((d([0.5, 0.5, 3.0]) - 3.0).abs() < 1e-6, "intérieur : hauteur");
        assert!((d([-1.0, -1.0, 0.0]) - 2f32.sqrt()).abs() < 1e-6, "sommet a");
        assert!((d([1.0, -1.0, 0.0]) - 1.0).abs() < 1e-6, "arête ab");
        assert!((d([2.0, 2.0, 0.0]) - 2f32.sqrt()).abs() < 1e-6, "arête bc");
        assert!((d([3.0, 0.0, 4.0]) - (1.0f32 + 16.0).sqrt()).abs() < 1e-6, "sommet b en hauteur");
        // dégénéré : les 3 sommets confondus
        assert!((point_triangle_d2([1.0, 0.0, 0.0], a, a, a) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn step_error_is_distance_to_surface() {
        // deux triangles plats (carré) ; sommet 4 retiré, à 2 au-dessus du centre
        let pos = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.5, 0.5, 2.0]];
        let prev = [0, 1, 4, 1, 3, 4, 3, 2, 4, 2, 0, 4];
        let next = [0, 1, 2, 2, 1, 3];
        assert!((step_error(&pos, &prev, &next) - 2.0).abs() < 1e-6);
        // sommet retiré dans le plan : erreur 0 (la version « sommet le plus
        // proche » donnait 0,707)
        let pos2 = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.5, 0.5, 0.0]];
        assert_eq!(step_error(&pos2, &prev, &next), 0.0);
    }

    /// Coutures : un « toit » (deux pans à 90°, arête faîtière dupliquée avec
    /// des normales différentes) se simplifie avec une borne d'erreur par
    /// effondrement (le pli n'est pas replié), et chaque coin de chaque
    /// triangle de chaque niveau garde un sommet d'origine dont la normale est
    /// celle de sa face (dot > 0,9). Puis un cube à 24 sommets : exact au
    /// niveau 0 (l'ancienne version prenait le premier sommet de la position,
    /// donc la normale d'une autre face) ; aux niveaux grossiers, un cube
    /// replié n'a plus de normale de face exacte, on vérifie que le sommet
    /// choisi est le meilleur disponible à cette position.
    #[test]
    fn seams_keep_face_normals() {
        // pan A : y = 0, x ∈ [0, 16], z ∈ [0, 8] (normale +y) ; pan B : z = 8,
        // y ∈ [0, 8] (normale +z) ; l'arête z = 8 est dupliquée.
        let n = 17usize;
        let m = 9usize;
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();
        for z in 0..m {
            for x in 0..n {
                positions.push([x as f32, 0.0, z as f32]);
                normals.push([0.0, 1.0, 0.0]);
            }
        }
        let off = positions.len() as u32;
        for y in 0..m {
            for x in 0..n {
                positions.push([x as f32, y as f32, 8.0]);
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        for (base, flip) in [(0u32, false), (off, true)] {
            for r in 0..m - 1 {
                for x in 0..n - 1 {
                    let a = base + (r * n + x) as u32;
                    let b = a + 1;
                    let c = a + n as u32;
                    let d = c + 1;
                    if flip {
                        indices.extend_from_slice(&[a, b, c, b, d, c]);
                    } else {
                        indices.extend_from_slice(&[a, c, b, b, c, d]);
                    }
                }
            }
        }
        let src = IndexedMesh::new(positions, indices);
        check_face_normals(&src, &normals, &LodParams { target_error: 0.02, ..Default::default() }, true);
        // cube à 24 sommets (4 par face, normales de face)
        let mut cp = Vec::new();
        let mut cn = Vec::new();
        let mut ci = Vec::new();
        for (nrm, u, v) in [
            ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            ([0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
            ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
            ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
            ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            ([0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        ] {
            let b = cp.len() as u32;
            for (su, sv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                cp.push([0, 1, 2].map(|k| nrm[k] + u[k] * su + v[k] * sv));
                cn.push(nrm);
            }
            ci.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
        }
        let cube = IndexedMesh::new(cp, ci);
        assert_eq!(cube.positions.len(), 24);
        check_face_normals(&cube, &cn, &loose(), false);
    }

    fn check_face_normals(src: &IndexedMesh, normals: &[[f32; 3]], params: &LodParams, strict: bool) {
        let chain = LodChain::build(src, params).unwrap();
        assert!(chain.len() >= 2, "niveaux : {}", chain.len());
        if strict {
            assert!(chain.len() >= 3, "niveaux : {}", chain.len());
        }
        let mut slivers = 0;
        for (li, l) in chain.levels.iter().enumerate() {
            assert_eq!(l.source_indices.len(), l.mesh.indices.len());
            for t in l.source_indices.as_chunks::<3>().0 {
                let (a, b, c) = (src.positions[t[0] as usize], src.positions[t[1] as usize], src.positions[t[2] as usize]);
                let (e1, e2) = (sub(b, a), sub(c, a));
                let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
                let len = len2(n).sqrt();
                if len <= 1e-6 {
                    // triangle d'aire nulle (trois sommets de bord alignés : la
                    // métrique quadrique est nulle dans le plan) : invisible,
                    // sans normale de face à vérifier
                    slivers += 1;
                    continue;
                }
                let n = [n[0] / len, n[1] / len, n[2] / len];
                for &i in t {
                    let d = dot(n, normals[i as usize]);
                    if strict || li == 0 {
                        assert!(d > 0.9, "niveau {li} : normale de sommet {:?} vs face {n:?} (dot {d})", normals[i as usize]);
                    }
                    // meilleur sommet disponible à cette position
                    let p = src.positions[i as usize];
                    for (j, q) in src.positions.iter().enumerate() {
                        if *q == p {
                            assert!(dot(n, normals[j]) <= d + 1e-6, "niveau {li} : sommet {j} meilleur que {i} pour la face {n:?}");
                        }
                    }
                }
            }
        }
        assert!(validate(&chain).is_empty());
        eprintln!("coutures : {} niveaux, triangles {:?}, {slivers} triangles d'aire nulle", chain.len(), chain.levels.iter().map(|l| l.mesh.triangle_count()).collect::<Vec<_>>());
    }

    /// Le choix du sommet d'origine par coin compare des *directions* : une
    /// petite face dont la normale est exactement celle du triangle doit
    /// l'emporter sur une grande face inclinée à 60°, quelle que soit l'aire
    /// de leur fan (la normale de fan est pondérée par l'aire).
    #[test]
    fn wedge_choice_ignores_fan_area() {
        // position 0 soudée, deux wedges : sommet 0 (fan minuscule, normale +y)
        // et sommet 1 (fan 1 000× plus grand, normale à 60° de +y)
        let ws = Workset {
            welded: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            wedge_start: vec![0, 2, 3, 4],
            wedges: vec![0, 1, 2, 3],
            normals: vec![[0.0, 0.01, 0.0], [0.0, 5.0, 8.66], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            dummies: None,
            meshopt_error: 1.0,
        };
        // triangle (0, 2, 1) : normale cross((0,0,1), (1,0,0)) = +y
        assert_eq!(ws.source_indices(&[0, 2, 1]), vec![0, 3, 2]);
    }

    #[test]
    fn source_indices_point_back_to_original_vertices() {
        // grille 8×8 avec un sommet dupliqué (couture) : indice 64 = copie du sommet 9
        let mut g = grid(8);
        g.positions.push(g.positions[9]);
        let n = g.positions.len();
        let chain = LodChain::build(&g, &loose()).unwrap();
        for l in &chain.levels {
            assert_eq!(l.source_indices.len(), l.mesh.indices.len());
            for (k, &s) in l.source_indices.iter().enumerate() {
                assert!((s as usize) < n, "indice source hors tampon");
                // la position d'origine référencée est celle du sommet compacté
                let p = l.mesh.positions[l.mesh.indices[k] as usize];
                assert_eq!(g.positions[s as usize], p);
            }
        }
        // la copie non référencée ne l'est toujours pas
        assert!(chain.levels.iter().all(|l| !l.source_indices.contains(&64)));
    }

    #[test]
    fn rejects_bad_inputs_instead_of_panicking() {
        let g = grid(8);
        let p = loose();
        // ratio ≥ 1 : meshopt-rs assertait `target_index_count <= indices.len()`
        assert_eq!(LodChain::build(&g, &LodParams { ratio: 2.0, ..p.clone() }).unwrap_err(), BuildError::InvalidRatio(2.0));
        assert_eq!(LodChain::build(&g, &LodParams { ratio: 1.0, ..p.clone() }).unwrap_err(), BuildError::InvalidRatio(1.0));
        assert_eq!(LodChain::build(&g, &LodParams { ratio: 0.0, ..p.clone() }).unwrap_err(), BuildError::InvalidRatio(0.0));
        assert!(LodChain::build(&g, &LodParams { ratio: f32::NAN, ..p.clone() }).is_err());
        assert_eq!(LodChain::build(&g, &LodParams { target_error: -1.0, ..p.clone() }).unwrap_err(), BuildError::InvalidParam("target_error"));
        // indices hors plage / non triangulaires / maillage vide
        let mut bad = grid(8);
        bad.indices[5] = 999;
        assert_eq!(LodChain::build(&bad, &p).unwrap_err(), BuildError::IndexOutOfRange { index: 999, vertices: 64 });
        let mut odd = grid(8);
        odd.indices.pop();
        assert!(matches!(LodChain::build(&odd, &p).unwrap_err(), BuildError::IndexCountNotTriangles { .. }));
        assert_eq!(LodChain::build(&IndexedMesh::default(), &p).unwrap_err(), BuildError::EmptyMesh);
        assert_eq!(LodChain::build(&IndexedMesh::new(vec![[0.0; 3]], vec![]), &p).unwrap_err(), BuildError::EmptyMesh);
        // tous les triangles dégénérés après soudure : vide, pas de panique
        assert_eq!(LodChain::build(&IndexedMesh::new(vec![[0.0; 3], [0.0; 3], [1e-6, 0.0, 0.0]], vec![0, 1, 2]), &p).unwrap_err(), BuildError::EmptyMesh);
        // max_levels = 0 : un seul niveau, pas une erreur
        assert_eq!(LodChain::build(&g, &LodParams { max_levels: 0, ..p.clone() }).unwrap().len(), 1);
        // un seul triangle : un seul niveau
        assert_eq!(LodChain::build(&IndexedMesh::new(vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], vec![0, 1, 2]), &p).unwrap().len(), 1);
    }

    /// target_error est maintenant appliqué (en unité meshoptimizer) : une
    /// tolérance infime sur la grille bosselée bloque la simplification, une
    /// large la laisse faire.
    #[test]
    fn target_error_limits_collapses() {
        let src = bumpy(32);
        let tight = LodChain::build(&src, &LodParams { target_error: 1e-6, ..Default::default() }).unwrap();
        let wide = LodChain::build(&src, &loose()).unwrap();
        assert!(tight.len() < wide.len(), "serré {} niveaux, large {}", tight.len(), wide.len());
    }

    /// Banc : grilles bosselées 16 k (128²) et 64 k (256²) sommets, chaîne
    /// complète (4 niveaux, bords verrouillés, erreur de surface). À lancer en
    /// release : `cargo test --release -p lodcells-core -- --ignored bench --nocapture`.
    #[test]
    #[ignore]
    fn bench_16k_64k_vertices() {
        for n in [128usize, 256] {
            let src = bumpy(n);
            let t0 = std::time::Instant::now();
            let chain = LodChain::build(&src, &loose()).unwrap();
            let dt = t0.elapsed();
            let tris: Vec<usize> = chain.levels.iter().map(|l| l.mesh.triangle_count()).collect();
            let borders: Vec<usize> = chain.levels.iter().map(|l| l.mesh.border_vertex_count()).collect();
            eprintln!(
                "bench {}×{} = {} sommets : {:.1} ms, triangles {:?}, bords {:?}, erreurs {:?}",
                n,
                n,
                n * n,
                dt.as_secs_f64() * 1e3,
                tris,
                borders,
                chain.levels.iter().map(|l| l.error).collect::<Vec<_>>()
            );
            assert!(borders.iter().all(|&b| b == borders[0]));
        }
    }
}
