//! Chaîne de LOD discrets d'un maillage.
//!
//! Chaque niveau est une simplification du précédent par `meshopt-rs` (port
//! Rust pur de meshoptimizer : effondrements d'arêtes à métrique quadrique,
//! bords de maillage préservés en priorité). L'erreur d'un niveau est
//! **additive** : erreur propre + erreur du niveau précédent, donc
//! strictement croissante — propriété qui rend la sélection monotone en
//! distance (idée reprise de nanite-webgpu / Godotite).
//!
//! L'erreur propre d'une étape est mesurée ici (meshopt-rs 0.1 ne la renvoie
//! pas) : distance maximale d'un sommet retiré au sommet conservé le plus
//! proche — une borne simple et déterministe de la déviation de surface.

use meshopt_rs::simplify::simplify;
use meshopt_rs::vertex::Position;

use crate::mesh::IndexedMesh;

#[derive(Debug, Clone, Copy)]
struct P([f32; 3]);

impl Position for P {
    fn pos(&self) -> [f32; 3] {
        self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LodLevel {
    pub mesh: IndexedMesh,
    /// Erreur géométrique cumulée, en unités du maillage (rayon de déviation).
    pub error: f32,
    /// Mêmes triangles que `mesh.indices`, mais indexés dans le tampon de
    /// sommets **d'origine** (avant soudure) : un moteur peut ainsi
    /// construire le niveau avec les attributs d'origine (normales, UV,
    /// matériau) sans toucher à ses sommets. Aux coutures soudées, c'est le
    /// premier sommet d'origine de la position qui est retenu.
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
    /// Erreur maximale tolérée par étape, relative à l'échelle du maillage
    /// (rayon de la sphère englobante).
    pub target_error: f32,
    /// Soudure par position avant simplification (0 = désactivée).
    pub weld_tolerance: f32,
}

impl Default for LodParams {
    fn default() -> Self {
        LodParams { ratio: 0.5, max_levels: 4, min_gain: 0.1, target_error: 0.05, weld_tolerance: 1e-4 }
    }
}

/// Entrée refusée par [`LodChain::build`]. Ces cas faisaient paniquer
/// meshopt-rs (assertion `target_index_count <= indices.len()` pour un ratio
/// ≥ 1, indexation hors tampon) ou produisaient une chaîne vide.
#[derive(Debug, Clone, PartialEq)]
pub enum BuildError {
    /// Aucun sommet ou aucun triangle.
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

impl LodChain {
    /// Construit la chaîne. Le niveau 0 est le maillage soudé (ou tel quel).
    /// Refuse (sans paniquer) un maillage vide, des indices hors plage ou non
    /// triangulaires, et des paramètres hors domaine.
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
        // base soudée + représentant d'origine de chaque sommet soudé
        let (base, representative) = if params.weld_tolerance > 0.0 {
            let (welded, remap) = source.weld(params.weld_tolerance);
            let mut rep = vec![u32::MAX; welded.positions.len()];
            for (src, &w) in remap.iter().enumerate() {
                if rep[w as usize] == u32::MAX {
                    rep[w as usize] = src as u32;
                }
            }
            (welded, rep)
        } else {
            (source.clone(), (0..source.positions.len() as u32).collect())
        };
        let (center, radius) = base.bounding_sphere();
        // chaque niveau garde, par sommet compacté, son indice dans la base soudée
        let mut to_base: Vec<u32> = (0..base.positions.len() as u32).collect();
        let base_src: Vec<u32> = base.indices.iter().map(|&i| representative[i as usize]).collect();
        let mut levels = vec![LodLevel { mesh: base, error: 0.0, source_indices: base_src }];
        for _ in 0..params.max_levels {
            let prev = levels.last().unwrap();
            let want = ((prev.mesh.indices.len() as f32) * params.ratio) as usize / 3 * 3;
            // garde-fou : meshopt-rs exige want ≤ indices (ratio déjà validé < 1)
            if want < 3 || want > prev.mesh.indices.len() || prev.mesh.triangle_count() < 2 {
                break;
            }
            let indices = simplify_indices(&prev.mesh, want, params.target_error * radius.max(1e-6));
            let removed = prev.mesh.indices.len().saturating_sub(indices.len()) as f32 / prev.mesh.indices.len() as f32;
            if indices.len() < 3 || removed < params.min_gain {
                break;
            }
            let step = step_error(&prev.mesh, &indices);
            let source_indices = indices.iter().map(|&i| representative[to_base[i as usize] as usize]).collect();
            let (mesh, kept) = compact(&prev.mesh, indices);
            to_base = kept.iter().map(|&k| to_base[k as usize]).collect();
            let error = prev.error + step.max(1e-7 * radius.max(1.0));
            levels.push(LodLevel { mesh, error, source_indices });
        }
        Ok(LodChain { levels, center, radius })
    }

    pub fn len(&self) -> usize {
        self.levels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.levels.is_empty()
    }
}

fn simplify_indices(m: &IndexedMesh, target_indices: usize, target_error: f32) -> Vec<u32> {
    let verts: Vec<P> = m.positions.iter().map(|&p| P(p)).collect();
    let mut out = vec![0u32; m.indices.len()];
    let n = simplify(&mut out, &m.indices, &verts, target_indices, target_error);
    out.truncate(n);
    out
}

/// Distance max d'un sommet retiré au sommet conservé le plus proche (le
/// maillage simplifié référence un sous-ensemble des sommets d'origine).
fn step_error(src: &IndexedMesh, kept_indices: &[u32]) -> f32 {
    let mut kept = vec![false; src.positions.len()];
    for &i in kept_indices {
        kept[i as usize] = true;
    }
    let kept_pts: Vec<[f32; 3]> = src.positions.iter().zip(&kept).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
    if kept_pts.is_empty() {
        return 0.0;
    }
    let grid = PointGrid::new(&kept_pts);
    let mut worst = 0.0f32;
    for (p, k) in src.positions.iter().zip(&kept) {
        if !*k {
            worst = worst.max(grid.nearest_d2(*p));
        }
    }
    worst.sqrt()
}

/// Grille uniforme de points pour les requêtes « plus proche voisin » de
/// `step_error` : la version naïve était en O(retirés × conservés), soit
/// plusieurs secondes par tuile de terrain de 16 k sommets.
struct PointGrid {
    pts: Vec<[f32; 3]>,
    min: [f32; 3],
    h: f32,
    dims: [usize; 3],
    /// Par cellule : indices des points qu'elle contient.
    cells: Vec<Vec<u32>>,
}

impl PointGrid {
    const MAX_DIM: usize = 64;

    fn new(pts: &[[f32; 3]]) -> Self {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for p in pts {
            for a in 0..3 {
                min[a] = min[a].min(p[a]);
                max[a] = max[a].max(p[a]);
            }
        }
        let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
        let longest = extent.iter().cloned().fold(0.0f32, f32::max).max(1e-6);
        // ~2 points par cellule occupée, sans dépasser MAX_DIM cellules par axe.
        let volume = extent.iter().map(|e| e.max(longest * 1e-3)).product::<f32>();
        let h = (volume / (pts.len() as f32 / 2.0)).cbrt().max(longest / Self::MAX_DIM as f32);
        let dims = [0, 1, 2].map(|a| ((extent[a] / h).floor() as usize + 1).min(Self::MAX_DIM));
        let mut g = PointGrid { pts: pts.to_vec(), min, h, dims, cells: vec![Vec::new(); dims[0] * dims[1] * dims[2]] };
        for (i, p) in pts.iter().enumerate() {
            let k = g.flat(g.cell_of(*p));
            g.cells[k].push(i as u32);
        }
        g
    }

    fn cell_of(&self, p: [f32; 3]) -> [usize; 3] {
        [0, 1, 2].map(|a| (((p[a] - self.min[a]) / self.h).floor().max(0.0) as usize).min(self.dims[a] - 1))
    }

    fn flat(&self, c: [usize; 3]) -> usize {
        (c[2] * self.dims[1] + c[1]) * self.dims[0] + c[0]
    }

    /// Carré de la distance de `p` au point le plus proche de la grille :
    /// anneaux de cellules de rayon croissant (Tchebychev), arrêt dès que le
    /// meilleur candidat est plus près que tout ce que l'anneau suivant peut
    /// contenir.
    fn nearest_d2(&self, p: [f32; 3]) -> f32 {
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
                        for &i in &self.cells[self.flat([x, y, z])] {
                            let q = self.pts[i as usize];
                            let d2 = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2);
                            best = best.min(d2);
                        }
                    }
                }
            }
            // Tout point d'un anneau > r est à au moins r·h de p (p est dans la cellule c).
            if best <= (r as f32 * self.h).powi(2) {
                break;
            }
        }
        best
    }
}

/// Garde seulement les sommets référencés, renumérote ; renvoie aussi, pour
/// chaque sommet compacté, son indice dans `src`.
fn compact(src: &IndexedMesh, indices: Vec<u32>) -> (IndexedMesh, Vec<u32>) {
    let mut remap = vec![u32::MAX; src.positions.len()];
    let mut positions = Vec::new();
    let mut kept = Vec::new();
    let indices = indices
        .into_iter()
        .map(|i| {
            let slot = &mut remap[i as usize];
            if *slot == u32::MAX {
                positions.push(src.positions[i as usize]);
                kept.push(i);
                *slot = (positions.len() - 1) as u32;
            }
            *slot
        })
        .collect();
    (IndexedMesh { positions, indices }, kept)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Grille plane N×N (2·(N−1)² triangles) : coplanaire, donc fortement
    /// simplifiable sans erreur de surface, mais avec une erreur de
    /// déplacement de sommets non nulle (notre borne).
    pub(crate) fn grid(n: usize) -> IndexedMesh {
        let mut positions = Vec::new();
        for z in 0..n {
            for x in 0..n {
                positions.push([x as f32, 0.0, z as f32]);
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

    #[test]
    fn chain_is_monotonic_and_shrinks() {
        let chain = LodChain::build(&grid(32), &LodParams { target_error: 1.0, ..Default::default() }).unwrap();
        assert!(chain.len() >= 3, "niveaux : {}", chain.len());
        assert_eq!(chain.levels[0].error, 0.0);
        for w in chain.levels.windows(2) {
            assert!(w[1].mesh.triangle_count() < w[0].mesh.triangle_count(), "chaque niveau est plus grossier");
            assert!(w[1].error > w[0].error, "erreur strictement croissante");
            let max = *w[1].mesh.indices.iter().max().unwrap() as usize;
            assert!(max < w[1].mesh.positions.len(), "indices renumérotés");
        }
        assert!(chain.radius > 0.0);
    }

    #[test]
    fn border_outline_is_preserved() {
        let g = grid(16);
        let chain = LodChain::build(&g, &LodParams { target_error: 1.0, ..Default::default() }).unwrap();
        let coarse = &chain.levels.last().unwrap().mesh;
        // les 4 coins de la grille sont des sommets de bord : ils survivent
        for corner in [[0.0, 0.0, 0.0], [15.0, 0.0, 0.0], [0.0, 0.0, 15.0], [15.0, 0.0, 15.0]] {
            assert!(coarse.positions.contains(&corner), "coin {corner:?} perdu");
        }
        // et l'enveloppe ne rétrécit pas
        let (lo, hi) = coarse.aabb().unwrap();
        assert_eq!((lo, hi), ([0.0, 0.0, 0.0], [15.0, 0.0, 15.0]));
    }

    #[test]
    fn source_indices_point_back_to_original_vertices() {
        // grille 8×8 avec un sommet dupliqué (couture) : indice 64 = copie du sommet 9
        let mut g = grid(8);
        g.positions.push(g.positions[9]);
        let n = g.positions.len();
        let chain = LodChain::build(&g, &LodParams { target_error: 1.0, ..Default::default() }).unwrap();
        for l in &chain.levels {
            assert_eq!(l.source_indices.len(), l.mesh.indices.len());
            for (k, &s) in l.source_indices.iter().enumerate() {
                assert!((s as usize) < n, "indice source hors tampon");
                // la position d'origine référencée est celle du sommet compacté
                let p = l.mesh.positions[l.mesh.indices[k] as usize];
                assert_eq!(g.positions[s as usize], p);
            }
        }
        // la copie soudée n'est jamais référencée : son représentant est le sommet 9
        assert!(chain.levels.iter().all(|l| !l.source_indices.contains(&64)));
    }

    #[test]
    fn rejects_bad_inputs_instead_of_panicking() {
        let g = grid(8);
        let p = LodParams { target_error: 1.0, ..Default::default() };
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
        // max_levels = 0 : un seul niveau, pas une erreur
        assert_eq!(LodChain::build(&g, &LodParams { max_levels: 0, ..p.clone() }).unwrap().len(), 1);
    }

    /// Mesure (pas une garantie) : meshopt-rs 0.1 n'a pas d'option de
    /// verrouillage des bords, il ne fait que les pondérer. Les sommets de
    /// bord d'une grille 16×16 sont donc bien réduits par la simplification ;
    /// seuls les coins et l'enveloppe survivent (cf. `border_outline_is_preserved`).
    #[test]
    fn border_vertices_are_not_locked_with_meshopt_rs_0_1() {
        let chain = LodChain::build(&grid(16), &LodParams { target_error: 1.0, ..Default::default() }).unwrap();
        let first = chain.levels[0].mesh.border_vertex_count();
        let last = chain.levels.last().unwrap().mesh.border_vertex_count();
        assert_eq!(first, 60, "grille 16×16 : 4·15 sommets de bord");
        assert!(last < first, "bords non verrouillés : {first} → {last} sommets de bord");
        eprintln!("sommets de bord grille 16×16 : {first} → {last} (niveau {})", chain.len() - 1);
    }

    #[test]
    fn step_error_bounds_removed_vertices() {
        let src = IndexedMesh::new(vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.5, 0.5, 2.0]], vec![0, 1, 2]);
        // le sommet 3 (hors des indices gardés) est à 2,06 du plus proche conservé
        assert!((step_error(&src, &[0, 1, 2]) - (0.25f32 + 0.25 + 4.0).sqrt()).abs() < 1e-5);
    }
}
