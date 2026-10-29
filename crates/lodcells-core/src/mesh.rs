//! Maillage indexé minimal et soudure par position.
//!
//! Les simplificateurs raisonnent en topologie : des sommets dupliqués aux
//! coutures (UV, normales à facettes) bloquent toute fusion d'arêtes
//! (leçon de nanite-webgpu). On soude donc par position sur une grille de
//! tolérance avant de simplifier, en gardant une table de correspondance pour
//! réattribuer les attributs au besoin.

use std::collections::HashMap;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct IndexedMesh {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl IndexedMesh {
    pub fn new(positions: Vec<[f32; 3]>, indices: Vec<u32>) -> Self {
        IndexedMesh { positions, indices }
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Boîte englobante (min, max) ; `None` si vide.
    pub fn aabb(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut it = self.positions.iter();
        let first = *it.next()?;
        Some(it.fold((first, first), |(lo, hi), p| {
            ([lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])], [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])])
        }))
    }

    /// Sphère englobante simple : centre de la boîte, rayon = distance max.
    pub fn bounding_sphere(&self) -> ([f32; 3], f32) {
        let Some((lo, hi)) = self.aabb() else { return ([0.0; 3], 0.0) };
        let c = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5];
        let r = self
            .positions
            .iter()
            .map(|p| ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt())
            .fold(0.0, f32::max);
        (c, r)
    }

    /// Sommets de bord (un drapeau par sommet) : sommets touchés par une
    /// demi-arête sans opposée (arête ouverte, portée par un seul triangle).
    pub fn border_vertices(&self) -> Vec<bool> {
        border_flags(&self.indices, self.positions.len())
    }

    /// Nombre de sommets de bord. Sert à mesurer ce que la simplification fait
    /// aux contours (coutures entre tuiles).
    pub fn border_vertex_count(&self) -> usize {
        self.border_vertices().iter().filter(|b| **b).count()
    }

    /// Positions des sommets de bord, triées (comparaison entre niveaux).
    pub fn border_positions(&self) -> Vec<[f32; 3]> {
        let mut out: Vec<[f32; 3]> = self.positions.iter().zip(self.border_vertices()).filter(|(_, b)| *b).map(|(p, _)| *p).collect();
        out.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        out
    }

    /// Soudure par position : les sommets dont les positions tombent dans la
    /// même cellule de `tolerance` sont fusionnés. Renvoie le maillage soudé
    /// et, pour chaque sommet d'origine, l'indice du sommet soudé.
    pub fn weld(&self, tolerance: f32) -> (IndexedMesh, Vec<u32>) {
        let inv = 1.0 / tolerance.max(1e-9);
        let mut cells: HashMap<[i64; 3], u32> = HashMap::new();
        let mut remap = Vec::with_capacity(self.positions.len());
        let mut positions = Vec::new();
        for p in &self.positions {
            let key = [(p[0] * inv).round() as i64, (p[1] * inv).round() as i64, (p[2] * inv).round() as i64];
            let idx = *cells.entry(key).or_insert_with(|| {
                positions.push(*p);
                (positions.len() - 1) as u32
            });
            remap.push(idx);
        }
        let mut indices: Vec<u32> = self.indices.iter().map(|&i| remap[i as usize]).collect();
        // triangles dégénérés par la soudure : retirés
        let mut kept = Vec::with_capacity(indices.len());
        for t in indices.as_chunks::<3>().0 {
            if t[0] != t[1] && t[1] != t[2] && t[0] != t[2] {
                kept.extend_from_slice(t);
            }
        }
        indices = kept;
        (IndexedMesh { positions, indices }, remap)
    }
}

/// Drapeaux de bord d'une liste de triangles sur `vertex_count` sommets : un
/// sommet est de bord s'il porte une demi-arête `(a, b)` sans opposée `(b, a)`.
/// La topologie est celle des indices tels quels : pour raisonner en
/// positions (coutures d'attributs exclues), passer des indices soudés.
pub fn border_flags(indices: &[u32], vertex_count: usize) -> Vec<bool> {
    let mut edges: HashMap<(u32, u32), u32> = HashMap::with_capacity(indices.len());
    for t in indices.as_chunks::<3>().0 {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *edges.entry((a, b)).or_default() += 1;
        }
    }
    let mut border = vec![false; vertex_count];
    for &(a, b) in edges.keys() {
        if !edges.contains_key(&(b, a)) {
            border[a as usize] = true;
            border[b as usize] = true;
        }
    }
    border
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn weld_merges_duplicates_and_drops_degenerates() {
        // deux triangles partageant une arête, sommets dupliqués à la couture
        let m = IndexedMesh::new(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0001], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
            vec![0, 1, 2, 3, 5, 4],
        );
        let (w, remap) = m.weld(0.001);
        assert_eq!(w.positions.len(), 4);
        assert_eq!(remap, vec![0, 1, 2, 1, 2, 3]);
        assert_eq!(w.triangle_count(), 2);
        // un triangle dégénéré disparaît
        let m2 = IndexedMesh::new(vec![[0.0; 3], [0.0; 3], [1.0, 0.0, 0.0]], vec![0, 1, 2]);
        assert_eq!(m2.weld(0.01).0.triangle_count(), 0);
    }

    #[test]
    fn border_vertex_count_of_a_quad() {
        // deux triangles formant un carré : les 4 sommets sont de bord
        let m = IndexedMesh::new(vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]], vec![0, 1, 2, 2, 1, 3]);
        assert_eq!(m.border_vertex_count(), 4);
        // tétraèdre fermé : aucun
        let t = IndexedMesh::new(vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3]);
        assert_eq!(t.border_vertex_count(), 0);
    }

    #[test]
    fn bounding_sphere_contains_all_points() {
        let m = IndexedMesh::new(vec![[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 3.0, 0.0]], vec![0, 1, 2]);
        let (c, r) = m.bounding_sphere();
        assert_eq!(c, [0.0, 1.5, 0.0]);
        for p in &m.positions {
            let d = ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt();
            assert!(d <= r + 1e-6);
        }
    }
}
