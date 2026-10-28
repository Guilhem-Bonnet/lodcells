//! Cellules spatiales : les exemplaires d'un même maillage sont groupés par
//! cellule (grille régulière en x/z) pour être dessinés en un lot (un
//! MultiMesh par maillage × niveau × cellule côté moteur) tout en gardant un
//! culling par cellule. Le pas de grille est aussi celui d'un éventuel
//! décalage d'origine.

use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CellKey {
    pub x: i32,
    pub z: i32,
}

/// Exemplaire d'un maillage : transformation 3×4 (colonnes = base, dernière = position).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Instance {
    pub mesh: u32,
    pub transform: [[f32; 3]; 4],
}

impl Instance {
    pub fn position(&self) -> [f32; 3] {
        self.transform[3]
    }
    /// Échelle maximale de la base (pour mettre le rayon d'erreur à l'échelle,
    /// ce que nanite-webgpu oublie).
    pub fn max_scale(&self) -> f32 {
        self.transform[..3].iter().map(|c| (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()).fold(0.0, f32::max)
    }
}

#[derive(Debug, Clone)]
pub struct CellGrid {
    pub cell_size: f32,
    /// (maillage, cellule) → indices d'exemplaires, ordre déterministe.
    pub buckets: BTreeMap<(u32, CellKey), Vec<u32>>,
}

impl CellGrid {
    pub fn key_of(cell_size: f32, p: [f32; 3]) -> CellKey {
        CellKey { x: (p[0] / cell_size).floor() as i32, z: (p[2] / cell_size).floor() as i32 }
    }

    pub fn build(cell_size: f32, instances: &[Instance]) -> CellGrid {
        let mut buckets: BTreeMap<(u32, CellKey), Vec<u32>> = BTreeMap::new();
        for (i, inst) in instances.iter().enumerate() {
            buckets.entry((inst.mesh, Self::key_of(cell_size, inst.position()))).or_default().push(i as u32);
        }
        CellGrid { cell_size, buckets }
    }

    pub fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// Centre (x, z) d'une cellule, y = 0.
    pub fn cell_center(&self, k: CellKey) -> [f32; 3] {
        [(k.x as f32 + 0.5) * self.cell_size, 0.0, (k.z as f32 + 0.5) * self.cell_size]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(mesh: u32, x: f32, z: f32) -> Instance {
        Instance { mesh, transform: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [x, 0.0, z]] }
    }

    #[test]
    fn groups_by_mesh_and_cell() {
        let g = CellGrid::build(100.0, &[at(0, 10.0, 10.0), at(0, 90.0, 20.0), at(0, 150.0, 20.0), at(1, 10.0, 10.0), at(0, -1.0, 0.0)]);
        assert_eq!(g.bucket_count(), 4);
        assert_eq!(g.buckets[&(0, CellKey { x: 0, z: 0 })], vec![0, 1]);
        assert_eq!(g.buckets[&(0, CellKey { x: 1, z: 0 })], vec![2]);
        assert_eq!(g.buckets[&(0, CellKey { x: -1, z: 0 })], vec![4]);
        assert_eq!(g.cell_center(CellKey { x: 1, z: 0 }), [150.0, 0.0, 50.0]);
    }

    #[test]
    fn max_scale_reads_the_basis() {
        let mut i = at(0, 0.0, 0.0);
        i.transform[1] = [0.0, 2.5, 0.0];
        assert!((i.max_scale() - 2.5).abs() < 1e-6);
    }
}
