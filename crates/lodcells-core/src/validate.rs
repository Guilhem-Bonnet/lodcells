//! Validateur (idée reprise de Godotite) : invariants d'une chaîne de LOD.

use crate::lod::LodChain;

#[derive(Debug, Clone, PartialEq)]
pub enum Issue {
    EmptyChain,
    NonMonotonicError { level: usize },
    NotCoarser { level: usize },
    IndexOutOfRange { level: usize, index: u32 },
    DegenerateTriangle { level: usize, tri: usize },
}

pub fn validate(chain: &LodChain) -> Vec<Issue> {
    let mut issues = Vec::new();
    if chain.levels.is_empty() {
        issues.push(Issue::EmptyChain);
        return issues;
    }
    for (i, l) in chain.levels.iter().enumerate() {
        if i > 0 {
            let p = &chain.levels[i - 1];
            if l.error <= p.error {
                issues.push(Issue::NonMonotonicError { level: i });
            }
            if l.mesh.triangle_count() >= p.mesh.triangle_count() {
                issues.push(Issue::NotCoarser { level: i });
            }
        }
        let n = l.mesh.positions.len() as u32;
        for (t, tri) in l.mesh.indices.as_chunks::<3>().0.iter().enumerate() {
            for &ix in tri {
                if ix >= n {
                    issues.push(Issue::IndexOutOfRange { level: i, index: ix });
                }
            }
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                issues.push(Issue::DegenerateTriangle { level: i, tri: t });
            }
        }
    }
    issues
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lod::LodLevel;
    use crate::mesh::IndexedMesh;

    #[test]
    fn detects_broken_invariants() {
        let tri = IndexedMesh::new(vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], vec![0, 1, 2]);
        let bad = IndexedMesh::new(vec![[0.0; 3]], vec![0, 0, 7]);
        let chain = LodChain {
            levels: vec![LodLevel { mesh: tri.clone(), error: 0.0, source_indices: vec![0, 1, 2] }, LodLevel { mesh: bad, error: 0.0, source_indices: vec![] }],
            center: [0.0; 3],
            radius: 1.0,
        };
        let issues = validate(&chain);
        assert!(issues.contains(&Issue::NonMonotonicError { level: 1 }));
        assert!(issues.contains(&Issue::NotCoarser { level: 1 }));
        assert!(issues.contains(&Issue::IndexOutOfRange { level: 1, index: 7 }));
        assert!(issues.contains(&Issue::DegenerateTriangle { level: 1, tri: 0 }));
        assert!(validate(&LodChain { levels: vec![LodLevel { mesh: tri, error: 0.0, source_indices: vec![0, 1, 2] }], center: [0.0; 3], radius: 1.0 }).is_empty());
    }
}
