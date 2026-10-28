//! Sélection du niveau de détail par erreur projetée en pixels.
//!
//! Erreur projetée d'un exemplaire (Ponchio, reprise par nanite-webgpu et
//! Godotite) : `px = error · k / d`, où `d` est la distance de la caméra à la
//! **coque** de l'exemplaire (distance au centre moins le rayon, bornée) et
//! `k = hauteur_écran / (2·tan(fov/2))`. Le niveau retenu est le plus grossier
//! dont l'erreur projetée reste sous le seuil.
//!
//! « Slack » (Godotite) : comme `px` est monotone en `d`, chaque décision reste
//! valable tant que la caméra n'a pas parcouru la distance qui la sépare du
//! seuil le plus proche ; les exemplaires dont le slack n'est pas épuisé sont
//! sautés — une caméra immobile ne coûte rien.

use crate::lod::LodChain;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub position: [f32; 3],
    /// Champ vertical, en radians.
    pub fov_y: f32,
    pub viewport_height: f32,
}

impl Camera {
    /// Facteur écran k = h / (2 tan(fov/2)).
    pub fn screen_factor(&self) -> f32 {
        self.viewport_height / (2.0 * (self.fov_y * 0.5).tan())
    }
}

/// Niveau choisi pour un exemplaire, avec le slack restant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Selection {
    pub level: usize,
    /// Distance (unités monde) que la caméra peut parcourir sans changer la décision.
    pub slack: f32,
}

/// Distance de la caméra à la coque d'une sphère (centre, rayon), bornée.
pub fn hull_distance(cam: [f32; 3], center: [f32; 3], radius: f32) -> f32 {
    let d = ((cam[0] - center[0]).powi(2) + (cam[1] - center[1]).powi(2) + (cam[2] - center[2]).powi(2)).sqrt();
    (d - radius).max(0.01)
}

/// Erreur projetée en pixels d'une erreur monde `error` vue à la distance `d`.
pub fn projected_error(error: f32, d: f32, screen_factor: f32) -> f32 {
    error * screen_factor / d
}

/// Choisit le niveau le plus grossier dont l'erreur projetée ≤ `threshold_px`,
/// pour un exemplaire de `chain` dont la sphère (centre monde, rayon mis à
/// l'échelle) est donnée. Renvoie aussi le slack.
pub fn select_level(chain: &LodChain, center: [f32; 3], radius: f32, cam: &Camera, threshold_px: f32) -> Selection {
    let k = cam.screen_factor();
    let d = hull_distance(cam.position, center, radius);
    // distance seuil de chaque niveau i ≥ 1 : d_i = error_i · k / threshold ;
    // le niveau i est admissible si d ≥ d_i (erreurs croissantes ⇒ d_i croissantes).
    let mut level = 0;
    let mut next_switch = f32::INFINITY; // d à partir de laquelle on passerait au niveau suivant
    let mut prev_switch = 0.0f32; // d en dessous de laquelle on reviendrait au niveau précédent
    for (i, l) in chain.levels.iter().enumerate().skip(1) {
        let d_i = projected_error(l.error, 1.0, k) / threshold_px;
        if d >= d_i {
            level = i;
            prev_switch = d_i;
        } else {
            next_switch = d_i;
            break;
        }
    }
    let slack = (d - prev_switch).min(next_switch - d).max(0.0);
    Selection { level, slack }
}

/// Sélecteur incrémental : mémorise la dernière décision et la position de la
/// caméra par exemplaire, ne recalcule que ceux dont le slack est épuisé.
#[derive(Debug, Default)]
pub struct Selector {
    last_cam: Option<[f32; 3]>,
    moved: f32,
    cache: Vec<Option<(Selection, f32)>>, // (décision, slack initial)
    pub evaluated: usize,
}

impl Selector {
    pub fn new(instances: usize) -> Self {
        Selector { cache: vec![None; instances], ..Default::default() }
    }

    /// Doit être appelé une fois par image avant `select` : accumule le
    /// déplacement de la caméra depuis la dernière réévaluation.
    pub fn begin_frame(&mut self, cam: &Camera) {
        if let Some(last) = self.last_cam {
            let p = cam.position;
            self.moved += ((p[0] - last[0]).powi(2) + (p[1] - last[1]).powi(2) + (p[2] - last[2]).powi(2)).sqrt();
        }
        self.last_cam = Some(cam.position);
        self.evaluated = 0;
    }

    /// Niveau de l'exemplaire `i` ; réévalué seulement si nécessaire.
    pub fn select(&mut self, i: usize, chain: &LodChain, center: [f32; 3], radius: f32, cam: &Camera, threshold_px: f32) -> Selection {
        if let Some((sel, slack0)) = self.cache[i] {
            if self.moved < slack0 {
                return Selection { level: sel.level, slack: slack0 - self.moved };
            }
        }
        let sel = select_level(chain, center, radius, cam, threshold_px);
        // le slack est mesuré depuis la position courante : on remet le compteur
        // de déplacement « à zéro » pour cet exemplaire en stockant slack + moved
        self.cache[i] = Some((sel, sel.slack + self.moved));
        self.evaluated += 1;
        sel
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lod::{LodChain, LodLevel};
    use crate::mesh::IndexedMesh;

    fn chain(errors: &[f32]) -> LodChain {
        LodChain {
            levels: errors.iter().map(|&e| LodLevel { mesh: IndexedMesh::default(), error: e }).collect(),
            center: [0.0; 3],
            radius: 1.0,
        }
    }

    fn cam(x: f32) -> Camera {
        Camera { position: [x, 0.0, 0.0], fov_y: std::f32::consts::FRAC_PI_2, viewport_height: 1000.0 }
    }

    #[test]
    fn coarser_with_distance_and_monotonic() {
        let c = chain(&[0.0, 0.01, 0.03, 0.1]);
        // k = 1000/(2·tan 45°) = 500 ; seuil 2 px ⇒ d_1 = 2,5 ; d_2 = 7,5 ; d_3 = 25 (+ rayon 1)
        let mut last = 0;
        for x in [1.0, 2.0, 4.0, 9.0, 30.0, 300.0] {
            let s = select_level(&c, [0.0; 3], 1.0, &cam(x), 2.0);
            assert!(s.level >= last, "à {x} : niveau {} < {last}", s.level);
            last = s.level;
        }
        assert_eq!(select_level(&c, [0.0; 3], 1.0, &cam(1.5), 2.0).level, 0);
        assert_eq!(select_level(&c, [0.0; 3], 1.0, &cam(5.0), 2.0).level, 1);
        assert_eq!(select_level(&c, [0.0; 3], 1.0, &cam(300.0), 2.0).level, 3);
    }

    #[test]
    fn slack_is_distance_to_nearest_switch() {
        let c = chain(&[0.0, 0.01, 0.03]);
        // d = 10 − 1 = 9 ; seuils 2,5 et 7,5 ⇒ niveau 2, slack = 9 − 7,5 = 1,5
        let s = select_level(&c, [0.0; 3], 1.0, &cam(10.0), 2.0);
        assert_eq!(s.level, 2);
        assert!((s.slack - 1.5).abs() < 1e-4, "slack {}", s.slack);
    }

    #[test]
    fn incremental_selector_skips_until_slack_used() {
        let c = chain(&[0.0, 0.01, 0.03]);
        let mut sel = Selector::new(1);
        sel.begin_frame(&cam(10.0));
        let a = sel.select(0, &c, [0.0; 3], 1.0, &cam(10.0), 2.0);
        assert_eq!(sel.evaluated, 1);
        // la caméra avance de 1,0 < slack 1,5 : pas de réévaluation, même niveau
        sel.begin_frame(&cam(9.0));
        let b = sel.select(0, &c, [0.0; 3], 1.0, &cam(9.0), 2.0);
        assert_eq!((a.level, sel.evaluated), (b.level, 0));
        // encore 1,0 : slack épuisé, réévaluation et passage au niveau 1 (d = 7)
        sel.begin_frame(&cam(8.0));
        let c2 = sel.select(0, &c, [0.0; 3], 1.0, &cam(8.0), 2.0);
        assert_eq!((c2.level, sel.evaluated), (1, 1));
    }
}
