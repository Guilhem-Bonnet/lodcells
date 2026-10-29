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
//!
//! Hystérésis : sans elle, une caméra qui oscille autour d'un seuil fait
//! clignoter le niveau à chaque image (le slack y est nul). Connaissant le
//! niveau précédent `p`, on exige pour s'éloigner vers un niveau plus grossier
//! `d ≥ d_i·(1+h)` et pour revenir vers un niveau plus fin `d < d_i·(1−h)` —
//! soit une marge de `h` sur l'erreur projetée (15 % par défaut).

use crate::lod::LodChain;

/// Marge d'hystérésis par défaut sur l'erreur projetée (fraction).
pub const DEFAULT_HYSTERESIS: f32 = 0.15;

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
/// l'échelle) est donnée. Renvoie aussi le slack. Sans hystérésis.
pub fn select_level(chain: &LodChain, center: [f32; 3], radius: f32, cam: &Camera, threshold_px: f32) -> Selection {
    select_level_with(chain, center, radius, cam, threshold_px, None, 0.0)
}

/// Variante avec hystérésis : `prev` est le niveau précédemment affiché
/// (`None` = première décision, neutre) et `hysteresis` la marge relative.
/// Le slack renvoyé est la distance au seuil le plus proche **de la bande du
/// niveau retenu** (celle qui s'appliquera à la prochaine réévaluation).
pub fn select_level_with(
    chain: &LodChain,
    center: [f32; 3],
    radius: f32,
    cam: &Camera,
    threshold_px: f32,
    prev: Option<usize>,
    hysteresis: f32,
) -> Selection {
    let k = cam.screen_factor();
    let d = hull_distance(cam.position, center, radius);
    let h = if hysteresis.is_finite() { hysteresis.clamp(0.0, 0.9) } else { 0.0 };
    // distance seuil de chaque niveau i ≥ 1 : d_i = error_i · k / threshold ;
    // le niveau i est admissible si d ≥ d_i (erreurs croissantes ⇒ d_i croissantes).
    let d_of = |i: usize| projected_error(chain.levels[i].error, 1.0, k) / threshold_px;
    // seuil effectif vu depuis le niveau `from` : les seuils ≤ from (retour
    // vers le fin) sont abaissés, ceux > from (passage au grossier) relevés ;
    // la suite reste croissante car d_from·(1−h) < d_from+1·(1+h).
    let eff = |i: usize, from: Option<usize>| match from {
        Some(p) if i <= p => d_of(i) * (1.0 - h),
        Some(_) => d_of(i) * (1.0 + h),
        None => d_of(i),
    };
    let mut level = 0;
    for i in 1..chain.levels.len() {
        if d >= eff(i, prev) {
            level = i;
        } else {
            break;
        }
    }
    // bande de validité du niveau retenu, telle qu'elle sera lue la prochaine fois
    let lo = if level == 0 { 0.0 } else { eff(level, Some(level)) };
    let hi = if level + 1 < chain.levels.len() { eff(level + 1, Some(level)) } else { f32::INFINITY };
    let slack = (d - lo).min(hi - d).max(0.0);
    Selection { level, slack }
}

/// Sélecteur incrémental : mémorise la dernière décision et la position de la
/// caméra par exemplaire, ne recalcule que ceux dont le slack est épuisé.
///
/// Le déplacement cumulé est tenu en `f64` : sur une longue session (100 000
/// images à 0,5 m = 50 km) l'accumulation en `f32` perdait ~4 mm par image et
/// le slack dérivait.
#[derive(Debug)]
pub struct Selector {
    last_cam: Option<[f32; 3]>,
    moved: f64,
    cache: Vec<Option<(Selection, f64)>>, // (décision, déplacement cumulé auquel le slack sera épuisé)
    pub evaluated: usize,
    /// Marge d'hystérésis relative (0 = aucune).
    pub hysteresis: f32,
}

impl Default for Selector {
    fn default() -> Self {
        Selector { last_cam: None, moved: 0.0, cache: Vec::new(), evaluated: 0, hysteresis: DEFAULT_HYSTERESIS }
    }
}

impl Selector {
    pub fn new(instances: usize) -> Self {
        Selector { cache: vec![None; instances], ..Default::default() }
    }

    /// Nombre d'exemplaires suivis.
    pub fn len(&self) -> usize {
        self.cache.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cache.is_empty()
    }

    /// Ajoute un exemplaire (index = ancien `len()`), sans toucher aux
    /// décisions en cache des autres : seul le nouveau sera évalué.
    pub fn push(&mut self) -> usize {
        self.cache.push(None);
        self.cache.len() - 1
    }

    /// Doit être appelé une fois par image avant `select` : accumule le
    /// déplacement de la caméra depuis la dernière réévaluation.
    pub fn begin_frame(&mut self, cam: &Camera) {
        if let Some(last) = self.last_cam {
            let p = cam.position;
            let dx = p[0] as f64 - last[0] as f64;
            let dy = p[1] as f64 - last[1] as f64;
            let dz = p[2] as f64 - last[2] as f64;
            self.moved += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        self.last_cam = Some(cam.position);
        self.evaluated = 0;
    }

    /// Niveau de l'exemplaire `i` ; réévalué seulement si nécessaire. Un index
    /// au-delà de `len()` étend le cache (jamais de panique).
    pub fn select(&mut self, i: usize, chain: &LodChain, center: [f32; 3], radius: f32, cam: &Camera, threshold_px: f32) -> Selection {
        if i >= self.cache.len() {
            self.cache.resize(i + 1, None);
        }
        let prev = self.cache[i].map(|(sel, _)| sel.level);
        if let Some((sel, until)) = self.cache[i] {
            if self.moved < until {
                return Selection { level: sel.level, slack: (until - self.moved) as f32 };
            }
        }
        let sel = select_level_with(chain, center, radius, cam, threshold_px, prev, self.hysteresis);
        // le slack est mesuré depuis la position courante : la décision tient
        // tant que le déplacement cumulé n'atteint pas moved + slack
        self.cache[i] = Some((sel, self.moved + sel.slack as f64));
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
            levels: errors.iter().map(|&e| LodLevel { mesh: IndexedMesh::default(), error: e, source_indices: vec![] }).collect(),
            center: [0.0; 3],
            radius: 1.0,
        }
    }

    fn cam(x: f32) -> Camera {
        Camera { position: [x, 0.0, 0.0], fov_y: std::f32::consts::FRAC_PI_2, viewport_height: 1000.0 }
    }

    fn cam_at(p: [f32; 3]) -> Camera {
        Camera { position: p, fov_y: std::f32::consts::FRAC_PI_2, viewport_height: 1000.0 }
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
        sel.hysteresis = 0.0;
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

    #[test]
    fn hysteresis_widens_the_band_around_previous_level() {
        let c = chain(&[0.0, 0.01, 0.03]);
        // seuil d_2 = 7,5 ; depuis le niveau 2, on ne redescend qu'en dessous de 7,5·0,85 = 6,375
        let s = select_level_with(&c, [0.0; 3], 1.0, &cam(8.0), 2.0, Some(2), 0.15);
        assert_eq!(s.level, 2, "d = 7 ≥ 6,375 : reste au niveau 2");
        assert!((s.slack - (7.0 - 6.375)).abs() < 1e-4, "slack {}", s.slack);
        // depuis le niveau 1, on ne monte au 2 qu'à partir de 7,5·1,15 = 8,625
        let s = select_level_with(&c, [0.0; 3], 1.0, &cam(9.0), 2.0, Some(1), 0.15);
        assert_eq!(s.level, 1, "d = 8 < 8,625 : reste au niveau 1");
        assert_eq!(select_level_with(&c, [0.0; 3], 1.0, &cam(9.7), 2.0, Some(1), 0.15).level, 2);
        // première décision (prev = None) : seuils nominaux
        assert_eq!(select_level_with(&c, [0.0; 3], 1.0, &cam(8.6), 2.0, None, 0.15).level, 2);
    }

    /// Caméra qui oscille de ±ε autour du seuil d_2 = 7,5 (x = 8,5) : sans
    /// hystérésis le niveau clignote à chaque image ; avec, il ne change plus.
    #[test]
    fn hysteresis_prevents_flicker_around_a_threshold() {
        let c = chain(&[0.0, 0.01, 0.03]);
        let flips = |h: f32| {
            let mut sel = Selector::new(1);
            sel.hysteresis = h;
            let mut last = usize::MAX;
            let mut flips = 0;
            for f in 0..1000 {
                let x = 8.5 + if f % 2 == 0 { 0.05 } else { -0.05 };
                sel.begin_frame(&cam(x));
                let l = sel.select(0, &c, [0.0; 3], 1.0, &cam(x), 2.0).level;
                if last != usize::MAX && l != last {
                    flips += 1;
                }
                last = l;
            }
            flips
        };
        assert!(flips(0.0) > 900, "sans hystérésis : {} bascules", flips(0.0));
        assert_eq!(flips(DEFAULT_HYSTERESIS), 0, "avec hystérésis : aucune bascule");
    }

    /// `push` après `begin_frame` n'invalide pas les décisions en cache :
    /// seule la nouvelle entrée est évaluée.
    #[test]
    fn push_is_incremental() {
        let c = chain(&[0.0, 0.01, 0.03]);
        let mut sel = Selector::new(2);
        sel.begin_frame(&cam(10.0));
        for i in 0..2 {
            sel.select(i, &c, [0.0; 3], 1.0, &cam(10.0), 2.0);
        }
        assert_eq!(sel.evaluated, 2);
        assert_eq!(sel.push(), 2);
        sel.begin_frame(&cam(10.0));
        for i in 0..3 {
            sel.select(i, &c, [0.0; 3], 1.0, &cam(10.0), 2.0);
        }
        assert_eq!((sel.len(), sel.evaluated), (3, 1), "seule la cellule ajoutée est évaluée");
        // index au-delà de len() : extension, pas de panique
        sel.select(7, &c, [0.0; 3], 1.0, &cam(10.0), 2.0);
        assert_eq!(sel.len(), 8);
    }

    /// 100 000 images à 0,5 m par image (50 km, aller-retour sur 1 km en
    /// diagonale) : le slack renvoyé doit rester égal à « slack à l'évaluation
    /// − distance parcourue depuis » à 1e-3 près, et le niveau en cache doit
    /// être celui qu'une évaluation fraîche donnerait.
    #[test]
    fn incremental_slack_stays_exact_over_100k_frames() {
        let c = chain(&[0.0, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0]);
        let mut sel = Selector::new(1);
        let dir = [0.6f64, 0.0, 0.8]; // unitaire, 0,5 m par pas non représentable exactement
        let span = 2000usize; // 2000 pas de 0,5 m = 1 km avant demi-tour
        let pos = |f: usize| {
            let t = f % (2 * span);
            let s = if t < span { t } else { 2 * span - t } as f64 * 0.5 + 10.0;
            [(dir[0] * s) as f32, 0.0, (dir[2] * s) as f32]
        };
        let mut travelled = 0.0f64; // depuis la dernière évaluation, référence f64
        let mut slack_at_eval = 0.0f64;
        let mut prev_pos = pos(0);
        let mut prev_level: Option<usize> = None;
        let mut worst = 0.0f64;
        let mut evaluations = 0;
        for f in 0..100_000 {
            let p = pos(f);
            let step = (0..3).map(|a| (p[a] as f64 - prev_pos[a] as f64).powi(2)).sum::<f64>().sqrt();
            travelled += step;
            prev_pos = p;
            let camera = cam_at(p);
            sel.begin_frame(&camera);
            let s = sel.select(0, &c, [0.0; 3], 1.0, &camera, 2.0);
            if sel.evaluated == 1 {
                evaluations += 1;
                slack_at_eval = s.slack as f64;
                travelled = 0.0;
                let fresh = select_level_with(&c, [0.0; 3], 1.0, &camera, 2.0, prev_level, sel.hysteresis);
                assert_eq!(s, fresh, "image {f} : l'évaluation suit select_level_with");
            } else {
                let expected = slack_at_eval - travelled;
                worst = worst.max((s.slack as f64 - expected).abs());
                assert!((s.slack as f64 - expected).abs() < 1e-3, "image {f} : slack {} attendu {expected}", s.slack);
                let fresh = select_level_with(&c, [0.0; 3], 1.0, &camera, 2.0, prev_level, sel.hysteresis);
                // à moins de 1e-3 du seuil (slack en cache presque épuisé), l'arrondi
                // f32 de d peut trancher autrement : toléré
                assert!(s.level == fresh.level || s.slack < 1e-3, "image {f} : niveau en cache {} ≠ frais {} (slack en cache {})", s.level, fresh.level, s.slack);
            }
            prev_level = Some(s.level);
        }
        eprintln!("100 000 images : {evaluations} évaluations, écart max du slack {worst:.2e}");
        assert!(evaluations < 100_000 / 10, "le slack doit épargner la plupart des images ({evaluations})");
    }
}
