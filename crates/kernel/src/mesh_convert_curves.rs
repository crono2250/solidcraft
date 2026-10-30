//! Analytic section certificates independent of a cylinder's tessellation phase.
use super::{MAX_BOUNDARY, MAX_LOOPS, certify_nesting, fit_loop, invalid};
use crate::{Body, Result, meshbody::TriMesh};
use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug)]
pub(super) struct Circle {
    center: Vec2,
    radius: f64,
}

fn section(mesh: &TriMesh, plane: &Plane, height: f64, epsilon: f64) -> Option<Vec<Vec<Vec2>>> {
    let axis = plane.normal();
    let mut edges = BTreeMap::<(u32, u32), usize>::new();
    for t in &mesh.triangles {
        if !t.iter().all(|&i| mesh.positions.get(i as usize).is_some_and(|p| (p.dot(axis) - height).abs() <= epsilon * 3.)) {
            continue;
        }
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *edges.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let mut adjacency = BTreeMap::<u32, Vec<u32>>::new();
    for ((a, b), count) in edges {
        if count == 2 {
            continue;
        }
        if count != 1 {
            return None;
        }
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }
    if adjacency.is_empty() || adjacency.len() > MAX_BOUNDARY || adjacency.values().any(|v| v.len() != 2) {
        return None;
    }
    let mut out = Vec::new();
    while let Some((&start, _)) = adjacency.first_key_value() {
        if out.len() >= MAX_LOOPS {
            return None;
        }
        let mut at = start;
        let mut previous = None;
        let mut points = Vec::new();
        loop {
            let next = adjacency.remove(&at)?.into_iter().find(|v| Some(*v) != previous)?;
            points.push(plane.to_local(*mesh.positions.get(at as usize)?));
            if next == start {
                break;
            }
            previous = Some(at);
            at = next;
        }
        if points.len() < 3 {
            return None;
        }
        out.push(points);
    }
    Some(out)
}

/// Only circles already certified on an original planar face become reconstruction hints.
pub(super) fn hints(mesh: &TriMesh, axis: Vec3, tolerance: f64, epsilon: f64) -> Vec<Circle> {
    let Some(plane) = Plane::from_normal(Vec3::ZERO, axis) else { return Vec::new() };
    let mut levels = Vec::new();
    for t in &mesh.triangles {
        let Some(h) = t.iter().map(|&i| mesh.positions.get(i as usize).map(|p| p.dot(axis))).collect::<Option<Vec<_>>>() else { return Vec::new() };
        if h.iter().copied().fold(f64::NEG_INFINITY, f64::max) - h.iter().copied().fold(f64::INFINITY, f64::min) <= epsilon * 3. {
            levels.push(h.iter().sum::<f64>() / 3.);
        }
    }
    levels.sort_by(f64::total_cmp);
    levels.dedup_by(|a, b| (*a - *b).abs() <= epsilon * 3.);
    if levels.len() > 64 {
        return Vec::new();
    }
    let mut out: Vec<Circle> = Vec::new();
    for height in levels {
        let Some(loops) = section(mesh, &plane, height, epsilon) else { continue };
        for points in loops {
            for seg in fit_loop(&points, tolerance, epsilon).segs {
                if let Seg2::Arc { center, radius, .. } = seg {
                    if !out.iter().any(|c| c.center.dist(center) <= epsilon * 10. && (c.radius - radius).abs() <= epsilon * 10.) {
                        out.push(Circle { center, radius });
                    }
                    if out.len() > 128 {
                        return Vec::new();
                    }
                }
            }
        }
    }
    out
}

fn arc(points: &[Vec2], circle: Circle, tolerance: f64, epsilon: f64) -> Option<Seg2> {
    if points.len() < 5 {
        return None;
    }
    let Circle { center, radius } = circle;
    let residual = points.iter().map(|p| (p.dist(center) - radius).abs()).fold(0., f64::max);
    if residual + epsilon * 10. > tolerance {
        return None;
    }
    let mut sweep = 0.;
    let mut sign = 0.;
    for pair in points.windows(2) {
        let (a, b) = (pair[0] - center, pair[1] - center);
        let angle = a.cross(b).atan2(a.dot(b));
        if sign == 0. {
            sign = angle.signum();
        }
        if angle * sign <= 0.
            || angle.abs() > std::f64::consts::PI / 6. + 1e-8
            || radius * (1. - (angle / 2.).cos()) + residual + epsilon * 10. > tolerance
        {
            return None;
        }
        sweep += angle;
    }
    let p = *points.first()? - center;
    Some(Seg2::Arc { center, radius, start: p.y.atan2(p.x), sweep })
}

fn circle_of(s: &Seg2) -> Option<Circle> {
    if let Seg2::Arc { center, radius, .. } = s { Some(Circle { center: *center, radius: *radius }) } else { None }
}
fn join(a: &Seg2, b: &Seg2, near: Vec2, tolerance: f64) -> Option<Vec2> {
    let mut points = Vec::new();
    match (circle_of(a), circle_of(b)) {
        (Some(a), Some(b)) => {
            let d = b.center - a.center;
            let length = d.len();
            if length < 1e-7 && (a.radius - b.radius).abs() < 1e-7 {
                let v = (near - a.center).normalized()?;
                return Some(a.center + v * a.radius);
            }
            if length <= 1e-12 {
                return None;
            }
            let x = (a.radius * a.radius - b.radius * b.radius + length * length) / (2. * length);
            let h2 = a.radius * a.radius - x * x;
            if h2 < -1e-9 {
                return None;
            }
            let n = d / length;
            let middle = a.center + n * x;
            let perpendicular = Vec2::new(-n.y, n.x) * h2.max(0.).sqrt();
            points.extend([middle + perpendicular, middle - perpendicular]);
        }
        (Some(c), None) | (None, Some(c)) => {
            let line = if circle_of(a).is_none() { a } else { b };
            let direction = (line.end() - line.start()).normalized()?;
            let offset = line.start() - c.center;
            let along = -offset.dot(direction);
            let h2 = c.radius * c.radius - offset.len2() + along * along;
            if h2 < -1e-9 {
                return None;
            }
            for t in [along + h2.max(0.).sqrt(), along - h2.max(0.).sqrt()] {
                points.push(line.start() + direction * t);
            }
        }
        (None, None) => return Some(near),
    }
    points.into_iter().filter(|p| p.dist(near) <= tolerance).min_by(|a, b| a.dist(near).total_cmp(&b.dist(near)))
}

fn fitted(points: &[Vec2], models: &[Circle], tolerance: f64, epsilon: f64) -> Loop2 {
    let original = fit_loop(points, tolerance, epsilon);
    if points.len() < 5 || models.is_empty() {
        return original;
    }
    let mut ring = points.to_vec();
    let n = ring.len();
    let offset = (0..n)
        .max_by(|&a, &b| {
            let turn = |i: usize| {
                let p = ring[i] - ring[(i + n - 1) % n];
                let q = ring[(i + 1) % n] - ring[i];
                p.cross(q).atan2(p.dot(q)).abs()
            };
            turn(a).total_cmp(&turn(b))
        })
        .unwrap_or(0);
    ring.rotate_left(offset);
    let Some(&first) = ring.first() else { return original };
    ring.push(first);
    // A whole circle has no preferred seam and is represented by two analytic half-arcs.
    for &model in models {
        if let Some(Seg2::Arc { center, radius, sweep, .. }) = arc(&ring, model, tolerance, epsilon)
            && (sweep.abs() - std::f64::consts::TAU).abs() < 1e-5
        {
            return Loop2::circle(center, radius);
        }
    }
    let mut at = 0;
    let mut segs = Vec::new();
    let mut vertices = Vec::new();
    let mut work = 0usize;
    while at < n {
        let mut best = None;
        for &model in models {
            for end in at + 4..=n {
                work = work.saturating_add((end - at + 1) * 2);
                if work > 4_000_000 {
                    return original;
                }
                let Some(run) = ring.get(at..=end) else { break };
                let Some(segment) = arc(run, model, tolerance, epsilon) else { break };
                if best.as_ref().is_none_or(|(old, _)| end > *old) {
                    best = Some((end, segment));
                }
            }
        }
        let Some(&a) = ring.get(at) else { return original };
        vertices.push(a);
        if let Some((end, segment)) = best {
            segs.push(segment);
            at = end;
        } else {
            let Some(&b) = ring.get(at + 1) else { return original };
            segs.push(Seg2::Line { a, b });
            at += 1;
        }
    }
    let mut joints = Vec::new();
    for (i, &near) in vertices.iter().enumerate() {
        let (Some(a), Some(b)) = (segs.get((i + segs.len() - 1) % segs.len()), segs.get(i)) else { return original };
        let Some(p) = join(a, b, near, tolerance) else { return original };
        joints.push(p);
    }
    let mut out = Vec::new();
    for (i, segment) in segs.iter().enumerate() {
        let (Some(&a), Some(&b)) = (joints.get(i), joints.get((i + 1) % joints.len())) else { return original };
        match segment {
            Seg2::Line { .. } => out.push(Seg2::Line { a, b }),
            Seg2::Arc { center, radius, sweep, .. } => {
                let start = (a - *center).y.atan2((a - *center).x);
                let end = (b - *center).y.atan2((b - *center).x);
                let angle =
                    if *sweep > 0. { (end - start).rem_euclid(std::f64::consts::TAU) } else { -(start - end).rem_euclid(std::f64::consts::TAU) };
                out.push(Seg2::Arc { center: *center, radius: *radius, start, sweep: angle });
            }
            _ => return original,
        }
    }
    Loop2 { segs: out }
}

fn line_distance(a: Vec2, b: Vec2, p: Vec2) -> f64 {
    let line = Seg2::Line { a, b };
    line.point_at(line.closest_param(p)).dist(p)
}
fn certified(points: &[Vec2], curves: &[Seg2], tolerance: f64) -> bool {
    curves.iter().any(|curve| match curve {
        Seg2::Line { a, b } => points.iter().all(|p| line_distance(*a, *b, *p) <= tolerance),
        Seg2::Arc { center, radius, start, sweep } => {
            if points.iter().any(|p| p.dist(*center) > radius + tolerance) {
                return false;
            }
            let mut minimum = f64::INFINITY;
            for (&a, &b) in points.iter().zip(points.iter().cycle().skip(1)).take(points.len()) {
                minimum = minimum.min(line_distance(a, b, *center));
            }
            if points.len() == 3 {
                let mut signs = points.iter().zip(points.iter().cycle().skip(1)).take(3).map(|(a, b)| (*b - *a).cross(*center - *a));
                let first = signs.next().unwrap_or(0.);
                if first.abs() > 1e-14 && signs.all(|s| s * first > 0.) {
                    minimum = 0.;
                }
            }
            if minimum < radius - tolerance {
                return false;
            }
            let full = curves
                .iter()
                .filter_map(|s| {
                    if let Seg2::Arc { center: c, radius: r, sweep: w, .. } = s {
                        if c.dist(*center) < 1e-8 && (*r - radius).abs() < 1e-8 { Some(w.abs()) } else { None }
                    } else {
                        None
                    }
                })
                .sum::<f64>()
                >= std::f64::consts::TAU - 1e-8;
            if full {
                return true;
            }
            if points.iter().any(|p| curve.point_at(curve.closest_param(*p)).dist(*p) > tolerance) {
                return false;
            }
            let angular = tolerance / radius.max(tolerance);
            let mut angles = Vec::new();
            for p in points {
                let d = *p - *center;
                let mut angle = ((d.y.atan2(d.x) - start) * sweep.signum()).rem_euclid(std::f64::consts::TAU);
                if angle > std::f64::consts::TAU - angular {
                    angle -= std::f64::consts::TAU;
                }
                if angle < -angular || angle > sweep.abs() + angular {
                    return false;
                }
                angles.push(angle);
            }
            angles.iter().copied().fold(f64::NEG_INFINITY, f64::max) - angles.iter().copied().fold(f64::INFINITY, f64::min)
                <= std::f64::consts::PI / 6. + angular
        }
        _ => false,
    })
}

pub(super) fn extrusion(mesh: &TriMesh, axis: Vec3, tolerance: f64, models: impl FnOnce() -> Vec<Circle>) -> Result<Option<Body>> {
    let (lo, hi) = mesh.positions.iter().map(|p| p.dot(axis)).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), h| (a.min(h), b.max(h)));
    let bounds = mesh.positions.iter().fold(solidcraft_geom::Aabb3::EMPTY, |mut b, p| {
        b.add(*p);
        b
    });
    let size = bounds.size().len();
    let epsilon = (size.max(1.) * 1e-7).min(tolerance * 0.01).max(1e-9);
    if hi - lo <= epsilon {
        return Ok(None);
    }
    let Some(plane) = Plane::from_normal(axis * lo, axis) else { return Ok(None) };
    let (Some(lower), Some(upper)) = (section(mesh, &plane, lo, epsilon), section(mesh, &plane, hi, epsilon)) else { return Ok(None) };
    if lower.len() != upper.len() {
        return Ok(None);
    }
    // Cap certificates can reject a direction without fitting circles on the entire root mesh.
    let models = models();
    if models.is_empty() {
        return Ok(None);
    }
    let a: Vec<_> = lower.iter().map(|p| fitted(p, &models, tolerance, epsilon)).collect();
    let b: Vec<_> = upper.iter().map(|p| fitted(p, &models, tolerance, epsilon)).collect();
    let count = |p: &[Loop2]| p.iter().map(|p| p.segs.len()).sum::<usize>();
    let loops = if count(&a) <= count(&b) { a } else { b };
    let curves: Vec<_> = loops.iter().flat_map(|p| p.segs.iter().cloned()).collect();
    if curves.len() > 128 || mesh.triangles.len().saturating_mul(curves.len()) > 4_000_000 {
        return Ok(None);
    }
    // Every cap loop is paired bijectively and every entire source chord is bounded.
    let mut lower_matched = Vec::new();
    for (cap_index, cap) in [&lower, &upper].into_iter().enumerate() {
        let mut used = std::collections::BTreeSet::new();
        for raw in cap {
            let matches: Vec<_> = loops
                .iter()
                .enumerate()
                .filter(|(_, lp)| raw.iter().zip(raw.iter().cycle().skip(1)).take(raw.len()).all(|(&a, &b)| certified(&[a, b], &lp.segs, tolerance)))
                .map(|(i, _)| i)
                .collect();
            let [index] = matches.as_slice() else { return Ok(None) };
            if !used.insert(*index) {
                return Ok(None);
            }
            if cap_index == 0 {
                let Some(lp) = loops.get(*index) else { return Ok(None) };
                lower_matched.push(lp.clone());
            }
        }
    }
    for t in &mesh.triangles {
        let Some(points) = t.iter().map(|&i| mesh.positions.get(i as usize).copied()).collect::<Option<Vec<_>>>() else {
            return Err(invalid("missing analytic section vertex"));
        };
        if points.iter().all(|p| (p.dot(axis) - lo).abs() <= epsilon * 3.) || points.iter().all(|p| (p.dot(axis) - hi).abs() <= epsilon * 3.) {
            continue;
        }
        let projected: Vec<_> = points.iter().map(|p| plane.to_local(*p)).collect();
        if !certified(&projected, &curves, tolerance) {
            return Ok(None);
        }
    }
    let raw: Vec<_> = lower.iter().map(|p| Loop2::polygon(p)).collect();
    certify_nesting(&raw, &lower_matched, epsilon, size)?;
    let mut regions = Vec::new();
    for lp in &loops {
        let Some(probe) = lp.segs.first().map(Seg2::start) else { return Ok(None) };
        let owners = loops.iter().filter(|p| !std::ptr::eq(*p, lp) && p.contains(probe)).count();
        if owners % 2 == 0 {
            regions.push(Region2 { outer: lp.ccw(), holes: Vec::new() });
        }
    }
    for lp in &loops {
        let Some(probe) = lp.segs.first().map(Seg2::start) else { return Ok(None) };
        let owners = loops.iter().filter(|p| !std::ptr::eq(*p, lp) && p.contains(probe)).count();
        if owners % 2 == 1 {
            let owner = regions
                .iter()
                .enumerate()
                .filter(|(_, r)| r.outer.contains(probe))
                .min_by(|(_, a), (_, b)| a.area().total_cmp(&b.area()))
                .map(|(i, _)| i);
            let Some(region) = owner.and_then(|i| regions.get_mut(i)) else { return Ok(None) };
            region.holes.push(lp.ccw().reversed());
        }
    }
    let body = crate::extrude_planar_regions(&plane, &regions, 0., hi - lo)?;
    let original = mesh.to_mesh().measure().volume;
    let volume = super::validated_volume(&body)?;
    let perimeter: f64 = curves.iter().map(Seg2::length).sum();
    if (volume - original).abs() > perimeter * (hi - lo) * tolerance + epsilon * size * size {
        return Err(invalid("analytic section failed solid or volume validation"));
    }
    Ok(Some(body))
}

/// Recover a right circular frustum only when both complete caps and every side
/// triangle certify the same linear radius law. Coarse polygonal tapers stay refused.
pub(super) fn conical_frustum(mesh: &TriMesh, axis: Vec3, tolerance: f64) -> Result<Option<Body>> {
    let (lo, hi) = mesh.positions.iter().map(|p| p.dot(axis)).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), h| (a.min(h), b.max(h)));
    let height = hi - lo;
    let bounds = mesh.positions.iter().fold(solidcraft_geom::Aabb3::EMPTY, |mut b, p| {
        b.add(*p);
        b
    });
    let size = bounds.size().len();
    let epsilon = (size.max(1.) * 1e-7).min(tolerance * 0.01).max(1e-9);
    if height <= epsilon {
        return Ok(None);
    }
    let Some(plane) = Plane::from_normal(axis * lo, axis) else { return Ok(None) };
    let (Some(lower), Some(upper)) = (section(mesh, &plane, lo, epsilon), section(mesh, &plane, hi, epsilon)) else { return Ok(None) };
    let ([bottom], [top]) = (lower.as_slice(), upper.as_slice()) else { return Ok(None) };
    let (Some(a), Some(b)) = (section_circle(bottom, tolerance, epsilon), section_circle(top, tolerance, epsilon)) else { return Ok(None) };
    if a.center.dist(b.center) > epsilon * 5. || (a.radius - b.radius).abs() <= epsilon * 10. {
        return Ok(None);
    }
    let radius = |h: f64| a.radius + (b.radius - a.radius) * ((h - lo) / height);
    for triangle in &mesh.triangles {
        let Some(points) = triangle.iter().map(|&i| mesh.positions.get(i as usize).copied()).collect::<Option<Vec<_>>>() else {
            return Err(invalid("missing conical vertex"));
        };
        if points.iter().all(|p| (p.dot(axis) - lo).abs() <= epsilon * 3.) || points.iter().all(|p| (p.dot(axis) - hi).abs() <= epsilon * 3.) {
            continue;
        }
        let projected: Vec<_> = points.iter().map(|p| plane.to_local(*p) - a.center).collect();
        if points.iter().zip(&projected).any(|(p, q)| (q.len() - radius(p.dot(axis))).abs() > epsilon * 5.) {
            return Ok(None);
        }
        let mut span = 0_f64;
        for u in &projected {
            for v in &projected {
                let denominator = u.len() * v.len();
                if denominator <= epsilon * epsilon {
                    return Ok(None);
                }
                span = span.max((u.dot(*v) / denominator).clamp(-1., 1.).acos());
            }
        }
        // Projection on the angular bisector bounds the deviation over the
        // complete triangle, not just its sampled vertices.
        if span > std::f64::consts::PI / 6. + 1e-8 || a.radius.max(b.radius) * (1. - (span * 0.5).cos()) + epsilon * 10. > tolerance {
            return Ok(None);
        }
    }
    let taper = ((b.radius - a.radius) / height).atan();
    if taper.abs() >= std::f64::consts::FRAC_PI_2 - 1e-3 {
        return Ok(None);
    }
    let region = Region2 { outer: Loop2::circle(a.center, a.radius), holes: vec![] };
    let body = crate::extrude_tapered(&plane, &region, height, 1., taper)?;
    let measured = super::validated_volume(&body)?;
    let original = mesh.to_mesh().measure();
    if (measured - original.volume).abs() > original.area * tolerance + epsilon * size * size {
        return Err(invalid("conical reconstruction failed solid or volume validation"));
    }
    Ok(Some(body))
}

fn section_circle(raw: &[Vec2], tolerance: f64, epsilon: f64) -> Option<Circle> {
    let fitted = fit_loop(raw, tolerance, epsilon);
    let mut result: Option<Circle> = None;
    let mut sweep = 0.;
    for seg in fitted.segs {
        let Seg2::Arc { center, radius, sweep: angle, .. } = seg else { return None };
        match result {
            None => result = Some(Circle { center, radius }),
            Some(c) if c.center.dist(center) <= epsilon * 5. && (c.radius - radius).abs() <= epsilon * 5. => {}
            _ => return None,
        }
        sweep += angle.abs();
    }
    if (sweep - std::f64::consts::TAU).abs() > 1e-6 {
        return None;
    }
    result
}

#[derive(Clone, Copy)]
struct ConicalBore {
    center: Vec2,
    small: f64,
    large: f64,
    knee: f64,
    end: f64,
    sign: f64,
}
impl ConicalBore {
    fn slope(self) -> f64 {
        (self.large - self.small) / (self.end - self.knee)
    }
    fn radius(self, h: f64) -> f64 {
        self.small + (self.sign * h - self.knee).max(0.) * self.slope()
    }
}

/// The circular loft bands are right cones/cylinders by construction. Store
/// their bore surfaces as exact revolutions, retaining the same trim wires.
/// This avoids relying on STEP's conservative B-spline recognition heuristics.
fn canonical_bore_band(body: &Body, bores: &[ConicalBore], plane: &Plane, axis: Vec3, from: f64, to: f64, epsilon: f64) -> Result<Body> {
    use truck_modeling as mt;
    use truck_modeling::{ParametricSurface, ParametricSurface3D, SearchNearestParameter};
    let point = |p: Vec3| mt::Point3::new(p.x, p.y, p.z);
    let vector = |p: Vec3| mt::Vector3::new(p.x, p.y, p.z);
    let convert = |p: mt::Point3| Vec3::new(p.x, p.y, p.z);
    let mut faces = Vec::new();
    for face in body.solid.face_iter() {
        let vertices: Vec<_> = face.vertex_iter().map(|v| convert(v.point())).collect();
        let bore = bores.iter().find(|b| {
            !vertices.is_empty() && vertices.iter().all(|p| (plane.to_local(*p).dist(b.center) - b.radius(p.dot(axis))).abs() <= epsilon * 5.)
        });
        let Some(bore) = bore else {
            faces.push(face.clone());
            continue;
        };
        let centre = plane.to_world(bore.center) + axis * (from - plane.origin.dot(axis));
        let a = centre + plane.x * bore.radius(from);
        let b = centre + axis * (to - from) + plane.x * bore.radius(to);
        let line = mt::builder::line(&mt::builder::vertex(point(a)), &mt::builder::vertex(point(b))).oriented_curve();
        let mut surface = mt::Surface::RevolutedCurve(mt::Processor::new(mt::RevolutedCurve::by_revolution(line, point(centre), vector(axis))));
        let old = face.surface();
        let (Some((u0, u1)), Some((v0, v1))) = old.try_range_tuple() else { return Err(invalid("unbounded certified bore surface")) };
        let (u, v) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        let sample = old.subs(u, v);
        let old_normal = old.normal(u, v);
        let Some((s, t)) = surface.search_nearest_parameter(sample, None, 100) else {
            return Err(invalid("analytic bore surface does not match its loft"));
        };
        let n = surface.normal(s, t);
        let alignment = n.x * old_normal.x + n.y * old_normal.y + n.z * old_normal.z;
        if !alignment.is_finite() || alignment.abs() <= 1e-20 {
            return Err(invalid("singular analytic bore surface"));
        }
        if alignment < 0. {
            surface = mt::Invertible::inverse(&surface);
        }
        let mut replacement =
            mt::Face::try_new(face.absolute_boundaries().clone(), surface).map_err(|_| invalid("analytic bore boundary does not close"))?;
        if !face.orientation() {
            replacement.invert();
        }
        faces.push(replacement);
    }
    Ok(crate::surfaces::surface_body(faces))
}

/// A countersink is accepted only with complete concentric cap circles and a
/// certified cone-to-cylinder radius law. Straighten its bore, certify the
/// remaining extrusion, then loft its analytic bands with shared caps. The scratch mesh
/// never becomes the output solid.
pub(super) fn countersunk_bores(mesh: &TriMesh, axis: Vec3, tolerance: f64) -> Result<Option<Body>> {
    let (lo, hi) = mesh.positions.iter().map(|p| p.dot(axis)).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), h| (a.min(h), b.max(h)));
    let bounds = mesh.positions.iter().fold(solidcraft_geom::Aabb3::EMPTY, |mut b, p| {
        b.add(*p);
        b
    });
    let size = bounds.size().len();
    let epsilon = (size.max(1.) * 1e-7).min(tolerance * 0.01).max(1e-9);
    if hi - lo <= epsilon {
        return Ok(None);
    }
    let Some(plane) = Plane::from_normal(axis * lo, axis) else {
        return Ok(None);
    };
    let (Some(lower), Some(upper)) = (section(mesh, &plane, lo, epsilon), section(mesh, &plane, hi, epsilon)) else {
        return Ok(None);
    };
    let mut bores: Vec<ConicalBore> = Vec::new();
    for (index, raw) in lower.iter().enumerate() {
        let Some(probe) = raw.first().copied() else {
            return Ok(None);
        };
        let owners = lower.iter().enumerate().filter(|(i, p)| *i != index && Loop2::polygon(p).contains(probe)).count();
        if owners % 2 == 0 {
            continue;
        }
        let Some(a) = section_circle(raw, tolerance, epsilon) else { continue };
        let matches: Vec<_> =
            upper.iter().filter_map(|p| section_circle(p, tolerance, epsilon)).filter(|b| a.center.dist(b.center) <= epsilon * 5.).collect();
        let [b] = matches.as_slice() else { continue };
        if (a.radius - b.radius).abs() <= epsilon * 10. {
            continue;
        }
        let sign = if b.radius > a.radius { 1. } else { -1. };
        let (small, large) = (a.radius.min(b.radius), a.radius.max(b.radius));
        let (start, end) = ((sign * lo).min(sign * hi), (sign * lo).max(sign * hi));
        let knee = mesh
            .positions
            .iter()
            .filter(|p| (plane.to_local(**p).dist(a.center) - small).abs() <= epsilon * 5.)
            .map(|p| sign * p.dot(axis))
            .fold(start, f64::max);
        if end - knee <= epsilon * 10. || bores.len() >= 8 {
            return Ok(None);
        }
        let bore = ConicalBore { center: a.center, small, large, knee, end, sign };
        if !bore.slope().is_finite() || bore.slope() <= 0. || bore.slope().atan() >= std::f64::consts::FRAC_PI_2 - 1e-3 {
            return Ok(None);
        }
        // Overlapping countersinks need a different surface-intersection proof.
        if bores.iter().any(|old| old.center.dist(bore.center) <= old.large + bore.large + epsilon * 10.) {
            return Ok(None);
        }
        bores.push(bore);
    }
    if bores.is_empty() {
        return Ok(None);
    }
    let mut straight = TriMesh { positions: mesh.positions.clone(), triangles: mesh.triangles.clone() };
    for bore in &bores {
        let matches = |p: Vec3| (plane.to_local(p).dist(bore.center) - bore.radius(p.dot(axis))).abs() <= epsilon * 5.;
        let changed = |p: Vec3| matches(p) && plane.to_local(p).dist(bore.center) > bore.small + epsilon * 10.;
        let mut side_count = 0;
        for t in &mesh.triangles {
            let Some(points) = t.iter().map(|&i| mesh.positions.get(i as usize).copied()).collect::<Option<Vec<_>>>() else {
                return Err(invalid("missing countersink vertex"));
            };
            if !points.iter().any(|p| changed(*p)) {
                continue;
            }
            if points.iter().all(|p| (bore.sign * p.dot(axis) - bore.end).abs() <= epsilon * 3.) {
                continue;
            }
            if !points.iter().all(|p| matches(*p) && bore.sign * p.dot(axis) >= bore.knee - epsilon * 5.) {
                return Ok(None);
            }
            let radial: Vec<_> = points.iter().map(|p| plane.to_local(*p) - bore.center).collect();
            let mut span = 0_f64;
            for u in &radial {
                for v in &radial {
                    let den = u.len() * v.len();
                    if den <= epsilon * epsilon {
                        return Ok(None);
                    }
                    span = span.max((u.dot(*v) / den).clamp(-1., 1.).acos());
                }
            }
            if span > std::f64::consts::PI / 6. + 1e-8 || bore.large * (1. - (span * 0.5).cos()) + epsilon * 10. > tolerance {
                return Ok(None);
            }
            side_count += 1;
        }
        if side_count < 12 {
            return Ok(None);
        }
        for (old, new) in mesh.positions.iter().zip(&mut straight.positions) {
            if changed(*old) {
                let radial = plane.to_local(*old) - bore.center;
                *new = plane.to_world(bore.center + radial * (bore.small / radial.len())) + axis * (old.dot(axis) - lo);
            }
        }
    }
    // Certify the straightened boundary before constructing any loft bands.
    let certified_body = match super::extrusion(&straight, axis, tolerance)? {
        Some(b) => Some(b),
        None => extrusion(&straight, axis, tolerance, || hints(&straight, axis, tolerance, epsilon))?,
    };
    if certified_body.is_none() {
        return Ok(None);
    }
    let Some(raw) = section(&straight, &plane, lo, epsilon) else {
        return Ok(None);
    };
    let loops: Vec<_> = raw.iter().map(|p| fit_loop(p, tolerance, epsilon)).collect();
    let all_curves: Vec<_> = loops.iter().flat_map(|p| p.segs.iter().cloned()).collect();
    if all_curves.len() > 128 || straight.triangles.len().saturating_mul(all_curves.len()) > 4_000_000 {
        return Ok(None);
    }
    for triangle in &straight.triangles {
        let Some(points) = triangle.iter().map(|&i| straight.positions.get(i as usize).copied()).collect::<Option<Vec<_>>>() else {
            return Err(invalid("missing straightened vertex"));
        };
        if points.iter().all(|p| (p.dot(axis) - lo).abs() <= epsilon * 3.) || points.iter().all(|p| (p.dot(axis) - hi).abs() <= epsilon * 3.) {
            continue;
        }
        let projected: Vec<_> = points.iter().map(|p| plane.to_local(*p)).collect();
        if !certified(&projected, &all_curves, tolerance) {
            return Ok(None);
        }
    }
    let mut regions = Vec::new();
    for (i, lp) in loops.iter().enumerate() {
        let Some(probe) = lp.segs.first().map(Seg2::start) else {
            return Ok(None);
        };
        if loops.iter().enumerate().filter(|(j, p)| *j != i && p.contains(probe)).count() % 2 == 0 {
            regions.push(Region2 { outer: lp.ccw(), holes: Vec::new() });
        }
    }
    for (i, lp) in loops.iter().enumerate() {
        let Some(probe) = lp.segs.first().map(Seg2::start) else {
            return Ok(None);
        };
        if loops.iter().enumerate().filter(|(j, p)| *j != i && p.contains(probe)).count() % 2 == 1 {
            let owner = regions.iter_mut().filter(|r| r.outer.contains(probe)).min_by(|a, b| a.area().total_cmp(&b.area()));
            let Some(owner) = owner else {
                return Ok(None);
            };
            owner.holes.push(lp.ccw().reversed());
        }
    }
    let [region] = regions.as_slice() else {
        return Ok(None);
    };
    let same_circle = |hole: &Loop2, bore: &ConicalBore| {
        !hole.segs.is_empty()
            && hole
                .segs
                .iter()
                .all(|s| matches!(s,Seg2::Arc {center,radius,..} if center.dist(bore.center)<=epsilon*5. && (*radius-bore.small).abs()<=epsilon*5.))
    };
    if bores.iter().any(|b| region.holes.iter().filter(|h| same_circle(h, b)).count() != 1) {
        return Ok(None);
    }
    let at = |h: f64| {
        let mut r = region.clone();
        for hole in &mut r.holes {
            for bore in &bores {
                if same_circle(hole, bore) {
                    *hole = Loop2::circle(bore.center, bore.radius(h)).reversed();
                    break;
                }
            }
        }
        (plane.offset(h - lo), r)
    };
    // Match the actual end profiles bijectively to the original caps, including
    // the large countersink mouth: straightened-bore nesting alone is not enough.
    for (raw_cap, height) in [(&lower, lo), (&upper, hi)] {
        let (_, profile) = at(height);
        let wanted: Vec<_> = std::iter::once(&profile.outer).chain(&profile.holes).collect();
        let mut used = std::collections::BTreeSet::new();
        let mut paired = Vec::new();
        for raw_loop in raw_cap {
            let choices: Vec<_> = wanted
                .iter()
                .enumerate()
                .filter(|(_, lp)| {
                    raw_loop.iter().zip(raw_loop.iter().cycle().skip(1)).take(raw_loop.len()).all(|(&a, &b)| certified(&[a, b], &lp.segs, tolerance))
                })
                .map(|(i, _)| i)
                .collect();
            let [i] = choices.as_slice() else { return Ok(None) };
            if !used.insert(*i) {
                return Ok(None);
            }
            let Some(lp) = wanted.get(*i) else { return Ok(None) };
            paired.push((*lp).clone());
        }
        if paired.len() != wanted.len() {
            return Ok(None);
        }
        let raw_loops: Vec<_> = raw_cap.iter().map(|p| Loop2::polygon(p)).collect();
        certify_nesting(&raw_loops, &paired, epsilon, size)?;
    }
    let mut levels = vec![lo, hi];
    for bore in &bores {
        let h = bore.sign * bore.knee;
        if h > lo + epsilon && h < hi - epsilon {
            levels.push(h);
        }
    }
    levels.sort_by(f64::total_cmp);
    levels.dedup_by(|a, b| a.to_bits() == b.to_bits());
    if levels.windows(2).any(|p| match p {
        [a, b] => b - a <= epsilon * 10.,
        _ => true,
    }) {
        return Ok(None);
    }
    let mut bands = Vec::new();
    for pair in levels.windows(2) {
        let [a, b] = pair else { return Ok(None) };
        let loft = crate::loft_regions(&[at(*a), at(*b)])?;
        bands.push(canonical_bore_band(&loft, &bores, &plane, axis, *a, *b, epsilon)?);
    }
    // Remove only the matched interior caps and sew their exactly shared curves.
    // The prismatic part sewer does not accept a tapering band.
    if bands.iter().map(Body::face_count).sum::<usize>() > 256 {
        return Ok(None);
    }
    let body = if bands.len() == 1 {
        let Some(b) = bands.pop() else {
            return Ok(None);
        };
        crate::surfaces::stitch(&[&b], epsilon)?
    } else {
        let mut skins = Vec::new();
        for band in &bands {
            let internal: Vec<_> = (0..band.face_count())
                .filter(|&i| {
                    band.planar_face(i).is_some_and(|p| {
                        p.normal().dot(axis).abs() > 1. - 1e-8
                            && levels.iter().skip(1).take(levels.len().saturating_sub(2)).any(|h| (p.origin.dot(axis) - h).abs() <= epsilon * 3.)
                    })
                })
                .collect();
            if internal.is_empty() {
                return Ok(None);
            }
            skins.push(crate::surfaces::remove_faces(band, &internal)?);
        }
        crate::surfaces::stitch(&skins.iter().collect::<Vec<_>>(), epsilon)?
    };
    let measured = super::validated_volume(&body)?;
    let original = mesh.to_mesh().measure();
    if (measured - original.volume).abs() > original.area * tolerance + epsilon * size * size {
        return Err(invalid("countersink reconstruction failed solid or volume validation"));
    }
    Ok(Some(body))
}
