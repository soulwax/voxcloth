import 'dart:math' as math;

import 'colliders.dart';
import 'constraints.dart';
import 'math.dart';

/// Extended Position-Based Dynamics simulation world for cloth & rope systems.
final class XpbdWorld {
  XpbdVec3 gravity;
  XpbdVec3 wind;
  double windDrag;
  double airDamping;
  int substeps;
  int constraintIterations;

  final List<XpbdParticle> particles = [];
  final List<XpbdConstraint> constraints = [];
  final List<XpbdCollider> colliders = [];

  /// Optional CCW triangle indices `(a, b, c)` used for aerodynamic wind lift & normal computation.
  final List<(int, int, int)> triangles = [];

  double _elapsedTime = 0.0;
  double windTurbulence;

  XpbdWorld({
    this.gravity = const XpbdVec3(0.0, -9.81, 0.0),
    this.wind = XpbdVec3.zero,
    this.windDrag = 1.2,
    this.windTurbulence = 0.0,
    this.airDamping = 0.02,
    this.substeps = 6,
    this.constraintIterations = 2,
  });

  double get elapsedTime => _elapsedTime;

  /// Computes average normalized stretch error across structural [DistanceConstraint]s.
  double get averageStretchError {
    var count = 0;
    var sum = 0.0;
    for (final c in constraints) {
      if (c is DistanceConstraint && !c.isBending) {
        sum += c.currentStretchRatio(particles);
        count++;
      }
    }
    return count == 0 ? 0.0 : sum / count;
  }

  /// Computes worst normalized stretch error across structural [DistanceConstraint]s.
  double get maxStretchError {
    var worst = 0.0;
    for (final c in constraints) {
      if (c is DistanceConstraint && !c.isBending) {
        final e = c.currentStretchRatio(particles);
        if (e > worst) worst = e;
      }
    }
    return worst;
  }

  void addParticle(XpbdParticle p) => particles.add(p);
  void addConstraint(XpbdConstraint c) => constraints.add(c);
  void addCollider(XpbdCollider col) => colliders.add(col);

  /// Advances XPBD integration by [dt] seconds over [substeps] iterations.
  void step(double dt) {
    if (dt <= 0.0 || particles.isEmpty) return;

    final steps = math.max(1, substeps);
    final sdt = dt / steps;
    final iters = math.max(1, constraintIterations);
    final dampFactor = math.exp(-airDamping * sdt);

    // Scratch per-particle acceleration array for triangle wind lift.
    final aeroForces = wind.lengthSquared > 1e-8 && triangles.isNotEmpty
        ? List<XpbdVec3>.filled(particles.length, XpbdVec3.zero)
        : null;

    for (int s = 0; s < steps; s++) {
      _elapsedTime += sdt;

      if (aeroForces != null) {
        for (int i = 0; i < aeroForces.length; i++) {
          aeroForces[i] = XpbdVec3.zero;
        }
        final gust = windTurbulence > 0.0
            ? 1.0 + math.sin(_elapsedTime * 5.3) * windTurbulence
            : 1.0;
        final activeWind = wind * gust;

        for (final (ia, ib, ic) in triangles) {
          final pa = particles[ia].position;
          final pb = particles[ib].position;
          final pc = particles[ic].position;
          final cross = (pb - pa).cross(pc - pa);
          final area2 = cross.length;
          if (area2 < 1e-8) continue;
          final normal = cross / area2;
          final avgVel = (particles[ia].velocity +
                  particles[ib].velocity +
                  particles[ic].velocity) /
              3.0;
          final relWind = activeWind - avgVel;
          final normalLoad = normal.dot(relWind);
          final triForce = normal * (normalLoad * area2 * 0.5 * windDrag * (1.0 / 3.0));
          aeroForces[ia] = aeroForces[ia] + triForce;
          aeroForces[ib] = aeroForces[ib] + triForce;
          aeroForces[ic] = aeroForces[ic] + triForce;
        }
      }

      // Predict positions
      for (int i = 0; i < particles.length; i++) {
        final p = particles[i];
        p.prevPosition = p.position;
        if (p.invMass <= 0.0) continue;

        var accel = gravity;
        if (aeroForces != null) {
          accel = accel + aeroForces[i] * p.invMass;
        }
        p.velocity = (p.velocity + accel * sdt) * dampFactor;
        p.position = p.position + p.velocity * sdt;
      }

      // Reset XPBD Lagrange multipliers & solve constraints
      for (final c in constraints) {
        c.resetLambda();
      }
      for (int it = 0; it < iters; it++) {
        for (final c in constraints) {
          c.solve(particles, sdt);
        }
        for (final col in colliders) {
          for (final p in particles) {
            col.resolveParticle(p);
          }
        }
      }

      // Update velocities from corrected positions
      final invSdt = 1.0 / sdt;
      for (final p in particles) {
        if (p.invMass <= 0.0) {
          p.velocity = XpbdVec3.zero;
        } else {
          p.velocity = (p.position - p.prevPosition) * invSdt;
        }
      }
    }
  }

  /// Computes smooth area-weighted vertex normals across [triangles].
  List<XpbdVec3> computeVertexNormals() {
    final normals = List<XpbdVec3>.filled(particles.length, XpbdVec3.zero);
    for (final (ia, ib, ic) in triangles) {
      final pa = particles[ia].position;
      final pb = particles[ib].position;
      final pc = particles[ic].position;
      final cross = (pb - pa).cross(pc - pa);
      normals[ia] = normals[ia] + cross;
      normals[ib] = normals[ib] + cross;
      normals[ic] = normals[ic] + cross;
    }
    for (int i = 0; i < normals.length; i++) {
      final lenSq = normals[i].lengthSquared;
      normals[i] = lenSq > 1e-10 ? normals[i].normalized : XpbdVec3.up;
    }
    return normals;
  }
}
