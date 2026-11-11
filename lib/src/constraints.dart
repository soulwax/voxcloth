import 'dart:math' as math;

import 'math.dart';

/// Dynamic particle node simulated by [XpbdWorld].
final class XpbdParticle {
  final int index;
  XpbdVec3 position;
  XpbdVec3 prevPosition;
  XpbdVec3 velocity;
  double invMass;
  final double u;
  final double v;

  double _unpinnedInvMass;

  XpbdParticle({
    required this.index,
    required this.position,
    double mass = 1.0,
    this.velocity = XpbdVec3.zero,
    this.u = 0.0,
    this.v = 0.0,
  })  : prevPosition = position,
        invMass = mass <= 0.0 ? 0.0 : 1.0 / mass,
        _unpinnedInvMass = mass <= 0.0 ? 1.0 : 1.0 / mass;

  bool get pinned => invMass == 0.0;

  set pinned(bool value) {
    if (value) {
      if (invMass > 0.0) _unpinnedInvMass = invMass;
      invMass = 0.0;
      velocity = XpbdVec3.zero;
    } else {
      invMass = _unpinnedInvMass;
    }
  }
}

/// Base class for Extended Position-Based Dynamics constraints.
abstract class XpbdConstraint {
  double compliance;
  double lambda = 0.0;

  XpbdConstraint({this.compliance = 0.0});

  void resetLambda() {
    lambda = 0.0;
  }

  /// Solves this constraint in-place across [particles] for substep [sdt].
  void solve(List<XpbdParticle> particles, double sdt);
}

/// Maintains rest separation [restLength] between particles [a] and [b].
final class DistanceConstraint extends XpbdConstraint {
  final int a;
  final int b;
  double restLength;
  final bool isBending;

  DistanceConstraint(
    this.a,
    this.b, {
    required this.restLength,
    this.isBending = false,
    super.compliance = 0.0,
  });

  double currentStretchRatio(List<XpbdParticle> particles) {
    if (restLength <= 1e-8) return 0.0;
    final d = particles[a].position.distanceTo(particles[b].position);
    return (d - restLength).abs() / restLength;
  }

  @override
  void solve(List<XpbdParticle> particles, double sdt) {
    final pa = particles[a];
    final pb = particles[b];
    final wSum = pa.invMass + pb.invMass;
    if (wSum <= 0.0) return;

    final delta = pa.position - pb.position;
    final len = delta.length;
    if (len < 1e-8) return;

    final c = len - restLength;
    final alphaTilde = compliance / (sdt * sdt);
    final dLambda = (-c - alphaTilde * lambda) / (wSum + alphaTilde);
    lambda += dLambda;

    final dir = delta / len;
    if (pa.invMass > 0.0) {
      pa.position = pa.position + dir * (pa.invMass * dLambda);
    }
    if (pb.invMass > 0.0) {
      pb.position = pb.position - dir * (pb.invMass * dLambda);
    }
  }
}

/// Anchors a particle smoothly toward [targetPosition] with configurable [compliance].
final class PinConstraint extends XpbdConstraint {
  final int particleIndex;
  XpbdVec3 targetPosition;

  PinConstraint(
    this.particleIndex,
    this.targetPosition, {
    super.compliance = 0.0,
  });

  @override
  void solve(List<XpbdParticle> particles, double sdt) {
    final p = particles[particleIndex];
    if (p.invMass <= 0.0) return;
    final delta = p.position - targetPosition;
    final len = delta.length;
    if (len < 1e-8) return;

    final alphaTilde = compliance / (sdt * sdt);
    final dLambda = (-len - alphaTilde * lambda) / (p.invMass + alphaTilde);
    lambda += dLambda;
    p.position = p.position + (delta / len) * (p.invMass * dLambda);
  }
}

/// Preserves dihedral fold angle across two triangles sharing hinge `(p0, p1)`
/// and opposite vertices `p2`, `p3`.
final class DihedralBendingConstraint extends XpbdConstraint {
  final int p0;
  final int p1;
  final int p2;
  final int p3;
  double restAngle;

  DihedralBendingConstraint(
    this.p0,
    this.p1,
    this.p2,
    this.p3, {
    required this.restAngle,
    super.compliance = 0.02,
  });

  @override
  void solve(List<XpbdParticle> particles, double sdt) {
    final v0 = particles[p0];
    final v1 = particles[p1];
    final v2 = particles[p2];
    final v3 = particles[p3];

    final w0 = v0.invMass;
    final w1 = v1.invMass;
    final w2 = v2.invMass;
    final w3 = v3.invMass;
    if (w0 + w1 + w2 + w3 <= 0.0) return;

    final p1r = v1.position - v0.position;
    final p2r = v2.position - v0.position;
    final p3r = v3.position - v0.position;

    final c1 = p1r.cross(p2r);
    final c2 = p1r.cross(p3r);
    final l1 = c1.length;
    final l2 = c2.length;
    if (l1 < 1e-7 || l2 < 1e-7) return;

    final n1 = c1 / l1;
    final n2 = c2 / l2;
    final d = n1.dot(n2).clamp(-1.0, 1.0);
    final oneMinusDSq = 1.0 - d * d;
    if (oneMinusDSq < 1e-6) return;

    final q3 = (p1r.cross(n1) + n2.cross(p1r) * d) / l2;
    final q2 = (p1r.cross(n2) + n1.cross(p1r) * d) / l1;
    final q1 = -(p2r.cross(n2) + n1.cross(p2r) * d) / l1 -
        (p3r.cross(n1) + n2.cross(p3r) * d) / l2;
    final q0 = -(q1 + q2 + q3);

    final sumGradSq = w0 * q0.lengthSquared +
        w1 * q1.lengthSquared +
        w2 * q2.lengthSquared +
        w3 * q3.lengthSquared;
    if (sumGradSq < 1e-10) return;

    final phi = math.acos(d);
    final c = phi - restAngle;
    final sqrtTerm = math.sqrt(oneMinusDSq);
    final alphaTilde = compliance / (sdt * sdt);

    // Note grad(acos(d)) = -(1 / sqrt(1 - d^2)) * grad(d), factored smoothly for XPBD.
    final factor = -sqrtTerm * c - alphaTilde * oneMinusDSq * lambda;
    final denom = sumGradSq + alphaTilde * oneMinusDSq;
    final dLambda = factor / math.max(1e-10, denom);
    lambda += dLambda;

    if (w0 > 0.0) v0.position = v0.position + q0 * (w0 * dLambda);
    if (w1 > 0.0) v1.position = v1.position + q1 * (w1 * dLambda);
    if (w2 > 0.0) v2.position = v2.position + q2 * (w2 * dLambda);
    if (w3 > 0.0) v3.position = v3.position + q3 * (w3 * dLambda);
  }
}
