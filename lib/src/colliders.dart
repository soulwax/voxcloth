import 'constraints.dart';
import 'math.dart';

/// Collision proxy pushed outward on cloth/rope particles during XPBD resolution.
sealed class XpbdCollider {
  double friction;

  XpbdCollider({this.friction = 0.2});

  void resolveParticle(XpbdParticle particle);
}

final class SphereConstraintCollider extends XpbdCollider {
  XpbdVec3 center;
  double radius;

  SphereConstraintCollider({
    required this.center,
    required this.radius,
    super.friction = 0.2,
  });

  @override
  void resolveParticle(XpbdParticle particle) {
    if (particle.invMass <= 0.0) return;
    final delta = particle.position - center;
    final distSq = delta.lengthSquared;
    final minDist = radius;
    if (distSq >= minDist * minDist) return;

    final n = distSq > 1e-10 ? delta.normalized : XpbdVec3.up;
    final target = center + n * minDist;
    _applyProjectionWithFriction(particle, target, n, friction);
  }
}

final class CapsuleConstraintCollider extends XpbdCollider {
  XpbdVec3 start;
  XpbdVec3 end;
  double radius;

  CapsuleConstraintCollider({
    required this.start,
    required this.end,
    required this.radius,
    super.friction = 0.25,
  });

  @override
  void resolveParticle(XpbdParticle particle) {
    if (particle.invMass <= 0.0) return;
    final ab = end - start;
    final lenSq = ab.lengthSquared;
    final t = lenSq > 1e-10
        ? ((particle.position - start).dot(ab) / lenSq).clamp(0.0, 1.0)
        : 0.0;
    final axisPt = start + ab * t;
    final delta = particle.position - axisPt;
    final distSq = delta.lengthSquared;
    if (distSq >= radius * radius) return;

    final n = distSq > 1e-10 ? delta.normalized : XpbdVec3.up;
    final target = axisPt + n * radius;
    _applyProjectionWithFriction(particle, target, n, friction);
  }
}

final class PlaneConstraintCollider extends XpbdCollider {
  XpbdVec3 point;
  XpbdVec3 normal;

  PlaneConstraintCollider({
    this.point = XpbdVec3.zero,
    XpbdVec3 normal = XpbdVec3.up,
    super.friction = 0.4,
  }) : normal = normal.normalized;

  @override
  void resolveParticle(XpbdParticle particle) {
    if (particle.invMass <= 0.0) return;
    final d = (particle.position - point).dot(normal);
    if (d >= 0.0) return;
    final target = particle.position - normal * d;
    _applyProjectionWithFriction(particle, target, normal, friction);
  }
}

void _applyProjectionWithFriction(
  XpbdParticle p,
  XpbdVec3 surfacePoint,
  XpbdVec3 normal,
  double friction,
) {
  final disp = surfacePoint - p.prevPosition;
  final normalDisp = normal * disp.dot(normal);
  final tangentDisp = disp - normalDisp;
  final dampedTangent = tangentDisp * (1.0 - friction.clamp(0.0, 1.0));
  p.position = surfacePoint;
  p.prevPosition = surfacePoint - (normalDisp + dampedTangent);
}
