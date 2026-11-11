import 'dart:math' as math;

/// Immutable 3D vector used across `voxcloth`.
final class XpbdVec3 {
  final double x;
  final double y;
  final double z;

  const XpbdVec3(this.x, this.y, this.z);

  static const XpbdVec3 zero = XpbdVec3(0.0, 0.0, 0.0);
  static const XpbdVec3 up = XpbdVec3(0.0, 1.0, 0.0);

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  XpbdVec3 get normalized {
    final len = length;
    if (len < 1e-10) return zero;
    final inv = 1.0 / len;
    return XpbdVec3(x * inv, y * inv, z * inv);
  }

  XpbdVec3 operator +(XpbdVec3 o) => XpbdVec3(x + o.x, y + o.y, z + o.z);
  XpbdVec3 operator -(XpbdVec3 o) => XpbdVec3(x - o.x, y - o.y, z - o.z);
  XpbdVec3 operator -() => XpbdVec3(-x, -y, -z);
  XpbdVec3 operator *(double s) => XpbdVec3(x * s, y * s, z * s);
  XpbdVec3 operator /(double s) => XpbdVec3(x / s, y / s, z / s);

  double dot(XpbdVec3 o) => x * o.x + y * o.y + z * o.z;

  XpbdVec3 cross(XpbdVec3 o) => XpbdVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double distanceTo(XpbdVec3 o) => (this - o).length;

  static XpbdVec3 lerp(XpbdVec3 a, XpbdVec3 b, double t) => XpbdVec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      );

  @override
  String toString() =>
      'XpbdVec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}
