import 'dart:math' as math;

import 'constraints.dart';
import 'math.dart';
import 'world.dart';

/// Generated particle grid & triangle topology produced by [ClothGridBuilder].
final class ClothPatch {
  final int columns;
  final int rows;
  final List<XpbdParticle> particles;
  final List<int> triangleIndices;

  const ClothPatch({
    required this.columns,
    required this.rows,
    required this.particles,
    required this.triangleIndices,
  });

  XpbdParticle at(int col, int row) => particles[row * columns + col];
}

/// Builds a grid cloth mesh with structural, shear, and optional dihedral bending constraints.
final class ClothGridBuilder {
  const ClothGridBuilder._();

  static ClothPatch populate(
    XpbdWorld world, {
    required int columns,
    required int rows,
    required double width,
    required double height,
    XpbdVec3 origin = XpbdVec3.zero,
    XpbdVec3 rightAxis = const XpbdVec3(1, 0, 0),
    XpbdVec3 downAxis = const XpbdVec3(0, -1, 0),
    double particleMass = 0.2,
    double stretchCompliance = 0.0,
    double shearCompliance = 1e-4,
    double bendingCompliance = 0.05,
    bool pinTopEdge = false,
    bool pinTopCorners = true,
    bool addShearConstraints = true,
    bool addDihedralBending = true,
  }) {
    if (columns < 2 || rows < 2) {
      throw ArgumentError('Cloth grid requires at least 2x2 particles');
    }

    final right = rightAxis.normalized;
    final down = downAxis.normalized;
    final dx = width / (columns - 1);
    final dy = height / (rows - 1);
    final baseIndex = world.particles.length;
    final patchParticles = <XpbdParticle>[];

    int gridIdx(int c, int r) => baseIndex + r * columns + c;

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        final pos = origin + right * (c * dx) + down * (r * dy);
        final isTopEdge = r == 0;
        final isCorner = r == 0 && (c == 0 || c == columns - 1);
        final pinned = (pinTopEdge && isTopEdge) || (pinTopCorners && isCorner);

        final p = XpbdParticle(
          index: baseIndex + patchParticles.length,
          position: pos,
          mass: pinned ? 0.0 : particleMass,
          u: c / (columns - 1),
          v: r / (rows - 1),
        );
        world.addParticle(p);
        patchParticles.add(p);
      }
    }

    // Structural horizontal & vertical constraints
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        final i = gridIdx(c, r);
        if (c + 1 < columns) {
          final j = gridIdx(c + 1, r);
          world.addConstraint(DistanceConstraint(
            i,
            j,
            restLength: world.particles[i].position.distanceTo(world.particles[j].position),
            compliance: stretchCompliance,
          ));
        }
        if (r + 1 < rows) {
          final j = gridIdx(c, r + 1);
          world.addConstraint(DistanceConstraint(
            i,
            j,
            restLength: world.particles[i].position.distanceTo(world.particles[j].position),
            compliance: stretchCompliance,
          ));
        }
        if (addShearConstraints && c + 1 < columns && r + 1 < rows) {
          final i11 = gridIdx(c + 1, r + 1);
          final i10 = gridIdx(c + 1, r);
          final i01 = gridIdx(c, r + 1);
          world.addConstraint(DistanceConstraint(
            i,
            i11,
            restLength: world.particles[i].position.distanceTo(world.particles[i11].position),
            compliance: shearCompliance,
          ));
          world.addConstraint(DistanceConstraint(
            i10,
            i01,
            restLength: world.particles[i10].position.distanceTo(world.particles[i01].position),
            compliance: shearCompliance,
          ));
        }
      }
    }

    // Triangles & optional dihedral bending constraints across quad diagonals
    final triIndices = <int>[];
    for (int r = 0; r < rows - 1; r++) {
      for (int c = 0; c < columns - 1; c++) {
        final i00 = gridIdx(c, r);
        final i10 = gridIdx(c + 1, r);
        final i01 = gridIdx(c, r + 1);
        final i11 = gridIdx(c + 1, r + 1);

        triIndices.addAll([i00, i01, i10]);
        triIndices.addAll([i10, i01, i11]);
        world.triangles.add((i00, i01, i10));
        world.triangles.add((i10, i01, i11));

        if (addDihedralBending) {
          world.addConstraint(DihedralBendingConstraint(
            i10,
            i01,
            i00,
            i11,
            restAngle: math.pi,
            compliance: bendingCompliance,
          ));
        }
      }
    }

    return ClothPatch(
      columns: columns,
      rows: rows,
      particles: patchParticles,
      triangleIndices: triIndices,
    );
  }
}

/// Builds a 1D XPBD rope or chain between [start] and [end].
final class RopeChainBuilder {
  const RopeChainBuilder._();

  static List<XpbdParticle> populate(
    XpbdWorld world, {
    required XpbdVec3 start,
    required XpbdVec3 end,
    int segments = 10,
    double particleMass = 0.25,
    double stretchCompliance = 0.0,
    double bendingCompliance = 0.02,
    bool pinStart = true,
    bool pinEnd = false,
  }) {
    if (segments < 1) {
      throw ArgumentError.value(segments, 'segments', 'must be >= 1');
    }
    final rope = <XpbdParticle>[];
    final base = world.particles.length;

    for (int i = 0; i <= segments; i++) {
      final t = i / segments;
      final pinned = (i == 0 && pinStart) || (i == segments && pinEnd);
      final p = XpbdParticle(
        index: base + i,
        position: XpbdVec3.lerp(start, end, t),
        mass: pinned ? 0.0 : particleMass,
        u: t,
      );
      world.addParticle(p);
      rope.add(p);
    }

    for (int i = 0; i < segments; i++) {
      world.addConstraint(DistanceConstraint(
        base + i,
        base + i + 1,
        restLength: rope[i].position.distanceTo(rope[i + 1].position),
        compliance: stretchCompliance,
      ));
    }

    // Skip-1 bending distance springs keep rope curves smooth.
    if (bendingCompliance < double.infinity) {
      for (int i = 0; i + 2 <= segments; i++) {
        world.addConstraint(DistanceConstraint(
          base + i,
          base + i + 2,
          restLength: rope[i].position.distanceTo(rope[i + 2].position),
          isBending: true,
          compliance: bendingCompliance,
        ));
      }
    }

    return rope;
  }
}
