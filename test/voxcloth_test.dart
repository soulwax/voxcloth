import 'package:test/test.dart';
import 'package:voxcloth/voxcloth.dart';

void main() {
  group('voxcloth XPBD Solver', () {
    test('ClothGridBuilder settles stably under gravity, wind and SphereConstraintCollider', () {
      final world = XpbdWorld(
        gravity: const XpbdVec3(0, -9.81, 0),
        wind: const XpbdVec3(0, 0, 4.0),
        substeps: 8,
        constraintIterations: 2,
      );

      final patch = ClothGridBuilder.populate(
        world,
        columns: 6,
        rows: 6,
        width: 2.5,
        height: 2.5,
        origin: const XpbdVec3(-1.25, 3.0, 0.0),
        downAxis: const XpbdVec3(0, -1, 0),
        pinTopCorners: true,
        stretchCompliance: 0.0,
      );

      // Sphere directly behind the banner deflects lower cloth particles toward +Z.
      world.addCollider(SphereConstraintCollider(
        center: const XpbdVec3(0.0, 1.5, 0.15),
        radius: 0.65,
      ));

      for (int i = 0; i < 45; i++) {
        world.step(1.0 / 60.0);
      }

      // Top corners remain strictly pinned.
      expect(patch.at(0, 0).position.y, closeTo(3.0, 1e-6));
      expect(patch.at(5, 0).position.y, closeTo(3.0, 1e-6));

      // Cloth stretches negligibly thanks to sub-stepped XPBD.
      expect(world.averageStretchError, lessThan(0.03));

      // Wind + sphere collider push bottom-center cloth forward along +Z without NaN.
      final centerBottom = patch.at(3, 5);
      expect(centerBottom.position.z, greaterThan(0.2));
      expect(world.computeVertexNormals().length, patch.particles.length);
    });

    test('RopeChainBuilder maintains inextensible segments and collides against ground plane', () {
      final world = XpbdWorld(
        gravity: const XpbdVec3(0, -9.81, 0),
        substeps: 10,
      );
      final rope = RopeChainBuilder.populate(
        world,
        start: const XpbdVec3(0, 2.0, 0),
        end: const XpbdVec3(3.0, 2.0, 0),
        segments: 10,
        pinStart: true,
        pinEnd: false,
        bendingCompliance: 0.1,
      );
      world.addCollider(PlaneConstraintCollider(
        point: const XpbdVec3(0, 0.0, 0),
        normal: XpbdVec3.up,
      ));

      for (int i = 0; i < 90; i++) {
        world.step(1.0 / 60.0);
      }

      expect(rope.first.position.y, closeTo(2.0, 1e-6));
      for (final p in rope) {
        expect(p.position.y, greaterThanOrEqualTo(-1e-4));
      }
      expect(rope.last.position.y, lessThan(0.25));
      expect(world.maxStretchError, lessThan(0.02));
    });
  });
}
