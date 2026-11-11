import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import '../math/phys_aabb2.dart';
import 'phys_shape.dart';

class SegmentShape extends PhysShape {
  final PhysVec2 p1;
  final PhysVec2 p2;

  SegmentShape({
    required this.p1,
    required this.p2,
  });

  @override
  ShapeType get type => ShapeType.segment;

  PhysVec2 get direction => (p2 - p1).normalized;
  PhysVec2 get normal => (p2 - p1).perpCw.normalized;
  double get length => (p2 - p1).length;

  @override
  PhysAabb2 computeAabb(PhysTransform transform) {
    final wp1 = transform.transformPoint(p1);
    final wp2 = transform.transformPoint(p2);
    return PhysAabb2.fromPoints([wp1, wp2]);
  }

  @override
  MassData computeMass(double density) {
    // Segments have 0 2D area, typically used for static static scenery/terrain
    return MassData(
      mass: 0.0,
      center: (p1 + p2) * 0.5,
      rotationalInertia: 0.0,
    );
  }

  @override
  bool containsPoint(PhysTransform transform, PhysVec2 point) {
    // 1D segments don't contain volume
    return false;
  }

  @override
  (double fraction, PhysVec2 normal)? raycast(
    PhysTransform transform,
    PhysVec2 rayP1,
    PhysVec2 rayP2,
  ) {
    final segP1 = transform.transformPoint(p1);
    final segP2 = transform.transformPoint(p2);

    final v1 = segP1 - rayP1;
    final v2 = segP2 - segP1;
    final v3 = rayP2 - rayP1;

    final cross32 = v3.cross(v2);
    if (cross32.abs() < 1e-12) return null;

    final t = v1.cross(v2) / cross32;
    final u = v1.cross(v3) / cross32;

    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
      final worldNormal = (transform.transformVector(normal));
      return (t, worldNormal);
    }

    return null;
  }
}
