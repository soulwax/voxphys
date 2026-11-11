import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import '../math/phys_aabb2.dart';

enum ShapeType {
  circle,
  polygon,
  segment,
}

class MassData {
  final double mass;
  final PhysVec2 center;
  final double rotationalInertia;

  const MassData({
    required this.mass,
    required this.center,
    required this.rotationalInertia,
  });

  static const MassData zero = MassData(
    mass: 0.0,
    center: PhysVec2.zero,
    rotationalInertia: 0.0,
  );
}

/// Abstract 2D collision geometry.
abstract class PhysShape {
  ShapeType get type;

  /// Computes world-space AABB given a transform.
  PhysAabb2 computeAabb(PhysTransform transform);

  /// Computes mass properties given material density.
  MassData computeMass(double density);

  /// Tests if a world-space point is inside the shape.
  bool containsPoint(PhysTransform transform, PhysVec2 point);

  /// Raycasts against the shape in world space.
  /// Returns distance fraction [0..1] and hit surface normal if hit.
  (double fraction, PhysVec2 normal)? raycast(
    PhysTransform transform,
    PhysVec2 p1,
    PhysVec2 p2,
  );
}
