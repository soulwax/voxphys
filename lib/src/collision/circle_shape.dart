import 'dart:math' as math;
import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import '../math/phys_aabb2.dart';
import 'phys_shape.dart';

class CircleShape extends PhysShape {
  final double radius;
  final PhysVec2 center;

  CircleShape({
    required this.radius,
    this.center = PhysVec2.zero,
  }) : assert(radius > 0.0, 'Radius must be positive');

  @override
  ShapeType get type => ShapeType.circle;

  @override
  PhysAabb2 computeAabb(PhysTransform transform) {
    final worldCenter = transform.transformPoint(center);
    final r = PhysVec2(radius, radius);
    return PhysAabb2(worldCenter - r, worldCenter + r);
  }

  @override
  MassData computeMass(double density) {
    final mass = density * math.pi * radius * radius;
    // Moment of inertia for disk about center + parallel axis theorem for offset
    final inertiaAboutCenter = 0.5 * mass * radius * radius;
    final totalInertia = inertiaAboutCenter + mass * center.lengthSquared;
    return MassData(
      mass: mass,
      center: center,
      rotationalInertia: totalInertia,
    );
  }

  @override
  bool containsPoint(PhysTransform transform, PhysVec2 point) {
    final worldCenter = transform.transformPoint(center);
    return worldCenter.distanceSquared(point) <= radius * radius;
  }

  @override
  (double fraction, PhysVec2 normal)? raycast(
    PhysTransform transform,
    PhysVec2 p1,
    PhysVec2 p2,
  ) {
    final worldCenter = transform.transformPoint(center);
    final s = p1 - worldCenter;
    final b = s.dot(s) - radius * radius;

    final r = p2 - p1;
    final c = s.dot(r);
    final rr = r.dot(r);
    final sigma = c * c - rr * b;

    // Check for negative discriminant or ray pointing away
    if (sigma < 0.0 || rr < 1e-12) return null;

    // Find the intersection point
    var a = -(c + math.sqrt(sigma));

    // Is the intersection along the ray?
    if (0.0 <= a && a <= rr) {
      final fraction = a / rr;
      final hitPoint = p1 + r * fraction;
      final normal = (hitPoint - worldCenter).normalized;
      return (fraction, normal);
    }

    return null;
  }
}

typedef PhysCircle = CircleShape;
typedef PhysCircleShape = CircleShape;
