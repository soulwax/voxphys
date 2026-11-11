import 'dart:math' as math;

/// A 2D vector for physics calculations.
class PhysVec2 {
  final double x;
  final double y;

  const PhysVec2(this.x, this.y);

  static const PhysVec2 zero = PhysVec2(0.0, 0.0);

  PhysVec2 operator +(PhysVec2 other) => PhysVec2(x + other.x, y + other.y);
  PhysVec2 operator -(PhysVec2 other) => PhysVec2(x - other.x, y - other.y);
  PhysVec2 operator *(double scalar) => PhysVec2(x * scalar, y * scalar);
  PhysVec2 operator /(double scalar) => PhysVec2(x / scalar, y / scalar);
  PhysVec2 operator -() => PhysVec2(-x, -y);

  double dot(PhysVec2 other) => x * other.x + y * other.y;

  /// 2D cross product of this vector with [other] returning the scalar z component.
  double cross(PhysVec2 other) => x * other.y - y * other.x;

  /// Vector cross product with scalar: v x s = (s * v.y, -s * v.x).
  PhysVec2 crossScalar(double s) => PhysVec2(s * y, -s * x);

  /// Scalar cross product with vector: s x v = (-s * v.y, s * v.x).
  static PhysVec2 scalarCross(double s, PhysVec2 v) => PhysVec2(-s * v.y, s * v.x);

  double get lengthSquared => x * x + y * y;
  double get length => math.sqrt(lengthSquared);

  PhysVec2 get normalized {
    final len = length;
    if (len < 1e-12) return PhysVec2.zero;
    return PhysVec2(x / len, y / len);
  }

  /// Returns the counter-clockwise perpendicular vector (-y, x).
  PhysVec2 get perp => PhysVec2(-y, x);

  /// Returns the clockwise perpendicular vector (y, -x).
  PhysVec2 get perpCw => PhysVec2(y, -x);

  double distance(PhysVec2 other) => (this - other).length;
  double distanceSquared(PhysVec2 other) => (this - other).lengthSquared;

  PhysVec2 lerp(PhysVec2 other, double t) => PhysVec2(
        x + (other.x - x) * t,
        y + (other.y - y) * t,
      );

  PhysVec2 abs() => PhysVec2(x.abs(), y.abs());

  PhysVec2 min(PhysVec2 other) => PhysVec2(
        math.min(x, other.x),
        math.min(y, other.y),
      );

  PhysVec2 max(PhysVec2 other) => PhysVec2(
        math.max(x, other.x),
        math.max(y, other.y),
      );

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is PhysVec2 &&
          (x - other.x).abs() < 1e-9 &&
          (y - other.y).abs() < 1e-9;

  @override
  int get hashCode => Object.hash(x, y);

  @override
  String toString() => 'PhysVec2(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)})';
}
