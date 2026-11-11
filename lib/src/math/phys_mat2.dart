import 'dart:math' as math;
import 'phys_vec2.dart';

/// 2x2 rotation matrix for 2D rigid body orientation.
class PhysMat2 {
  final double c;
  final double s;

  const PhysMat2(this.c, this.s);

  factory PhysMat2.fromAngle(double radians) {
    return PhysMat2(math.cos(radians), math.sin(radians));
  }

  static const PhysMat2 identity = PhysMat2(1.0, 0.0);

  double get angle => math.atan2(s, c);

  /// Multiplies matrix with vector: R * v
  PhysVec2 mul(PhysVec2 v) => PhysVec2(c * v.x - s * v.y, s * v.x + c * v.y);

  /// Multiplies transposed matrix (inverse rotation) with vector: R^T * v
  PhysVec2 mulTransposed(PhysVec2 v) => PhysVec2(c * v.x + s * v.y, -s * v.x + c * v.y);

  PhysMat2 transpose() => PhysMat2(c, -s);

  PhysMat2 multiply(PhysMat2 other) {
    // [c -s] * [oc -os] = [c*oc - s*os, -(c*os + s*oc)]
    return PhysMat2(c * other.c - s * other.s, s * other.c + c * other.s);
  }

  @override
  String toString() => 'PhysMat2(angle: ${angle.toStringAsFixed(3)} rad)';
}
