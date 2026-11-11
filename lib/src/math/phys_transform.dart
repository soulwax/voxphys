import 'phys_vec2.dart';
import 'phys_mat2.dart';

/// Represents a 2D position and rotation transform.
class PhysTransform {
  PhysVec2 position;
  PhysMat2 rotation;

  PhysTransform({
    this.position = PhysVec2.zero,
    PhysMat2? rotation,
  }) : rotation = rotation ?? PhysMat2.identity;

  factory PhysTransform.fromPosAngle(PhysVec2 pos, double angle) {
    return PhysTransform(position: pos, rotation: PhysMat2.fromAngle(angle));
  }

  double get angle => rotation.angle;
  set angle(double a) => rotation = PhysMat2.fromAngle(a);

  /// Transforms a local point into world space.
  PhysVec2 transformPoint(PhysVec2 p) => position + rotation.mul(p);

  /// Inverse transforms a world point into local space.
  PhysVec2 inverseTransformPoint(PhysVec2 p) => rotation.mulTransposed(p - position);

  /// Transforms a local vector into world space (rotation only).
  PhysVec2 transformVector(PhysVec2 v) => rotation.mul(v);

  /// Inverse transforms a world vector into local space (inverse rotation only).
  PhysVec2 inverseTransformVector(PhysVec2 v) => rotation.mulTransposed(v);

  PhysTransform copy() => PhysTransform(position: position, rotation: rotation);
}
