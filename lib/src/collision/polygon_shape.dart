import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import '../math/phys_mat2.dart';
import '../math/phys_aabb2.dart';
import 'phys_shape.dart';

class PolygonShape extends PhysShape {
  final List<PhysVec2> vertices;
  final List<PhysVec2> normals;
  final PhysVec2 centroid;

  PolygonShape._({
    required this.vertices,
    required this.normals,
    required this.centroid,
  });

  /// Creates a convex polygon from vertices arranged in counter-clockwise order.
  factory PolygonShape(List<PhysVec2> rawVertices) {
    if (rawVertices.length < 3) {
      throw ArgumentError('Polygon must have at least 3 vertices.');
    }

    // Compute centroid
    var area = 0.0;
    var c = PhysVec2.zero;
    final count = rawVertices.length;

    for (var i = 0; i < count; i++) {
      final p1 = rawVertices[i];
      final p2 = rawVertices[(i + 1) % count];
      final d = p1.cross(p2);
      area += d;
      c += (p1 + p2) * (d / 3.0);
    }

    area *= 0.5;
    if (area.abs() < 1e-9) {
      throw ArgumentError('Polygon has near-zero area.');
    }

    // If clockwise (negative area), reverse
    List<PhysVec2> verts;
    if (area < 0.0) {
      verts = rawVertices.reversed.toList();
      area = -area;
    } else {
      verts = List.of(rawVertices);
    }

    final centroid = c / (2.0 * area);

    // Compute edge normals (outward pointing: perpCw(p2 - p1).normalized)
    final norms = <PhysVec2>[];
    for (var i = 0; i < count; i++) {
      final p1 = verts[i];
      final p2 = verts[(i + 1) % count];
      final edge = p2 - p1;
      norms.add(edge.perpCw.normalized);
    }

    return PolygonShape._(
      vertices: verts,
      normals: norms,
      centroid: centroid,
    );
  }

  /// Creates an oriented box polygon centered at [center] (default (0,0)) with dimensions 2*halfWidth x 2*halfHeight.
  factory PolygonShape.box(
    double halfWidth,
    double halfHeight, [
    PhysVec2? center,
    double angle = 0.0,
  ]) {
    final c = center ?? PhysVec2.zero;
    final rot = PhysMat2.fromAngle(angle);

    final raw = [
      rot.mul(PhysVec2(-halfWidth, -halfHeight)) + c,
      rot.mul(PhysVec2(halfWidth, -halfHeight)) + c,
      rot.mul(PhysVec2(halfWidth, halfHeight)) + c,
      rot.mul(PhysVec2(-halfWidth, halfHeight)) + c,
    ];

    return PolygonShape(raw);
  }

  @override
  ShapeType get type => ShapeType.polygon;

  @override
  PhysAabb2 computeAabb(PhysTransform transform) {
    return PhysAabb2.fromPoints(
      vertices.map((v) => transform.transformPoint(v)),
    );
  }

  @override
  MassData computeMass(double density) {
    // Polygon mass and rotational inertia calculation about centroid
    var area = 0.0;
    var inertia = 0.0;
    final count = vertices.length;

    for (var i = 0; i < count; i++) {
      final p1 = vertices[i] - centroid;
      final p2 = vertices[(i + 1) % count] - centroid;

      final d = p1.cross(p2);
      final triangleArea = 0.5 * d;
      area += triangleArea;

      // Inertia of triangle about origin
      final triInertia = (d / 6.0) * (p1.dot(p1) + p1.dot(p2) + p2.dot(p2));
      inertia += triInertia;
    }

    final mass = density * area.abs();
    final totalInertia = density * inertia.abs() + mass * centroid.lengthSquared;

    return MassData(
      mass: mass,
      center: centroid,
      rotationalInertia: totalInertia,
    );
  }

  @override
  bool containsPoint(PhysTransform transform, PhysVec2 point) {
    final localPoint = transform.inverseTransformPoint(point);

    for (var i = 0; i < vertices.length; i++) {
      final edge = localPoint - vertices[i];
      if (normals[i].dot(edge) > 0.0) {
        return false;
      }
    }
    return true;
  }

  @override
  (double fraction, PhysVec2 normal)? raycast(
    PhysTransform transform,
    PhysVec2 p1,
    PhysVec2 p2,
  ) {
    // Put ray into polygon's frame of reference
    final localP1 = transform.inverseTransformPoint(p1);
    final localP2 = transform.inverseTransformPoint(p2);
    final d = localP2 - localP1;

    var lower = 0.0;
    var upper = 1.0;
    var hitNormalIndex = -1;

    for (var i = 0; i < vertices.length; i++) {
      // Separating axis test for edge
      final numerator = normals[i].dot(vertices[i] - localP1);
      final denominator = normals[i].dot(d);

      if (denominator == 0.0) {
        if (numerator < 0.0) return null;
      } else {
        if (denominator < 0.0 && numerator < lower * denominator) {
          // Increase lower
          lower = numerator / denominator;
          hitNormalIndex = i;
        } else if (denominator > 0.0 && numerator < upper * denominator) {
          // Decrease upper
          upper = numerator / denominator;
        }
      }

      if (upper < lower) return null;
    }

    if (hitNormalIndex >= 0) {
      final worldNormal = transform.transformVector(normals[hitNormalIndex]);
      return (lower, worldNormal);
    }

    return null;
  }
}
