import 'dart:math' as math;
import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import 'phys_shape.dart';
import 'circle_shape.dart';
import 'polygon_shape.dart';
import 'segment_shape.dart';
import 'contact_manifold.dart';

class CollisionDetector {
  /// Detects collision between two generic shapes with transforms.
  static ContactManifold? detect(
    PhysShape shapeA,
    PhysTransform xfA,
    PhysShape shapeB,
    PhysTransform xfB,
  ) {
    if (shapeA is CircleShape && shapeB is CircleShape) {
      return collideCircles(shapeA, xfA, shapeB, xfB);
    }
    if (shapeA is PolygonShape && shapeB is CircleShape) {
      return collidePolygonAndCircle(shapeA, xfA, shapeB, xfB);
    }
    if (shapeA is CircleShape && shapeB is PolygonShape) {
      final m = collidePolygonAndCircle(shapeB, xfB, shapeA, xfA);
      if (m == null) return null;
      // Flip normal for A -> B
      return ContactManifold(
        type: ManifoldType.faceB,
        normal: -m.normal,
        points: m.points
            .map((p) => ContactPoint(
                  position: p.position,
                  localPointA: p.localPointB,
                  localPointB: p.localPointA,
                  penetration: p.penetration,
                  normalImpulse: p.normalImpulse,
                  tangentImpulse: p.tangentImpulse,
                  id: p.id,
                ))
            .toList(),
      );
    }
    if (shapeA is PolygonShape && shapeB is PolygonShape) {
      return collidePolygons(shapeA, xfA, shapeB, xfB);
    }
    if (shapeA is SegmentShape && shapeB is CircleShape) {
      return collideSegmentAndCircle(shapeA, xfA, shapeB, xfB);
    }
    if (shapeA is CircleShape && shapeB is SegmentShape) {
      final m = collideSegmentAndCircle(shapeB, xfB, shapeA, xfA);
      if (m == null) return null;
      return ContactManifold(
        type: ManifoldType.faceB,
        normal: -m.normal,
        points: m.points,
      );
    }
    return null;
  }

  static ContactManifold? collideCircles(
    CircleShape circleA,
    PhysTransform xfA,
    CircleShape circleB,
    PhysTransform xfB,
  ) {
    final pA = xfA.transformPoint(circleA.center);
    final pB = xfB.transformPoint(circleB.center);

    final d = pB - pA;
    final distSq = d.lengthSquared;
    final radiusSum = circleA.radius + circleB.radius;

    if (distSq > radiusSum * radiusSum) {
      return null;
    }

    final dist = math.sqrt(distSq);
    PhysVec2 normal;
    double penetration;
    PhysVec2 contactPoint;

    if (dist < 1e-9) {
      normal = const PhysVec2(1.0, 0.0);
      penetration = radiusSum;
      contactPoint = pA;
    } else {
      normal = d / dist;
      penetration = radiusSum - dist;
      contactPoint = pA + normal * (circleA.radius - penetration * 0.5);
    }

    return ContactManifold(
      type: ManifoldType.circles,
      normal: normal,
      points: [
        ContactPoint(
          position: contactPoint,
          localPointA: xfA.inverseTransformPoint(contactPoint),
          localPointB: xfB.inverseTransformPoint(contactPoint),
          penetration: penetration,
        ),
      ],
    );
  }

  static ContactManifold? collidePolygonAndCircle(
    PolygonShape polyA,
    PhysTransform xfA,
    CircleShape circleB,
    PhysTransform xfB,
  ) {
    // Transform circle center into polyA local coordinates
    final cWorld = xfB.transformPoint(circleB.center);
    final cLocal = xfA.inverseTransformPoint(cWorld);

    var maxSeparation = -double.infinity;
    var faceIndex = 0;
    final count = polyA.vertices.length;

    for (var i = 0; i < count; i++) {
      final s = polyA.normals[i].dot(cLocal - polyA.vertices[i]);
      if (s > circleB.radius) {
        return null; // Separating axis found
      }
      if (s > maxSeparation) {
        maxSeparation = s;
        faceIndex = i;
      }
    }

    // Vertices of the edge of minimum separation
    final v1 = polyA.vertices[faceIndex];
    final v2 = polyA.vertices[(faceIndex + 1) % count];

    PhysVec2 normal;
    double penetration;
    PhysVec2 contactPoint;

    if (maxSeparation < 0.0) {
      // Circle center is inside polygon
      normal = xfA.transformVector(polyA.normals[faceIndex]);
      penetration = circleB.radius - maxSeparation;
      contactPoint = cWorld - normal * circleB.radius;
    } else {
      // Circle center is outside edge
      final u1 = (cLocal - v1).dot(v2 - v1);
      final u2 = (cLocal - v2).dot(v1 - v2);

      if (u1 <= 0.0) {
        // Closest to vertex 1
        final diff = cLocal - v1;
        if (diff.lengthSquared > circleB.radius * circleB.radius) return null;
        final len = diff.length;
        if (len < 1e-9) return null;
        normal = xfA.transformVector(diff / len);
        penetration = circleB.radius - len;
        contactPoint = xfA.transformPoint(v1);
      } else if (u2 <= 0.0) {
        // Closest to vertex 2
        final diff = cLocal - v2;
        if (diff.lengthSquared > circleB.radius * circleB.radius) return null;
        final len = diff.length;
        if (len < 1e-9) return null;
        normal = xfA.transformVector(diff / len);
        penetration = circleB.radius - len;
        contactPoint = xfA.transformPoint(v2);
      } else {
        // Closest to face
        normal = xfA.transformVector(polyA.normals[faceIndex]);
        penetration = circleB.radius - maxSeparation;
        contactPoint = cWorld - normal * circleB.radius;
      }
    }

    return ContactManifold(
      type: ManifoldType.faceA,
      normal: normal,
      points: [
        ContactPoint(
          position: contactPoint,
          localPointA: xfA.inverseTransformPoint(contactPoint),
          localPointB: xfB.inverseTransformPoint(contactPoint),
          penetration: penetration,
        ),
      ],
    );
  }

  static ContactManifold? collidePolygons(
    PolygonShape polyA,
    PhysTransform xfA,
    PolygonShape polyB,
    PhysTransform xfB,
  ) {
    // 1. Find max separation on A's faces
    final (sepA, faceA) = _findMaxSeparation(polyA, xfA, polyB, xfB);
    if (sepA > 0.0) return null;

    // 2. Find max separation on B's faces
    final (sepB, faceB) = _findMaxSeparation(polyB, xfB, polyA, xfA);
    if (sepB > 0.0) return null;

    PolygonShape refPoly;
    PolygonShape incPoly;
    PhysTransform refXf;
    PhysTransform incXf;
    int refFace;
    bool flip;

    if (sepB > sepA + 0.001) {
      refPoly = polyB;
      refXf = xfB;
      incPoly = polyA;
      incXf = xfA;
      refFace = faceB;
      flip = true;
    } else {
      refPoly = polyA;
      refXf = xfA;
      incPoly = polyB;
      incXf = xfB;
      refFace = faceA;
      flip = false;
    }

    // Reference face vertices & normal in world space
    final v1Local = refPoly.vertices[refFace];
    final v2Local = refPoly.vertices[(refFace + 1) % refPoly.vertices.length];
    final refV1 = refXf.transformPoint(v1Local);
    final refV2 = refXf.transformPoint(v2Local);
    final refNormal = refXf.transformVector(refPoly.normals[refFace]);

    // Find incident edge on incPoly (most anti-parallel to refNormal)
    final incNormalTarget = incXf.inverseTransformVector(refNormal);
    var incFace = 0;
    var minDot = double.infinity;
    for (var i = 0; i < incPoly.normals.length; i++) {
      final dot = incPoly.normals[i].dot(incNormalTarget);
      if (dot < minDot) {
        minDot = dot;
        incFace = i;
      }
    }

    final incV1 = incXf.transformPoint(incPoly.vertices[incFace]);
    final incV2 = incXf.transformPoint(incPoly.vertices[(incFace + 1) % incPoly.vertices.length]);

    // Clip incident edge against reference face side planes
    final refTangent = (refV2 - refV1).normalized;

    // Side 1: normal = -refTangent, offset = -refTangent.dot(refV1)
    var clipped = _clipSegmentToLine(incV1, incV2, -refTangent, -refTangent.dot(refV1));
    if (clipped.length < 2) return null;

    // Side 2: normal = refTangent, offset = refTangent.dot(refV2)
    clipped = _clipSegmentToLine(clipped[0], clipped[1], refTangent, refTangent.dot(refV2));
    if (clipped.length < 2) return null;

    // Filter points behind reference face
    final refPlaneOffset = refNormal.dot(refV1);
    final contactPoints = <ContactPoint>[];

    for (var i = 0; i < clipped.length; i++) {
      final cp = clipped[i];
      final separation = refNormal.dot(cp) - refPlaneOffset;
      if (separation <= 0.0) {
        final penetration = -separation;
        final worldPoint = cp;
        contactPoints.add(
          ContactPoint(
            position: worldPoint,
            localPointA: xfA.inverseTransformPoint(worldPoint),
            localPointB: xfB.inverseTransformPoint(worldPoint),
            penetration: penetration,
            id: i,
          ),
        );
      }
    }

    if (contactPoints.isEmpty) return null;

    final worldNormal = flip ? -refNormal : refNormal;

    return ContactManifold(
      type: flip ? ManifoldType.faceB : ManifoldType.faceA,
      normal: worldNormal,
      points: contactPoints,
    );
  }

  static (double, int) _findMaxSeparation(
    PolygonShape poly1,
    PhysTransform xf1,
    PolygonShape poly2,
    PhysTransform xf2,
  ) {
    var maxSep = -double.infinity;
    var bestFace = 0;

    for (var i = 0; i < poly1.vertices.length; i++) {
      final nWorld = xf1.transformVector(poly1.normals[i]);
      final vWorld = xf1.transformPoint(poly1.vertices[i]);

      // Find support point on poly2 in direction -nWorld
      var minDot = double.infinity;
      for (final v2 in poly2.vertices) {
        final v2World = xf2.transformPoint(v2);
        final dot = nWorld.dot(v2World - vWorld);
        if (dot < minDot) {
          minDot = dot;
        }
      }

      if (minDot > maxSep) {
        maxSep = minDot;
        bestFace = i;
      }
    }

    return (maxSep, bestFace);
  }

  static List<PhysVec2> _clipSegmentToLine(
    PhysVec2 p1,
    PhysVec2 p2,
    PhysVec2 normal,
    double offset,
  ) {
    final d1 = normal.dot(p1) - offset;
    final d2 = normal.dot(p2) - offset;
    final out = <PhysVec2>[];

    if (d1 <= 0.0) out.add(p1);
    if (d1 * d2 < 0.0) {
      final alpha = d1 / (d1 - d2);
      final intersect = p1.lerp(p2, alpha);
      out.add(intersect);
    }
    if (d2 <= 0.0) out.add(p2);

    return out;
  }

  static ContactManifold? collideSegmentAndCircle(
    SegmentShape segA,
    PhysTransform xfA,
    CircleShape circleB,
    PhysTransform xfB,
  ) {
    final p1 = xfA.transformPoint(segA.p1);
    final p2 = xfA.transformPoint(segA.p2);
    final c = xfB.transformPoint(circleB.center);

    final edge = p2 - p1;
    final len = edge.length;
    if (len < 1e-9) return null;

    final dir = edge / len;
    final t = math.max(0.0, math.min(len, (c - p1).dot(dir)));
    final closest = p1 + dir * t;

    final diff = c - closest;
    final distSq = diff.lengthSquared;

    if (distSq > circleB.radius * circleB.radius) {
      return null;
    }

    final dist = math.sqrt(distSq);
    PhysVec2 normal;
    double penetration;

    if (dist < 1e-9) {
      normal = xfA.transformVector(segA.normal);
      penetration = circleB.radius;
    } else {
      normal = -diff / dist; // From seg to circle
      penetration = circleB.radius - dist;
    }

    return ContactManifold(
      type: ManifoldType.faceA,
      normal: normal,
      points: [
        ContactPoint(
          position: closest,
          localPointA: xfA.inverseTransformPoint(closest),
          localPointB: xfB.inverseTransformPoint(closest),
          penetration: penetration,
        ),
      ],
    );
  }
}
