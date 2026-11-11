import 'dart:math' as math;
import 'phys_vec2.dart';

/// An axis-aligned bounding box in 2D space.
class PhysAabb2 {
  final PhysVec2 lowerBound;
  final PhysVec2 upperBound;

  const PhysAabb2(this.lowerBound, this.upperBound);

  factory PhysAabb2.fromCenterExtents(PhysVec2 center, PhysVec2 halfExtents) {
    return PhysAabb2(center - halfExtents, center + halfExtents);
  }

  factory PhysAabb2.fromPoints(Iterable<PhysVec2> points) {
    var minX = double.infinity;
    var minY = double.infinity;
    var maxX = -double.infinity;
    var maxY = -double.infinity;

    for (final p in points) {
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }

    return PhysAabb2(PhysVec2(minX, minY), PhysVec2(maxX, maxY));
  }

  double get width => upperBound.x - lowerBound.x;
  double get height => upperBound.y - lowerBound.y;
  PhysVec2 get center => (lowerBound + upperBound) * 0.5;
  PhysVec2 get extents => (upperBound - lowerBound) * 0.5;

  bool containsPoint(PhysVec2 point) {
    return point.x >= lowerBound.x &&
        point.x <= upperBound.x &&
        point.y >= lowerBound.y &&
        point.y <= upperBound.y;
  }

  bool overlaps(PhysAabb2 other) {
    if (upperBound.x < other.lowerBound.x || lowerBound.x > other.upperBound.x) {
      return false;
    }
    if (upperBound.y < other.lowerBound.y || lowerBound.y > other.upperBound.y) {
      return false;
    }
    return true;
  }

  PhysAabb2 combined(PhysAabb2 other) {
    return PhysAabb2(
      PhysVec2(
        math.min(lowerBound.x, other.lowerBound.x),
        math.min(lowerBound.y, other.lowerBound.y),
      ),
      PhysVec2(
        math.max(upperBound.x, other.upperBound.x),
        math.max(upperBound.y, other.upperBound.y),
      ),
    );
  }

  PhysAabb2 expand(double margin) {
    final m = PhysVec2(margin, margin);
    return PhysAabb2(lowerBound - m, upperBound + m);
  }

  /// Raycasts against this AABB. Returns distance fraction t in [0..1], or null if missed.
  double? raycast(PhysVec2 p1, PhysVec2 p2) {
    final d = p2 - p1;
    var tMin = 0.0;
    var tMax = 1.0;

    // X axis
    if (d.x.abs() < 1e-9) {
      if (p1.x < lowerBound.x || p1.x > upperBound.x) return null;
    } else {
      final invD = 1.0 / d.x;
      var t1 = (lowerBound.x - p1.x) * invD;
      var t2 = (upperBound.x - p1.x) * invD;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return null;
    }

    // Y axis
    if (d.y.abs() < 1e-9) {
      if (p1.y < lowerBound.y || p1.y > upperBound.y) return null;
    } else {
      final invD = 1.0 / d.y;
      var t1 = (lowerBound.y - p1.y) * invD;
      var t2 = (upperBound.y - p1.y) * invD;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return null;
    }

    return tMin;
  }

  @override
  String toString() => 'PhysAabb2($lowerBound, $upperBound)';
}
