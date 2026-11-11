import '../math/phys_vec2.dart';

enum ManifoldType {
  circles,
  faceA,
  faceB,
}

class ContactPoint {
  PhysVec2 position;
  PhysVec2 localPointA;
  PhysVec2 localPointB;
  double penetration;
  double normalImpulse;
  double tangentImpulse;
  int id;

  ContactPoint({
    required this.position,
    this.localPointA = PhysVec2.zero,
    this.localPointB = PhysVec2.zero,
    required this.penetration,
    this.normalImpulse = 0.0,
    this.tangentImpulse = 0.0,
    this.id = 0,
  });
}

class ContactManifold {
  final ManifoldType type;
  final PhysVec2 normal; // from A to B
  final List<ContactPoint> points;

  ContactManifold({
    required this.type,
    required this.normal,
    required this.points,
  });

  bool get isColliding => points.isNotEmpty;
}
