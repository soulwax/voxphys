import '../math/phys_vec2.dart';
import '../dynamics/rigid_body.dart';

class PhysRaycastHit2D {
  final Fixture fixture;
  final PhysVec2 point;
  final PhysVec2 normal;
  final double fraction;

  PhysRaycastHit2D({
    required this.fixture,
    required this.point,
    required this.normal,
    required this.fraction,
  });

  RigidBody get body => fixture.body;
}
