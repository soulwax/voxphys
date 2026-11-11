import 'dart:math' as math;
import '../collision/contact_manifold.dart';
import '../collision/collision_detector.dart';
import 'rigid_body.dart';

class Contact {
  final Fixture fixtureA;
  final Fixture fixtureB;

  ContactManifold? manifold;
  bool isTouching = false;
  bool enabled = true;

  double friction;
  double restitution;

  Contact({
    required this.fixtureA,
    required this.fixtureB,
  })  : friction = math.sqrt(fixtureA.friction * fixtureB.friction),
        restitution = math.max(fixtureA.restitution, fixtureB.restitution);

  RigidBody get bodyA => fixtureA.body;
  RigidBody get bodyB => fixtureB.body;
  bool get isSensor => fixtureA.isSensor || fixtureB.isSensor;

  void update() {
    final oldManifold = manifold;
    manifold = CollisionDetector.detect(
      fixtureA.shape,
      fixtureA.body.transform,
      fixtureB.shape,
      fixtureB.body.transform,
    );

    final wasTouching = isTouching;
    isTouching = manifold != null && manifold!.isColliding;

    // Warm starting match: persist normal and tangent impulses across frames
    if (isTouching && oldManifold != null) {
      for (final newPt in manifold!.points) {
        for (final oldPt in oldManifold.points) {
          if (newPt.id == oldPt.id) {
            newPt.normalImpulse = oldPt.normalImpulse;
            newPt.tangentImpulse = oldPt.tangentImpulse;
            break;
          }
        }
      }
    }

    if (!wasTouching && isTouching) {
      // Just started touching
    }
  }
}
