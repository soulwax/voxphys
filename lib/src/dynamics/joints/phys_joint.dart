import '../../math/phys_vec2.dart';
import '../rigid_body.dart';

enum JointType {
  distance,
  revolute,
  prismatic,
  spring,
}

abstract class PhysJoint {
  final RigidBody bodyA;
  final RigidBody bodyB;
  final bool collideConnected;
  Object? userData;

  PhysJoint({
    required this.bodyA,
    required this.bodyB,
    this.collideConnected = false,
    this.userData,
  });

  JointType get type;

  PhysVec2 get anchorA;
  PhysVec2 get anchorB;

  PhysVec2 get reactionForce;
  double get reactionTorque;

  void initVelocityConstraints(double dt);
  void warmStart();
  void solveVelocityConstraints();
  bool solvePositionConstraints();
}
