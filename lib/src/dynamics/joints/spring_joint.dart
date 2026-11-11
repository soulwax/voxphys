import '../../math/phys_vec2.dart';
import '../rigid_body.dart';
import 'phys_joint.dart';

class SpringJointDef {
  final RigidBody bodyA;
  final RigidBody bodyB;
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  final double restLength;
  final double stiffness;
  final double damping;
  final bool collideConnected;
  final Object? userData;

  const SpringJointDef({
    required this.bodyA,
    required this.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.restLength = 1.0,
    this.stiffness = 50.0,
    this.damping = 2.0,
    this.collideConnected = false,
    this.userData,
  });
}

class SpringJoint extends PhysJoint {
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  double restLength;
  double stiffness;
  double damping;

  PhysVec2 _force = PhysVec2.zero;

  SpringJoint({
    required super.bodyA,
    required super.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.restLength = 1.0,
    this.stiffness = 50.0,
    this.damping = 2.0,
    super.collideConnected,
    super.userData,
  });

  factory SpringJoint.fromDef(SpringJointDef def) {
    return SpringJoint(
      bodyA: def.bodyA,
      bodyB: def.bodyB,
      localAnchorA: def.localAnchorA,
      localAnchorB: def.localAnchorB,
      restLength: def.restLength,
      stiffness: def.stiffness,
      damping: def.damping,
      collideConnected: def.collideConnected,
      userData: def.userData,
    );
  }

  @override
  JointType get type => JointType.spring;

  @override
  PhysVec2 get anchorA => bodyA.getWorldPoint(localAnchorA);

  @override
  PhysVec2 get anchorB => bodyB.getWorldPoint(localAnchorB);

  @override
  PhysVec2 get reactionForce => _force;

  @override
  double get reactionTorque => 0.0;

  @override
  void initVelocityConstraints(double dt) {
    final pA = anchorA;
    final pB = anchorB;
    final delta = pB - pA;
    final currentLength = delta.length;

    if (currentLength < 1e-6) {
      _force = PhysVec2.zero;
      return;
    }

    final dir = delta / currentLength;
    final displacement = currentLength - restLength;

    final rA = pA - bodyA.position;
    final rB = pB - bodyB.position;
    final vA = bodyA.linearVelocity + PhysVec2.scalarCross(bodyA.angularVelocity, rA);
    final vB = bodyB.linearVelocity + PhysVec2.scalarCross(bodyB.angularVelocity, rB);
    final relVel = (vB - vA).dot(dir);

    // Spring-damper force F = -k * x - c * v
    final springForceMag = stiffness * displacement + damping * relVel;
    _force = dir * springForceMag;

    bodyA.applyForce(_force, pA);
    bodyB.applyForce(-_force, pB);
  }

  @override
  void warmStart() {}

  @override
  void solveVelocityConstraints() {}

  @override
  bool solvePositionConstraints() => true;
}
