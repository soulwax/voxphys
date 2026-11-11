import 'dart:math' as math;
import '../../math/phys_vec2.dart';
import '../rigid_body.dart';
import 'phys_joint.dart';

class RevoluteJointDef {
  final RigidBody bodyA;
  final RigidBody bodyB;
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  final double referenceAngle;
  final bool enableLimit;
  final double lowerAngle;
  final double upperAngle;
  final bool enableMotor;
  final double motorSpeed;
  final double maxMotorTorque;
  final bool collideConnected;
  final Object? userData;

  const RevoluteJointDef({
    required this.bodyA,
    required this.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.referenceAngle = 0.0,
    this.enableLimit = false,
    this.lowerAngle = 0.0,
    this.upperAngle = 0.0,
    this.enableMotor = false,
    this.motorSpeed = 0.0,
    this.maxMotorTorque = 0.0,
    this.collideConnected = false,
    this.userData,
  });
}

class RevoluteJoint extends PhysJoint {
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  double referenceAngle;

  bool enableLimit;
  double lowerAngle;
  double upperAngle;

  bool enableMotor;
  double motorSpeed;
  double maxMotorTorque;

  // Solver cache
  PhysVec2 _rA = PhysVec2.zero;
  PhysVec2 _rB = PhysVec2.zero;
  PhysVec2 _impulse = PhysVec2.zero;
  double _motorImpulse = 0.0;
  double _limitImpulse = 0.0;

  // 2x2 point-to-point effective mass matrix K
  double _k11 = 0.0;
  double _k12 = 0.0;
  double _k22 = 0.0;
  double _motorMass = 0.0;

  RevoluteJoint({
    required super.bodyA,
    required super.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.referenceAngle = 0.0,
    this.enableLimit = false,
    this.lowerAngle = 0.0,
    this.upperAngle = 0.0,
    this.enableMotor = false,
    this.motorSpeed = 0.0,
    this.maxMotorTorque = 0.0,
    super.collideConnected,
    super.userData,
  });

  factory RevoluteJoint.fromDef(RevoluteJointDef def) {
    return RevoluteJoint(
      bodyA: def.bodyA,
      bodyB: def.bodyB,
      localAnchorA: def.localAnchorA,
      localAnchorB: def.localAnchorB,
      referenceAngle: def.referenceAngle,
      enableLimit: def.enableLimit,
      lowerAngle: def.lowerAngle,
      upperAngle: def.upperAngle,
      enableMotor: def.enableMotor,
      motorSpeed: def.motorSpeed,
      maxMotorTorque: def.maxMotorTorque,
      collideConnected: def.collideConnected,
      userData: def.userData,
    );
  }

  @override
  JointType get type => JointType.revolute;

  @override
  PhysVec2 get anchorA => bodyA.getWorldPoint(localAnchorA);

  @override
  PhysVec2 get anchorB => bodyB.getWorldPoint(localAnchorB);

  @override
  PhysVec2 get reactionForce => _impulse;

  @override
  double get reactionTorque => _motorImpulse + _limitImpulse;

  double get jointAngle => bodyB.angle - bodyA.angle - referenceAngle;
  double get jointSpeed => bodyB.angularVelocity - bodyA.angularVelocity;

  @override
  void initVelocityConstraints(double dt) {
    _rA = bodyA.transform.transformVector(localAnchorA);
    _rB = bodyB.transform.transformVector(localAnchorB);

    final mA = bodyA.invMass;
    final mB = bodyB.invMass;
    final iA = bodyA.invInertia;
    final iB = bodyB.invInertia;

    _k11 = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    _k12 = -_rA.x * _rA.y * iA - _rB.x * _rB.y * iB;
    _k22 = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;

    _motorMass = iA + iB > 0.0 ? 1.0 / (iA + iB) : 0.0;
  }

  @override
  void warmStart() {
    if (bodyA.isDynamic) {
      bodyA.linearVelocity -= _impulse * bodyA.invMass;
      bodyA.angularVelocity -= bodyA.invInertia * (_rA.cross(_impulse) + _motorImpulse + _limitImpulse);
    }
    if (bodyB.isDynamic) {
      bodyB.linearVelocity += _impulse * bodyB.invMass;
      bodyB.angularVelocity += bodyB.invInertia * (_rB.cross(_impulse) + _motorImpulse + _limitImpulse);
    }
  }

  @override
  void solveVelocityConstraints() {
    // 1. Solve motor constraint
    if (enableMotor) {
      final cDot = bodyB.angularVelocity - bodyA.angularVelocity - motorSpeed;
      var impulse = -_motorMass * cDot;
      final oldImpulse = _motorImpulse;
      final maxImpulse = maxMotorTorque;
      _motorImpulse = math.max(-maxImpulse, math.min(maxImpulse, _motorImpulse + impulse));
      impulse = _motorImpulse - oldImpulse;

      if (bodyA.isDynamic) bodyA.angularVelocity -= bodyA.invInertia * impulse;
      if (bodyB.isDynamic) bodyB.angularVelocity += bodyB.invInertia * impulse;
    }

    // 2. Solve limit constraint
    if (enableLimit) {
      final angle = jointAngle;
      if (angle <= lowerAngle) {
        final cDot = bodyB.angularVelocity - bodyA.angularVelocity;
        var impulse = -_motorMass * cDot;
        final oldImpulse = _limitImpulse;
        _limitImpulse = math.max(0.0, _limitImpulse + impulse);
        impulse = _limitImpulse - oldImpulse;

        if (bodyA.isDynamic) bodyA.angularVelocity -= bodyA.invInertia * impulse;
        if (bodyB.isDynamic) bodyB.angularVelocity += bodyB.invInertia * impulse;
      } else if (angle >= upperAngle) {
        final cDot = bodyB.angularVelocity - bodyA.angularVelocity;
        var impulse = -_motorMass * cDot;
        final oldImpulse = _limitImpulse;
        _limitImpulse = math.min(0.0, _limitImpulse + impulse);
        impulse = _limitImpulse - oldImpulse;

        if (bodyA.isDynamic) bodyA.angularVelocity -= bodyA.invInertia * impulse;
        if (bodyB.isDynamic) bodyB.angularVelocity += bodyB.invInertia * impulse;
      }
    }

    // 3. Solve point-to-point constraint
    final vA = bodyA.linearVelocity + PhysVec2.scalarCross(bodyA.angularVelocity, _rA);
    final vB = bodyB.linearVelocity + PhysVec2.scalarCross(bodyB.angularVelocity, _rB);
    final cDot = vB - vA;

    // Solve 2x2 linear system K * impulse = -cDot
    final det = _k11 * _k22 - _k12 * _k12;
    if (det.abs() > 1e-12) {
      final impulseX = (_k22 * -cDot.x - _k12 * -cDot.y) / det;
      final impulseY = (-_k12 * -cDot.x + _k11 * -cDot.y) / det;
      final impulse = PhysVec2(impulseX, impulseY);
      _impulse += impulse;

      if (bodyA.isDynamic) {
        bodyA.linearVelocity -= impulse * bodyA.invMass;
        bodyA.angularVelocity -= bodyA.invInertia * _rA.cross(impulse);
      }
      if (bodyB.isDynamic) {
        bodyB.linearVelocity += impulse * bodyB.invMass;
        bodyB.angularVelocity += bodyB.invInertia * _rB.cross(impulse);
      }
    }
  }

  @override
  bool solvePositionConstraints() {
    final rA = bodyA.transform.transformVector(localAnchorA);
    final rB = bodyB.transform.transformVector(localAnchorB);
    final c = (bodyB.position + rB) - (bodyA.position + rA);

    final mA = bodyA.invMass;
    final mB = bodyB.invMass;
    final iA = bodyA.invInertia;
    final iB = bodyB.invInertia;

    final k11 = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    final k12 = -rA.x * rA.y * iA - rB.x * rB.y * iB;
    final k22 = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;

    final det = k11 * k22 - k12 * k12;
    if (det.abs() > 1e-12) {
      final impulseX = (k22 * -c.x - k12 * -c.y) / det;
      final impulseY = (-k12 * -c.x + k11 * -c.y) / det;
      final impulse = PhysVec2(impulseX, impulseY);

      if (bodyA.isDynamic) {
        bodyA.position -= impulse * mA;
        bodyA.angle -= iA * rA.cross(impulse);
      }
      if (bodyB.isDynamic) {
        bodyB.position += impulse * mB;
        bodyB.angle += iB * rB.cross(impulse);
      }
    }

    return c.length < 0.005;
  }
}
