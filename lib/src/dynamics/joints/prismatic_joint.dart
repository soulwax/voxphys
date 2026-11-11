import 'dart:math' as math;
import '../../math/phys_vec2.dart';
import '../rigid_body.dart';
import 'phys_joint.dart';

class PrismaticJointDef {
  final RigidBody bodyA;
  final RigidBody bodyB;
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  final PhysVec2 localAxisA;
  final double referenceAngle;
  final bool enableLimit;
  final double lowerTranslation;
  final double upperTranslation;
  final bool enableMotor;
  final double maxMotorForce;
  final double motorSpeed;
  final bool collideConnected;
  final Object? userData;

  const PrismaticJointDef({
    required this.bodyA,
    required this.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.localAxisA = const PhysVec2(1.0, 0.0),
    this.referenceAngle = 0.0,
    this.enableLimit = false,
    this.lowerTranslation = 0.0,
    this.upperTranslation = 0.0,
    this.enableMotor = false,
    this.maxMotorForce = 0.0,
    this.motorSpeed = 0.0,
    this.collideConnected = false,
    this.userData,
  });
}

class PrismaticJoint extends PhysJoint {
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  final PhysVec2 localAxisA;
  double referenceAngle;

  bool enableLimit;
  double lowerTranslation;
  double upperTranslation;

  bool enableMotor;
  double maxMotorForce;
  double motorSpeed;

  // Solver cache
  PhysVec2 _axis = PhysVec2.zero;
  PhysVec2 _perp = PhysVec2.zero;
  double _s1 = 0.0;
  double _s2 = 0.0;
  double _a1 = 0.0;
  double _a2 = 0.0;
  double _kPerp = 0.0;
  double _kAngular = 0.0;
  double _kMotor = 0.0;

  double _impulsePerp = 0.0;
  double _impulseAngular = 0.0;
  double _motorImpulse = 0.0;
  double _limitImpulse = 0.0;

  PrismaticJoint({
    required super.bodyA,
    required super.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.localAxisA = const PhysVec2(1.0, 0.0),
    this.referenceAngle = 0.0,
    this.enableLimit = false,
    this.lowerTranslation = 0.0,
    this.upperTranslation = 0.0,
    this.enableMotor = false,
    this.maxMotorForce = 0.0,
    this.motorSpeed = 0.0,
    super.collideConnected,
    super.userData,
  });

  factory PrismaticJoint.fromDef(PrismaticJointDef def) {
    return PrismaticJoint(
      bodyA: def.bodyA,
      bodyB: def.bodyB,
      localAnchorA: def.localAnchorA,
      localAnchorB: def.localAnchorB,
      localAxisA: def.localAxisA.normalized,
      referenceAngle: def.referenceAngle,
      enableLimit: def.enableLimit,
      lowerTranslation: def.lowerTranslation,
      upperTranslation: def.upperTranslation,
      enableMotor: def.enableMotor,
      maxMotorForce: def.maxMotorForce,
      motorSpeed: def.motorSpeed,
      collideConnected: def.collideConnected,
      userData: def.userData,
    );
  }

  @override
  JointType get type => JointType.prismatic;

  @override
  PhysVec2 get anchorA => bodyA.getWorldPoint(localAnchorA);

  @override
  PhysVec2 get anchorB => bodyB.getWorldPoint(localAnchorB);

  @override
  PhysVec2 get reactionForce => _perp * _impulsePerp + _axis * (_motorImpulse + _limitImpulse);

  @override
  double get reactionTorque => _impulseAngular;

  double get jointTranslation {
    final pA = bodyA.getWorldPoint(localAnchorA);
    final pB = bodyB.getWorldPoint(localAnchorB);
    final axis = bodyA.getWorldVector(localAxisA);
    return (pB - pA).dot(axis);
  }

  double get jointSpeed {
    final axis = bodyA.getWorldVector(localAxisA);
    final rA = bodyA.transform.transformVector(localAnchorA);
    final rB = bodyB.transform.transformVector(localAnchorB);
    final vA = bodyA.linearVelocity + PhysVec2.scalarCross(bodyA.angularVelocity, rA);
    final vB = bodyB.linearVelocity + PhysVec2.scalarCross(bodyB.angularVelocity, rB);
    return (vB - vA).dot(axis);
  }

  @override
  void initVelocityConstraints(double dt) {
    _axis = bodyA.transform.transformVector(localAxisA);
    _perp = _axis.perp;

    final rA = bodyA.transform.transformVector(localAnchorA);
    final rB = bodyB.transform.transformVector(localAnchorB);
    final d = (bodyB.position + rB) - (bodyA.position + rA);

    _s1 = (rA + d).cross(_perp);
    _s2 = rB.cross(_perp);
    _a1 = (rA + d).cross(_axis);
    _a2 = rB.cross(_axis);

    final mA = bodyA.invMass;
    final mB = bodyB.invMass;
    final iA = bodyA.invInertia;
    final iB = bodyB.invInertia;

    _kPerp = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
    _kAngular = iA + iB;
    _kMotor = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
  }

  @override
  void warmStart() {
    final p = _perp * _impulsePerp + _axis * (_motorImpulse + _limitImpulse);
    final lA = _impulsePerp * _s1 + (_motorImpulse + _limitImpulse) * _a1 + _impulseAngular;
    final lB = _impulsePerp * _s2 + (_motorImpulse + _limitImpulse) * _a2;

    if (bodyA.isDynamic) {
      bodyA.linearVelocity -= p * bodyA.invMass;
      bodyA.angularVelocity -= bodyA.invInertia * lA;
    }
    if (bodyB.isDynamic) {
      bodyB.linearVelocity += p * bodyB.invMass;
      bodyB.angularVelocity += bodyB.invInertia * lB;
    }
  }

  @override
  void solveVelocityConstraints() {
    // 1. Solve motor constraint
    if (enableMotor && _kMotor > 0.0) {
      final speed = jointSpeed;
      final cDot = speed - motorSpeed;
      var impulse = -cDot / _kMotor;
      final oldImpulse = _motorImpulse;
      _motorImpulse = math.max(-maxMotorForce, math.min(maxMotorForce, _motorImpulse + impulse));
      impulse = _motorImpulse - oldImpulse;

      final p = _axis * impulse;
      if (bodyA.isDynamic) {
        bodyA.linearVelocity -= p * bodyA.invMass;
        bodyA.angularVelocity -= bodyA.invInertia * impulse * _a1;
      }
      if (bodyB.isDynamic) {
        bodyB.linearVelocity += p * bodyB.invMass;
        bodyB.angularVelocity += bodyB.invInertia * impulse * _a2;
      }
    }

    // 2. Solve limit constraint
    if (enableLimit && _kMotor > 0.0) {
      final translation = jointTranslation;
      if (translation <= lowerTranslation) {
        final cDot = jointSpeed;
        var impulse = -cDot / _kMotor;
        final oldImpulse = _limitImpulse;
        _limitImpulse = math.max(0.0, _limitImpulse + impulse);
        impulse = _limitImpulse - oldImpulse;

        final p = _axis * impulse;
        if (bodyA.isDynamic) {
          bodyA.linearVelocity -= p * bodyA.invMass;
          bodyA.angularVelocity -= bodyA.invInertia * impulse * _a1;
        }
        if (bodyB.isDynamic) {
          bodyB.linearVelocity += p * bodyB.invMass;
          bodyB.angularVelocity += bodyB.invInertia * impulse * _a2;
        }
      } else if (translation >= upperTranslation) {
        final cDot = jointSpeed;
        var impulse = -cDot / _kMotor;
        final oldImpulse = _limitImpulse;
        _limitImpulse = math.min(0.0, _limitImpulse + impulse);
        impulse = _limitImpulse - oldImpulse;

        final p = _axis * impulse;
        if (bodyA.isDynamic) {
          bodyA.linearVelocity -= p * bodyA.invMass;
          bodyA.angularVelocity -= bodyA.invInertia * impulse * _a1;
        }
        if (bodyB.isDynamic) {
          bodyB.linearVelocity += p * bodyB.invMass;
          bodyB.angularVelocity += bodyB.invInertia * impulse * _a2;
        }
      }
    }

    // 3. Solve 2D perpendicular and angular constraint
    if (_kPerp > 0.0) {
      final rA = bodyA.transform.transformVector(localAnchorA);
      final rB = bodyB.transform.transformVector(localAnchorB);
      final vA = bodyA.linearVelocity + PhysVec2.scalarCross(bodyA.angularVelocity, rA);
      final vB = bodyB.linearVelocity + PhysVec2.scalarCross(bodyB.angularVelocity, rB);
      final cDot = (vB - vA).dot(_perp);

      final impulse = -cDot / _kPerp;
      _impulsePerp += impulse;

      final p = _perp * impulse;
      if (bodyA.isDynamic) {
        bodyA.linearVelocity -= p * bodyA.invMass;
        bodyA.angularVelocity -= bodyA.invInertia * impulse * _s1;
      }
      if (bodyB.isDynamic) {
        bodyB.linearVelocity += p * bodyB.invMass;
        bodyB.angularVelocity += bodyB.invInertia * impulse * _s2;
      }
    }

    if (_kAngular > 0.0) {
      final cDot = bodyB.angularVelocity - bodyA.angularVelocity;
      final impulse = -cDot / _kAngular;
      _impulseAngular += impulse;

      if (bodyA.isDynamic) bodyA.angularVelocity -= bodyA.invInertia * impulse;
      if (bodyB.isDynamic) bodyB.angularVelocity += bodyB.invInertia * impulse;
    }
  }

  @override
  bool solvePositionConstraints() {
    final rA = bodyA.transform.transformVector(localAnchorA);
    final rB = bodyB.transform.transformVector(localAnchorB);
    final d = (bodyB.position + rB) - (bodyA.position + rA);
    final axis = bodyA.transform.transformVector(localAxisA);
    final perp = axis.perp;

    final cPerp = d.dot(perp);
    final cAngle = bodyB.angle - bodyA.angle - referenceAngle;

    if (_kPerp > 0.0) {
      final impulse = -cPerp / _kPerp;
      final p = perp * impulse;
      if (bodyA.isDynamic) {
        bodyA.position -= p * bodyA.invMass;
        bodyA.angle -= bodyA.invInertia * impulse * _s1;
      }
      if (bodyB.isDynamic) {
        bodyB.position += p * bodyB.invMass;
        bodyB.angle += bodyB.invInertia * impulse * _s2;
      }
    }

    if (_kAngular > 0.0) {
      final impulse = -cAngle / _kAngular;
      if (bodyA.isDynamic) bodyA.angle -= bodyA.invInertia * impulse;
      if (bodyB.isDynamic) bodyB.angle += bodyB.invInertia * impulse;
    }

    return cPerp.abs() < 0.005 && cAngle.abs() < 0.01;
  }
}
