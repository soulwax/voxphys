import 'dart:math' as math;
import '../../math/phys_vec2.dart';
import '../rigid_body.dart';
import 'phys_joint.dart';

class DistanceJointDef {
  final RigidBody bodyA;
  final RigidBody bodyB;
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  final double? length;
  final double frequencyHz;
  final double dampingRatio;
  final bool collideConnected;
  final Object? userData;

  const DistanceJointDef({
    required this.bodyA,
    required this.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    this.length,
    this.frequencyHz = 0.0,
    this.dampingRatio = 0.0,
    this.collideConnected = false,
    this.userData,
  });
}

class DistanceJoint extends PhysJoint {
  final PhysVec2 localAnchorA;
  final PhysVec2 localAnchorB;
  double length;
  double frequencyHz;
  double dampingRatio;

  // Solver cache
  PhysVec2 _u = PhysVec2.zero;
  PhysVec2 _rA = PhysVec2.zero;
  PhysVec2 _rB = PhysVec2.zero;
  double _mass = 0.0;
  double _impulse = 0.0;
  double _gamma = 0.0;
  double _bias = 0.0;

  DistanceJoint({
    required super.bodyA,
    required super.bodyB,
    this.localAnchorA = PhysVec2.zero,
    this.localAnchorB = PhysVec2.zero,
    double? length,
    this.frequencyHz = 0.0,
    this.dampingRatio = 0.0,
    super.collideConnected,
    super.userData,
  }) : length = length ?? (bodyA.getWorldPoint(localAnchorA) - bodyB.getWorldPoint(localAnchorB)).length;

  factory DistanceJoint.fromDef(DistanceJointDef def) {
    return DistanceJoint(
      bodyA: def.bodyA,
      bodyB: def.bodyB,
      localAnchorA: def.localAnchorA,
      localAnchorB: def.localAnchorB,
      length: def.length,
      frequencyHz: def.frequencyHz,
      dampingRatio: def.dampingRatio,
      collideConnected: def.collideConnected,
      userData: def.userData,
    );
  }

  @override
  JointType get type => JointType.distance;

  @override
  PhysVec2 get anchorA => bodyA.getWorldPoint(localAnchorA);

  @override
  PhysVec2 get anchorB => bodyB.getWorldPoint(localAnchorB);

  @override
  PhysVec2 get reactionForce => _u * _impulse;

  @override
  double get reactionTorque => 0.0;

  @override
  void initVelocityConstraints(double dt) {
    _rA = bodyA.transform.transformVector(localAnchorA);
    _rB = bodyB.transform.transformVector(localAnchorB);

    final d = (bodyB.position + _rB) - (bodyA.position + _rA);
    final currentLength = d.length;

    if (currentLength > 1e-6) {
      _u = d / currentLength;
    } else {
      _u = PhysVec2.zero;
    }

    final crAu = _rA.cross(_u);
    final crBu = _rB.cross(_u);
    var invMass = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crAu * crAu + bodyB.invInertia * crBu * crBu;

    if (frequencyHz > 0.0) {
      final c = currentLength - length;
      final omega = 2.0 * math.pi * frequencyHz;
      final k = invMass > 0.0 ? 1.0 / invMass : 0.0;
      final dParam = 2.0 * k * dampingRatio * omega;
      final kParam = k * omega * omega;

      _gamma = dt * (dParam + dt * kParam);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = c * dt * kParam * _gamma;
      invMass += _gamma;
      _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      _gamma = 0.0;
      _bias = 0.0;
      _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    }
  }

  @override
  void warmStart() {
    final p = _u * _impulse;
    if (bodyA.isDynamic) {
      bodyA.linearVelocity -= p * bodyA.invMass;
      bodyA.angularVelocity -= bodyA.invInertia * _rA.cross(p);
    }
    if (bodyB.isDynamic) {
      bodyB.linearVelocity += p * bodyB.invMass;
      bodyB.angularVelocity += bodyB.invInertia * _rB.cross(p);
    }
  }

  @override
  void solveVelocityConstraints() {
    final vA = bodyA.linearVelocity + PhysVec2.scalarCross(bodyA.angularVelocity, _rA);
    final vB = bodyB.linearVelocity + PhysVec2.scalarCross(bodyB.angularVelocity, _rB);
    final cDot = _u.dot(vB - vA);

    final impulse = -_mass * (cDot + _bias + _gamma * _impulse);
    _impulse += impulse;

    final p = _u * impulse;
    if (bodyA.isDynamic) {
      bodyA.linearVelocity -= p * bodyA.invMass;
      bodyA.angularVelocity -= bodyA.invInertia * _rA.cross(p);
    }
    if (bodyB.isDynamic) {
      bodyB.linearVelocity += p * bodyB.invMass;
      bodyB.angularVelocity += bodyB.invInertia * _rB.cross(p);
    }
  }

  @override
  bool solvePositionConstraints() {
    if (frequencyHz > 0.0) return true; // Soft constraint handled in velocity solver

    final rA = bodyA.transform.transformVector(localAnchorA);
    final rB = bodyB.transform.transformVector(localAnchorB);

    final d = (bodyB.position + rB) - (bodyA.position + rA);
    final currentLength = d.length;
    if (currentLength < 1e-6) return true;

    final u = d / currentLength;
    final c = currentLength - length;

    final crAu = rA.cross(u);
    final crBu = rB.cross(u);
    final k = bodyA.invMass + bodyB.invMass + bodyA.invInertia * crAu * crAu + bodyB.invInertia * crBu * crBu;
    if (k == 0.0) return true;

    final impulse = -c / k;
    final p = u * impulse;

    if (bodyA.isDynamic) {
      bodyA.position -= p * bodyA.invMass;
      bodyA.angle -= bodyA.invInertia * rA.cross(p);
    }
    if (bodyB.isDynamic) {
      bodyB.position += p * bodyB.invMass;
      bodyB.angle += bodyB.invInertia * rB.cross(p);
    }

    return c.abs() < 0.01;
  }
}
