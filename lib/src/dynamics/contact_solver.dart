import 'dart:math' as math;
import '../math/phys_vec2.dart';
import 'rigid_body.dart';
import 'contact.dart';

class VelocityConstraintPoint {
  PhysVec2 rA = PhysVec2.zero;
  PhysVec2 rB = PhysVec2.zero;
  double normalMass = 0.0;
  double tangentMass = 0.0;
  double normalImpulse = 0.0;
  double tangentImpulse = 0.0;
  double velocityBias = 0.0;
}

class ContactVelocityConstraint {
  final Contact contact;
  final RigidBody bodyA;
  final RigidBody bodyB;
  PhysVec2 normal = PhysVec2.zero;
  PhysVec2 tangent = PhysVec2.zero;
  double friction = 0.0;
  double restitution = 0.0;
  final List<VelocityConstraintPoint> points = [];

  ContactVelocityConstraint({
    required this.contact,
    required this.bodyA,
    required this.bodyB,
  });
}

class ContactSolver {
  final List<Contact> contacts;
  final List<ContactVelocityConstraint> constraints = [];

  ContactSolver(this.contacts);

  void initVelocityConstraints(double dt) {
    constraints.clear();

    for (final contact in contacts) {
      if (!contact.isTouching || contact.isSensor) continue;
      final manifold = contact.manifold;
      if (manifold == null || manifold.points.isEmpty) continue;

      final bA = contact.bodyA;
      final bB = contact.bodyB;
      final normal = manifold.normal;
      final tangent = normal.perpCw;

      final vc = ContactVelocityConstraint(
        contact: contact,
        bodyA: bA,
        bodyB: bB,
      );
      vc.normal = normal;
      vc.tangent = tangent;
      vc.friction = contact.friction;
      vc.restitution = contact.restitution;

      for (final cp in manifold.points) {
        final vcp = VelocityConstraintPoint();
        vcp.rA = cp.position - bA.position;
        vcp.rB = cp.position - bB.position;

        final rnA = vcp.rA.cross(normal);
        final rnB = vcp.rB.cross(normal);
        final kNormal = bA.invMass + bB.invMass + bA.invInertia * rnA * rnA + bB.invInertia * rnB * rnB;
        vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        final rtA = vcp.rA.cross(tangent);
        final rtB = vcp.rB.cross(tangent);
        final kTangent = bA.invMass + bB.invMass + bA.invInertia * rtA * rtA + bB.invInertia * rtB * rtB;
        vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

        // Velocity bias for restitution
        final vA = bA.linearVelocity + PhysVec2.scalarCross(bA.angularVelocity, vcp.rA);
        final vB = bB.linearVelocity + PhysVec2.scalarCross(bB.angularVelocity, vcp.rB);
        final vRelNormal = (vB - vA).dot(normal);

        if (vRelNormal < -1.0) {
          vcp.velocityBias = -vc.restitution * vRelNormal;
        }

        // Retain warm starting impulses
        vcp.normalImpulse = cp.normalImpulse;
        vcp.tangentImpulse = cp.tangentImpulse;

        vc.points.add(vcp);
      }

      constraints.add(vc);
    }
  }

  void warmStart() {
    for (final vc in constraints) {
      final bA = vc.bodyA;
      final bB = vc.bodyB;
      final normal = vc.normal;
      final tangent = vc.tangent;

      for (final vcp in vc.points) {
        final p = normal * vcp.normalImpulse + tangent * vcp.tangentImpulse;

        if (bA.isDynamic) {
          bA.linearVelocity -= p * bA.invMass;
          bA.angularVelocity -= bA.invInertia * vcp.rA.cross(p);
        }
        if (bB.isDynamic) {
          bB.linearVelocity += p * bB.invMass;
          bB.angularVelocity += bB.invInertia * vcp.rB.cross(p);
        }
      }
    }
  }

  void solveVelocityConstraints() {
    for (final vc in constraints) {
      final bA = vc.bodyA;
      final bB = vc.bodyB;
      final normal = vc.normal;
      final tangent = vc.tangent;

      for (final vcp in vc.points) {
        // 1. Solve tangent (friction) constraint
        final vA = bA.linearVelocity + PhysVec2.scalarCross(bA.angularVelocity, vcp.rA);
        final vB = bB.linearVelocity + PhysVec2.scalarCross(bB.angularVelocity, vcp.rB);
        final vRelTangent = (vB - vA).dot(tangent);

        var deltaTangentImpulse = -vcp.tangentMass * vRelTangent;
        final maxFriction = vc.friction * vcp.normalImpulse;
        final newTangentImpulse = math.max(-maxFriction, math.min(maxFriction, vcp.tangentImpulse + deltaTangentImpulse));
        deltaTangentImpulse = newTangentImpulse - vcp.tangentImpulse;
        vcp.tangentImpulse = newTangentImpulse;

        final pTangent = tangent * deltaTangentImpulse;
        if (bA.isDynamic) {
          bA.linearVelocity -= pTangent * bA.invMass;
          bA.angularVelocity -= bA.invInertia * vcp.rA.cross(pTangent);
        }
        if (bB.isDynamic) {
          bB.linearVelocity += pTangent * bB.invMass;
          bB.angularVelocity += bB.invInertia * vcp.rB.cross(pTangent);
        }

        // 2. Solve normal constraint
        final vA2 = bA.linearVelocity + PhysVec2.scalarCross(bA.angularVelocity, vcp.rA);
        final vB2 = bB.linearVelocity + PhysVec2.scalarCross(bB.angularVelocity, vcp.rB);
        final vRelNormal = (vB2 - vA2).dot(normal);

        var deltaNormalImpulse = -vcp.normalMass * (vRelNormal - vcp.velocityBias);
        final newNormalImpulse = math.max(0.0, vcp.normalImpulse + deltaNormalImpulse);
        deltaNormalImpulse = newNormalImpulse - vcp.normalImpulse;
        vcp.normalImpulse = newNormalImpulse;

        final pNormal = normal * deltaNormalImpulse;
        if (bA.isDynamic) {
          bA.linearVelocity -= pNormal * bA.invMass;
          bA.angularVelocity -= bA.invInertia * vcp.rA.cross(pNormal);
        }
        if (bB.isDynamic) {
          bB.linearVelocity += pNormal * bB.invMass;
          bB.angularVelocity += bB.invInertia * vcp.rB.cross(pNormal);
        }
      }
    }
  }

  void storeImpulses() {
    for (final vc in constraints) {
      final manifold = vc.contact.manifold;
      if (manifold == null) continue;

      for (var i = 0; i < vc.points.length && i < manifold.points.length; i++) {
        manifold.points[i].normalImpulse = vc.points[i].normalImpulse;
        manifold.points[i].tangentImpulse = vc.points[i].tangentImpulse;
      }
    }
  }

  bool solvePositionConstraints() {
    const baumgarte = 0.2;
    const slop = 0.005;
    const maxLinearCorrection = 0.2;
    var minSeparation = 0.0;

    for (final vc in constraints) {
      final bA = vc.bodyA;
      final bB = vc.bodyB;
      final normal = vc.normal;

      for (final cp in vc.contact.manifold!.points) {
        final rA = cp.position - bA.position;
        final rB = cp.position - bB.position;

        final separation = -cp.penetration;
        minSeparation = math.min(minSeparation, separation);

        final C = math.max(0.0, math.min(maxLinearCorrection, -separation - slop)) * baumgarte;
        final rnA = rA.cross(normal);
        final rnB = rB.cross(normal);
        final k = bA.invMass + bB.invMass + bA.invInertia * rnA * rnA + bB.invInertia * rnB * rnB;
        if (k <= 0.0) continue;

        final impulse = normal * (C / k);
        if (bA.isDynamic) {
          bA.position -= impulse * bA.invMass;
          bA.angle -= bA.invInertia * rA.cross(impulse);
        }
        if (bB.isDynamic) {
          bB.position += impulse * bB.invMass;
          bB.angle += bB.invInertia * rB.cross(impulse);
        }
      }
    }

    return minSeparation >= -3.0 * slop;
  }
}
