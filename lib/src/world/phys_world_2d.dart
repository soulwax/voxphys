import '../math/phys_vec2.dart';
import '../math/phys_aabb2.dart';
import '../dynamics/rigid_body.dart';
import '../dynamics/contact.dart';
import '../dynamics/contact_solver.dart';
import '../dynamics/joints/phys_joint.dart';
import 'raycast_hit_2d.dart';

typedef ContactCallback = void Function(Contact contact);

class PhysWorld2D {
  PhysVec2 gravity;

  final List<RigidBody> bodies = [];
  final List<PhysJoint> joints = [];
  final List<Contact> contacts = [];

  ContactCallback? onBeginContact;
  ContactCallback? onEndContact;

  PhysWorld2D({
    this.gravity = const PhysVec2(0.0, -9.8),
    this.onBeginContact,
    this.onEndContact,
  });

  RigidBody createBody([RigidBodyDef? def]) {
    final body = def != null ? RigidBody.fromDef(def) : RigidBody();
    bodies.add(body);
    return body;
  }

  void destroyBody(RigidBody body) {
    // Remove attached joints
    joints.removeWhere((j) => j.bodyA == body || j.bodyB == body);

    // Remove contacts
    contacts.removeWhere((c) {
      final involves = c.bodyA == body || c.bodyB == body;
      if (involves && c.isTouching) {
        onEndContact?.call(c);
      }
      return involves;
    });

    bodies.remove(body);
  }

  T createJoint<T extends PhysJoint>(T joint) {
    joints.add(joint);
    return joint;
  }

  void destroyJoint(PhysJoint joint) {
    joints.remove(joint);
  }

  void step(
    double dt, {
    int velocityIterations = 8,
    int positionIterations = 3,
  }) {
    if (dt <= 0.0) return;

    // 1. Broadphase & Fixture AABB synchronization
    for (final b in bodies) {
      b.synchronizeFixtures();
    }
    _findNewContacts();

    // 2. Narrowphase collision updates & contact callbacks
    for (var i = contacts.length - 1; i >= 0; i--) {
      final c = contacts[i];
      final wasTouching = c.isTouching;

      // Check if AABBs still overlap
      if (!c.fixtureA.aabb.overlaps(c.fixtureB.aabb)) {
        c.isTouching = false;
        if (wasTouching) onEndContact?.call(c);
        contacts.removeAt(i);
        continue;
      }

      c.update();

      if (!wasTouching && c.isTouching) {
        onBeginContact?.call(c);
      } else if (wasTouching && !c.isTouching) {
        onEndContact?.call(c);
      }
    }

    // 3. Integrate forces & apply damping
    for (final b in bodies) {
      if (!b.isDynamic) continue;

      // Integrate linear velocity: v += (F * invMass + gravity * gravityScale) * dt
      final effGravity = gravity * b.gravityScale;
      b.linearVelocity += (b.force * b.invMass + effGravity) * dt;
      b.angularVelocity += (b.torque * b.invInertia) * dt;

      // Damping
      if (b.linearDamping > 0.0) {
        final factor = (1.0 - dt * b.linearDamping).clamp(0.0, 1.0);
        b.linearVelocity *= factor;
      }
      if (b.angularDamping > 0.0) {
        final factor = (1.0 - dt * b.angularDamping).clamp(0.0, 1.0);
        b.angularVelocity *= factor;
      }
    }

    // 4. Init velocity constraints
    final solver = ContactSolver(contacts);
    solver.initVelocityConstraints(dt);
    for (final j in joints) {
      j.initVelocityConstraints(dt);
    }

    // 5. Warm starting
    solver.warmStart();
    for (final j in joints) {
      j.warmStart();
    }

    // 6. Velocity iterations
    for (var i = 0; i < velocityIterations; i++) {
      for (final j in joints) {
        j.solveVelocityConstraints();
      }
      solver.solveVelocityConstraints();
    }
    solver.storeImpulses();

    // 7. Integrate positions
    for (final b in bodies) {
      if (b.isStatic) continue;

      b.position += b.linearVelocity * dt;
      b.angle += b.angularVelocity * dt;
      b.synchronizeFixtures();
    }

    // 8. Position iterations
    for (var i = 0; i < positionIterations; i++) {
      final contactsSolved = solver.solvePositionConstraints();
      var jointsSolved = true;
      for (final j in joints) {
        if (!j.solvePositionConstraints()) {
          jointsSolved = false;
        }
      }
      if (contactsSolved && jointsSolved) break;
    }

    // 9. Clear applied forces
    for (final b in bodies) {
      b.force = PhysVec2.zero;
      b.torque = 0.0;
    }
  }

  void _findNewContacts() {
    for (var i = 0; i < bodies.length; i++) {
      final bA = bodies[i];
      for (var j = i + 1; j < bodies.length; j++) {
        final bB = bodies[j];

        // Static vs Static or Kinematic vs Static bodies don't collide
        if (bA.isStatic && bB.isStatic) continue;
        if (bA.isKinematic && bB.isStatic) continue;
        if (bA.isStatic && bB.isKinematic) continue;

        // Check if joint disables collision between connected bodies
        var canCollide = true;
        for (final joint in joints) {
          if (!joint.collideConnected &&
              ((joint.bodyA == bA && joint.bodyB == bB) || (joint.bodyA == bB && joint.bodyB == bA))) {
            canCollide = false;
            break;
          }
        }
        if (!canCollide) continue;

        for (final fA in bA.fixtures) {
          for (final fB in bB.fixtures) {
            // Layer filter mask
            if ((fA.categoryBits & fB.maskBits) == 0 || (fB.categoryBits & fA.maskBits) == 0) {
              continue;
            }

            if (fA.aabb.overlaps(fB.aabb)) {
              // Check if contact already exists
              var exists = false;
              for (final c in contacts) {
                if ((c.fixtureA == fA && c.fixtureB == fB) || (c.fixtureA == fB && c.fixtureB == fA)) {
                  exists = true;
                  break;
                }
              }

              if (!exists) {
                contacts.add(Contact(fixtureA: fA, fixtureB: fB));
              }
            }
          }
        }
      }
    }
  }

  List<Fixture> queryAabb(PhysAabb2 aabb) {
    final results = <Fixture>[];
    for (final b in bodies) {
      for (final f in b.fixtures) {
        if (f.aabb.overlaps(aabb)) {
          results.add(f);
        }
      }
    }
    return results;
  }

  PhysRaycastHit2D? raycastClosest(
    PhysVec2 p1,
    PhysVec2 p2, {
    int mask = 0xFFFF,
  }) {
    PhysRaycastHit2D? closestHit;
    var closestFraction = 1.0;

    for (final b in bodies) {
      for (final f in b.fixtures) {
        if ((f.categoryBits & mask) == 0) continue;
        if (f.aabb.raycast(p1, p2) == null) continue;

        final hit = f.shape.raycast(b.transform, p1, p2);
        if (hit != null) {
          final (fraction, normal) = hit;
          if (fraction < closestFraction) {
            closestFraction = fraction;
            final hitPoint = p1.lerp(p2, fraction);
            closestHit = PhysRaycastHit2D(
              fixture: f,
              point: hitPoint,
              normal: normal,
              fraction: fraction,
            );
          }
        }
      }
    }

    return closestHit;
  }

  List<PhysRaycastHit2D> raycastAll(
    PhysVec2 p1,
    PhysVec2 p2, {
    int mask = 0xFFFF,
  }) {
    final hits = <PhysRaycastHit2D>[];

    for (final b in bodies) {
      for (final f in b.fixtures) {
        if ((f.categoryBits & mask) == 0) continue;
        if (f.aabb.raycast(p1, p2) == null) continue;

        final hit = f.shape.raycast(b.transform, p1, p2);
        if (hit != null) {
          final (fraction, normal) = hit;
          final hitPoint = p1.lerp(p2, fraction);
          hits.add(
            PhysRaycastHit2D(
              fixture: f,
              point: hitPoint,
              normal: normal,
              fraction: fraction,
            ),
          );
        }
      }
    }

    hits.sort((a, b) => a.fraction.compareTo(b.fraction));
    return hits;
  }
}
