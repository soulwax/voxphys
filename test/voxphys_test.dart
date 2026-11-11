import 'package:test/test.dart';
import 'package:voxphys/voxphys.dart';

void main() {
  group('voxphys: Math & Transforms', () {
    test('PhysVec2 arithmetic and cross products', () {
      const v1 = PhysVec2(3.0, 4.0);
      const v2 = PhysVec2(1.0, 2.0);

      expect(v1.length, closeTo(5.0, 1e-6));
      expect(v1.normalized.length, closeTo(1.0, 1e-6));
      expect(v1.dot(v2), equals(11.0)); // 3*1 + 4*2
      expect(v1.cross(v2), equals(2.0)); // 3*2 - 4*1 = 2
      expect(v1.perp, equals(const PhysVec2(-4.0, 3.0)));

      final crossed = v1.crossScalar(2.0);
      expect(crossed, equals(const PhysVec2(8.0, -6.0)));

      final scalarCrossed = PhysVec2.scalarCross(2.0, v1);
      expect(scalarCrossed, equals(const PhysVec2(-8.0, 6.0)));
    });

    test('PhysMat2 rotation and transformation', () {
      final rot = PhysMat2.fromAngle(3.141592653589793 / 2.0); // 90 deg
      final v = rot.mul(const PhysVec2(1.0, 0.0));
      expect(v.x, closeTo(0.0, 1e-6));
      expect(v.y, closeTo(1.0, 1e-6));

      final vInv = rot.mulTransposed(v);
      expect(vInv.x, closeTo(1.0, 1e-6));
      expect(vInv.y, closeTo(0.0, 1e-6));
    });

    test('PhysTransform point and vector transforms', () {
      final xf = PhysTransform.fromPosAngle(const PhysVec2(5.0, 10.0), 0.0);
      final p = xf.transformPoint(const PhysVec2(2.0, 3.0));
      expect(p, equals(const PhysVec2(7.0, 13.0)));
      expect(xf.inverseTransformPoint(p), equals(const PhysVec2(2.0, 3.0)));
    });

    test('PhysAabb2 overlaps and raycast', () {
      final aabb1 = PhysAabb2.fromCenterExtents(PhysVec2.zero, const PhysVec2(2.0, 2.0));
      final aabb2 = PhysAabb2.fromCenterExtents(const PhysVec2(3.0, 0.0), const PhysVec2(2.0, 2.0));
      final aabb3 = PhysAabb2.fromCenterExtents(const PhysVec2(10.0, 0.0), const PhysVec2(1.0, 1.0));

      expect(aabb1.overlaps(aabb2), isTrue);
      expect(aabb1.overlaps(aabb3), isFalse);

      final hitFraction = aabb1.raycast(const PhysVec2(-5.0, 0.0), const PhysVec2(5.0, 0.0));
      expect(hitFraction, isNotNull);
      expect(hitFraction!, closeTo(0.3, 1e-3)); // -5 + 0.3 * 10 = -2
    });
  });

  group('voxphys: Shapes & Raycasting', () {
    test('CircleShape properties and raycast', () {
      final circle = CircleShape(radius: 2.0);
      final mass = circle.computeMass(1.0);
      expect(mass.mass, closeTo(3.14159265 * 4.0, 1e-3));

      final xf = PhysTransform.fromPosAngle(const PhysVec2(5.0, 0.0), 0.0);
      expect(circle.containsPoint(xf, const PhysVec2(5.5, 0.0)), isTrue);
      expect(circle.containsPoint(xf, const PhysVec2(10.0, 0.0)), isFalse);

      final rayHit = circle.raycast(xf, PhysVec2.zero, const PhysVec2(10.0, 0.0));
      expect(rayHit, isNotNull);
      final (fraction, normal) = rayHit!;
      expect(fraction, closeTo(0.3, 1e-3)); // hits at x = 3.0
      expect(normal.x, closeTo(-1.0, 1e-3));
    });

    test('PolygonShape box properties and raycast', () {
      final box = PolygonShape.box(1.0, 1.0); // 2x2 box
      final mass = box.computeMass(1.0);
      expect(mass.mass, closeTo(4.0, 1e-3));

      final xf = PhysTransform.fromPosAngle(const PhysVec2(0.0, 0.0), 0.0);
      expect(box.containsPoint(xf, const PhysVec2(0.5, 0.5)), isTrue);
      expect(box.containsPoint(xf, const PhysVec2(2.0, 0.0)), isFalse);

      final rayHit = box.raycast(xf, const PhysVec2(-5.0, 0.0), const PhysVec2(5.0, 0.0));
      expect(rayHit, isNotNull);
      final (fraction, normal) = rayHit!;
      expect(fraction, closeTo(0.4, 1e-3)); // -5 + 0.4 * 10 = -1
      expect(normal.x, closeTo(-1.0, 1e-3));
    });
  });

  group('voxphys: Narrowphase Collision Detection (SAT)', () {
    test('Circle vs Circle collision', () {
      final c1 = CircleShape(radius: 1.0);
      final c2 = CircleShape(radius: 1.0);
      final xf1 = PhysTransform.fromPosAngle(PhysVec2.zero, 0.0);
      final xf2 = PhysTransform.fromPosAngle(const PhysVec2(1.5, 0.0), 0.0);

      final m = CollisionDetector.detect(c1, xf1, c2, xf2);
      expect(m, isNotNull);
      expect(m!.isColliding, isTrue);
      expect(m.points.length, equals(1));
      expect(m.points.first.penetration, closeTo(0.5, 1e-4));
      expect(m.normal.x, closeTo(1.0, 1e-4));
    });

    test('Polygon vs Polygon (box vs box) SAT collision', () {
      final b1 = PolygonShape.box(1.0, 1.0); // [-1,1] x [-1,1]
      final b2 = PolygonShape.box(1.0, 1.0);
      final xf1 = PhysTransform.fromPosAngle(PhysVec2.zero, 0.0);
      final xf2 = PhysTransform.fromPosAngle(const PhysVec2(1.8, 0.0), 0.0);

      final m = CollisionDetector.detect(b1, xf1, b2, xf2);
      expect(m, isNotNull);
      expect(m!.isColliding, isTrue);
      expect(m.points.isNotEmpty, isTrue);
      expect(m.points.first.penetration, closeTo(0.2, 1e-3));
      expect(m.normal.x.abs(), closeTo(1.0, 1e-3));
    });

    test('Polygon vs Circle collision', () {
      final poly = PolygonShape.box(2.0, 0.5); // ground plane
      final circle = CircleShape(radius: 0.5);
      final xfPoly = PhysTransform.fromPosAngle(PhysVec2.zero, 0.0);
      final xfCircle = PhysTransform.fromPosAngle(const PhysVec2(0.0, 0.8), 0.0);

      final m = CollisionDetector.detect(poly, xfPoly, circle, xfCircle);
      expect(m, isNotNull);
      expect(m!.isColliding, isTrue);
      expect(m.points.first.penetration, closeTo(0.2, 1e-3)); // 0.5 - (0.8 - 0.5) = 0.2
    });
  });

  group('voxphys: Dynamics & Constraint Solver in World', () {
    test('RigidBody free-fall under gravity', () {
      final world = PhysWorld2D(gravity: const PhysVec2(0.0, -10.0));
      final body = world.createBody(RigidBodyDef(
        type: BodyType.dynamicBody,
        position: const PhysVec2(0.0, 10.0),
      ));
      body.createFixtureFromShape(CircleShape(radius: 0.5));

      // 10 steps of 0.1s = 1.0s total: y(t) = y0 + 0.5*a*t^2 = 10 - 5 = 5
      for (var i = 0; i < 10; i++) {
        world.step(0.1);
      }

      expect(body.linearVelocity.y, closeTo(-10.0, 0.1));
      expect(body.position.y, closeTo(4.5, 0.5)); // Euler integration discrete step approximation
    });

    test('RigidBody rests on static ground', () {
      final world = PhysWorld2D(gravity: const PhysVec2(0.0, -10.0));

      // Static ground at y = 0, halfHeight = 0.5 (top edge at y = 0.5)
      final ground = world.createBody(RigidBodyDef(
        type: BodyType.staticBody,
        position: const PhysVec2(0.0, 0.0),
      ));
      ground.createFixture(FixtureDef(
        shape: PolygonShape.box(10.0, 0.5),
        friction: 0.5,
        restitution: 0.0,
      ));

      // Dynamic box at y = 2.0, halfHeight = 0.5
      final box = world.createBody(RigidBodyDef(
        type: BodyType.dynamicBody,
        position: const PhysVec2(0.0, 2.0),
      ));
      box.createFixture(FixtureDef(
        shape: PolygonShape.box(0.5, 0.5),
        friction: 0.5,
        restitution: 0.0,
      ));

      // Step 60 frames (1 second)
      for (var i = 0; i < 60; i++) {
        world.step(1.0 / 60.0);
      }

      // Box should come to rest resting on ground: box center should be at y = 0.5 + 0.5 = 1.0
      expect(box.position.y, closeTo(1.0, 0.05));
      expect(box.linearVelocity.y.abs(), lessThan(0.05));
    });

    test('DistanceJoint constrains distance between two bodies', () {
      final world = PhysWorld2D(gravity: const PhysVec2(0.0, -10.0));

      // Static anchor at (0, 5)
      final anchor = world.createBody(RigidBodyDef(
        type: BodyType.staticBody,
        position: const PhysVec2(0.0, 5.0),
      ));

      // Dynamic bob at (3, 5), distance = 3
      final bob = world.createBody(RigidBodyDef(
        type: BodyType.dynamicBody,
        position: const PhysVec2(3.0, 5.0),
      ));
      bob.createFixtureFromShape(CircleShape(radius: 0.2));

      world.createJoint(DistanceJoint(
        bodyA: anchor,
        bodyB: bob,
        length: 3.0,
      ));

      // Step simulation
      for (var i = 0; i < 60; i++) {
        world.step(1.0 / 60.0);
      }

      // Distance should remain approximately 3.0
      final dist = (bob.position - anchor.position).length;
      expect(dist, closeTo(3.0, 0.05));
    });

    test('RevoluteJoint pin constraint maintains pivot point', () {
      final world = PhysWorld2D(gravity: const PhysVec2(0.0, -10.0));

      final pivot = world.createBody(RigidBodyDef(
        type: BodyType.staticBody,
        position: const PhysVec2(0.0, 5.0),
      ));

      final rod = world.createBody(RigidBodyDef(
        type: BodyType.dynamicBody,
        position: const PhysVec2(2.0, 5.0),
      ));
      rod.createFixtureFromShape(PolygonShape.box(2.0, 0.2));

      // Pin at pivot: localAnchorA = (0,0), localAnchorB = (-2, 0)
      world.createJoint(RevoluteJoint(
        bodyA: pivot,
        bodyB: rod,
        localAnchorA: PhysVec2.zero,
        localAnchorB: const PhysVec2(-2.0, 0.0),
      ));

      for (var i = 0; i < 30; i++) {
        world.step(1.0 / 60.0);
      }

      final anchorA = pivot.getWorldPoint(PhysVec2.zero);
      final anchorB = rod.getWorldPoint(const PhysVec2(-2.0, 0.0));
      expect((anchorB - anchorA).length, lessThan(0.05));
    });

    test('PrismaticJoint constrains motion along horizontal axis', () {
      final world = PhysWorld2D(gravity: const PhysVec2(0.0, -10.0));

      final rail = world.createBody(RigidBodyDef(
        type: BodyType.staticBody,
        position: PhysVec2.zero,
      ));

      final cart = world.createBody(RigidBodyDef(
        type: BodyType.dynamicBody,
        position: const PhysVec2(0.0, 0.0),
      ));
      cart.createFixtureFromShape(PolygonShape.box(0.5, 0.5));

      // Prismatic along X axis
      world.createJoint(PrismaticJoint(
        bodyA: rail,
        bodyB: cart,
        localAxisA: const PhysVec2(1.0, 0.0),
      ));

      // Apply initial horizontal velocity
      cart.linearVelocity = const PhysVec2(5.0, 0.0);

      for (var i = 0; i < 30; i++) {
        world.step(1.0 / 60.0);
      }

      // Even with gravity pointing down, cart's Y position stays clamped to 0
      expect(cart.position.y.abs(), lessThan(0.05));
      expect(cart.position.x, greaterThan(1.0));
    });

    test('PhysWorld2D raycast returns closest hit and surface normal', () {
      final world = PhysWorld2D();
      final body = world.createBody(RigidBodyDef(
        type: BodyType.staticBody,
        position: const PhysVec2(5.0, 0.0),
      ));
      body.createFixtureFromShape(CircleShape(radius: 1.0));

      final hit = world.raycastClosest(PhysVec2.zero, const PhysVec2(10.0, 0.0));
      expect(hit, isNotNull);
      expect(hit!.point.x, closeTo(4.0, 1e-3));
      expect(hit.normal.x, closeTo(-1.0, 1e-3));
    });
  });
}
