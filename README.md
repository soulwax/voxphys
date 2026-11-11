# voxphys

A fast, lightweight, pure-Dart 2D rigid body physics engine with sequential impulse constraint solver, SAT contact manifolds, joints, and broadphase queries.

## Features

- **Sequential Impulse Solver**: Warm-started velocity solver, Coulomb friction cone, restitution, and Baumgarte position stabilization.
- **Convex Collision Detection (SAT)**: Separating Axis Theorem with edge clipping contact manifold generation for Box/Polygon vs Polygon, Polygon vs Circle, and Circle vs Circle.
- **Rigid Bodies**: Dynamic, Kinematic, and Static bodies with mass, inertia, damping, and continuous forces/impulses.
- **Constraints & Joints**: DistanceJoint (with soft spring option), RevoluteJoint (pin/hinge with motor and angle limits), PrismaticJoint (slider with motor and limits), and SpringJoint.
- **Broadphase & Raycasts**: Fast AABB overlap checks, AABB spatial queries, and segment raycasting with hit point and surface normals.
- **Zero Platform Dependencies**: 100% pure Dart, runs anywhere Dart runs (VM, Flutter, Web, Native AOT).

## Example

```dart
import 'package:voxphys/voxphys.dart';

void main() {
  final world = PhysWorld2D(gravity: PhysVec2(0.0, -9.8));

  // Static ground
  final ground = world.createBody(RigidBodyDef(type: BodyType.staticBody));
  ground.createFixtureFromShape(PolygonShape.box(10.0, 0.5));

  // Dynamic falling box
  final box = world.createBody(RigidBodyDef(
    type: BodyType.dynamicBody,
    position: PhysVec2(0.0, 5.0),
  ));
  box.createFixture(FixtureDef(
    shape: PolygonShape.box(0.5, 0.5),
    density: 1.0,
    restitution: 0.3,
  ));

  // Step simulation
  for (var i = 0; i < 60; i++) {
    world.step(1.0 / 60.0);
  }
}
```
