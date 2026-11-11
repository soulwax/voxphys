import '../math/phys_vec2.dart';
import '../math/phys_transform.dart';
import '../math/phys_aabb2.dart';
import '../collision/phys_shape.dart';

enum BodyType {
  staticBody,
  kinematicBody,
  dynamicBody,
}

class FixtureDef {
  final PhysShape shape;
  final double density;
  final double friction;
  final double restitution;
  final bool isSensor;
  final int categoryBits;
  final int maskBits;
  final Object? userData;

  const FixtureDef({
    required this.shape,
    this.density = 1.0,
    this.friction = 0.2,
    this.restitution = 0.0,
    this.isSensor = false,
    this.categoryBits = 0x0001,
    this.maskBits = 0xFFFF,
    this.userData,
  });
}

class Fixture {
  final RigidBody body;
  final PhysShape shape;
  double density;
  double friction;
  double restitution;
  bool isSensor;
  int categoryBits;
  int maskBits;
  Object? userData;

  PhysAabb2 aabb = const PhysAabb2(PhysVec2.zero, PhysVec2.zero);

  Fixture({
    required this.body,
    required this.shape,
    this.density = 1.0,
    this.friction = 0.2,
    this.restitution = 0.0,
    this.isSensor = false,
    this.categoryBits = 0x0001,
    this.maskBits = 0xFFFF,
    this.userData,
  });

  void synchronizeAabb() {
    aabb = shape.computeAabb(body.transform);
  }
}

class RigidBodyDef {
  final BodyType type;
  final PhysVec2 position;
  final double angle;
  final PhysVec2 linearVelocity;
  final double angularVelocity;
  final double linearDamping;
  final double angularDamping;
  final double gravityScale;
  final bool fixedRotation;
  final bool bullet;
  final Object? userData;

  const RigidBodyDef({
    this.type = BodyType.dynamicBody,
    this.position = PhysVec2.zero,
    this.angle = 0.0,
    this.linearVelocity = PhysVec2.zero,
    this.angularVelocity = 0.0,
    this.linearDamping = 0.0,
    this.angularDamping = 0.0,
    this.gravityScale = 1.0,
    this.fixedRotation = false,
    this.bullet = false,
    this.userData,
  });
}

class RigidBody {
  BodyType type;
  final PhysTransform transform;

  PhysVec2 linearVelocity;
  double angularVelocity;
  PhysVec2 force = PhysVec2.zero;
  double torque = 0.0;

  double mass = 0.0;
  double invMass = 0.0;
  double inertia = 0.0;
  double invInertia = 0.0;

  double linearDamping;
  double angularDamping;
  double gravityScale;
  bool _fixedRotation;
  bool bullet;
  Object? userData;

  final List<Fixture> fixtures = [];

  RigidBody({
    this.type = BodyType.dynamicBody,
    PhysVec2? position,
    double angle = 0.0,
    PhysVec2? linearVelocity,
    this.angularVelocity = 0.0,
    this.linearDamping = 0.0,
    this.angularDamping = 0.0,
    this.gravityScale = 1.0,
    bool fixedRotation = false,
    this.bullet = false,
    this.userData,
  })  : transform = PhysTransform.fromPosAngle(position ?? PhysVec2.zero, angle),
        linearVelocity = linearVelocity ?? PhysVec2.zero,
        // ignore: prefer_initializing_formals
        _fixedRotation = fixedRotation;

  factory RigidBody.fromDef(RigidBodyDef def) {
    return RigidBody(
      type: def.type,
      position: def.position,
      angle: def.angle,
      linearVelocity: def.linearVelocity,
      angularVelocity: def.angularVelocity,
      linearDamping: def.linearDamping,
      angularDamping: def.angularDamping,
      gravityScale: def.gravityScale,
      fixedRotation: def.fixedRotation,
      bullet: def.bullet,
      userData: def.userData,
    );
  }

  PhysVec2 get position => transform.position;
  set position(PhysVec2 p) => transform.position = p;

  double get angle => transform.angle;
  set angle(double a) => transform.angle = a;

  bool get fixedRotation => _fixedRotation;
  set fixedRotation(bool val) {
    _fixedRotation = val;
    resetMassData();
  }

  bool get isDynamic => type == BodyType.dynamicBody;
  bool get isStatic => type == BodyType.staticBody;
  bool get isKinematic => type == BodyType.kinematicBody;

  Fixture createFixture(FixtureDef def) {
    final fixture = Fixture(
      body: this,
      shape: def.shape,
      density: def.density,
      friction: def.friction,
      restitution: def.restitution,
      isSensor: def.isSensor,
      categoryBits: def.categoryBits,
      maskBits: def.maskBits,
      userData: def.userData,
    );
    fixtures.add(fixture);
    fixture.synchronizeAabb();
    resetMassData();
    return fixture;
  }

  Fixture createFixtureFromShape(PhysShape shape, [double density = 1.0]) {
    return createFixture(FixtureDef(shape: shape, density: density));
  }

  void destroyFixture(Fixture fixture) {
    fixtures.remove(fixture);
    resetMassData();
  }

  void resetMassData() {
    mass = 0.0;
    invMass = 0.0;
    inertia = 0.0;
    invInertia = 0.0;

    if (type == BodyType.staticBody || type == BodyType.kinematicBody) {
      return;
    }

    var totalMass = 0.0;
    var totalInertia = 0.0;

    for (final f in fixtures) {
      if (f.density <= 0.0) continue;
      final massData = f.shape.computeMass(f.density);
      totalMass += massData.mass;
      totalInertia += massData.rotationalInertia;
    }

    if (totalMass > 0.0) {
      mass = totalMass;
      invMass = 1.0 / totalMass;
    }

    if (totalInertia > 0.0 && !_fixedRotation) {
      inertia = totalInertia;
      invInertia = 1.0 / totalInertia;
    }
  }

  void applyForce(PhysVec2 appliedForce, PhysVec2 worldPoint) {
    if (!isDynamic) return;
    force += appliedForce;
    torque += (worldPoint - position).cross(appliedForce);
  }

  void applyForceToCenter(PhysVec2 appliedForce) {
    if (!isDynamic) return;
    force += appliedForce;
  }

  void applyTorque(double appliedTorque) {
    if (!isDynamic) return;
    torque += appliedTorque;
  }

  void applyLinearImpulse(PhysVec2 impulse, PhysVec2 worldPoint) {
    if (!isDynamic) return;
    linearVelocity += impulse * invMass;
    angularVelocity += invInertia * (worldPoint - position).cross(impulse);
  }

  void applyLinearImpulseToCenter(PhysVec2 impulse) {
    if (!isDynamic) return;
    linearVelocity += impulse * invMass;
  }

  void applyAngularImpulse(double impulse) {
    if (!isDynamic) return;
    angularVelocity += invInertia * impulse;
  }

  PhysVec2 getVelocityAtWorldPoint(PhysVec2 worldPoint) {
    final r = worldPoint - position;
    return linearVelocity + PhysVec2.scalarCross(angularVelocity, r);
  }

  PhysVec2 getWorldPoint(PhysVec2 localPoint) => transform.transformPoint(localPoint);
  PhysVec2 getLocalPoint(PhysVec2 worldPoint) => transform.inverseTransformPoint(worldPoint);
  PhysVec2 getWorldVector(PhysVec2 localVector) => transform.transformVector(localVector);
  PhysVec2 getLocalVector(PhysVec2 worldVector) => transform.inverseTransformVector(worldVector);

  void synchronizeFixtures() {
    for (final f in fixtures) {
      f.synchronizeAabb();
    }
  }
}
