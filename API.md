# kfizzix API Reference

This document provides a comprehensive reference for the kfizzix library.

## Dynamics

### World
The `World` class manages all physics entities, dynamic simulation, and asynchronous queries.

**Constructor:**
*   `World(gravity: Vec2)`: Creates a world with the given gravity vector.

**Properties:**
*   `gravity`: The global gravity vector.
*   `allowSleep`: Boolean. Enable/disable sleeping of inactive bodies.
*   `warmStarting`: Boolean. Enable/disable warm starting of contacts.
*   `continuousPhysics`: Boolean. Enable/disable continuous physics (TOI).
*   `subStepping`: Boolean. Enable/disable sub-stepping for particles.
*   `particleSystem`: The default `ParticleSystem`.
*   `bodyCount`: Number of bodies in the world.
*   `jointCount`: Number of joints in the world.
*   `contactCount`: Number of contacts in the world.
*   `contactManager`: Manages contact creation and persistence.
*   `contactListener`: Callback for contact events (`ContactListener`).
*   `contactFilter`: Callback for contact filtering (`ContactFilter`).
*   `destructionListener`: Callback for body/joint destruction (`DestructionListener`).
*   `particleDestructionListener`: Callback for particle destruction (`ParticleDestructionListener`).
*   `debugDraw`: The `DebugDraw` instance for rendering debug data.

**Methods:**
*   `step(dt: Float, velocityIterations: Int, positionIterations: Int)`: Advances the simulation by `dt` seconds using the specified iteration counts.
*   `createBody(def: BodyDef): Body`: Creates a rigid body from a definition.
*   `destroyBody(body: Body)`: Destroys a rigid body.
*   `createJoint(def: JointDef): Joint?`: Creates a joint from a definition.
*   `destroyJoint(joint: Joint)`: Destroys a joint.
*   `createParticleSystem(def: ParticleSystemDef): ParticleSystem`: Creates a new particle system (currently mostly managed internally).
*   `queryAABB(callback: QueryCallback, aabb: AABB)`: Queries the world for fixtures within a bounding box.
*   `raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2)`: Casts a ray through the world.
*   `drawDebugData()`: Renders the world using the attached `DebugDraw`.
*   `setGravity(gravity: Vec2)`: Updates the gravity vector.
*   `getGravity()`: Returns the current gravity vector.

### Body
A rigid body.

**Properties:**
*   `position`: The world position of the body (`Vec2`).
*   `angle`: The world rotation angle in radians.
*   `linearVelocity`: The linear velocity vector (`Vec2`).
*   `angularVelocity`: The angular velocity in radians/second.
*   `linearDamping`: Linear damping coefficient.
*   `angularDamping`: Angular damping coefficient.
*   `gravityScale`: Scale factor for gravity applied to this body.
*   `type`: The body type (`BodyType.STATIC`, `BodyType.KINEMATIC`, `BodyType.DYNAMIC`).
*   `isBullet`: Boolean. Treat as a bullet for continuous collision detection.
*   `isSleepingAllowed`: Boolean.
*   `isAwake`: Boolean.
*   `isActive`: Boolean.
*   `isFixedRotation`: Boolean. Prevent rotation.
*   `fixtureList`: Linked list of fixtures attached to this body.
*   `jointList`: Linked list of joints attached to this body.
*   `contactList`: Linked list of contacts involving this body.
*   `world`: The parent `World`.
*   `userData`: Custom user data object.

**Methods:**
*   `createFixture(def: FixtureDef): Fixture?`: Creates a fixture and attaches it to this body.
*   `createFixture(shape: Shape, density: Float): Fixture?`: Shortcut to create a fixture.
*   `destroyFixture(fixture: Fixture)`: Destroys a fixture.
*   `setTransform(position: Vec2, angle: Float)`: Sets the position and rotation.
*   `applyForce(force: Vec2, point: Vec2)`: Applies a force at a world point.
*   `applyForceToCenter(force: Vec2)`: Applies a force to the center of mass.
*   `applyTorque(torque: Float)`: Applies a torque.
*   `applyLinearImpulse(impulse: Vec2, point: Vec2)`: Applies an impulse at a point (modifies velocity immediately).
*   `applyAngularImpulse(impulse: Float)`: Applies an angular impulse.
*   `getWorldPoint(localPoint: Vec2): Vec2`: Converts a local point to world coordinates.
*   `getWorldVector(localVector: Vec2): Vec2`: Converts a local vector to world coordinates.
*   `getLocalPoint(worldPoint: Vec2): Vec2`: Converts a world point to local coordinates.
*   `getLocalVector(worldVector: Vec2): Vec2`: Converts a world vector to local coordinates.
*   `getLinearVelocityFromWorldPoint(worldPoint: Vec2): Vec2`: Gets the velocity of a specific point on the body.
*   `resetMassData()`: Recalculates mass properties from fixtures.

### Fixture
A fixture binds a shape to a body and adds material properties.

**Properties:**
*   `shape`: The geometric shape (`Shape`).
*   `density`: The density (used to compute mass).
*   `friction`: The friction coefficient.
*   `restitution`: The restitution (bounciness).
*   `isSensor`: Whether this fixture is a sensor (detects overlap but doesn't collide).
*   `filter`: The contact filter data (`Filter`).
*   `body`: The parent `Body`.
*   `userData`: Custom user data object.

**Methods:**
*   `testPoint(p: Vec2): Boolean`: Checks if a point is inside the fixture.
*   `raycast(output: RayCastOutput, input: RayCastInput, childIndex: Int): Boolean`: Casts a ray against the fixture.
*   `getMassData(massData: MassData)`: Gets the mass data.
*   `getAABB(childIndex: Int): AABB`: Gets the axis-aligned bounding box.

### BodyDef
Definition for constructing a `Body`.

**Properties:**
*   `type`: `BodyType` (default `STATIC`).
*   `position`: Initial position (`Vec2`).
*   `angle`: Initial angle (radians).
*   `linearVelocity`: Initial linear velocity (`Vec2`).
*   `angularVelocity`: Initial angular velocity.
*   `linearDamping`: Linear damping.
*   `angularDamping`: Angular damping.
*   `allowSleep`: Boolean.
*   `awake`: Boolean (default `true`).
*   `fixedRotation`: Boolean.
*   `bullet`: Boolean.
*   `active`: Boolean (default `true`).
*   `gravityScale`: Float (default `1.0`).
*   `userData`: Object.

### FixtureDef
Definition for constructing a `Fixture`.

**Properties:**
*   `shape`: The `Shape`.
*   `userData`: Object.
*   `friction`: Float (default `0.2`).
*   `restitution`: Float (default `0.0`).
*   `density`: Float (default `0.0`).
*   `isSensor`: Boolean (default `false`).
*   `filter`: `Filter` (collision groups/masks).

## Collision

### Shapes
All shapes inherit from `Shape`.

*   **`CircleShape`**:
    *   `radius`: Float.
    *   `p`: Position offset (`Vec2`).
*   **`PolygonShape`**:
    *   `setAsBox(hx: Float, hy: Float)`: Create a box centered at local origin.
    *   `setAsBox(hx: Float, hy: Float, center: Vec2, angle: Float)`: Create an oriented box.
    *   `set(vertices: Array<Vec2>, count: Int)`: Create a convex polygon from vertices.
    *   `vertices`: The array of vertices.
    *   `count`: The number of vertices.
*   **`EdgeShape`**:
    *   `set(v1: Vec2, v2: Vec2)`: Set the endpoints.
    *   `vertex1`: Start point (`Vec2`).
    *   `vertex2`: End point (`Vec2`).
    *   `hasVertex0`, `vertex0`: Previous vertex (for ghost collisions).
    *   `hasVertex3`, `vertex3`: Next vertex (for ghost collisions).
*   **`ChainShape`**:
    *   `createLoop(vertices: Array<Vec2>, count: Int)`: Create a closed loop.
    *   `createChain(vertices: Array<Vec2>, count: Int)`: Create an open chain.
    *   `vertices`: The array of vertices.
    *   `count`: The vertex count.

### Joint Definitions
All inherit from `JointDef`.

*   **`RevoluteJointDef`**: Hinge joint.
    *   `initialize(bodyA, bodyB, anchor)`: Helper to set up.
    *   `localAnchorA`, `localAnchorB`: Local connection points.
    *   `referenceAngle`: Initial angle difference.
    *   `enableLimit`, `lowerAngle`, `upperAngle`: Rotation limits.
    *   `enableMotor`, `motorSpeed`, `maxMotorTorque`: Motor properties.
*   **`PrismaticJointDef`**: Sliding joint.
    *   `initialize(bodyA, bodyB, anchor, axis)`: Helper to set up.
    *   `localAnchorA`, `localAnchorB`: Local connection points.
    *   `localAxisA`: The axis of translation.
    *   `enableLimit`, `lowerTranslation`, `upperTranslation`: Translation limits.
    *   `enableMotor`, `motorSpeed`, `maxMotorForce`: Motor properties.
*   **`DistanceJointDef`**: Fixed distance constraint.
    *   `initialize(bodyA, bodyB, anchorA, anchorB)`: Helper to set up.
    *   `length`: The distance to maintain.
    *   `frequencyHz`, `dampingRatio`: Spring properties.
*   **`MouseJointDef`**: For dragging bodies.
    *   `target`: The world target point (`Vec2`).
    *   `maxForce`: Maximum force.
    *   `frequencyHz`, `dampingRatio`: Spring properties.
*   **`WeldJointDef`**: Glues bodies together.
    *   `initialize(bodyA, bodyB, anchor)`: Helper to set up.
    *   `frequencyHz`, `dampingRatio`: Softness.
*   **`RopeJointDef`**: Limits maximum distance.
    *   `localAnchorA`, `localAnchorB`.
    *   `maxLength`: The max distance.
*   **`PulleyJointDef`**: Pulley system.
    *   `initialize(...)`: Complex setup helper.
    *   `ratio`: The pulley ratio.
*   **`GearJointDef`**: Connects two other joints (Revolute/Prismatic).
    *   `joint1`, `joint2`.
    *   `ratio`.
*   **`WheelJointDef`**: Vehicle suspension/wheel.
    *   `initialize(bodyA, bodyB, anchor, axis)`.
    *   `enableMotor`, `motorSpeed`, `maxMotorTorque`.
    *   `frequencyHz`, `dampingRatio`.
*   **`FrictionJointDef`**: Simulates top-down friction.
    *   `maxForce`, `maxTorque`.
*   **`MotorJointDef`**: Controls relative motion.
    *   `linearOffset`, `angularOffset`.
    *   `maxForce`, `maxTorque`, `correctionFactor`.

### ContactListener
Interface for collision callbacks.

*   `beginContact(contact: Contact)`: Called when two fixtures begin to touch.
*   `endContact(contact: Contact)`: Called when two fixtures cease to touch.
*   `preSolve(contact: Contact, oldManifold: Manifold)`: Called before the solver updates. Useful for disabling contact.
*   `postSolve(contact: Contact, impulse: ContactImpulse)`: Called after the solver. Contains impulse data.

## Particles
LiquidFun-style particle simulation.

### ParticleSystem
**Methods:**
*   `createParticle(def: ParticleDef): Int`: Creates a single particle.
*   `destroyParticle(index: Int)`: Destroys a particle.
*   `createParticleGroup(def: ParticleGroupDef): ParticleGroup`: Creates a group of particles.
*   `joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup)`: Merges two groups.
*   `destroyParticleGroup(group: ParticleGroup)`: Destroys a group.
*   `solve(step: TimeStep)`: Solves the particle system (internal, called by World).
*   `queryAABB(callback: ParticleQueryCallback, aabb: AABB)`: Finds particles in a box.
*   `raycast(callback: ParticleRaycastCallback, p1: Vec2, p2: Vec2)`: Casts a ray against particles.

**Properties:**
*   `count`: The number of particles.
*   `particleRadius`: The radius of particles (half of diameter).
*   `particleDensity`: The density of particles.
*   `gravityScale`: Gravity scale for particles.
*   `dampingStrength`, `elasticStrength`, `viscousStrength`, `powderStrength`, `ejectionStrength`, `staticPressureStrength`: Physics parameters for different particle types.
*   `particlePositionBuffer`: Array of `Vec2` positions.
*   `particleVelocityBuffer`: Array of `Vec2` velocities.
*   `particleColorBuffer`: Array of `ParticleColor`.

### ParticleDef
Definition for a single particle.
*   `flags`: `ParticleType` flags.
*   `position`: `Vec2`.
*   `velocity`: `Vec2`.
*   `color`: `ParticleColor`.
*   `userData`: Object.

### ParticleGroupDef
Definition for a group of particles.
*   `shape`: The `Shape` defining the group's volume.
*   `flags`: `ParticleType` flags for particles in the group.
*   `groupFlags`: `ParticleGroupType` flags.
*   `position`: Offset position.
*   `angle`: Rotation angle.
*   `linearVelocity`, `angularVelocity`.
*   `color`: `ParticleColor`.
*   `strength`: Cohesion strength.

### ParticleType (Flags)
*   `waterParticle`
*   `zombieParticle`
*   `wallParticle`
*   `springParticle`
*   `elasticParticle`
*   `viscousParticle`
*   `powderParticle`
*   `tensileParticle`
*   `colorMixingParticle`
*   `destructionListener`
*   `barrierParticle`
*   `staticPressureParticle`
*   `reactiveParticle`
*   `repulsiveParticle`
*   `fixtureContactListenerParticle`
*   `particleContactListenerParticle`

## Common
*   `Vec2(x, y)`: A 2D vector. Methods: `set`, `add`, `sub`, `mul`, `length`, `normalize`, `skew`, `dot`, `cross`.
*   `Vec3(x, y, z)`: A 3D vector.
*   `Mat22`: A 2x2 matrix.
*   `Transform`: Position (`p`) and rotation (`q`).
*   `Rot`: Rotation (sin/cos).
*   `MathUtils`: Helper functions (`sin`, `cos`, `atan2`, `clamp`, `random`).
*   `Settings`: Global configuration (e.g., `maxPolygonVertices`, `velocityThreshold`).
