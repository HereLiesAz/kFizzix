# kfizzix Architecture

This document provides a high-level overview of the kfizzix architecture, explaining how the core components interact to simulate 2D physics.

## Core Components

The engine is built around a few central classes that manage the simulation state and logic.

### 1. World (`com.hereliesaz.kfizzix.dynamics.World`)
The `World` is the container for all physics entities. It is responsible for:
*   **Memory Management:** Managing the lifecycle of bodies, joints, and particles.
*   **Simulation Loop:** The `step()` method advances the simulation by integrating equations of motion, detecting collisions, and resolving constraints.
*   **Queries:** Providing spatial query methods like `raycast` and `queryAABB`.
*   **Gravity:** Applying global gravity to dynamic bodies.

### 2. Body (`com.hereliesaz.kfizzix.dynamics.Body`)
A `Body` represents a rigid object in the simulation. It has properties like position, velocity, and mass.
*   **Types:**
    *   **Static:** Zero mass, zero velocity, moved only manually. Examples: Ground, walls.
    *   **Kinematic:** Zero mass, non-zero velocity set by user, moved by solver. Examples: Moving platforms.
    *   **Dynamic:** Positive mass, non-zero velocity determined by forces. Examples: Crates, balls.
*   **Fixtures:** A body can have multiple fixtures attached to it, defining its shape.

### 3. Fixture (`com.hereliesaz.kfizzix.dynamics.Fixture`)
A `Fixture` binds a `Shape` to a `Body` and adds material properties:
*   **Density:** Used to compute the mass of the parent body.
*   **Friction:** Determines how slippery the object is.
*   **Restitution:** Determines how bouncy the object is.
*   **Filters:** Collision filtering (categories and masks) to determine what hits what.

### 4. Shape (`com.hereliesaz.kfizzix.collision.shapes.Shape`)
The geometry of a fixture.
*   `CircleShape`: A simple circle.
*   `PolygonShape`: A convex polygon (used for boxes).
*   `EdgeShape`: A single line segment.
*   `ChainShape`: A chain of line segments.

### 5. Joints (`com.hereliesaz.kfizzix.dynamics.joints.Joint`)
Joints constrain the motion of two bodies relative to each other.
*   `RevoluteJoint`: Allows rotation around a common point (hinge).
*   `PrismaticJoint`: Allows translation along an axis (slider).
*   `DistanceJoint`: Maintains a fixed distance.
*   `MouseJoint`: Pulls a body towards a target point.

## Simulation Pipeline

When `world.step()` is called, the following happens:

1.  **New Fixtures:** New fixtures are added to the BroadPhase.
2.  **BroadPhase:** Potential collision pairs are identified using a dynamic tree (AABB tree).
3.  **NarrowPhase:** Exact collisions are computed for the pairs identified by the BroadPhase. `Contact` objects are created.
4.  **Solve Velocity:** The `ContactSolver` calculates impulses to resolve overlaps and satisfy joint constraints (modifying velocities).
5.  **Integrate Positions:** Bodies are moved based on their new velocities.
6.  **Solve Position:** The solver adjusts positions to correct any remaining overlap (constraint drift).
7.  **Clear Forces:** Applied forces are cleared for the next step.

## Collision Processing

*   **BroadPhase:** Uses a `DynamicTree` for efficient AABB queries.
*   **NarrowPhase:** Uses algorithms like SAT (Separating Axis Theorem) and GJK (Gilbert-Johnson-Keerthi) via `Collision.collidePolygons` and `Collision.collideCircles`.
*   **Manifolds:** A `Manifold` describes the contact points and normal vector for a collision.

## Object Pooling
To avoid garbage collection overhead, kfizzix uses object pooling heavily.
*   **WorldPool:** A central pool interface for retrieving temporary vectors, matrices, and internal solver objects.
*   **Stacks:** Vectors (`Vec2`) are often allocated from thread-local or world-local stacks (`IWorldPool.popVec2()`) and must be pushed back (`IWorldPool.pushVec2()`).

## Particle System
The particle system is a separate solver integrated into the world. It simulates fluids and soft bodies using a different algorithm (SPH-like) but interacts with rigid bodies.
