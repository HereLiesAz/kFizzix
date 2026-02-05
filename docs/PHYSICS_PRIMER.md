# Physics Primer

This document explains the fundamental physics concepts used in kfizzix. It is intended for developers who want to understand the "why" behind the math.

## 1. Simulation Loop (Discrete Time Integration)

Physics engines simulate the continuous flow of time by breaking it into small, discrete chunks called "time steps" (`dt`).

The basic equation of motion is Newton's Second Law: $F = ma$ (Force = mass * acceleration).

kfizzix uses **Symplectic Euler Integration** (or Semi-Implicit Euler):
1.  **Integrate Velocity:** $v_{new} = v_{old} + (F/m) * dt$
2.  **Integrate Position:** $p_{new} = p_{old} + v_{new} * dt$

This method is stable and energy-conserving enough for games.

## 2. Collision Detection

Collision detection answers two questions:
1.  **Are they touching?** (Boolean result)
2.  **How are they touching?** (Manifold generation)

### AABB (Axis-Aligned Bounding Box)
A simple box that encloses a shape, aligned with the X and Y axes. It is fast to check if two AABBs overlap. If they don't, the shapes inside definitely don't overlap.

### Manifold
When two shapes collide, the engine generates a "Manifold" containing:
*   **Normal:** The direction to push the bodies apart.
*   **Points:** The specific points on the shapes where contact occurs.
*   **Separation:** How deep the overlap is (negative value).

## 3. Constraint Solving (The "Solver")

kfizzix is an **Impulse-Based Constraint Solver**.

### Constraints
A constraint is a rule that limits motion.
*   **Contact Constraint:** "These two objects must not overlap."
*   **Joint Constraint:** "These two points must stay together."

### Impulses
Instead of applying forces (which act over time), the solver applies **impulses** (instantaneous changes in momentum).
$Impulse (J) = Force * Time$
$Velocity Change (\Delta v) = J / mass$

### Sequential Impulse
The solver iterates over all constraints multiple times per step. For each constraint, it calculates the impulse needed to satisfy it *at that moment*. By doing this repeatedly, the system converges to a global solution where all constraints are mostly satisfied.

### Warm Starting
Physics simulations have "temporal coherence"—things don't move much between frames. The solver uses the solution from the *previous* frame as a starting point for the *current* frame. This makes the simulation much more stable (stacks of boxes don't jitter).

## 4. Continuous Physics (TOI)
Fast-moving objects can "tunnel" through thin walls because they jump completely past the wall in a single time step.
**TOI (Time of Impact)** solves this by calculating the exact time ($0 \le t \le 1$) when a collision will occur and advancing the simulation only to that point for that specific object.
