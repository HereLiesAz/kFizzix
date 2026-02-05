# Contributing to kfizzix

We welcome contributions! Because this is a high-performance physics library, we have strict guidelines to maintain quality and speed.

## Code Style

*   **Kotlin:** We use idiomatic Kotlin but avoid features that generate excessive garbage (e.g., extensive lambdas in hot loops, `Sequence` overhead in critical paths).
*   **Documentation:** Every line of code must be documented. Yes, every line. We want to explain *why* we are doing something, not just *what*.
*   **Formatting:** Use standard Kotlin coding conventions.

## Performance Rules

1.  **No Allocations in Loop:** Do not use `new` or create objects inside the physics step (`World.step`). Use the `WorldPool` to get temporary vectors and return them.
2.  **Primitives:** Prefer `Float` and `Int` over boxed types.
3.  **Inlining:** Use `final` classes and methods where possible to encourage inlining (though Kotlin handles much of this).

## Documentation Standard

When writing code, assume the reader is a junior developer who knows Kotlin but nothing about physics engines.

**Bad:**
```kotlin
// Add velocity
pos.addLocal(vel)
```

**Good:**
```kotlin
// Integrate the position by adding the velocity scaled by the timestep.
// This is the Symplectic Euler integration step.
position.x += velocity.x * dt
position.y += velocity.y * dt
```

## Running Tests

Run the full suite:
```bash
./gradlew test
```

We use JUnit 5.
