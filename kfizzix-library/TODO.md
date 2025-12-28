# kfizzix-library To-Do List

This document outlines the tasks required to analyze, refactor, and test the kfizzix-library.

## 1. Analysis

- [x] Analyze `common` package (`Vec2.kt`, `Mat22.kt`, `Rot.kt`, `Transform.kt`)
- [x] Analyze `collision` package and its subpackages (`shapes`, `broadphase`)
- [x] Analyze `dynamics` package and its subpackages (`contacts`, `joints`)
- [x] Analyze `particle` package
- [x] Analyze `pooling` package
- [x] Analyze `serialization` package
- [x] Analyze existing test infrastructure in `src/test`

## 2. Refactoring and API Improvements

### General
- [ ] **Idiomatic Kotlin:** Refactor code to be more idiomatic.
    - [ ] Replace `clone()` with `copy()` or copy constructors.
    - [ ] Evaluate `...Local` methods and consider alternatives like `apply` or operator assignments (`plusAssign`).
    - [ ] Review `companion object` methods and consider converting them to extension functions where appropriate.
- [ ] **API Cleanup:**
    - [ ] Review redundant methods (e.g., `abs()` on instance vs. `abs()` in companion object) and consolidate where possible.
    - [ ] Add `@RequiresOptIn` annotation or clear warnings for `...Unsafe` methods.
    - [ ] Address thread-safety issues with static pools (e.g., `Transform.pool`).

### Specific Classes
- [ ] **Vec2.kt:**
    - [ ] It's a `data class`, but has a manual copy constructor. Rely on the generated `copy()` method.
- [ ] **Mat22.kt:**
    - [ ] Consider converting to a `data class` to get `equals`, `hashCode`, `toString`, and `copy()` for free.
    - [ ] Improve the `toString()` implementation.
- [ ] **Transform.kt:**
    - [ ] Address the non-thread-safe static `pool` field. Consider making it thread-local or using object pooling in a more controlled way.
- [ ] **World.kt:**
    - [ ] Convert public fields (e.g., `gravity`, `allowSleep`) to Kotlin properties.
    - [ ] Refactor the main `step()` method to improve readability, potentially breaking out parts of the simulation loop into smaller, private functions.
    - [ ] Replace manual `ContactRegister` initialization with a more robust, type-safe mechanism.
- [ ] **Body.kt:**
    - [ ] Convert public fields to Kotlin properties (e.g., `linearVelocity`, `angularVelocity`, `mass`).
    - [ ] Convert accessor methods like `isAwake()` and `isActive()` to properties.
    - [ ] Simplify the complex `resetMassData()` method.
- [ ] **Fixture.kt:**
    - [ ] Convert public fields to Kotlin properties (`density`, `friction`, `restitution`).
    - [ ] Convert `isSensor()` to a property.
- [ ] **ContactSolver.kt:**
    - [ ] Carefully refactor this performance-critical class.
    - [ ] Add detailed comments to the block solver logic in `solveVelocityConstraints` to explain the algorithm.
    - [ ] Replace manual array resizing with a more idiomatic Kotlin approach, such as using `MutableList` or a better pre-allocation strategy.
- [ ] **Joint.kt:**
    - [ ] Refactor the `create` factory method to be more idiomatic, perhaps using a map of constructors instead of a `when` expression.
- [ ] **ParticleSystem.kt:**
    - [ ] This is a massive class. Break down the monolithic `solve()` method into smaller, more manageable private functions (e.g., `solvePressure`, `solveDamping`).
    - [ ] Replace the manual, error-prone buffer management (`reallocateBuffer`, etc.) with `MutableList` or a more robust, idiomatic Kotlin approach.
    - [ ] Convert public fields to properties where appropriate.
- [ ] **DefaultWorldPool.kt:**
    - [ ] Refactor the custom `OrderedStack` and `MutableStack` implementations to use standard Kotlin collections or a simplified, more readable pooling mechanism.
    - [ ] Improve the type safety of the array caches, which currently use `HashMap<Integer, ...>`.

## 3. Testing

- [ ] **Setup:**
    - [ ] Configure JUnit 5 as the testing framework.
    - [ ] Create a base test class with helper methods if needed.
- [ ] **Test Suites:**
    - [ ] **common:**
        - [ ] `Vec2`: Test all arithmetic, normalization, dot/cross products, and edge cases (zero vector).
        - [ ] `Mat22`: Test all matrix operations (creation, inversion, multiplication) and `solve()`.
        - [ ] `Rot`: Test all rotation operations.
        - [ ] `Transform`: Test vector and transform multiplication.
        - [ ] Test all `...ToOut` and `...Unsafe` variants to ensure correctness.
    - [ ] **collision:**
        - [ ] `AABB`: Test overlap and containment logic.
        - [ ] `shapes`: Test all shape types (Circle, Polygon, Edge, Chain) for mass properties, ray casting, and point testing.
        - [ ] `Distance`: Test distance calculations between all shape combinations.
        - [ ] `TimeOfImpact`: Test TOI calculations.
        - [ ] `DynamicTree`: Test proxy creation, destruction, moving, and querying.
    - [ ] **dynamics:**
        - [ ] `World`: Test body creation/destruction, stepping, and querying.
        - [ ] `Body`: Test setting/getting state, applying forces/impulses.
        - [ ] `Fixture`: Test fixture creation and properties.
        - [ ] `joints`: Test all joint types for constraints and reactions.
        - [ ] `contacts`: Test contact creation, updates, and solving.
    - [ ] **particle:**
        - [ ] `ParticleSystem`: Test particle creation/destruction, stepping, and interaction with bodies.
    - [ ] **serialization:**
        - [ ] Test serialization and deserialization of a `World` with various bodies, joints, and particles.

## 4. Documentation
- [ ] Update KDoc to explain the purpose of `...Local`, `...ToOut`, and `...Unsafe` method variants.
- [ ] Add warnings about thread-safety and usage of unsafe methods.
- [ ] Document any significant API changes made during refactoring.
