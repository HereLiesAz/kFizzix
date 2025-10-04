# kfizzix-library To-Do List

This document outlines the tasks required to analyze, refactor, and test the kfizzix-library.

## 1. Analysis

- [x] Analyze `common` package (`Vec2.kt`, `Mat22.kt`, `Rot.kt`, `Transform.kt`)
- [x] Analyze `collision` package and its subpackages (`shapes`, `broadphase`)
- [ ] Analyze `dynamics` package and its subpackages (`contacts`, `joints`)
- [ ] Analyze `particle` package
- [ ] Analyze `pooling` package
- [ ] Analyze `serialization` package
- [ ] Analyze existing test infrastructure in `src/test`

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