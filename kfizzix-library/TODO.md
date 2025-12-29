# kfizzix-library To-Do List

This document outlines the tasks required to analyze, refactor, and test the kfizzix-library.

## Status Summary
- **common**: Ported and tested (`Vec2`, `Mat22`, `Mat33`, `Rot`, `Transform`, `MathUtils`, `Settings`, `Timer`, `OBBViewportTransform`). Tests (`RotTest`, `Vec2Test`, `Mat22Test`, `TransformTest`) are passing when isolated.
- **pooling**: Ported (`OrderedStack`, `CircleStack`, `DynamicStack`, `MutableStack`, `WorldPool`, `DefaultWorldPool`, `arrays/*`). Note: `DefaultWorldPool` compilation is currently broken due to dependencies on unported `collision` and `dynamics`.
- **dynamics/contacts**: Ported (`Contact`, `ContactEdge`, and concrete contact classes).
- **dynamics**: **UNPORTED** (Java files masquerading as Kotlin). Needs full porting (`World`, `Body`, `Fixture`, `Joints`).
- **collision**: **UNPORTED** (Java files masquerading as Kotlin). Needs full porting (`Collision`, `Distance`, `TimeOfImpact`, `shapes/*`, `broadphase/*`).
- **particle**: **UNPORTED** (Java files masquerading as Kotlin). Needs full porting (`ParticleSystem`, etc.).
- **callbacks**: **UNPORTED**.

**Note:** The project currently fails to compile because of the dependencies on unported packages (`dynamics`, `collision`, etc.). To verify `common` package tests, you must temporarily exclude the broken packages from the build.

## 1. Analysis

- [x] Analyze `common` package (`Vec2.kt`, `Mat22.kt`, `Rot.kt`, `Transform.kt`)
- [x] Analyze `collision` package and its subpackages (`shapes`, `broadphase`)
- [x] Analyze `dynamics` package and its subpackages (`contacts`, `joints`)
- [x] Analyze `particle` package
- [x] Analyze `pooling` package
- [x] Analyze `serialization` package (Removed from library module as it was broken/misplaced).
- [x] Analyze existing test infrastructure in `src/test`

## 2. Refactoring and Porting (Java -> Kotlin)

### Common Package (Completed)
- [x] `Vec2.kt`
- [x] `Mat22.kt`
- [x] `Mat33.kt`
- [x] `Rot.kt`
- [x] `Transform.kt`
- [x] `MathUtils.kt`
- [x] `Settings.kt`
- [x] `Timer.kt`
- [x] `OBBViewportTransform.kt`
- [x] `BufferUtils.kt`
- [x] `RaycastResult.kt`
- [x] `PlatformMathUtils.kt`

### Pooling Package (Completed*)
- [x] `OrderedStack.kt` (Interface & Implementation)
- [x] `CircleStack.kt`
- [x] `DynamicStack.kt`
- [x] `MutableStack.kt`
- [x] `WorldPool.kt` (Interface)
- [x] `DefaultWorldPool.kt` (*Compilation fails due to missing dependencies in collision/dynamics*)
- [x] `arrays/*` (`Vec2Array`, `IntArray`, `FloatArray`, `GeneratorArray`)

### Dynamics Package (In Progress)
- [x] `contacts/*` (Ported `Contact`, `ContactEdge`, and all subclasses)
- [ ] `World.kt`
- [ ] `Body.kt`
- [ ] `BodyDef.kt`
- [ ] `Fixture.kt`
- [ ] `FixtureDef.kt`
- [ ] `joints/*` (All joint classes)
- [ ] `ContactManager.kt`
- [ ] `Island.kt`
- [ ] `TimeStep.kt`

### Collision Package (ToDo)
- [ ] `Collision.kt`
- [ ] `Distance.kt`
- [ ] `TimeOfImpact.kt`
- [ ] `shapes/*`
- [ ] `broadphase/*`
- [ ] `AABB.kt`
- [ ] `RayCastInput.kt` / `RayCastOutput.kt`
- [ ] `Manifold.kt`

### Particle Package (ToDo)
- [ ] `ParticleSystem.kt`
- [ ] `VoronoiDiagram.kt` (Fixed compilation for now)
- [ ] Other particle files.

### Callbacks Package (ToDo)
- [ ] All callback interfaces and classes.

## 3. Testing

- [ ] **Setup:**
    - [x] Configure JUnit 5 as the testing framework.
- [ ] **Test Suites:**
    - [x] **common:**
        - [x] `Vec2Test`
        - [x] `Mat22Test`
        - [x] `RotTest`
        - [x] `TransformTest`
    - [ ] **collision:**
        - [ ] `AABB`: Test overlap and containment logic.
        - [ ] `shapes`: Test all shape types.
        - [ ] `Distance`: Test distance calculations.
        - [ ] `TimeOfImpact`: Test TOI calculations.
        - [ ] `DynamicTree`: Test proxy creation, destruction, moving, and querying.
    - [ ] **dynamics:**
        - [ ] `World`: Test body creation/destruction, stepping, and querying.
        - [ ] `Body`: Test setting/getting state, applying forces/impulses.
        - [ ] `Fixture`: Test fixture creation and properties.
        - [ ] `joints`: Test all joint types.
        - [ ] `contacts`: Test contact creation, updates, and solving.
    - [ ] **particle:**
        - [ ] `ParticleSystem`: Test particle creation/destruction, stepping.

## 4. Documentation
- [ ] Update KDoc to explain the purpose of `...Local`, `...ToOut`, and `...Unsafe` method variants.
- [ ] Add warnings about thread-safety and usage of unsafe methods.
