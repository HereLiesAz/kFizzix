# Remaining Tasks for kfizzix-library Merge

The following issues need to be resolved to complete the compilation of the `kfizzix-library` module. These errors stem from merging different versions/ports of Box2D/LiquidFun.

1.  **`DynamicTreeFlatNodes.kt` Overload Ambiguity**:
    *   `MathUtils.max` calls are ambiguous (Float vs Int).
    *   `height` property conflict (var vs val).

2.  **`Contact.kt` Type Mismatch**:
    *   `worldManifold.initialize(...)` call fails with `Type mismatch: inferred type is Shape? but Shape was expected`.
    *   Likely due to `fixture.shape` being nullable platform type or import mismatch.

3.  **`World.kt` & `BroadPhase` Mismatch**:
    *   `World.kt` calls `contactManager.broadPhase.query(...)` and `raycast(...)`.
    *   `BroadPhase` interface seems to match, but compiler reports `Unresolved reference`. Check for signature mismatch (e.g. nullable callback).

4.  **`Island.kt` Array Covariance**:
    *   `solverDef.contacts = contacts` fails. `Island` uses `Array<Contact?>?` (nullable elements), while `ContactSolverDef` expects `Array<Contact>?` (non-null elements).
    *   Requires careful casting or array copy.

5.  **`PolygonShape.kt` / `Transform`**:
    *   `Unresolved reference: mul` / `mulT`.
    *   Ensure `Transform` instance methods `mul(Vec2)` are visible.

6.  **`ParticleSystem.kt` Ambiguity**:
    *   `solveViscous` method has unresolved references `va`, `vb` likely due to previous compilation failures aborting analysis.
    *   Double-check variable definitions in that method.
