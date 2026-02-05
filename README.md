# kfizzix

**Hello!** 👋

Imagine you have a big box of blocks, balls, and toy cars. You can stack them up, knock them over, and watch them roll.

**kfizzix** (say "Kay-Fizz-Icks") is a special computer program that lets you do exactly that inside your computer screen!

*   🍎 **Gravity:** It knows how things fall down.
*   💥 **Bumping:** It knows when two things hit each other (Boink!).
*   🧱 **Stacking:** It lets you build tall towers.

It's like magic math that makes your games feel real.

---

## For the Grown-Ups (Developers) 🤓

kfizzix is a rigid body physics engine written in 100% Kotlin. It is a port of JBox2D, optimized for the JVM and Android.

### How to Install

If you are including this in your project, add the library dependency:

```kotlin
implementation("com.hereliesaz.kfizzix:kfizzix-library:1.0.0-SNAPSHOT")
```

### Features

*   **Rigid Bodies:** Dynamic (moving), Static (walls), and Kinematic (moving platforms).
*   **Joints:** Connect bodies together with hinges, springs, and sliders.
*   **Collision Detection:** Efficiently detects when Polygons, Circles, and Chains overlap.
*   **Particle Simulation:** Simulate fluids and soft particles (ported from LiquidFun).

### Simple Example

Here is how you create a world where a box falls onto the ground:

```kotlin
import com.hereliesaz.kfizzix.dynamics.*
import com.hereliesaz.kfizzix.collision.shapes.*
import com.hereliesaz.kfizzix.common.*

// 1. Make a World. Gravity pulls down!
val gravity = Vec2(0.0f, -10.0f)
val world = World(gravity)

// 2. Make the Ground (so things don't fall forever)
val groundDef = BodyDef()
groundDef.position.set(0.0f, -10.0f) // 10 meters down
val groundBody = world.createBody(groundDef)
val groundShape = PolygonShape()
groundShape.setAsBox(50.0f, 10.0f) // A huge floor
groundBody.createFixture(groundShape, 0.0f) // 0 mass (static)

// 3. Make a Box that falls
val boxDef = BodyDef()
boxDef.type = BodyType.DYNAMIC // It moves!
boxDef.position.set(0.0f, 4.0f) // 4 meters up
val boxBody = world.createBody(boxDef)
val boxShape = PolygonShape()
boxShape.setAsBox(1.0f, 1.0f) // A 2x2 meter box
boxBody.createFixture(boxShape, 1.0f) // Density 1.0

// 4. Time step! (Move the world forward a tiny bit)
// In a game loop, you call this 60 times a second.
world.step(1.0f / 60.0f, 6, 2)
```

Have fun making things crash!
