# kfizzix

## What is this?
kfizzix is a toy box for your computer code! It helps you make things move and bump into each other, just like in real life. If you want to make a ball bounce or a block fall, kfizzix does the math for you.

It is written in a language called Kotlin. It is very fast and works on phones (Android) and computers.

## How to use it
Ask a grown-up (or a developer) to put this in your `build.gradle.kts` file:

```kotlin
dependencies {
    implementation("com.hereliesaz.kfizzix:kfizzix-library:1.0.0-SNAPSHOT")
}
```

## Make Code (Example)
Here is how you make a world with gravity and a box:

```kotlin
// 1. Make a World. Gravity pulls down!
val gravity = Vec2(0.0f, -10.0f)
val world = World(gravity)

// 2. Make the Ground (so things don't fall forever)
val groundDef = BodyDef()
groundDef.position.set(0.0f, -10.0f)
val groundBody = world.createBody(groundDef)
val groundShape = PolygonShape()
groundShape.setAsBox(50.0f, 10.0f)
groundBody.createFixture(groundShape, 0.0f)

// 3. Make a Box that falls
val boxDef = BodyDef()
boxDef.type = BodyType.DYNAMIC // Dynamic means it moves!
boxDef.position.set(0.0f, 4.0f)
val boxBody = world.createBody(boxDef)
val boxShape = PolygonShape()
boxShape.setAsBox(1.0f, 1.0f)
boxBody.createFixture(boxShape, 1.0f) // 1.0f is how heavy it is

// 4. Time step! (Move the world forward a tiny bit)
// Do this in a loop, like in a video game.
world.step(1.0f / 60.0f, 6, 2)
```

Have fun making things crash!
