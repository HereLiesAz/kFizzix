[![Maven Central](https://img.shields.io/maven-central/v/com.hereliesaz.kfizzix/kfizzix-library.svg?style=flat)](https://central.sonatype.com/artifact/com.hereliesaz.kfizzix/kfizzix-library)
[![javadoc](https://javadoc.io/badge2/com.hereliesaz.kfizzix/kfizzix-library/javadoc.svg)](https://javadoc.io/doc/com.hereliesaz.kfizzix/kfizzix-library)

# kfizzix

kfizzix is a pure Kotlin port of the C++ physics engines [LiquidFun](http://google.github.io/liquidfun/) and [Box2d](http://box2d.org).

This project is a fork of the original [jbox2d](https://github.com/jbox2d/jbox2d) library, which was a Java port. kfizzix has been fully converted to idiomatic Kotlin, with a focus on providing a modern, easy-to-use, and well-documented physics library for the Kotlin ecosystem.

## Features

-   **Pure Kotlin:** The library is written entirely in Kotlin, providing a modern and expressive API.
-   **Comprehensive KDoc:** The public API is fully documented with KDoc, making it easy to understand and use.
-   **Gradle-based:** The project is built with Gradle, making it easy to integrate into your projects.
-   **Modular:** The project is divided into several modules, allowing you to include only the parts you need.

## Modules

-   `kfizzix-library`: The core physics library.
-   `kfizzix-serialization-kt`: Serialization tools for saving and loading worlds.
-   `kfizzix-testbed-javafx`: A testbed for creating and running physics tests using JavaFX.
-   `jbox2d-testbed-jogl`: The original testbed with OpenGL rendering.

## Usage

To use kfizzix in your project, add the following dependency to your `build.gradle.kts` file:

```kotlin
dependencies {
    implementation("com.hereliesaz.kfizzix:kfizzix-library:1.0.0-SNAPSHOT")
}
```

### Public API Guide

Here are a few examples of how to use the core classes in kfizzix:

**Creating a 2D Vector:**

```kotlin
import com.hereliesaz.kfizzix.common.Vec2

// Create a zero vector
val v1 = Vec2()

// Create a vector with specific coordinates
val v2 = Vec2(1.0f, 2.0f)

// Add two vectors
val v3 = v1 + v2
```

**Creating a 2x2 Matrix:**

```kotlin
import com.hereliesaz.kfizzix.common.Mat22
import com.hereliesaz.kfizzix.common.Vec2

// Create a matrix from two column vectors
val col1 = Vec2(1.0f, 2.0f)
val col2 = Vec2(3.0f, 4.0f)
val m1 = Mat22(col1, col2)

// Create a matrix from four floats
val m2 = Mat22(1.0f, 3.0f, 2.0f, 4.0f)
```

**Creating a Transform:**

```kotlin
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.common.Rot

// Create a transform with a position and rotation
val position = Vec2(1.0f, 1.0f)
val rotation = Rot(0.5f) // 0.5 radians
val xf = Transform(position, rotation)
```

For more detailed information, please refer to the KDoc documentation for each class.

## Tests / Demos

The original demos from jbox2d are available in the `jbox2d-testbed-jogl` and `kfizzix-testbed-javafx` modules.

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/BulletTest.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Car.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/DamBreak.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Dominos.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/LiquidTimer.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Particles.gif)
![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/WaveMachine.gif)
