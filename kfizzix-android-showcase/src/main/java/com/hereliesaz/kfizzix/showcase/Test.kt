package com.hereliesaz.kfizzix.showcase

import android.view.MotionEvent
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.World

abstract class Test {
    lateinit var world: World
    val debugDraw = AndroidDebugDraw()

    // Default gravity
    open val gravity = Vec2(0f, -10f)

    open fun initialize() {
        world = World(gravity)
        world.debugDraw = debugDraw
        debugDraw.setFlags(com.hereliesaz.kfizzix.callbacks.DebugDraw.shapeBit or com.hereliesaz.kfizzix.callbacks.DebugDraw.jointBit or com.hereliesaz.kfizzix.callbacks.DebugDraw.pairBit)
        // Set up camera defaults (center at 0, 10, scale 10 pixels per meter)
        debugDraw.setCamera(0f, 10f, 20f)
    }

    open fun step() {
        world.step(1f / 60f, 8, 3)
    }

    open fun render() {
        world.drawDebugData()
    }

    open fun onTouch(event: MotionEvent) {
        // Optional override
    }

    abstract fun getTestName(): String
}
