package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.dynamics.BodyType
import com.hereliesaz.kfizzix.showcase.Test

class HelloWorldTest : Test() {

    override fun initialize() {
        super.initialize()

        // Ground
        val groundDef = BodyDef()
        groundDef.position.set(0f, -10f)
        val ground = world.createBody(groundDef)
        val groundShape = PolygonShape()
        groundShape.setAsBox(50f, 10f)
        ground.createFixture(groundShape, 0f)

        // Box
        val boxDef = BodyDef()
        boxDef.type = BodyType.DYNAMIC
        boxDef.position.set(0f, 4f)
        val box = world.createBody(boxDef)
        val boxShape = PolygonShape()
        boxShape.setAsBox(1f, 1f)
        box.createFixture(boxShape, 1f)
    }

    override fun getTestName(): String {
        return "Hello World"
    }
}
