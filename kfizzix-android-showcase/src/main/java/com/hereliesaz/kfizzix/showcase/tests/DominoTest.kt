package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.dynamics.BodyType
import com.hereliesaz.kfizzix.showcase.Test

class DominoTest : Test() {

    override fun initialize() {
        super.initialize()

        // Ground
        val groundDef = BodyDef()
        groundDef.position.set(0f, -10f)
        val ground = world.createBody(groundDef)
        val groundShape = PolygonShape()
        groundShape.setAsBox(50f, 10f)
        ground.createFixture(groundShape, 0f)

        // Dominos
        val shape = PolygonShape()
        shape.setAsBox(0.125f, 2f)
        val d = 1f // density

        for (i in 0 until 10) {
            val bd = BodyDef()
            bd.type = BodyType.DYNAMIC
            bd.position.set(20f - i * 1.5f, 2f)
            val body = world.createBody(bd)
            body.createFixture(shape, d)
        }
    }

    override fun getTestName(): String {
        return "Dominos"
    }
}
