package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.collision.shapes.ChainShape
import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.collision.shapes.EdgeShape
import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.dynamics.BodyType
import com.hereliesaz.kfizzix.showcase.Test

class ShapesTest : Test() {

    override fun initialize() {
        super.initialize()

        // Ground
        val groundDef = BodyDef()
        val ground = world.createBody(groundDef)
        val groundShape = EdgeShape()
        groundShape.set(Vec2(-40f, 0f), Vec2(40f, 0f))
        ground.createFixture(groundShape, 0f)

        // Circle
        val circleDef = BodyDef()
        circleDef.type = BodyType.DYNAMIC
        circleDef.position.set(-10f, 10f)
        val circleBody = world.createBody(circleDef)
        val circleShape = CircleShape()
        circleShape.radius = 2f
        circleBody.createFixture(circleShape, 1f)

        // Polygon (Triangle)
        val polyDef = BodyDef()
        polyDef.type = BodyType.DYNAMIC
        polyDef.position.set(0f, 10f)
        val polyBody = world.createBody(polyDef)
        val polyShape = PolygonShape()
        val vertices = arrayOf(Vec2(0f, 2f), Vec2(-2f, -2f), Vec2(2f, -2f))
        polyShape.set(vertices, 3)
        polyBody.createFixture(polyShape, 1f)

        // Chain Loop
        val chainDef = BodyDef()
        chainDef.type = BodyType.DYNAMIC
        chainDef.position.set(10f, 10f)
        val chainBody = world.createBody(chainDef)
        val chainShape = ChainShape()
        val chainVerts = arrayOf(
            Vec2(0f, 0f),
            Vec2(2f, 0f),
            Vec2(2f, 2f),
            Vec2(0f, 2f)
        )
        chainShape.createLoop(chainVerts, 4)
        chainBody.createFixture(chainShape, 1f)
    }

    override fun getTestName(): String {
        return "Shapes"
    }
}
