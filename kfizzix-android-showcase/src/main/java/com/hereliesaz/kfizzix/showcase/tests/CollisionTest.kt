package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.callbacks.QueryCallback
import com.hereliesaz.kfizzix.callbacks.RayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.dynamics.BodyType
import com.hereliesaz.kfizzix.dynamics.Fixture
import com.hereliesaz.kfizzix.showcase.Test

class CollisionTest : Test() {

    override fun initialize() {
        super.initialize()

        // Populate world
        val def = BodyDef()
        def.type = BodyType.DYNAMIC
        val shape = CircleShape()
        shape.radius = 1f

        for (i in 0 until 10) {
            def.position.set((Math.random() * 20 - 10).toFloat(), (Math.random() * 20).toFloat())
            val body = world.createBody(def)
            body.createFixture(shape, 1f)
        }
    }

    override fun step() {
        super.step()

        // Demonstrate Raycast
        val p1 = Vec2(-20f, 10f)
        val p2 = Vec2(20f, 10f)
        debugDraw.drawSegment(p1, p2, com.hereliesaz.kfizzix.common.Color3f(1f, 1f, 1f))

        world.raycast(object : RayCastCallback {
            override fun reportFixture(fixture: Fixture, point: Vec2, normal: Vec2, fraction: Float): Float {
                debugDraw.drawPoint(point, 0.5f, com.hereliesaz.kfizzix.common.Color3f(1f, 0f, 0f))
                debugDraw.drawSegment(point, point + normal, com.hereliesaz.kfizzix.common.Color3f(0f, 1f, 0f))
                return 1f // Continue
            }
        }, p1, p2)

        // Demonstrate AABB Query
        val aabb = AABB()
        aabb.lowerBound.set(-5f, -5f)
        aabb.upperBound.set(5f, 5f)

        // Draw AABB box
        val v = arrayOf(
            Vec2(aabb.lowerBound.x, aabb.lowerBound.y),
            Vec2(aabb.upperBound.x, aabb.lowerBound.y),
            Vec2(aabb.upperBound.x, aabb.upperBound.y),
            Vec2(aabb.lowerBound.x, aabb.upperBound.y)
        )
        debugDraw.drawPolygon(v, 4, com.hereliesaz.kfizzix.common.Color3f(1f, 1f, 0f))

        world.queryAABB(object : QueryCallback {
            override fun reportFixture(fixture: Fixture): Boolean {
                val body = fixture.body
                debugDraw.drawPoint(body!!.position, 0.2f, com.hereliesaz.kfizzix.common.Color3f(0f, 0f, 1f))
                return true
            }
        }, aabb)
    }

    override fun getTestName(): String {
        return "Collision Query"
    }
}
