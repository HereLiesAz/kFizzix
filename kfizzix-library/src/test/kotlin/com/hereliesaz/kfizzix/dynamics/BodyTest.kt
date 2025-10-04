package com.hereliesaz.kfizzix.dynamics

import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.common.Vec2
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class BodyTest {

    private lateinit var world: World
    private lateinit var body: Body

    @BeforeEach
    fun setUp() {
        world = World(Vec2(0f, -9.8f))
        val bodyDef = BodyDef().apply {
            type = BodyType.DYNAMIC
            position.set(0f, 0f)
        }
        body = world.createBody(bodyDef)
    }

    @Test
    fun `test body creation`() {
        assertNotNull(body)
        assertEquals(BodyType.DYNAMIC, body.type)
        assertEquals(0f, body.position.x)
        assertEquals(0f, body.position.y)
    }

    @Test
    fun `test create and destroy fixture`() {
        val shape = CircleShape().apply { radius = 1f }
        val fixture = body.createFixture(shape, 1f)
        assertNotNull(fixture)
        assertEquals(1, body.fixtureCount)
        assertNotNull(body.fixtureList)

        body.destroyFixture(fixture)
        assertEquals(0, body.fixtureCount)
    }

    @Test
    fun `test apply force`() {
        val force = Vec2(10f, 0f)
        val point = body.worldCenter
        body.applyForce(force, point)

        assertEquals(10f, body.force.x)
        assertEquals(0f, body.force.y)
    }

    @Test
    fun `test apply torque`() {
        body.applyTorque(10f)
        assertEquals(10f, body.torque)
    }

    @Test
    fun `test apply linear impulse`() {
        val impulse = Vec2(10f, 0f)
        val point = body.worldCenter
        body.applyLinearImpulse(impulse, point, true)

        assertEquals(10f, body.linearVelocity.x, 1e-6f)
        assertEquals(0f, body.linearVelocity.y, 1e-6f)
    }
}