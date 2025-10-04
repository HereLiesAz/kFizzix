package com.hereliesaz.kfizzix.dynamics

import com.hereliesaz.kfizzix.common.Vec2
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Test

class WorldTest {

    @Test
    fun `test world creation`() {
        val gravity = Vec2(0f, -9.8f)
        val world = World(gravity)
        assertNotNull(world)
        assertEquals(gravity, world.gravity)
    }
}