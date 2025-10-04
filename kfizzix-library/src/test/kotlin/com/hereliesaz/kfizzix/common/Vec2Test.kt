package com.hereliesaz.kfizzix.common

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class Vec2Test {

    @Test
    fun `new vector is zero`() {
        val v = Vec2()
        assertEquals(0f, v.x)
        assertEquals(0f, v.y)
    }
}