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

    @Test
    fun `add vectors`() {
        val v1 = Vec2(1f, 2f)
        val v2 = Vec2(3f, 4f)
        val v3 = v1 + v2
        assertEquals(4f, v3.x)
        assertEquals(6f, v3.y)
    }

    @Test
    fun `subtract vectors`() {
        val v1 = Vec2(1f, 2f)
        val v2 = Vec2(3f, 4f)
        val v3 = v1 - v2
        assertEquals(-2f, v3.x)
        assertEquals(-2f, v3.y)
    }

    @Test
    fun `multiply vector by scalar`() {
        val v1 = Vec2(1f, 2f)
        val v2 = v1 * 3f
        assertEquals(3f, v2.x)
        assertEquals(6f, v2.y)
    }

    @Test
    fun `negate vector`() {
        val v1 = Vec2(1f, 2f)
        val v2 = -v1
        assertEquals(-1f, v2.x)
        assertEquals(-2f, v2.y)
    }

    @Test
    fun `test copy`() {
        val v1 = Vec2(5f, 10f)
        val v2 = v1.copy()
        assertEquals(5f, v2.x)
        assertEquals(10f, v2.y)
        assertEquals(v1, v2)
    }

    @Test
    fun `addLocal vectors`() {
        val v1 = Vec2(1f, 2f)
        val v2 = Vec2(3f, 4f)
        v1.addLocal(v2)
        assertEquals(4f, v1.x)
        assertEquals(6f, v1.y)
    }

    @Test
    fun `subLocal vectors`() {
        val v1 = Vec2(1f, 2f)
        val v2 = Vec2(3f, 4f)
        v1.subLocal(v2)
        assertEquals(-2f, v1.x)
        assertEquals(-2f, v1.y)
    }

    @Test
    fun `mulLocal vector by scalar`() {
        val v1 = Vec2(1f, 2f)
        v1.mulLocal(3f)
        assertEquals(3f, v1.x)
        assertEquals(6f, v1.y)
    }

    @Test
    fun `negateLocal vector`() {
        val v1 = Vec2(1f, 2f)
        v1.negateLocal()
        assertEquals(-1f, v1.x)
        assertEquals(-2f, v1.y)
    }

    @Test
    fun `set vector`() {
        val v1 = Vec2()
        v1.set(1f, 2f)
        assertEquals(1f, v1.x)
        assertEquals(2f, v1.y)
    }

    @Test
    fun `setZero vector`() {
        val v1 = Vec2(1f, 2f)
        v1.setZero()
        assertEquals(0f, v1.x)
        assertEquals(0f, v1.y)
    }

    @Test
    fun `vector length`() {
        val v = Vec2(3f, 4f)
        assertEquals(5f, v.length(), 1e-6f)
    }

    @Test
    fun `vector length squared`() {
        val v = Vec2(3f, 4f)
        assertEquals(25f, v.lengthSquared(), 1e-6f)
    }

    @Test
    fun `normalize vector`() {
        val v = Vec2(3f, 4f)
        val length = v.normalize()
        assertEquals(5f, length, 1e-6f)
        assertEquals(0.6f, v.x, 1e-6f)
        assertEquals(0.8f, v.y, 1e-6f)
        assertEquals(1f, v.length(), 1e-6f)
    }

    @Test
    fun `normalize zero vector`() {
        val v = Vec2(0f, 0f)
        val length = v.normalize()
        assertEquals(0f, length, 1e-6f)
        assertEquals(0f, v.x, 1e-6f)
        assertEquals(0f, v.y, 1e-6f)
    }

    @Test
    fun `isValid vector`() {
        val v1 = Vec2(1f, 2f)
        val v2 = Vec2(Float.NaN, 2f)
        val v3 = Vec2(1f, Float.POSITIVE_INFINITY)
        assert(v1.isValid())
        assert(!v2.isValid())
        assert(!v3.isValid())
    }

    @Test
    fun `skew vector`() {
        val v1 = Vec2(2f, 3f)
        val v2 = v1.skew()
        assertEquals(-3f, v2.x)
        assertEquals(2f, v2.y)
    }

    @Test
    fun `skew to out`() {
        val v1 = Vec2(2f, 3f)
        val out = Vec2()
        v1.skew(out)
        assertEquals(-3f, out.x)
        assertEquals(2f, out.y)
    }

    @Test
    fun `abs of vector`() {
        val v1 = Vec2(-1f, -2f)
        val v2 = v1.abs()
        assertEquals(1f, v2.x)
        assertEquals(2f, v2.y)
    }

    @Test
    fun `absLocal of vector`() {
        val v1 = Vec2(-1f, -2f)
        v1.absLocal()
        assertEquals(1f, v1.x)
        assertEquals(2f, v1.y)
    }

    @Test
    fun `static abs of vector`() {
        val v1 = Vec2(-1f, -2f)
        val v2 = Vec2.abs(v1)
        assertEquals(1f, v2.x)
        assertEquals(2f, v2.y)
    }

    @Test
    fun `static absToOut of vector`() {
        val v1 = Vec2(-1f, -2f)
        val out = Vec2()
        Vec2.absToOut(v1, out)
        assertEquals(1f, out.x)
        assertEquals(2f, out.y)
    }

    @Test
    fun `dot product`() {
        val v1 = Vec2(2f, 3f)
        val v2 = Vec2(4f, 5f)
        assertEquals(23f, Vec2.dot(v1, v2), 1e-6f)
    }

    @Test
    fun `cross product of vectors`() {
        val v1 = Vec2(2f, 3f)
        val v2 = Vec2(4f, 5f)
        assertEquals(-2f, Vec2.cross(v1, v2), 1e-6f)
    }

    @Test
    fun `cross product of vector and scalar`() {
        val v1 = Vec2(2f, 3f)
        val v2 = Vec2.cross(v1, 2f)
        assertEquals(6f, v2.x)
        assertEquals(-4f, v2.y)
    }

    @Test
    fun `cross product of scalar and vector`() {
        val v1 = Vec2(2f, 3f)
        val v2 = Vec2.cross(2f, v1)
        assertEquals(-6f, v2.x)
        assertEquals(4f, v2.y)
    }

    @Test
    fun `crossToOut of vector and scalar`() {
        val v1 = Vec2(2f, 3f)
        val out = Vec2()
        Vec2.crossToOut(v1, 2f, out)
        assertEquals(6f, out.x)
        assertEquals(-4f, out.y)
    }

    @Test
    fun `crossToOut of scalar and vector`() {
        val v1 = Vec2(2f, 3f)
        val out = Vec2()
        Vec2.crossToOut(2f, v1, out)
        assertEquals(-6f, out.x)
        assertEquals(4f, out.y)
    }

    @Test
    fun `min of vectors`() {
        val v1 = Vec2(1f, 5f)
        val v2 = Vec2(3f, 2f)
        val v3 = Vec2.min(v1, v2)
        assertEquals(1f, v3.x)
        assertEquals(2f, v3.y)
    }

    @Test
    fun `max of vectors`() {
        val v1 = Vec2(1f, 5f)
        val v2 = Vec2(3f, 2f)
        val v3 = Vec2.max(v1, v2)
        assertEquals(3f, v3.x)
        assertEquals(5f, v3.y)
    }
}