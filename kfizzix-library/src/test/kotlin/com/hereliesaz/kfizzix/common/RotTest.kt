/*
 * Copyright (c) 2024, Herelies-Az
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.common

import kotlin.math.PI
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotSame
import org.junit.jupiter.api.Test

class RotTest {

    private fun assertRotEquals(expected: Rot, actual: Rot, delta: Float) {
        assertEquals(expected.s, actual.s, delta)
        assertEquals(expected.c, actual.c, delta)
    }

    @Test
    fun `test default constructor`() {
        val rot = Rot()
        assertEquals(0f, rot.s)
        assertEquals(1f, rot.c)
        assertEquals(0f, rot.angle)
    }

    @Test
    fun `test constructor with angle`() {
        val angle = PI.toFloat() / 2
        val rot = Rot(angle)
        assertEquals(1f, rot.s, 1e-7f)
        assertEquals(0f, rot.c, 1e-7f)
    }

    @Test
    fun `test set with angle`() {
        val rot = Rot()
        val angle = PI.toFloat() / 2
        rot.set(angle)
        assertEquals(1f, rot.s, 1e-7f)
        assertEquals(0f, rot.c, 1e-7f)
    }

    @Test
    fun `test set with another Rot`() {
        val rot1 = Rot(PI.toFloat() / 2)
        val rot2 = Rot()
        rot2.set(rot1)
        assertRotEquals(rot1, rot2, 1e-7f)
    }

    @Test
    fun `test setIdentity`() {
        val rot = Rot(PI.toFloat() / 2)
        rot.setIdentity()
        assertEquals(0f, rot.s)
        assertEquals(1f, rot.c)
    }

    @Test
    fun `test getAngle`() {
        val angle = PI.toFloat() / 4
        val rot = Rot(angle)
        assertEquals(angle, rot.angle, 1e-7f)
    }

    @Test
    fun `test getXAxis`() {
        val rot = Rot(PI.toFloat() / 2)
        val xAxis = Vec2()
        rot.getXAxis(xAxis)
        assertEquals(0f, xAxis.x, 1e-7f)
        assertEquals(1f, xAxis.y, 1e-7f)
    }

    @Test
    fun `test getYAxis`() {
        val rot = Rot(PI.toFloat() / 2)
        val yAxis = Vec2()
        rot.getYAxis(yAxis)
        assertEquals(-1f, yAxis.x, 1e-7f)
        assertEquals(0f, yAxis.y, 1e-7f)
    }

    @Test
    fun `test clone`() {
        val rot1 = Rot(PI.toFloat() / 2)
        val rot2 = rot1.clone()
        assertEquals(rot1, rot2)
        assertNotSame(rot1, rot2)
    }

    @Test
    fun `test copy`() {
        val rot1 = Rot(PI.toFloat() / 2)
        val rot2 = rot1.copy()
        assertEquals(rot1, rot2)
        assertNotSame(rot1, rot2)
    }

    @Test
    fun `test mul`() {
        val rot1 = Rot(PI.toFloat() / 2) // 90 deg
        val rot2 = Rot(PI.toFloat() / 4) // 45 deg
        val out = Rot()
        Rot.mul(rot1, rot2, out)

        // Expected: 135 deg
        val expected = Rot(3 * PI.toFloat() / 4)
        assertRotEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulUnsafe`() {
        val rot1 = Rot(PI.toFloat() / 2)
        val rot2 = Rot(PI.toFloat() / 4)
        val out = Rot()
        Rot.mulUnsafe(rot1, rot2, out)

        val expected = Rot(3 * PI.toFloat() / 4)
        assertRotEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulTrans`() {
        val rot1 = Rot(PI.toFloat() / 2) // 90 deg
        val rot2 = Rot(3 * PI.toFloat() / 4) // 135 deg
        val out = Rot()
        // Transpose of rot1 is -90 deg. -90 + 135 = 45 deg.
        Rot.mulTrans(rot1, rot2, out)

        val expected = Rot(PI.toFloat() / 4)
        assertRotEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulTransUnsafe`() {
        val rot1 = Rot(PI.toFloat() / 2)
        val rot2 = Rot(3 * PI.toFloat() / 4)
        val out = Rot()
        Rot.mulTransUnsafe(rot1, rot2, out)

        val expected = Rot(PI.toFloat() / 4)
        assertRotEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulToOut`() {
        val rot = Rot(PI.toFloat() / 2) // 90 deg
        val v = Vec2(1f, 0f)
        val out = Vec2()
        // Rotate (1, 0) by 90 deg -> (0, 1)
        Rot.mulToOut(rot, v, out)

        assertEquals(0f, out.x, 1e-7f)
        assertEquals(1f, out.y, 1e-7f)
    }

    @Test
    fun `test mulToOutUnsafe`() {
        val rot = Rot(PI.toFloat() / 2)
        val v = Vec2(1f, 0f)
        val out = Vec2()
        Rot.mulToOutUnsafe(rot, v, out)

        assertEquals(0f, out.x, 1e-7f)
        assertEquals(1f, out.y, 1e-7f)
    }

    @Test
    fun `test mulTrans vector`() {
        val rot = Rot(PI.toFloat() / 2) // 90 deg
        val v = Vec2(0f, 1f)
        val out = Vec2()
        // Rotate (0, 1) by -90 deg -> (1, 0)
        Rot.mulTrans(rot, v, out)

        assertEquals(1f, out.x, 1e-7f)
        assertEquals(0f, out.y, 1e-7f)
    }

    @Test
    fun `test mulTransUnsafe vector`() {
        val rot = Rot(PI.toFloat() / 2)
        val v = Vec2(0f, 1f)
        val out = Vec2()
        Rot.mulTransUnsafe(rot, v, out)

        assertEquals(1f, out.x, 1e-7f)
        assertEquals(0f, out.y, 1e-7f)
    }
}
