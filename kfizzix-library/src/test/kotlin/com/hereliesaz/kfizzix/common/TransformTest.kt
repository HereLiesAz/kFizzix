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
import kotlin.test.Test
import kotlin.test.assertEquals

class TransformTest {

    private fun assertTransformEquals(expected: Transform, actual: Transform, delta: Float) {
        assertEquals(expected.p.x, actual.p.x, delta)
        assertEquals(expected.p.y, actual.p.y, delta)
        assertEquals(expected.q.s, actual.q.s, delta)
        assertEquals(expected.q.c, actual.q.c, delta)
    }

    @Test
    fun `test default constructor`() {
        val transform = Transform()
        assertEquals(0f, transform.p.x)
        assertEquals(0f, transform.p.y)
        assertEquals(0f, transform.q.s)
        assertEquals(1f, transform.q.c)
    }

    @Test
    fun `test constructor with position and rotation`() {
        val pos = Vec2(1f, 2f)
        val rot = Rot(PI.toFloat() / 2)
        val transform = Transform(pos, rot)
        assertEquals(1f, transform.p.x)
        assertEquals(2f, transform.p.y)
        assertEquals(1f, transform.q.s, 1e-7f)
        assertEquals(0f, transform.q.c, 1e-7f)
    }

    @Test
    fun `test set with another transform`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform()
        transform2.set(transform1)
        assertTransformEquals(transform1, transform2, 1e-7f)
    }

    @Test
    fun `test set with position and angle`() {
        val transform = Transform()
        val pos = Vec2(1f, 2f)
        transform.set(pos, PI.toFloat() / 2)
        assertEquals(1f, transform.p.x)
        assertEquals(2f, transform.p.y)
        assertEquals(1f, transform.q.s, 1e-7f)
        assertEquals(0f, transform.q.c, 1e-7f)
    }

    @Test
    fun `test setIdentity`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        transform.setIdentity()
        assertEquals(0f, transform.p.x)
        assertEquals(0f, transform.p.y)
        assertEquals(0f, transform.q.s)
        assertEquals(1f, transform.q.c)
    }

    @Test
    fun `test mul with vector`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val vec = Vec2(3f, 4f)
        val result = Transform.mul(transform, vec)
        assertEquals(-3f, result.x, 1e-7f)
        assertEquals(5f, result.y, 1e-7f)
    }

    @Test
    fun `test mulToOut with vector`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val vec = Vec2(3f, 4f)
        val out = Vec2()
        Transform.mulToOut(transform, vec, out)
        assertEquals(-3f, out.x, 1e-7f)
        assertEquals(5f, out.y, 1e-7f)
    }

    @Test
    fun `test mulTrans with vector`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val vec = Vec2(3f, 4f)
        val result = Transform.mulTrans(transform, vec)
        assertEquals(2f, result.x, 1e-7f)
        assertEquals(5f, result.y, 1e-7f)
    }

    @Test
    fun `test mulTransToOut with vector`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val vec = Vec2(3f, 4f)
        val out = Vec2()
        Transform.mulTransToOut(transform, vec, out)
        assertEquals(2f, out.x, 1e-7f)
        assertEquals(5f, out.y, 1e-7f)
    }

    @Test
    fun `test mul with transform`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform(Vec2(3f, 4f), Rot(PI.toFloat() / 4))
        val result = Transform.mul(transform1, transform2)
        val expected = Transform(Vec2(-3f, 5f), Rot(3 * PI.toFloat() / 4))
        assertTransformEquals(expected, result, 1e-7f)
    }

    @Test
    fun `test mulToOut with transform`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform(Vec2(3f, 4f), Rot(PI.toFloat() / 4))
        val out = Transform()
        Transform.mulToOut(transform1, transform2, out)
        val expected = Transform(Vec2(-3f, 5f), Rot(3 * PI.toFloat() / 4))
        assertTransformEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulTrans with transform`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform(Vec2(3f, 4f), Rot(PI.toFloat() / 4))
        val result = Transform.mulTrans(transform1, transform2)
        val expected = Transform(Vec2(2f, 2f), Rot(-PI.toFloat() / 4))
        assertTransformEquals(expected, result, 1e-7f)
    }

    @Test
    fun `test mulTransToOut with transform`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform(Vec2(3f, 4f), Rot(PI.toFloat() / 4))
        val out = Transform()
        Transform.mulTransToOut(transform1, transform2, out)
        val expected = Transform(Vec2(2f, 2f), Rot(-PI.toFloat() / 4))
        assertTransformEquals(expected, out, 1e-7f)
    }

    @Test
    fun `test equals and hashCode`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform3 = Transform(Vec2(3f, 4f), Rot(PI.toFloat() / 4))
        assertEquals(transform1, transform2)
        assert(transform1 != transform3)
        assertEquals(transform1.hashCode(), transform2.hashCode())
        assert(transform1.hashCode() != transform3.hashCode())
    }

    @Test
    fun `test copy`() {
        val transform1 = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val transform2 = transform1.copy()
        assertEquals(transform1, transform2)
        assert(transform1 !== transform2)
    }

    @Test
    fun `test toString`() {
        val transform = Transform(Vec2(1f, 2f), Rot(PI.toFloat() / 2))
        val expectedString = "XForm:\nPosition: ${transform.p}\nR: \n${transform.q}\n"
        assertEquals(expectedString, transform.toString())
    }
}