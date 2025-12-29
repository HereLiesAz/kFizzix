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
import org.junit.jupiter.api.Test

class Mat22Test {

    private fun assertMat22Equals(expected: Mat22, actual: Mat22, delta: Float) {
        assertEquals(expected.ex.x, actual.ex.x, delta)
        assertEquals(expected.ex.y, actual.ex.y, delta)
        assertEquals(expected.ey.x, actual.ey.x, delta)
        assertEquals(expected.ey.y, actual.ey.y, delta)
    }

    @Test
    fun `test default constructor`() {
        val mat = Mat22()
        assertEquals(0f, mat.ex.x)
        assertEquals(0f, mat.ex.y)
        assertEquals(0f, mat.ey.x)
        assertEquals(0f, mat.ey.y)
    }

    @Test
    fun `test constructor with column vectors`() {
        val col1 = Vec2(1f, 2f)
        val col2 = Vec2(3f, 4f)
        val mat = Mat22(col1, col2)
        assertEquals(1f, mat.ex.x)
        assertEquals(2f, mat.ex.y)
        assertEquals(3f, mat.ey.x)
        assertEquals(4f, mat.ey.y)
    }

    @Test
    fun `test constructor with floats`() {
        val mat = Mat22(1f, 3f, 2f, 4f)
        assertEquals(1f, mat.ex.x)
        assertEquals(2f, mat.ex.y)
        assertEquals(3f, mat.ey.x)
        assertEquals(4f, mat.ey.y)
    }

    @Test
    fun `test set with another matrix`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22()
        mat2.set(mat1)
        assertEquals(1f, mat2.ex.x)
        assertEquals(3f, mat2.ex.y)
        assertEquals(2f, mat2.ey.x)
        assertEquals(4f, mat2.ey.y)
    }

    @Test
    fun `test set with floats`() {
        val mat = Mat22()
        mat.set(1f, 2f, 3f, 4f)
        assertEquals(1f, mat.ex.x)
        assertEquals(3f, mat.ex.y)
        assertEquals(2f, mat.ey.x)
        assertEquals(4f, mat.ey.y)
    }

    @Test
    fun `test setIdentity`() {
        val mat = Mat22()
        mat.setIdentity()
        assertEquals(1f, mat.ex.x)
        assertEquals(0f, mat.ex.y)
        assertEquals(0f, mat.ey.x)
        assertEquals(1f, mat.ey.y)
    }

    @Test
    fun `test setZero`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        mat.setZero()
        assertEquals(0f, mat.ex.x)
        assertEquals(0f, mat.ex.y)
        assertEquals(0f, mat.ey.x)
        assertEquals(0f, mat.ey.y)
    }

    @Test
    fun `test set with angle`() {
        val mat = Mat22()
        mat.set(PI.toFloat() / 2)
        assertMat22Equals(Mat22(0f, -1f, 1f, 0f), mat, 1e-7f)
    }

    @Test
    fun `test getAngle`() {
        val mat = Mat22()
        mat.set(PI.toFloat() / 4)
        assertEquals(PI.toFloat() / 4, mat.angle, 1e-7f)
    }

    @Test
    fun `test invert`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val inverted = mat.invert()
        val expected = Mat22(-2f, 1f, 1.5f, -0.5f)
        assertMat22Equals(expected, inverted, 1e-7f)
    }

    @Test
    fun `test invertLocal`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        mat.invertLocal()
        val expected = Mat22(-2f, 1f, 1.5f, -0.5f)
        assertMat22Equals(expected, mat, 1e-7f)
    }

    @Test
    fun `test invertToOut`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val out = Mat22()
        mat.invertToOut(out)
        val expected = Mat22(-2f, 1f, 1.5f, -0.5f)
        assertMat22Equals(expected, out, 1e-7f)
    }

    @Test
    fun `test solve`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val b = Vec2(5f, 6f)
        val x = mat.solve(b)
        assertEquals(-4f, x.x, 1e-7f)
        assertEquals(4.5f, x.y, 1e-7f)
    }

    @Test
    fun `test solveToOut`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val b = Vec2(5f, 6f)
        val out = Vec2()
        mat.solveToOut(b, out)
        assertEquals(-4f, out.x, 1e-7f)
        assertEquals(4.5f, out.y, 1e-7f)
    }

    @Test
    fun `test add`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val result = mat1.add(mat2)
        val expected = Mat22(6f, 8f, 10f, 12f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test addLocal`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        mat1.addLocal(mat2)
        val expected = Mat22(6f, 8f, 10f, 12f)
        assertMat22Equals(expected, mat1, 1e-7f)
    }

    @Test
    fun `test mul with vector`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val vec = Vec2(5f, 6f)
        val result = mat.mul(vec)
        assertEquals(17f, result.x, 1e-7f)
        assertEquals(39f, result.y, 1e-7f)
    }

    @Test
    fun `test mulToOut with vector`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val vec = Vec2(5f, 6f)
        val out = Vec2()
        mat.mulToOut(vec, out)
        assertEquals(17f, out.x, 1e-7f)
        assertEquals(39f, out.y, 1e-7f)
    }

    @Test
    fun `test mul with matrix`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val result = mat1.mul(mat2)
        val expected = Mat22(19f, 22f, 43f, 50f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test mulToOut with matrix`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val out = Mat22()
        mat1.mulToOut(mat2, out)
        val expected = Mat22(19f, 22f, 43f, 50f)
        assertMat22Equals(expected, out, 1e-7f)
    }

    @Test
    fun `test mulTrans with vector`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val vec = Vec2(5f, 6f)
        val result = mat.mulTrans(vec)
        assertEquals(23f, result.x, 1e-7f)
        assertEquals(34f, result.y, 1e-7f)
    }

    @Test
    fun `test mulTransToOut with vector`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        val vec = Vec2(5f, 6f)
        val out = Vec2()
        mat.mulTransToOut(vec, out)
        assertEquals(23f, out.x, 1e-7f)
        assertEquals(34f, out.y, 1e-7f)
    }

    @Test
    fun `test mulTrans with matrix`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val result = mat1.mulTrans(mat2)
        val expected = Mat22(23f, 31f, 34f, 46f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test mulTransToOut with matrix`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val out = Mat22()
        mat1.mulTransToOut(mat2, out)
        val expected = Mat22(23f, 31f, 34f, 46f)
        assertMat22Equals(expected, out, 1e-7f)
    }

    @Test
    fun `test abs`() {
        val mat = Mat22(-1f, 2f, -3f, -4f)
        val result = mat.abs()
        val expected = Mat22(1f, 2f, 3f, 4f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test absLocal`() {
        val mat = Mat22(-1f, 2f, -3f, -4f)
        mat.absLocal()
        val expected = Mat22(1f, 2f, 3f, 4f)
        assertMat22Equals(expected, mat, 1e-7f)
    }

    @Test
    fun `test companion mul`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val result = Mat22.mul(mat1, mat2)
        val expected = Mat22(19f, 22f, 43f, 50f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test companion mulToOut`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val out = Mat22()
        Mat22.mulToOut(mat1, mat2, out)
        val expected = Mat22(19f, 22f, 43f, 50f)
        assertMat22Equals(expected, out, 1e-7f)
    }

    @Test
    fun `test companion mulTrans`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val result = Mat22.mulTrans(mat1, mat2)
        val expected = Mat22(17f, 23f, 39f, 53f)
        assertMat22Equals(expected, result, 1e-7f)
    }

    @Test
    fun `test companion mulTransToOut`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(5f, 6f, 7f, 8f)
        val out = Mat22()
        Mat22.mulTransToOut(mat1, mat2, out)
        val expected = Mat22(17f, 23f, 39f, 53f)
        assertMat22Equals(expected, out, 1e-7f)
    }

    @Test
    fun `test equals and hashCode`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = Mat22(1f, 2f, 3f, 4f)
        val mat3 = Mat22(5f, 6f, 7f, 8f)
        assertEquals(mat1, mat2)
        assert(mat1 != mat3)
        assertEquals(mat1.hashCode(), mat2.hashCode())
        assert(mat1.hashCode() != mat3.hashCode())
    }

    @Test
    fun `test copy`() {
        val mat1 = Mat22(1f, 2f, 3f, 4f)
        val mat2 = mat1.copy()
        assertEquals(mat1, mat2)
        assert(mat1 !== mat2)
    }

    @Test
    fun `test toString`() {
        val mat = Mat22(1f, 2f, 3f, 4f)
        assertEquals("[(${mat.ex.x}, ${mat.ey.x}), (${mat.ex.y}, ${mat.ey.y})]", mat.toString())
    }
}
