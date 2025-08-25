/*
 * Copyright (c) 2013, Daniel Murphy
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

import java.io.Serializable

/**
 * Represents a rotation.
 * This class is used to represent a 2D rotation and is composed of a sine and cosine value.
 *
 * @param angle the angle in radians
 * @constructor Creates a new rotation from the given angle.
 * @author Daniel Murphy
 */
class Rot(angle: Float? = null) : Serializable {

    /**
     * The sine component of the rotation.
     */
    @JvmField
    var s: Float = 0f

    /**
     * The cosine component of the rotation.
     */
    @JvmField
    var c: Float = 0f

    init {
        if (angle != null) {
            set(angle)
        } else {
            setIdentity()
        }
    }

    /**
     * The sine component of the rotation.
     */
    val sin: Float
        get() = s

    override fun toString(): String {
        return "Rot(s:$s, c:$c)"
    }

    /**
     * The cosine component of the rotation.
     */
    val cos: Float
        get() = c

    /**
     * Sets the rotation to the given angle.
     *
     * @param angle the angle in radians
     * @return this rotation for chaining
     */
    fun set(angle: Float): Rot {
        s = MathUtils.sin(angle)
        c = MathUtils.cos(angle)
        return this
    }

    /**
     * Copies the values from another rotation.
     *
     * @param other the rotation to copy from
     * @return this rotation for chaining
     */
    fun set(other: Rot): Rot {
        s = other.s
        c = other.c
        return this
    }

    /**
     * Sets the rotation to the identity rotation (no rotation).
     *
     * @return this rotation for chaining
     */
    fun setIdentity(): Rot {
        s = 0f
        c = 1f
        return this
    }

    /**
     * The angle of the rotation in radians.
     */
    val angle: Float
        get() = MathUtils.atan2(s, c)

    /**
     * Get the x-axis of this rotation.
     *
     * @param xAxis the vector to store the x-axis in
     */
    fun getXAxis(xAxis: Vec2) {
        xAxis.set(c, s)
    }

    /**
     * Get the y-axis of this rotation.
     *
     * @param yAxis the vector to store the y-axis in
     */
    fun getYAxis(yAxis: Vec2) {
        yAxis.set(-s, c)
    }

    /**
     * Creates a copy of this rotation.
     *
     * @return a new rotation with the same values
     */
    public fun clone(): Rot {
        val copy = Rot()
        copy.s = s
        copy.c = c
        return copy
    }

    companion object {
        private const val serialVersionUID = 1L

        /**
         * Multiplies two rotations.
         *
         * @param q the first rotation
         * @param r the second rotation
         * @param out the output rotation
         */
        @JvmStatic
        fun mul(q: Rot, r: Rot, out: Rot) {
            val tempC = q.c * r.c - q.s * r.s
            out.s = q.s * r.c + q.c * r.s
            out.c = tempC
        }

        /**
         * Multiplies two rotations, assuming the output is not the same as the input.
         * This is an unsafe version of [mul] that is faster but has more restrictions.
         *
         * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L535-L546
         *
         * @param q the first rotation
         * @param r the second rotation
         * @param out the output rotation
         */
        @JvmStatic
        fun mulUnsafe(q: Rot, r: Rot, out: Rot) {
            assert(r !== out)
            assert(q !== out)
            out.s = q.s * r.c + q.c * r.s
            out.c = q.c * r.c - q.s * r.s
        }

        /**
         * Multiplies the transpose of the first rotation by the second rotation.
         *
         * @param q the first rotation (will be transposed)
         * @param r the second rotation
         * @param out the output rotation
         */
        @JvmStatic
        fun mulTrans(q: Rot, r: Rot, out: Rot) {
            val tempC = q.c * r.c + q.s * r.s
            out.s = q.c * r.s - q.s * r.c
            out.c = tempC
        }

        /**
         * Multiplies the transpose of the first rotation by the second rotation, assuming the output is not the same as the input.
         * This is an unsafe version of [mulTrans] that is faster but has more restrictions.
         *
         * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L548-L559
         *
         * @param q the first rotation (will be transposed)
         * @param r the second rotation
         * @param out the output rotation
         */
        @JvmStatic
        fun mulTransUnsafe(q: Rot, r: Rot, out: Rot) {
            out.s = q.c * r.s - q.s * r.c
            out.c = q.c * r.c + q.s * r.s
        }

        /**
         * Multiplies a rotation by a vector.
         *
         * @param q the rotation
         * @param v the vector
         * @param out the output vector
         */
        @JvmStatic
        fun mulToOut(q: Rot, v: Vec2, out: Vec2) {
            val tempY = q.s * v.x + q.c * v.y
            out.x = q.c * v.x - q.s * v.y
            out.y = tempY
        }

        /**
         * Multiplies a rotation by a vector, assuming the output is not the same as the input.
         * This is an unsafe version of [mulToOut] that is faster but has more restrictions.
         *
         * @param q the rotation
         * @param v the vector
         * @param out the output vector
         */
        @JvmStatic
        fun mulToOutUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x - q.s * v.y
            out.y = q.s * v.x + q.c * v.y
        }

        /**
         * Multiplies the transpose of a rotation by a vector.
         *
         * @param q the rotation (will be transposed)
         * @param v the vector
         * @param out the output vector
         */
        @JvmStatic
        fun mulTrans(q: Rot, v: Vec2, out: Vec2) {
            val tempY = -q.s * v.x + q.c * v.y
            out.x = q.c * v.x + q.s * v.y
            out.y = tempY
        }

        /**
         * Multiplies the transpose of a rotation by a vector, assuming the output is not the same as the input.
         * This is an unsafe version of [mulTrans] that is faster but has more restrictions.
         *
         * @param q the rotation (will be transposed)
         * @param v the vector
         * @param out the output vector
         */
        @JvmStatic
        fun mulTransUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x + q.s * v.y
            out.y = -q.s * v.x + q.c * v.y
        }
    }
}
