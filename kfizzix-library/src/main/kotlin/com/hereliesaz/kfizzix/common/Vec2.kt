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
 * A 2D column vector.
 * This class represents a 2D vector with x and y components.
 *
 * @param x the x-component of the vector, defaults to 0.
 * @param y the y-component of the vector, defaults to 0.
 * @constructor Creates a new vector with the given components.
 * @author Daniel Murphy
 * @see [Box2D b2Math.h](https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L40-L129)
 */
data class Vec2(
    @JvmField var x: Float = 0f,
    @JvmField var y: Float = 0f
) : Serializable {

    /**
     * Creates a new vector by copying the values from another vector.
     *
     * @param toCopy the vector to copy from
     */
    constructor(toCopy: Vec2) : this(toCopy.x, toCopy.y)

    /**
     * Zero out this vector.
     * After this call, both x and y components will be 0.
     */
    fun setZero() {
        x = 0.0f
        y = 0.0f
    }

    /**
     * Set the vector component-wise.
     *
     * @param x the new x-component
     * @param y the new y-component
     * @return this vector for chaining
     */
    fun set(x: Float, y: Float): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    /**
     * Set this vector to another vector.
     *
     * @param v the vector to copy from
     * @return this vector for chaining
     */
    fun set(v: Vec2): Vec2 {
        this.x = v.x
        this.y = v.y
        return this
    }

    /**
     * Return the sum of this vector and another; does not alter either one.
     *
     * @param v the vector to add
     * @return a new vector containing the sum
     */
    operator fun plus(v: Vec2): Vec2 {
        return Vec2(x + v.x, y + v.y)
    }

    /**
     * Return the difference of this vector and another; does not alter either one.
     *
     * @param v the vector to subtract
     * @return a new vector containing the difference
     */
    operator fun minus(v: Vec2): Vec2 {
        return Vec2(x - v.x, y - v.y)
    }

    /**
     * Return this vector multiplied by a scalar; does not alter this vector.
     *
     * @param a the scalar to multiply by
     * @return a new vector containing the result
     */
    operator fun times(a: Float): Vec2 {
        return Vec2(x * a, y * a)
    }

    /**
     * Return the negation of this vector; does not alter this vector.
     */
    operator fun unaryMinus(): Vec2 {
        return Vec2(-x, -y)
    }

    /**
     * Flip the vector and return it - alters this vector.
     *
     * @return this vector for chaining
     */
    fun negateLocal(): Vec2 {
        x = -x
        y = -y
        return this
    }

    /**
     * Add another vector to this one and returns result - alters this vector.
     *
     * @param v the vector to add
     * @return this vector for chaining
     */
    fun addLocal(v: Vec2): Vec2 {
        x += v.x
        y += v.y
        return this
    }

    /**
     * Adds values to this vector and returns result - alters this vector.
     *
     * @param x the x-component to add
     * @param y the y-component to add
     * @return this vector for chaining
     */
    fun addLocal(x: Float, y: Float): Vec2 {
        this.x += x
        this.y += y
        return this
    }

    /**
     * Subtract another vector from this one and return result - alters this vector.
     *
     * @param v the vector to subtract
     * @return this vector for chaining
     */
    fun subLocal(v: Vec2): Vec2 {
        x -= v.x
        y -= v.y
        return this
    }

    /**
     * Multiply this vector by a number and return result - alters this vector.
     *
     * @param a the scalar to multiply by
     * @return this vector for chaining
     */
    fun mulLocal(a: Float): Vec2 {
        x *= a
        y *= a
        return this
    }

    /**
     * Get the skew vector such that `dot(skew_vec, other) == cross(vec, other)`.
     * The skew vector is `(-y, x)`.
     *
     * @return a new vector containing the skew vector
     */
    fun skew(): Vec2 {
        return Vec2(-y, x)
    }

    /**
     * Get the skew vector such that `dot(skew_vec, other) == cross(vec, other)`.
     * The skew vector is `(-y, x)`.
     *
     * @param out the vector to store the result in
     */
    fun skew(out: Vec2) {
        out.x = -y
        out.y = x
    }

    /**
     * Get the length of this vector (the norm).
     *
     * @return the length of this vector
     */
    fun length(): Float {
        return MathUtils.sqrt(x * x + y * y)
    }

    /**
     * Get the length squared. For performance, use this instead of [length].
     *
     * @return the squared length of this vector
     */
    fun lengthSquared(): Float {
        return x * x + y * y
    }

    /**
     * Convert this vector into a unit vector. Returns the length.
     * Normalize this vector and return the length before normalization. Alters this vector.
     *
     * @return the length before normalization
     */
    fun normalize(): Float {
        val length = length()
        if (length < Settings.EPSILON) {
            return 0f
        }
        val invLength = 1.0f / length
        x *= invLength
        y *= invLength
        return length
    }

    /**
     * Does this vector contain finite coordinates?
     *
     * @return `true` if the vector represents a pair of valid, non-infinite floating point numbers.
     */
    fun isValid(): Boolean {
        return !x.isNaN() && !x.isInfinite() && !y.isNaN() && !y.isInfinite()
    }

    /**
     * Return a new vector that has positive components.
     *
     * @return a new vector with the absolute values of the components
     */
    fun abs(): Vec2 {
        return Vec2(MathUtils.abs(x), MathUtils.abs(y))
    }

    /**
     * Sets the components of this vector to their absolute values.
     */
    fun absLocal() {
        x = MathUtils.abs(x)
        y = MathUtils.abs(y)
    }

    companion object {
        private const val serialVersionUID = 1L

        /**
         * Returns a new vector with the absolute values of the components of the given vector.
         *
         * @param a the vector to take the absolute values of
         * @return a new vector with the absolute values
         */
        @JvmStatic
        fun abs(a: Vec2): Vec2 {
            return Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))
        }

        /**
         * Sets the output vector to the absolute values of the components of the given vector.
         *
         * @param a the vector to take the absolute values of
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun absToOut(a: Vec2, out: Vec2) {
            out.x = MathUtils.abs(a.x)
            out.y = MathUtils.abs(a.y)
        }

        /**
         * Computes the dot product of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return the dot product
         */
        @JvmStatic
        fun dot(a: Vec2, b: Vec2): Float {
            return a.x * b.x + a.y * b.y
        }

        /**
         * Computes the cross product of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return the cross product
         */
        @JvmStatic
        fun cross(a: Vec2, b: Vec2): Float {
            return a.x * b.y - a.y * b.x
        }

        /**
         * Computes the cross product of a vector and a scalar.
         *
         * @param a the vector
         * @param s the scalar
         * @return a new vector containing the result
         */
        @JvmStatic
        fun cross(a: Vec2, s: Float): Vec2 {
            return Vec2(s * a.y, -s * a.x)
        }

        /**
         * Computes the cross product of a vector and a scalar, and stores the result in the output vector.
         *
         * @param a the vector
         * @param s the scalar
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            val tempY = -s * a.x
            out.x = s * a.y
            out.y = tempY
        }

        /**
         * Computes the cross product of a vector and a scalar, and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param a the vector
         * @param s the scalar
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            assert(out !== a)
            out.x = s * a.y
            out.y = -s * a.x
        }

        /**
         * Computes the cross product of a scalar and a vector.
         *
         * @param s the scalar
         * @param a the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun cross(s: Float, a: Vec2): Vec2 {
            return Vec2(-s * a.y, s * a.x)
        }

        /**
         * Computes the cross product of a scalar and a vector, and stores the result in the output vector.
         *
         * @param s the scalar
         * @param a the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            val tempY = s * a.x
            out.x = -s * a.y
            out.y = tempY
        }

        /**
         * Computes the cross product of a scalar and a vector, and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param s the scalar
         * @param a the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            assert(out !== a)
            out.x = -s * a.y
            out.y = s * a.x
        }

        /**
         * Sets the output vector to the negation of the input vector.
         *
         * @param a the vector to negate
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun negateToOut(a: Vec2, out: Vec2) {
            out.x = -a.x
            out.y = -a.y
        }

        /**
         * Returns a new vector with the minimum components of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return a new vector with the minimum components
         */
        @JvmStatic
        fun min(a: Vec2, b: Vec2): Vec2 {
            return Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y))
        }

        /**
         * Returns a new vector with the maximum components of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return a new vector with the maximum components
         */
        @JvmStatic
        fun max(a: Vec2, b: Vec2): Vec2 {
            return new Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y))
        }

        /**
         * Sets the output vector to the minimum components of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = Math.min(a.x, b.x)
            out.y = Math.min(a.y, b.y)
        }

        /**
         * Sets the output vector to the maximum components of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = Math.max(a.x, b.x)
            out.y = Math.max(a.y, b.y)
        }
    }
}
