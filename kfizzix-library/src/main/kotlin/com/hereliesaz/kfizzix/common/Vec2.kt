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
 *
 * This class represents a geometric vector in 2D space, containing an [x] and [y] component.
 * It is the fundamental building block for physics calculations, representing position,
 * velocity, force, and impulses.
 *
 * **Performance Note:**
 * Many methods have a standard version (e.g., [add]) which creates a new [Vec2] object,
 * and a "Local" or "ToOut" version (e.g., [addLocal], [addToOut]) which modifies an existing object.
 * In a physics engine, avoiding garbage collection (GC) is critical. Prefer using the "Local"
 * or "ToOut" methods in tight loops to reuse objects.
 *
 * @param x The x-coordinate (horizontal). Defaults to 0.0.
 * @param y The y-coordinate (vertical). Defaults to 0.0.
 * @constructor Creates a new vector with the given components.
 * @author Daniel Murphy
 */
class Vec2(
    @JvmField var x: Float = 0f,
    @JvmField var y: Float = 0f
) : Serializable {

    /**
     * Copy constructor. Creates a new vector with the same values as the input vector.
     * @param v The vector to copy.
     */
    constructor(v: Vec2) : this(v.x, v.y)

    /** Destructuring declaration for 'x' component. Allows `val (x, y) = vec`. */
    operator fun component1() = x
    /** Destructuring declaration for 'y' component. Allows `val (x, y) = vec`. */
    operator fun component2() = y

    /**
     * Creates a copy of this vector.
     * @param x New x value (optional).
     * @param y New y value (optional).
     * @return A new Vec2 instance.
     */
    fun copy(x: Float = this.x, y: Float = this.y) = Vec2(x, y)

    /**
     * Clones this vector.
     * @return A new Vec2 with the same values.
     */
    fun clone(): Vec2 {
        return Vec2(x, y)
    }

    /**
     * Sets this vector's coordinates to zero.
     * Use this to reset a vector for reuse.
     */
    fun setZero() {
        x = 0.0f
        y = 0.0f
    }

    /**
     * Sets this vector's coordinates to the given values.
     * @param x The new x coordinate.
     * @param y The new y coordinate.
     * @return This vector (for chaining).
     */
    fun set(x: Float, y: Float): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    /**
     * Sets this vector's coordinates to match another vector.
     * @param v The vector to copy values from.
     * @return This vector (for chaining).
     */
    fun set(v: Vec2): Vec2 {
        this.x = v.x
        this.y = v.y
        return this
    }

    /**
     * Adds two vectors and returns the result as a new vector.
     * Result = this + v.
     * @param v The vector to add.
     * @return A new vector containing the sum.
     */
    fun add(v: Vec2): Vec2 {
        return Vec2(x + v.x, y + v.y)
    }

    /**
     * Adds two vectors and stores the result in this vector (mutation).
     * this += v.
     * @param v The vector to add.
     * @return This vector (for chaining).
     */
    fun addLocal(v: Vec2): Vec2 {
        x += v.x
        y += v.y
        return this
    }

    /**
     * Adds specific x and y values to this vector (mutation).
     * @param x The x amount to add.
     * @param y The y amount to add.
     * @return This vector (for chaining).
     */
    fun addLocal(x: Float, y: Float): Vec2 {
        this.x += x
        this.y += y
        return this
    }

    /**
     * Subtracts a vector from this one and returns a new vector.
     * Result = this - v.
     * @param v The vector to subtract.
     * @return A new vector containing the difference.
     */
    fun sub(v: Vec2): Vec2 {
        return Vec2(x - v.x, y - v.y)
    }

    /**
     * Subtracts a vector from this one and stores the result in this vector (mutation).
     * this -= v.
     * @param v The vector to subtract.
     * @return This vector (for chaining).
     */
    fun subLocal(v: Vec2): Vec2 {
        x -= v.x
        y -= v.y
        return this
    }

    /**
     * Multiplies this vector by a scalar and returns a new vector.
     * Result = this * a.
     * @param a The scalar value.
     * @return A new scaled vector.
     */
    fun mul(a: Float): Vec2 {
        return Vec2(x * a, y * a)
    }

    /**
     * Multiplies this vector by a scalar and stores the result in this vector (mutation).
     * this *= a.
     * @param a The scalar value.
     * @return This vector (for chaining).
     */
    fun mulLocal(a: Float): Vec2 {
        x *= a
        y *= a
        return this
    }

    /**
     * Negates this vector and returns a new vector.
     * Result = -this.
     * @return A new vector (-x, -y).
     */
    fun negate(): Vec2 {
        return Vec2(-x, -y)
    }

    /**
     * Negates this vector in place (mutation).
     * this = -this.
     * @return This vector (for chaining).
     */
    fun negateLocal(): Vec2 {
        x = -x
        y = -y
        return this
    }

    /**
     * Calculates the skew vector (rotation by 90 degrees counter-clockwise).
     * If v = (x, y), then skew(v) = (-y, x).
     * This is useful for cross product calculations in 2D.
     *
     * Explanation:
     * `dot(skew_vec, other) == cross(vec, other)`
     *
     * @return A new vector representing the skew.
     */
    fun skew(): Vec2 {
        return Vec2(-y, x)
    }

    /**
     * Calculates the skew vector and stores it in the output vector.
     * @param out The vector to store the result in.
     */
    fun skew(out: Vec2) {
        out.x = -y
        out.y = x
    }

    /**
     * Calculates the length (magnitude) of the vector.
     * Uses sqrt(), so it is relatively slow.
     * @return The length.
     */
    fun length(): Float {
        return MathUtils.sqrt(x * x + y * y)
    }

    /**
     * Calculates the squared length of the vector.
     * Result = x*x + y*y.
     * Much faster than [length] because it avoids the square root.
     * Useful for distance comparisons (if dist^2 < range^2).
     * @return The squared length.
     */
    fun lengthSquared(): Float {
        return x * x + y * y
    }

    /**
     * Normalizes this vector (makes it unit length) and returns the previous length.
     * If the length is very small (close to zero), the vector becomes zero to avoid NaN.
     *
     * @return The original length of the vector.
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
     * Checks if the vector components are valid finite numbers.
     * @return True if x and y are not NaN and not Infinite.
     */
    fun isValid(): Boolean {
        return !x.isNaN() && !x.isInfinite() && !y.isNaN() && !y.isInfinite()
    }

    /**
     * Returns a new vector containing the absolute value of each component.
     * @return new Vec2(|x|, |y|).
     */
    fun abs(): Vec2 {
        return Vec2(MathUtils.abs(x), MathUtils.abs(y))
    }

    /**
     * Modifies this vector to contain the absolute value of its components (mutation).
     */
    fun absLocal() {
        x = MathUtils.abs(x)
        y = MathUtils.abs(y)
    }

    /**
     * Calculates the dot product with another vector.
     * Dot product = x1*x2 + y1*y2.
     * Geometric meaning: |a||b|cos(theta).
     * If dot > 0, angle is acute. If dot < 0, angle is obtuse. If dot == 0, vectors are perpendicular.
     * @param v The other vector.
     * @return The scalar dot product.
     */
    fun dot(v: Vec2): Float {
        return x * v.x + y * v.y
    }


    operator fun plus(v: Vec2) = add(v)
    operator fun minus(v: Vec2) = sub(v)
    operator fun times(a: Float) = mul(a)
    operator fun unaryMinus() = negate()

    override fun toString(): String {
        return "($x,$y)"
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Vec2

        if (x != other.x) return false
        if (y != other.y) return false

        return true
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    companion object {
        private const val serialVersionUID = 1L

        /**
         * Creates a new vector with absolute values of the input vector's components.
         * @param a The input vector.
         * @return New vector |a|.
         */
        @JvmStatic
        fun abs(a: Vec2): Vec2 {
            return Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))
        }

        /**
         * Computes absolute values of vector 'a' and stores them in 'out'.
         * @param a Input vector.
         * @param out Output vector (modified).
         */
        @JvmStatic
        fun absToOut(a: Vec2, out: Vec2) {
            out.x = MathUtils.abs(a.x)
            out.y = MathUtils.abs(a.y)
        }

        /**
         * Computes the dot product of two vectors.
         * Dot(a, b) = a.x * b.x + a.y * b.y.
         * @param a First vector.
         * @param b Second vector.
         * @return Scalar result.
         */
        @JvmStatic
        fun dot(a: Vec2, b: Vec2): Float {
            return a.x * b.x + a.y * b.y
        }

        /**
         * Computes the 2D Cross Product of two vectors.
         * In 2D, the cross product is a scalar (the Z-component of the 3D cross product).
         * Cross(a, b) = a.x * b.y - a.y * b.x.
         * Geometric meaning: |a||b|sin(theta). Represents the signed area of the parallelogram formed by a and b.
         * @param a First vector.
         * @param b Second vector.
         * @return Scalar result.
         */
        @JvmStatic
        fun cross(a: Vec2, b: Vec2): Float {
            return a.x * b.y - a.y * b.x
        }

        /**
         * Computes the Cross Product of a vector 'a' and a scalar 's'.
         * This effectively rotates the vector -90 degrees and scales it.
         * Result = (s * a.y, -s * a.x).
         * @param a The vector.
         * @param s The scalar.
         * @return New vector result.
         */
        @JvmStatic
        fun cross(a: Vec2, s: Float): Vec2 {
            return Vec2(s * a.y, -s * a.x)
        }

        /**
         * Computes Cross Product of a vector 'a' and scalar 's', storing in 'out'.
         * @param a The vector.
         * @param s The scalar.
         * @param out Output vector.
         */
        @JvmStatic
        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            val tempY = -s * a.x
            out.x = s * a.y
            out.y = tempY
        }

        /**
         * Unsafe version of crossToOut.
         * Assumes 'out' is not 'a'.
         */
        @DelicateFizzixApi
        @JvmStatic
        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            assert(out !== a)
            out.x = s * a.y
            out.y = -s * a.x
        }

        /**
         * Computes the Cross Product of a scalar 's' and a vector 'a'.
         * This effectively rotates the vector +90 degrees and scales it.
         * Result = (-s * a.y, s * a.x).
         * @param s The scalar.
         * @param a The vector.
         * @return New vector result.
         */
        @JvmStatic
        fun cross(s: Float, a: Vec2): Vec2 {
            return Vec2(-s * a.y, s * a.x)
        }

        /**
         * Computes Cross Product of scalar 's' and vector 'a', storing in 'out'.
         * @param s The scalar.
         * @param a The vector.
         * @param out Output vector.
         */
        @JvmStatic
        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            val tempY = s * a.x
            out.x = -s * a.y
            out.y = tempY
        }

        /**
         * Unsafe version of crossToOut.
         * Assumes 'out' is not 'a'.
         */
        @DelicateFizzixApi
        @JvmStatic
        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            assert(out !== a)
            out.x = -s * a.y
            out.y = s * a.x
        }

        /**
         * Negates vector 'a' and stores in 'out'.
         * out = -a.
         */
        @JvmStatic
        fun negateToOut(a: Vec2, out: Vec2) {
            out.x = -a.x
            out.y = -a.y
        }

        /**
         * Returns a new vector containing the component-wise minimum of 'a' and 'b'.
         * @return min(a, b).
         */
        @JvmStatic
        fun min(a: Vec2, b: Vec2): Vec2 {
            return Vec2(MathUtils.min(a.x, b.x), MathUtils.min(a.y, b.y))
        }

        /**
         * Returns a new vector containing the component-wise maximum of 'a' and 'b'.
         * @return max(a, b).
         */
        @JvmStatic
        fun max(a: Vec2, b: Vec2): Vec2 {
            return Vec2(MathUtils.max(a.x, b.x), MathUtils.max(a.y, b.y))
        }

        /**
         * Stores the component-wise minimum of 'a' and 'b' into 'out'.
         */
        @JvmStatic
        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = MathUtils.min(a.x, b.x)
            out.y = MathUtils.min(a.y, b.y)
        }

        /**
         * Stores the component-wise maximum of 'a' and 'b' into 'out'.
         */
        @JvmStatic
        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = MathUtils.max(a.x, b.x)
            out.y = MathUtils.max(a.y, b.y)
        }
    }
}
