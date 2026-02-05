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
class Vec2(
    // The x-coordinate of the vector.
    // Annotated with @JvmField to expose it as a public field in Java byte code for performance.
    @JvmField var x: Float = 0f,
    // The y-coordinate of the vector.
    @JvmField var y: Float = 0f
) : Serializable {

    // Copy constructor: creates a new Vec2 from an existing one.
    constructor(v: Vec2) : this(v.x, v.y)

    // Destructuring declaration for the x component.
    // Allows usage like: val (x, y) = vec2
    operator fun component1() = x

    // Destructuring declaration for the y component.
    operator fun component2() = y

    // Creates a copy of this vector, optionally modifying components.
    fun copy(x: Float = this.x, y: Float = this.y) = Vec2(x, y)

    // Clones the vector.
    fun clone(): Vec2 {
        // Return a new instance with the same values.
        return Vec2(x, y)
    }

    // Checks equality with another object.
    override fun equals(other: Any?): Boolean {
        // If they are the same instance in memory, they are equal.
        if (this === other) return true
        // If the other object is null or a different class, they are not equal.
        if (javaClass != other?.javaClass) return false

        // Cast the other object to Vec2.
        other as Vec2

        // Compare the x components.
        if (x != other.x) return false
        // Compare the y components.
        if (y != other.y) return false

        // If both components match, the vectors are equal.
        return true
    }

    // Computes a hash code for the vector.
    override fun hashCode(): Int {
        // Start with the hash of x.
        var result = x.hashCode()
        // Combine with the hash of y using a prime multiplier (31).
        result = 31 * result + y.hashCode()
        return result
    }

    // Returns a string representation of the vector.
    override fun toString(): String {
        return "Vec2(x=$x, y=$y)"
    }

    /**
     * Zero out this vector.
     * After this call, both x and y components will be 0.
     */
    fun setZero() {
        // Set x to zero.
        x = 0.0f
        // Set y to zero.
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
        // Assign the new x value.
        this.x = x
        // Assign the new y value.
        this.y = y
        // Return this instance to allow chaining method calls.
        return this
    }

    /**
     * Set this vector to another vector.
     *
     * @param v the vector to copy from
     * @return this vector for chaining
     */
    fun set(v: Vec2): Vec2 {
        // Copy the x value from the source vector.
        this.x = v.x
        // Copy the y value from the source vector.
        this.y = v.y
        // Return this instance.
        return this
    }

    /**
     * Return the sum of this vector and another; does not alter either one.
     *
     * @param v the vector to add
     * @return a new vector containing the sum
     */
    operator fun plus(v: Vec2): Vec2 {
        // Create and return a new Vec2 where each component is the sum of the inputs.
        return Vec2(x + v.x, y + v.y)
    }

    /**
     * Return the difference of this vector and another; does not alter either one.
     *
     * @param v the vector to subtract
     * @return a new vector containing the difference
     */
    operator fun minus(v: Vec2): Vec2 {
        // Create and return a new Vec2 where each component is the difference.
        return Vec2(x - v.x, y - v.y)
    }

    /**
     * Return this vector multiplied by a scalar; does not alter this vector.
     *
     * @param a the scalar to multiply by
     * @return a new vector containing the result
     */
    operator fun times(a: Float): Vec2 {
        // Create and return a new Vec2 where each component is scaled by 'a'.
        return Vec2(x * a, y * a)
    }

    /**
     * Return the negation of this vector; does not alter this vector.
     */
    operator fun unaryMinus(): Vec2 {
        // Create and return a new Vec2 with negated components.
        return Vec2(-x, -y)
    }

    /**
     * Flip the vector and return it. This method alters the original vector for chaining.
     * For a non-mutating version, see [unaryMinus].
     *
     * @return this vector for chaining
     */
    fun negateLocal(): Vec2 {
        // Negate the x component in place.
        x = -x
        // Negate the y component in place.
        y = -y
        // Return this instance.
        return this
    }

    /**
     * Add another vector to this one and returns result. This method alters the original vector for chaining.
     * For a non-mutating version, see [plus].
     *
     * @param v the vector to add
     * @return this vector for chaining
     */
    fun addLocal(v: Vec2): Vec2 {
        // Add the source vector's x to this vector's x.
        x += v.x
        // Add the source vector's y to this vector's y.
        y += v.y
        // Return this instance.
        return this
    }

    /**
     * Adds values to this vector and returns result. This method alters the original vector for chaining.
     *
     * @param x the x-component to add
     * @param y the y-component to add
     * @return this vector for chaining
     */
    fun addLocal(x: Float, y: Float): Vec2 {
        // Add the given x value to this vector's x.
        this.x += x
        // Add the given y value to this vector's y.
        this.y += y
        // Return this instance.
        return this
    }

    /**
     * Subtract another vector from this one and return result. This method alters the original vector for chaining.
     * For a non-mutating version, see [minus].
     *
     * @param v the vector to subtract
     * @return this vector for chaining
     */
    fun subLocal(v: Vec2): Vec2 {
        // Subtract the source vector's x from this vector's x.
        x -= v.x
        // Subtract the source vector's y from this vector's y.
        y -= v.y
        // Return this instance.
        return this
    }

    /**
     * Multiply this vector by a number and return result. This method alters the original vector for chaining.
     * For a non-mutating version, see [times].
     *
     * @param a the scalar to multiply by
     * @return this vector for chaining
     */
    fun mulLocal(a: Float): Vec2 {
        // Scale the x component by 'a'.
        x *= a
        // Scale the y component by 'a'.
        y *= a
        // Return this instance.
        return this
    }

    /**
     * Get the skew vector such that `dot(skew_vec, other) == cross(vec, other)`.
     * The skew vector is `(-y, x)`.
     *
     * @return a new vector containing the skew vector
     */
    fun skew(): Vec2 {
        // Return a new vector perpendicular to this one (-y, x).
        return Vec2(-y, x)
    }

    /**
     * Get the skew vector such that `dot(skew_vec, other) == cross(vec, other)`.
     * The skew vector is `(-y, x)`.
     *
     * @param out the vector to store the result in
     */
    fun skew(out: Vec2) {
        // Set the output x to -y.
        out.x = -y
        // Set the output y to x.
        out.y = x
    }

    /**
     * Get the length of this vector (the norm).
     *
     * @return the length of this vector
     */
    fun length(): Float {
        // Calculate the square root of the sum of squares.
        return MathUtils.sqrt(x * x + y * y)
    }

    /**
     * Get the length squared. For performance, use this instead of [length].
     *
     * @return the squared length of this vector
     */
    fun lengthSquared(): Float {
        // Calculate x^2 + y^2. avoiding the expensive sqrt() call.
        return x * x + y * y
    }

    /**
     * Convert this vector into a unit vector. Returns the length.
     * Normalize this vector and return the length before normalization. Alters this vector.
     *
     * @return the length before normalization
     */
    fun normalize(): Float {
        // Calculate the current length.
        val length = length()
        // If the length is too small, avoid division by zero/instability.
        if (length < Settings.EPSILON) {
            return 0f
        }
        // Calculate the inverse length (1/length) to use multiplication instead of division.
        val invLength = 1.0f / length
        // Scale x by the inverse length.
        x *= invLength
        // Scale y by the inverse length.
        y *= invLength
        // Return the original length.
        return length
    }

    /**
     * Does this vector contain finite coordinates?
     *
     * @return `true` if the vector represents a pair of valid, non-infinite floating point numbers.
     */
    fun isValid(): Boolean {
        // Check if x is not NaN and not Infinite, AND y is not NaN and not Infinite.
        return !x.isNaN() && !x.isInfinite() && !y.isNaN() && !y.isInfinite()
    }

    /**
     * Return a new vector that has positive components.
     *
     * @return a new vector with the absolute values of the components
     */
    fun abs(): Vec2 {
        // Return a new Vec2 with absolute values of components.
        return Vec2(MathUtils.abs(x), MathUtils.abs(y))
    }

    /**
     * Sets the components of this vector to their absolute values.
     */
    fun absLocal() {
        // Update x to its absolute value.
        x = MathUtils.abs(x)
        // Update y to its absolute value.
        y = MathUtils.abs(y)
    }

    /**
     * Subtract another vector from this one and return a new vector.
     *
     * @param v the vector to subtract
     * @return a new vector
     */
    fun sub(v: Vec2): Vec2 {
        // Create and return a new vector representing (this - v).
        return Vec2(x - v.x, y - v.y)
    }

    /**
     * Computes the dot product of this vector with another.
     * @param v The other vector.
     * @return The dot product.
     */
    fun dot(v: Vec2): Float {
        // The dot product is the sum of the products of corresponding components.
        return x * v.x + y * v.y
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
            // Create a new vector with absolute components of 'a'.
            return Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))
        }

        /**
         * Sets the output vector to the absolute values of the components of the given vector.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param a the vector to take the absolute values of
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun absToOut(a: Vec2, out: Vec2) {
            // Set out.x to absolute value of a.x.
            out.x = MathUtils.abs(a.x)
            // Set out.y to absolute value of a.y.
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
            // Calculate a.x * b.x + a.y * b.y.
            return a.x * b.x + a.y * b.y
        }

        /**
         * Computes the cross product of two vectors.
         * In 2D, this is a scalar representing the z-component of the 3D cross product.
         *
         * @param a the first vector
         * @param b the second vector
         * @return the cross product
         */
        @JvmStatic
        fun cross(a: Vec2, b: Vec2): Float {
            // Calculate determinant: a.x * b.y - a.y * b.x.
            return a.x * b.y - a.y * b.x
        }

        /**
         * Computes the cross product of a vector and a scalar.
         * This yields a vector perpendicular to 'a' scaled by 's'.
         *
         * @param a the vector
         * @param s the scalar
         * @return a new vector containing the result
         */
        @JvmStatic
        fun cross(a: Vec2, s: Float): Vec2 {
            // Result is (s * a.y, -s * a.x).
            return Vec2(s * a.y, -s * a.x)
        }

        /**
         * Computes the cross product of a vector and a scalar, and stores the result in the output vector.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param a the vector
         * @param s the scalar
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            // Calculate the new y component first to safe-guard against aliasing if a === out.
            // Wait, if a === out, then updating out.x first would corrupt a.x needed for out.y.
            // So we use a temp variable.
            val tempY = -s * a.x
            // Update out.x using the original a.y.
            out.x = s * a.y
            // Update out.y using the calculated tempY.
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
        @DelicateFizzixApi
        @JvmStatic
        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            // Assert that 'out' is not the same object as 'a'.
            assert(out !== a)
            // Directly assign x without temp variable.
            out.x = s * a.y
            // Directly assign y without temp variable.
            out.y = -s * a.x
        }

        /**
         * Computes the cross product of a scalar and a vector.
         * This yields a vector perpendicular to 'a' scaled by 's' (opposite direction to cross(a, s)).
         *
         * @param s the scalar
         * @param a the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun cross(s: Float, a: Vec2): Vec2 {
            // Result is (-s * a.y, s * a.x).
            return Vec2(-s * a.y, s * a.x)
        }

        /**
         * Computes the cross product of a scalar and a vector, and stores the result in the output vector.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param s the scalar
         * @param a the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            // Calculate temp y to handle potential aliasing.
            val tempY = s * a.x
            // Update out.x using original a.y.
            out.x = -s * a.y
            // Update out.y using tempY.
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
        @DelicateFizzixApi
        @JvmStatic
        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            // Assert that 'out' is not the same object as 'a'.
            assert(out !== a)
            // Directly assign x.
            out.x = -s * a.y
            // Directly assign y.
            out.y = s * a.x
        }

        /**
         * Sets the output vector to the negation of the input vector.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param a the vector to negate
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun negateToOut(a: Vec2, out: Vec2) {
            // Set out.x to negative a.x.
            out.x = -a.x
            // Set out.y to negative a.y.
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
            // Return new Vec2(min(ax, bx), min(ay, by)).
            return Vec2(MathUtils.min(a.x, b.x), MathUtils.min(a.y, b.y))
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
            // Return new Vec2(max(ax, bx), max(ay, by)).
            return Vec2(MathUtils.max(a.x, b.x), MathUtils.max(a.y, b.y))
        }

        /**
         * Sets the output vector to the minimum components of two vectors.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            // Update out.x to min(ax, bx).
            out.x = MathUtils.min(a.x, b.x)
            // Update out.y to min(ay, by).
            out.y = MathUtils.min(a.y, b.y)
        }

        /**
         * Sets the output vector to the maximum components of two vectors.
         * This is a performance-optimized version that avoids new object allocations.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            // Update out.x to max(ax, bx).
            out.x = MathUtils.max(a.x, b.x)
            // Update out.y to max(ay, by).
            out.y = MathUtils.max(a.y, b.y)
        }
    }
}
