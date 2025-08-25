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
 * A 3D column vector.
 * This class represents a 3D vector with x, y, and z components.
 *
 * @param x the x-component of the vector, defaults to 0.
 * @param y the y-component of the vector, defaults to 0.
 * @param z the z-component of the vector, defaults to 0.
 * @constructor Creates a new vector with the given components.
 * @author Daniel Murphy
 */
data class Vec3(
    @JvmField var x: Float = 0f,
    @JvmField var y: Float = 0f,
    @JvmField var z: Float = 0f
) : Serializable {

    /**
     * Creates a new vector by copying the values from another vector.
     *
     * @param toCopy the vector to copy from
     */
    constructor(toCopy: Vec3) : this(toCopy.x, toCopy.y, toCopy.z)

    /**
     * Sets this vector to the given values.
     *
     * @param x the new x-component
     * @param y the new y-component
     * @param z the new z-component
     * @return this vector for chaining
     */
    fun set(x: Float, y: Float, z: Float): Vec3 {
        this.x = x
        this.y = y
        this.z = z
        return this
    }

    /**
     * Sets this vector to be a copy of another vector.
     *
     * @param vec the vector to copy from
     * @return this vector for chaining
     */
    fun set(vec: Vec3): Vec3 {
        x = vec.x
        y = vec.y
        z = vec.z
        return this
    }

    /**
     * Adds another vector to this one and returns the result.
     * This operation modifies this vector.
     *
     * @param argVec the vector to add
     * @return this vector for chaining
     */
    fun addLocal(argVec: Vec3): Vec3 {
        x += argVec.x
        y += argVec.y
        z += argVec.z
        return this
    }

    /**
     * Subtracts another vector from this one and returns the result.
     * This operation modifies this vector.
     *
     * @param argVec the vector to subtract
     * @return this vector for chaining
     */
    fun subLocal(argVec: Vec3): Vec3 {
        x -= argVec.x
        y -= argVec.y
        z -= argVec.z
        return this
    }

    /**
     * Multiplies this vector by a scalar and returns the result.
     * This operation modifies this vector.
     *
     * @param argScalar the scalar to multiply by
     * @return this vector for chaining
     */
    fun mulLocal(argScalar: Float): Vec3 {
        x *= argScalar
        y *= argScalar
        z *= argScalar
        return this
    }

    /**
     * Negates this vector and returns the result.
     * This operation modifies this vector.
     *
     * @return this vector for chaining
     */
    fun negateLocal(): Vec3 {
        x = -x
        y = -y
        z = -z
        return this
    }

    /**
     * Sets this vector to zero.
     */
    fun setZero() {
        x = 0f
        y = 0f
        z = 0f
    }

    /**
     * Returns a new vector that is the sum of this vector and another.
     * This operation does not modify this vector.
     */
    operator fun plus(argVec: Vec3): Vec3 {
        return Vec3(x + argVec.x, y + argVec.y, z + argVec.z)
    }

    /**
     * Returns a new vector that is the difference of this vector and another.
     * This operation does not modify this vector.
     */
    operator fun minus(argVec: Vec3): Vec3 {
        return Vec3(x - argVec.x, y - argVec.y, z - argVec.z)
    }

    /**
     * Returns a new vector that is this vector multiplied by a scalar.
     * This operation does not modify this vector.
     */
    operator fun times(argScalar: Float): Vec3 {
        return Vec3(x * argScalar, y * argScalar, z * argScalar)
    }

    /**
     * Returns a new vector that is the negation of this vector.
     * This operation does not modify this vector.
     */
    operator fun unaryMinus(): Vec3 {
        return Vec3(-x, -y, -z)
    }

    companion object {
        private const val serialVersionUID = 1L

        /**
         * Computes the dot product of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return the dot product
         */
        @JvmStatic
        fun dot(a: Vec3, b: Vec3): Float {
            return a.x * b.x + a.y * b.y + a.z * b.z
        }

        /**
         * Computes the cross product of two vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @return a new vector containing the cross product
         */
        @JvmStatic
        fun cross(a: Vec3, b: Vec3): Vec3 {
            return Vec3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            )
        }

        /**
         * Computes the cross product of two vectors and stores the result in the output vector.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOut(a: Vec3, b: Vec3, out: Vec3) {
            val tempY = a.z * b.x - a.x * b.z
            val tempZ = a.x * b.y - a.y * b.x
            out.x = a.y * b.z - a.z * b.y
            out.y = tempY
            out.z = tempZ
        }

        /**
         * Computes the cross product of two vectors and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vectors.
         *
         * @param a the first vector
         * @param b the second vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun crossToOutUnsafe(a: Vec3, b: Vec3, out: Vec3) {
            assert(out !== b)
            assert(out !== a)
            out.x = a.y * b.z - a.z * b.y
            out.y = a.z * b.x - a.x * b.z
            out.z = a.x * b.y - a.y * b.x
        }
    }
}
