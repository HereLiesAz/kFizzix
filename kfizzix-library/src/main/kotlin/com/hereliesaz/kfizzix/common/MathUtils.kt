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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 *
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package com.hereliesaz.kfizzix.common

import java.util.Random

/**
 * A collection of math utility functions.
 *
 * @author Daniel Murphy
 */
object MathUtils : PlatformMathUtils() {
    const val PI = Math.PI.toFloat()
    const val TWOPI = (Math.PI * 2).toFloat()
    const val INV_PI = 1f / PI
    const val HALF_PI = PI / 2
    const val QUARTER_PI = PI / 4
    const val THREE_HALVES_PI = TWOPI - HALF_PI

    /**
     * Degrees to radians conversion factor
     */
    const val DEG2RAD = PI / 180

    /**
     * Radians to degrees conversion factor
     */
    const val RAD2DEG = 180 / PI

    val sinLUT = FloatArray(Settings.SINCOS_LUT_LENGTH)

    init {
        for (i in 0 until Settings.SINCOS_LUT_LENGTH) {
            sinLUT[i] = Math.sin(i * Settings.SINCOS_LUT_PRECISION).toFloat()
        }
    }

    /**
     * Calculates the sine of an angle.
     *
     * @param x the angle in radians
     * @return the sine of the angle
     */
    fun sin(x: Float): Float {
        return if (Settings.SINCOS_LUT_ENABLED) {
            sinLUT(x)
        } else {
            StrictMath.sin(x.toDouble()).toFloat()
        }
    }

    /**
     * Calculates the sine of an angle using a lookup table.
     *
     * @param x the angle in radians
     * @return the sine of the angle
     */
    fun sinLUT(x: Float): Float {
        var x = x % TWOPI
        if (x < 0) {
            x += TWOPI
        }
        return if (Settings.SINCOS_LUT_LERP) {
            x /= Settings.SINCOS_LUT_PRECISION
            val index = x.toInt()
            val newX = if (index != 0) {
                x % index
            } else {
                x
            }
            // the next index is 0
            if (index == Settings.SINCOS_LUT_LENGTH - 1) {
                (1 - newX) * sinLUT[index] + newX * sinLUT[0]
            } else {
                (1 - newX) * sinLUT[index] + newX * sinLUT[index + 1]
            }
        } else {
            sinLUT[round(x / Settings.SINCOS_LUT_PRECISION) % Settings.SINCOS_LUT_LENGTH]
        }
    }

    /**
     * Calculates the cosine of an angle.
     *
     * @param x the angle in radians
     * @return the cosine of the angle
     */
    fun cos(x: Float): Float {
        return if (Settings.SINCOS_LUT_ENABLED) {
            sinLUT(HALF_PI - x)
        } else {
            StrictMath.cos(x.toDouble()).toFloat()
        }
    }

    /**
     * Returns the absolute value of a float.
     *
     * @param x the value
     * @return the absolute value
     */
    fun abs(x: Float): Float {
        return if (Settings.FAST_ABS) {
            if (x > 0) x else -x
        } else {
            StrictMath.abs(x)
        }
    }

    /**
     * Returns the absolute value of a float using a faster method.
     *
     * @param x the value
     * @return the absolute value
     */
    fun fastAbs(x: Float): Float {
        return if (x > 0) x else -x
    }

    /**
     * Returns the absolute value of an integer.
     *
     * @param x the value
     * @return the absolute value
     */
    fun abs(x: Int): Int {
        val y = x shr 31
        return (x xor y) - y
    }

    /**
     * Returns the floor of a float.
     *
     * @param x the value
     * @return the floor value
     */
    fun floor(x: Float): Int {
        return if (Settings.FAST_FLOOR) {
            fastFloor(x)
        } else {
            StrictMath.floor(x.toDouble()).toInt()
        }
    }

    /**
     * Returns the floor of a float using a faster method.
     *
     * @param x the value
     * @return the floor value
     */
    fun fastFloor(x: Float): Int {
        val y = x.toInt()
        return if (x < y) {
            y - 1
        } else y
    }

    /**
     * Returns the ceiling of a float.
     *
     * @param x the value
     * @return the ceiling value
     */
    fun ceil(x: Float): Int {
        return if (Settings.FAST_CEIL) {
            fastCeil(x)
        } else {
            StrictMath.ceil(x.toDouble()).toInt()
        }
    }

    /**
     * Returns the ceiling of a float using a faster method.
     *
     * @param x the value
     * @return the ceiling value
     */
    fun fastCeil(x: Float): Int {
        val y = x.toInt()
        return if (x > y) {
            y + 1
        } else y
    }

    /**
     * Rounds a float to the nearest integer.
     *
     * @param x the value
     * @return the rounded integer
     */
    fun round(x: Float): Int {
        return if (Settings.FAST_ROUND) {
            floor(x + .5f)
        } else {
            StrictMath.round(x)
        }
    }

    /**
     * Rounds up the value to the nearest higher power of 2.
     *
     * @return power of 2 value
     */
    fun ceilPowerOf2(x: Int): Int {
        var pow2 = 1
        while (pow2 < x) {
            pow2 = pow2 shl 1
        }
        return pow2
    }

    /**
     * Returns the maximum of two floats.
     *
     * @param a the first value
     * @param b the second value
     * @return the maximum value
     */
    fun max(a: Float, b: Float): Float {
        return Math.max(a, b)
    }

    /**
     * Returns the maximum of two integers.
     *
     * @param a the first value
     * @param b the second value
     * @return the maximum value
     */
    fun max(a: Int, b: Int): Int {
        return Math.max(a, b)
    }

    /**
     * Returns the minimum of two floats.
     *
     * @param a the first value
     * @param b the second value
     * @return the minimum value
     */
    fun min(a: Float, b: Float): Float {
        return Math.min(a, b)
    }

    /**
     * Returns the minimum of two integers.
     *
     * @param a the first value
     * @param b the second value
     * @return the minimum value
     */
    fun min(a: Int, b: Int): Int {
        return Math.min(a, b)
    }

    /**
     * Maps a value from one range to another.
     *
     * @param value the value to map
     * @param fromMin the minimum of the original range
     * @param fromMax the maximum of the original range
     * @param toMin the minimum of the new range
     * @param toMax the maximum of the new range
     * @return the mapped value
     */
    fun map(value: Float, fromMin: Float, fromMax: Float, toMin: Float, toMax: Float): Float {
        val mult = (value - fromMin) / (fromMax - fromMin)
        return toMin + mult * (toMax - toMin)
    }

    /**
     * Returns the closest value to 'a' that is in between 'low' and 'high'.
     *
     * @param a the value to clamp
     * @param low the lower bound
     * @param high the upper bound
     * @return the clamped value
     */
    fun clamp(a: Float, low: Float, high: Float): Float {
        return max(low, min(a, high))
    }

    /**
     * Clamps a vector to be within a given range.
     *
     * @param a the vector to clamp
     * @param low the lower bound
     * @param high the upper bound
     * @return a new vector with the clamped values
     */
    fun clamp(a: Vec2, low: Vec2, high: Vec2): Vec2 {
        val min = Vec2()
        min.x = Math.min(a.x, high.x)
        min.y = Math.min(a.y, high.y)
        min.x = Math.max(low.x, min.x)
        min.y = Math.max(low.y, min.y)
        return min
    }

    /**
     * Clamps a vector to be within a given range and stores the result in the output vector.
     *
     * @param a the vector to clamp
     * @param low the lower bound
     * @param high the upper bound
     * @param dest the vector to store the result in
     */
    fun clampToOut(a: Vec2, low: Vec2, high: Vec2, dest: Vec2) {
        dest.x = Math.min(a.x, high.x)
        dest.y = Math.min(a.y, high.y)
        dest.x = Math.max(low.x, dest.x)
        dest.y = Math.max(low.y, dest.y)
    }

    /**
     * Next Largest Power of 2: Given a binary integer value x, the next largest
     * power of 2 can be computed by a SWAR algorithm that recursively "folds"
     * the upper bits into the lower bits. This process yields a bit vector with
     * the same most significant 1 as x, but all 1's below it. Adding 1 to that
     * value yields the next largest power of 2.
     */
    fun nextPowerOfTwo(x: Int): Int {
        var x = x
        x = x or (x shr 1)
        x = x or (x shr 2)
        x = x or (x shr 4)
        x = x or (x shr 8)
        x = x or (x shr 16)
        return x + 1
    }

    /**
     * Checks if a number is a power of two.
     *
     * @param x the number to check
     * @return true if the number is a power of two
     */
    fun isPowerOfTwo(x: Int): Boolean {
        return x > 0 && x and x - 1 == 0
    }

    /**
     * Calculates the power of a number.
     *
     * @param a the base
     * @param b the exponent
     * @return `a` raised to the power of `b`
     */
    fun pow(a: Float, b: Float): Float {
        return if (Settings.FAST_POW) {
            fastPow(a, b)
        } else {
            StrictMath.pow(a.toDouble(), b.toDouble()).toFloat()
        }
    }

    /**
     * Calculates the arc tangent of a value.
     *
     * @param y the y-coordinate
     * @param x the x-coordinate
     * @return the arc tangent
     */
    fun atan2(y: Float, x: Float): Float {
        return if (Settings.FAST_ATAN2) {
            fastAtan2(y, x)
        } else {
            StrictMath.atan2(y.toDouble(), x.toDouble()).toFloat()
        }
    }

    /**
     * Calculates the arc tangent of a value using a faster method.
     *
     * @param y the y-coordinate
     * @param x the x-coordinate
     * @return the arc tangent
     */
    fun fastAtan2(y: Float, x: Float): Float {
        if (x == 0.0f) {
            if (y > 0.0f) return HALF_PI
            return if (y == 0.0f) 0.0f else -HALF_PI
        }
        val atan: Float
        val z = y / x
        if (abs(z) < 1.0f) {
            atan = z / (1.0f + 0.28f * z * z)
            if (x < 0.0f) {
                return if (y < 0.0f) atan - PI else atan + PI
            }
        } else {
            atan = HALF_PI - z / (z * z + 0.28f)
            if (y < 0.0f) return atan - PI
        }
        return atan
    }

    /**
     * Reduces an angle to be within the range of -PI to PI.
     *
     * @param theta the angle to reduce
     * @return the reduced angle
     */
    fun reduceAngle(theta: Float): Float {
        var theta = theta % TWOPI
        if (abs(theta) > PI) {
            theta -= TWOPI
        }
        if (abs(theta) > HALF_PI) {
            theta = PI - theta
        }
        return theta
    }

    /**
     * Generates a random float between two values.
     *
     * @param low the lower bound
     * @param high the upper bound
     * @return a random float
     */
    fun randomFloat(low: Float, high: Float): Float {
        return (Math.random() * (high - low) + low).toFloat()
    }

    /**
     * Generates a random float between two values using a given random number generator.
     *
     * @param r the random number generator
     * @param low the lower bound
     * @param high the upper bound
     * @return a random float
     */
    fun randomFloat(r: Random, low: Float, high: Float): Float {
        return r.nextFloat() * (high - low) + low
    }

    /**
     * Calculates the square root of a number.
     *
     * @param x the number
     * @return the square root
     */
    fun sqrt(x: Float): Float {
        return StrictMath.sqrt(x.toDouble()).toFloat()
    }

    /**
     * Calculates the squared distance between two vectors.
     *
     * @param v1 the first vector
     * @param v2 the second vector
     * @return the squared distance
     */
    fun distanceSquared(v1: Vec2, v2: Vec2): Float {
        val dx = v1.x - v2.x
        val dy = v1.y - v2.y
        return dx * dx + dy * dy
    }

    /**
     * Calculates the distance between two vectors.
     *
     * @param v1 the first vector
     * @param v2 the second vector
     * @return the distance
     */
    fun distance(v1: Vec2, v2: Vec2): Float {
        return sqrt(distanceSquared(v1, v2))
    }
}
