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
 * ARISING IN ANY WAY OUT OF THE USE OF this SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.collision

import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.pooling.WorldPool
import com.hereliesaz.kfizzix.pooling.normal.DefaultWorldPool

/**
 * An axis-aligned bounding box.
 */
class AABB(
    /** The bottom left vertex of the bounding box. */
    val lowerBound: Vec2 = Vec2(),
    /** The top right vertex of the bounding box. */
    val upperBound: Vec2 = Vec2()
) {
    /**
     * Verify that the bounds are sorted.
     */
    val isValid: Boolean
        get() = lowerBound.isValid && upperBound.isValid && (upperBound - lowerBound).let { it.x >= 0 && it.y >= 0 }

    /**
     * Get the center of the AABB.
     */
    val center: Vec2
        get() = (lowerBound + upperBound) * 0.5f

    /**
     * Get the extents of the AABB (half-widths).
     */
    val extents: Vec2
        get() = (upperBound - lowerBound) * 0.5f

    /**
     * Gets the perimeter length.
     */
    val perimeter: Float
        get() = 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y)

    /**
     * Combines another aabb with this one, returning a new AABB.
     */
    fun combine(aabb: AABB): AABB {
        return combine(this, aabb)
    }

    /**
     * Does this aabb contain the provided AABB.
     */
    operator fun contains(aabb: AABB): Boolean {
        return lowerBound.x <= aabb.lowerBound.x &&
                lowerBound.y <= aabb.lowerBound.y &&
                aabb.upperBound.x <= upperBound.x &&
                aabb.upperBound.y <= upperBound.y
    }

    /**
     * From Real-time Collision Detection, p179.
     */
    fun raycast(output: RayCastOutput, input: RayCastInput, argPool: WorldPool = DefaultWorldPool(4, 4)): Boolean {
        var tMin = -Float.MAX_VALUE
        var tMax = Float.MAX_VALUE

        val p = input.p1
        val d = argPool.popVec2()
        val absD = argPool.popVec2()
        val normal = argPool.popVec2()

        d.set(input.p2).subLocal(input.p1)
        Vec2.absToOut(d, absD)

        try {
            // x then y
            if (absD.x < Settings.EPSILON) {
                // Parallel.
                if (p.x < lowerBound.x || upperBound.x < p.x) {
                    return false
                }
            } else {
                val inv_d = 1.0f / d.x
                var t1 = (lowerBound.x - p.x) * inv_d
                var t2 = (upperBound.x - p.x) * inv_d

                // Sign of the normal vector.
                var s = -1.0f
                if (t1 > t2) {
                    val temp = t1
                    t1 = t2
                    t2 = temp
                    s = 1.0f
                }

                // Push the min up
                if (t1 > tMin) {
                    normal.set(s, 0.0f)
                    tMin = t1
                }

                // Pull the max down
                tMax = minOf(tMax, t2)

                if (tMin > tMax) {
                    return false
                }
            }

            if (absD.y < Settings.EPSILON) {
                // Parallel.
                if (p.y < lowerBound.y || upperBound.y < p.y) {
                    return false
                }
            } else {
                val inv_d = 1.0f / d.y
                var t1 = (lowerBound.y - p.y) * inv_d
                var t2 = (upperBound.y - p.y) * inv_d

                // Sign of the normal vector.
                var s = -1.0f
                if (t1 > t2) {
                    val temp = t1
                    t1 = t2
                    t2 = temp
                    s = 1.0f
                }

                // Push the min up
                if (t1 > tMin) {
                    normal.set(0.0f, s)
                    tMin = t1
                }

                // Pull the max down
                tMax = minOf(tMax, t2)

                if (tMin > tMax) {
                    return false
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tMin < 0.0f || input.maxFraction < tMin) {
                return false
            }

            // Intersection.
            output.fraction = tMin
            output.normal.set(normal)
            return true
        } finally {
            argPool.pushVec2(3)
        }
    }

    override fun toString(): String {
        return "AABB(lowerBound=$lowerBound, upperBound=$upperBound)"
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as AABB
        if (lowerBound != other.lowerBound) return false
        if (upperBound != other.upperBound) return false
        return true
    }

    override fun hashCode(): Int {
        var result = lowerBound.hashCode()
        result = 31 * result + upperBound.hashCode()
        return result
    }

    companion object {
        fun testOverlap(a: AABB, b: AABB): Boolean {
            if (b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y) {
                return false
            }
            if (a.lowerBound.x > b.upperBound.x || a.lowerBound.y > b.upperBound.y) {
                return false
            }
            return true
        }

        fun combine(aabb1: AABB, aabb2: AABB): AABB {
            val lowerBoundX = minOf(aabb1.lowerBound.x, aabb2.lowerBound.x)
            val lowerBoundY = minOf(aabb1.lowerBound.y, aabb2.lowerBound.y)
            val upperBoundX = maxOf(aabb1.upperBound.x, aabb2.upperBound.x)
            val upperBoundY = maxOf(aabb1.upperBound.y, aabb2.upperBound.y)
            return AABB(Vec2(lowerBoundX, lowerBoundY), Vec2(upperBoundX, upperBoundY))
        }

        fun combine(aabb1: AABB, aabb2: AABB, out: AABB) {
            out.lowerBound.x = minOf(aabb1.lowerBound.x, aabb2.lowerBound.x)
            out.lowerBound.y = minOf(aabb1.lowerBound.y, aabb2.lowerBound.y)
            out.upperBound.x = maxOf(aabb1.upperBound.x, aabb2.upperBound.x)
            out.upperBound.y = maxOf(aabb1.upperBound.y, aabb2.upperBound.y)
        }
    }
}
