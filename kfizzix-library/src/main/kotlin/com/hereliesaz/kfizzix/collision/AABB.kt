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
 *
 * @author Daniel Murphy
 */
class AABB {
    /**
     * The bottom left vertex of the bounding box.
     */
    val lowerBound: Vec2

    /**
     * The top right vertex of the bounding box.
     */
    val upperBound: Vec2

    /**
     * Creates the default object, with vertices at 0,0 and 0,0.
     */
    constructor() {
        lowerBound = Vec2()
        upperBound = Vec2()
    }

    /**
     * Copies from the given object.
     *
     * @param copy The object to copy from.
     */
    constructor(copy: AABB) : this(copy.lowerBound, copy.upperBound) {}

    /**
     * Creates an AABB object using the given bounding vertices.
     *
     * @param lowerVertex The bottom left vertex of the bounding box.
     * @param upperVertex The top right vertex of the bounding box.
     */
    constructor(lowerVertex: Vec2, upperVertex: Vec2) {
        lowerBound = lowerVertex.clone() // clone to be safe
        upperBound = upperVertex.clone()
    }

    /**
     * Sets this object from the given object.
     *
     * @param aabb The object to copy from.
     */
    fun set(aabb: AABB) {
        val v = aabb.lowerBound
        lowerBound.x = v.x
        lowerBound.y = v.y
        val v1 = aabb.upperBound
        upperBound.x = v1.x
        upperBound.y = v1.y
    }

    /**
     * Verify that the bounds are sorted.
     */
    val isValid: Boolean
        get() {
            val dx = upperBound.x - lowerBound.x
            if (dx < 0f) {
                return false
            }
            val dy = upperBound.y - lowerBound.y
            if (dy < 0) {
                return false
            }
            return lowerBound.isValid && upperBound.isValid
        }

    /**
     * Get the center of the AABB.
     */
    val center: Vec2
        get() {
            val center = Vec2(lowerBound)
            center.addLocal(upperBound)
            center.mulLocal(.5f)
            return center
        }

    fun getCenterToOut(out: Vec2) {
        out.x = (lowerBound.x + upperBound.x) * .5f
        out.y = (lowerBound.y + upperBound.y) * .5f
    }

    /**
     * Get the extents of the AABB (half-widths).
     */
    val extents: Vec2
        get() {
            val center = Vec2(upperBound)
            center.subLocal(lowerBound)
            center.mulLocal(.5f)
            return center
        }

    fun getExtentsToOut(out: Vec2) {
        out.x = (upperBound.x - lowerBound.x) * .5f
        out.y = (upperBound.y - lowerBound.y) * .5f // thanks FDN1
    }

    fun getVertices(argRay: Array<Vec2>) {
        argRay[0].set(lowerBound)
        argRay[1].set(lowerBound)
        argRay[1].x += upperBound.x - lowerBound.x
        argRay[2].set(upperBound)
        argRay[3].set(upperBound)
        argRay[3].x -= upperBound.x - lowerBound.x
    }

    /**
     * Combine two AABBs into this one.
     */
    fun combine(aabb1: AABB, aab: AABB) {
        lowerBound.x = Math.min(aabb1.lowerBound.x, aab.lowerBound.x)
        lowerBound.y = Math.min(aabb1.lowerBound.y, aab.lowerBound.y)
        upperBound.x = Math.max(aabb1.upperBound.x, aab.upperBound.x)
        upperBound.y = Math.max(aabb1.upperBound.y, aab.upperBound.y)
    }

    /**
     * Gets the perimeter length.
     */
    val perimeter: Float
        get() = 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y)

    /**
     * Combines another aabb with this one.
     */
    fun combine(aabb: AABB) {
        lowerBound.x = Math.min(lowerBound.x, aabb.lowerBound.x)
        lowerBound.y = Math.min(lowerBound.y, aabb.lowerBound.y)
        upperBound.x = Math.max(upperBound.x, aabb.upperBound.x)
        upperBound.y = Math.max(upperBound.y, aabb.upperBound.y)
    }

    /**
     * Does this aabb contain the provided AABB.
     */
    fun contains(aabb: AABB): Boolean {
        /*
     * boolean result = true; result = result && lowerBound.x <=
     * aabb.lowerBound.x; result = result && lowerBound.y <=
     * aabb.lowerBound.y; result = result && aabb.upperBound.x <=
     * upperBound.x; result = result && aabb.upperBound.y <= upperBound.y;
     * return result;
     */
        // djm: faster putting all of them together, as if one is false we leave
        // the logic
        // early
        return lowerBound.x <= aabb.lowerBound.x && lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= upperBound.x && aabb.upperBound.y <= upperBound.y
    }

    /**
     * @deprecated please use
     * [.raycast] for better
     * performance
     */
    fun raycast(output: RayCastOutput, input: RayCastInput): Boolean {
        return raycast(output, input, DefaultWorldPool(4, 4))
    }

    /**
     * From Real-time Collision Detection, p179.
     */
    fun raycast(output: RayCastOutput, input: RayCastInput, argPool: WorldPool): Boolean {
        var tMin = -Float.MAX_VALUE
        var tMax = Float.MAX_VALUE
        val p = argPool.popVec2()
        val d = argPool.popVec2()
        val absD = argPool.popVec2()
        val normal = argPool.popVec2()
        p.set(input.p1)
        d.set(input.p2).subLocal(input.p1)
        Vec2.absToOut(d, absD)
        // x then y
        if (absD.x < Settings.EPSILON) {
            // Parallel.
            if (p.x < lowerBound.x || upperBound.x < p.x) {
                argPool.pushVec2(4)
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
                normal.setZero()
                normal.x = s
                tMin = t1
            }
            // Pull the max down
            tMax = MathUtils.min(tMax, t2)
            if (tMin > tMax) {
                argPool.pushVec2(4)
                return false
            }
        }
        if (absD.y < Settings.EPSILON) {
            // Parallel.
            if (p.y < lowerBound.y || upperBound.y < p.y) {
                argPool.pushVec2(4)
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
                normal.setZero()
                normal.y = s
                tMin = t1
            }
            // Pull the max down
            tMax = MathUtils.min(tMax, t2)
            if (tMin > tMax) {
                argPool.pushVec2(4)
                return false
            }
        }
        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tMin < 0.0f || input.maxFraction < tMin) {
            argPool.pushVec2(4)
            return false
        }
        // Intersection.
        output.fraction = tMin
        output.normal.x = normal.x
        output.normal.y = normal.y
        argPool.pushVec2(4)
        return true
    }

    override fun toString(): String {
        return "AABB[$lowerBound . $upperBound]"
    }

    companion object {
        fun testOverlap(a: AABB, b: AABB): Boolean {
            if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
                return false
            }
            return !(a.lowerBound.x - b.upperBound.x > 0.0f) && !(a.lowerBound.y - b.upperBound.y > 0.0f)
        }
    }
}
