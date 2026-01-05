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
package com.hereliesaz.kfizzix.collision

import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2

/** An axis-aligned bounding box.  */
class AABB {
    /** Bottom left vertex of bounding box.  */
    val lowerBound = Vec2()

    /** Top right vertex of bounding box.  */
    val upperBound = Vec2()

    /**
     * Creates the default object, with vertices at 0,0 and 0,0.
     */
    constructor() {
        lowerBound.setZero()
        upperBound.setZero()
    }

    /**
     * Copies from the given object
     *
     * @param copy the object to copy from
     */
    constructor(copy: AABB) : this(copy.lowerBound, copy.upperBound)

    /**
     * Creates an AABB object using the given bounding vertices.
     *
     * @param lowerVertex the bottom left vertex of the bounding box
     * @param upperVertex the top right vertex of the bounding box
     */
    constructor(lowerVertex: Vec2, upperVertex: Vec2) {
        this.lowerBound.set(lowerVertex)
        this.upperBound.set(upperVertex)
    }

    /**
     * Sets this object from the given object
     *
     * @param aabb the object to copy from
     */
    fun set(aabb: AABB) {
        val v = aabb.lowerBound
        lowerBound.x = v.x
        lowerBound.y = v.y
        val v1 = aabb.upperBound
        upperBound.x = v1.x
        upperBound.y = v1.y
    }

    /** Verify that the bounds are sorted  */
    fun isValid(): Boolean {
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
     * Get the center of the AABB
     *
     * @return
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
     *
     * @return
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
     *
     * @param aabb1
     * @param aab
     */
    fun combine(aabb1: AABB, aab: AABB) {
        lowerBound.x = if (aabb1.lowerBound.x < aab.lowerBound.x) aabb1.lowerBound.x else aab.lowerBound.x
        lowerBound.y = if (aabb1.lowerBound.y < aab.lowerBound.y) aabb1.lowerBound.y else aab.lowerBound.y
        upperBound.x = if (aabb1.upperBound.x > aab.upperBound.x) aabb1.upperBound.x else aab.upperBound.x
        upperBound.y = if (aabb1.upperBound.y > aab.upperBound.y) aabb1.upperBound.y else aab.upperBound.y
    }

    /**
     * Gets the perimeter length
     *
     * @return
     */
    val perimeter: Float
        get() = 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y)

    /**
     * Combines another aabb with this one
     *
     * @param aabb
     */
    fun combine(aabb: AABB) {
        lowerBound.x = if (lowerBound.x < aabb.lowerBound.x) lowerBound.x else aabb.lowerBound.x
        lowerBound.y = if (lowerBound.y < aabb.lowerBound.y) lowerBound.y else aabb.lowerBound.y
        upperBound.x = if (upperBound.x > aabb.upperBound.x) upperBound.x else aabb.upperBound.x
        upperBound.y = if (upperBound.y > aabb.upperBound.y) upperBound.y else aabb.upperBound.y
    }

    /**
     * Does this aabb contain the provided AABB.
     *
     * @return
     */
    fun contains(aabb: AABB): Boolean {
        /*
		 * boolean result = true; result = result && lowerBound.x <= aabb.lowerBound.x; result = result
		 * && lowerBound.y <= aabb.lowerBound.y; result = result && aabb.upperBound.x <= upperBound.x;
		 * result = result && aabb.upperBound.y <= upperBound.y; return result;
		 */
        // djm: faster, optimized
        return lowerBound.x <= aabb.lowerBound.x && lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= upperBound.x && aabb.upperBound.y <= upperBound.y
    }

    /**
     * @deprecated use contains
     * @param aabb
     * @return
     */
    @Deprecated("use contains")
    fun containsFast(aabb: AABB): Boolean {
        return contains(aabb)
    }

    fun raycast(output: RayCastOutput, input: RayCastInput): Boolean {
        var tmin = -Float.MAX_VALUE
        var tmax = Float.MAX_VALUE
        val p = input.p1
        val d = input.p2.sub(input.p1)
        val absD = MathUtils.abs(d)
        val normal = output.normal
        if (absD.x < Settings.EPSILON) {
            // Parallel.
            if (p.x < lowerBound.x || upperBound.x < p.x) {
                return false
            }
        } else {
            val invD = 1.0f / d.x
            var t1 = (lowerBound.x - p.x) * invD
            var t2 = (upperBound.x - p.x) * invD

            // Sign of the normal vector.
            var s = -1.0f
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
                s = 1.0f
            }

            // Push the min up
            if (t1 > tmin) {
                normal.setZero()
                normal.x = s
                tmin = t1
            }

            // Pull the max down
            tmax = MathUtils.min(tmax, t2)
            if (tmin > tmax) {
                return false
            }
        }
        if (absD.y < Settings.EPSILON) {
            // Parallel.
            if (p.y < lowerBound.y || upperBound.y < p.y) {
                return false
            }
        } else {
            val invD = 1.0f / d.y
            var t1 = (lowerBound.y - p.y) * invD
            var t2 = (upperBound.y - p.y) * invD

            // Sign of the normal vector.
            var s = -1.0f
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
                s = 1.0f
            }

            // Push the min up
            if (t1 > tmin) {
                normal.setZero()
                normal.y = s
                tmin = t1
            }

            // Pull the max down
            tmax = MathUtils.min(tmax, t2)
            if (tmin > tmax) {
                return false
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0.0f || input.maxFraction < tmin) {
            return false
        }

        // Intersection.
        output.fraction = tmin
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
            if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f) {
                return false
            }
            return true
        }

        fun combine(aabb1: AABB, aabb2: AABB, out: AABB) {
            out.combine(aabb1, aabb2)
        }
    }
}
