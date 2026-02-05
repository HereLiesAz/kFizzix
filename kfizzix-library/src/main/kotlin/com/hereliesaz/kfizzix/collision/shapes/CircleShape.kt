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
package com.hereliesaz.kfizzix.collision.shapes

import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2

/**
 * A circle shape.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/collision/b2_collide_circle.cpp
 *
 * @author Daniel Murphy
 */
class CircleShape(
    // The center of the circle in local coordinates.
    val p: Vec2 = Vec2()
) : Shape(ShapeType.CIRCLE) {

    // Creates a copy of the circle shape.
    fun copy(p: Vec2 = this.p): CircleShape {
        val copy = CircleShape(p.copy())
        copy.radius = radius
        return copy
    }

    // Clones the shape.
    override fun clone(): Shape {
        val shape = copy()
        shape.radius = radius
        return shape
    }

    // Checks for equality.
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as CircleShape
        // Check center position.
        if (p != other.p) return false
        // Check radius (from base class).
        if (radius != other.radius) return false
        return true
    }

    // Computes hash code.
    override fun hashCode(): Int {
        var result = p.hashCode()
        result = 31 * result + radius.hashCode()
        return result
    }

    override fun toString(): String {
        return "CircleShape(p=$p, radius=$radius)"
    }

    // A circle has one child (itself).
    override val childCount: Int
        get() = 1

    /**
     * Get the supporting vertex index in the given direction.
     * For a circle, this concept is ambiguous, so return 0.
     */
    fun getSupport(@Suppress("UNUSED_PARAMETER") d: Vec2): Int {
        return 0
    }

    /**
     * Get the supporting vertex in the given direction.
     * For a circle, this is always the center. The actual support point is calculated elsewhere.
     */
    fun getSupportVertex(@Suppress("UNUSED_PARAMETER") d: Vec2): Vec2 {
        return p
    }

    /**
     * Get the vertex count. Circles have one "vertex" (the center).
     */
    val vertexCount: Int
        get() = 1

    /**
     * Get a vertex by index.
     */
    fun getVertex(index: Int): Vec2 {
        assert(index == 0)
        return p
    }

    // Test if a point is inside the circle.
    override fun testPoint(xf: Transform, p: Vec2): Boolean {
        // Transform local center to world coordinates.
        val center = xf.mul(this.p)
        // Vector from center to point.
        val d = p - center
        // Check if length squared is less than radius squared.
        return d.dot(d) <= radius * radius
    }

    // Compute the distance from the circle's edge to a point.
    fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        // Transform local center to world.
        val center = xf.mul(this.p)
        // Vector from center to point.
        val d = p - center
        // Distance from center.
        val d1 = d.length()
        // If distance is significant, normalize d to get normal.
        if (d1 > Settings.EPSILON) {
            normalOut.set(d).mulLocal(1 / d1)
        } else {
            // Otherwise, zero out normal (point is at center).
            normalOut.set(0f, 0f)
        }
        // Return signed distance (negative if inside).
        return d1 - radius
    }

    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.1.2
    // x = s + a * r
    // norm(x) = radius
    override fun raycast(
        output: RayCastOutput, input: RayCastInput,
        transform: Transform, childIndex: Int
    ): Boolean {
        // Transform local center to world.
        val position = transform.mul(p)
        // Vector from circle center to ray start.
        val s = input.p1 - position
        // b = dot(s, s) - r^2
        val b = s.dot(s) - radius * radius

        // Solve quadratic equation.
        // Ray direction vector.
        val r = input.p2 - input.p1
        val c = s.dot(r)
        val rr = r.dot(r)
        val sigma = c * c - rr * b

        // Check for negative discriminant and short segment.
        if (sigma < 0.0f || rr < Settings.EPSILON) {
            return false
        }

        // Find the point of intersection of the line with the circle.
        var a = -(c + MathUtils.sqrt(sigma))

        // Is the intersection point on the segment?
        if (a in 0.0f..(input.maxFraction * rr)) {
            a /= rr
            output.fraction = a
            // Normal is vector from center to intersection point.
            output.normal.set(r).mulLocal(a).addLocal(s)
            output.normal.normalize()
            return true
        }
        return false
    }

    // Compute AABB.
    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        // Transform center.
        val center = xf.mul(p)
        // Expand by radius.
        aabb.lowerBound.x = center.x - radius
        aabb.lowerBound.y = center.y - radius
        aabb.upperBound.x = center.x + radius
        aabb.upperBound.y = center.y + radius
    }

    // Compute mass properties.
    override fun computeMass(massData: MassData, density: Float) {
        // Mass = density * Area
        massData.mass = density * Settings.PI * radius * radius
        // Center is the geometric center.
        massData.center.set(p)
        // Inertia about the local origin.
        // I = mass * (0.5 * r^2 + offset^2) (Parallel Axis Theorem)
        massData.i = massData.mass * (0.5f * radius * radius + p.dot(p))
    }
}
