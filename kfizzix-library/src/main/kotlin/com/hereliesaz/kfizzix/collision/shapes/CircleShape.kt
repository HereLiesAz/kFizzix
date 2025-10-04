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
data class CircleShape(val p: Vec2 = Vec2()) : Shape(ShapeType.CIRCLE) {

    public override fun clone(): Shape {
        val shape = copy()
        shape.radius = radius
        return shape
    }

    override val childCount: Int
        get() = 1

    /**
     * Get the supporting vertex index in the given direction.
     */
    fun getSupport(@Suppress("UNUSED_PARAMETER") d: Vec2): Int {
        return 0
    }

    /**
     * Get the supporting vertex in the given direction.
     */
    fun getSupportVertex(@Suppress("UNUSED_PARAMETER") d: Vec2): Vec2 {
        return p
    }

    /**
     * Get the vertex count.
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

    override fun testPoint(transform: Transform, p: Vec2): Boolean {
        val center = transform.mul(this.p)
        val d = p - center
        return d.dot(d) <= radius * radius
    }

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val center = xf.mul(this.p)
        val d = p - center
        val d1 = d.length()
        if (d1 > Settings.EPSILON) {
            normalOut.set(d).mulLocal(1 / d1)
        } else {
            normalOut.set(0f, 0f)
        }
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
        val position = transform.mul(p)
        val s = input.p1 - position
        val b = s.dot(s) - radius * radius

        // Solve quadratic equation.
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
            output.normal.set(r).mulLocal(a).addLocal(s)
            output.normal.normalize()
            return true
        }
        return false
    }

    override fun computeAABB(aabb: AABB, transform: Transform, childIndex: Int) {
        val center = transform.mul(p)
        aabb.lowerBound.x = center.x - radius
        aabb.lowerBound.y = center.y - radius
        aabb.upperBound.x = center.x + radius
        aabb.upperBound.y = center.y + radius
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = density * Settings.PI * radius * radius
        massData.center.set(p)
        // inertia about the local origin
        massData.i = massData.mass * (0.5f * radius * radius + p.dot(p))
    }
}