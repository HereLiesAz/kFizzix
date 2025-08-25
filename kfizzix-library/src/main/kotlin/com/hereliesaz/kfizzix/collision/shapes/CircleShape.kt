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
import com.hereliesaz.kfizzix.common.Rot
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
class CircleShape : Shape(ShapeType.CIRCLE) {
    val p: Vec2

    init {
        p = Vec2()
        radius = 0f
    }

    public override fun clone(): Shape {
        val shape = CircleShape()
        shape.p.x = p.x
        shape.p.y = p.y
        shape.radius = radius
        return shape
    }

    override val childCount: Int
        get() = 1

    /**
     * Get the supporting vertex index in the given direction.
     */
    fun getSupport(d: Vec2): Int {
        return 0
    }

    /**
     * Get the supporting vertex in the given direction.
     */
    fun getSupportVertex(d: Vec2): Vec2 {
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
        // Rot.mulToOutUnsafe(transform.q, p, center);
        // center.addLocal(transform.p);
        //
        // final Vec2 d = center.subLocal(p).negateLocal();
        // return Vec2.dot(d, d) <= radius * radius;
        val q = transform.q
        val tp = transform.p
        val centerx = -(q.c * this.p.x - q.s * this.p.y + tp.x - p.x)
        val centery = -(q.s * this.p.x + q.c * this.p.y + tp.y - p.y)
        return centerx * centerx + centery * centery <= radius * radius
    }

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val xfq = xf.q
        val centerx = xfq.c * this.p.x - xfq.s * this.p.y + xf.p.x
        val centery = xfq.s * this.p.x + xfq.c * this.p.y + xf.p.y
        val dx = p.x - centerx
        val dy = p.y - centery
        val d1 = MathUtils.sqrt(dx * dx + dy * dy)
        normalOut.x = dx * 1 / d1
        normalOut.y = dy * 1 / d1
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
        val inputP1 = input.p1
        val inputP2 = input.p2
        val tq = transform.q
        val tp = transform.p
        // Rot.mulToOutUnsafe(transform.q, p, position);
        // position.addLocal(transform.p);
        val positionX = tq.c * p.x - tq.s * p.y + tp.x
        val positionY = tq.s * p.x + tq.c * p.y + tp.y
        val sx = inputP1.x - positionX
        val sy = inputP1.y - positionY
        // final float b = Vec2.dot(s, s) - radius * radius;
        val b = sx * sx + sy * sy - radius * radius
        // Solve quadratic equation.
        val rx = inputP2.x - inputP1.x
        val ry = inputP2.y - inputP1.y
        // final float c = Vec2.dot(s, r);
        // final float rr = Vec2.dot(r, r);
        val c = sx * rx + sy * ry
        val rr = rx * rx + ry * ry
        val sigma = c * c - rr * b
        // Check for negative discriminant and short segment.
        if (sigma < 0.0f || rr < Settings.EPSILON) {
            return false
        }
        // Find the point of intersection of the line with the circle.
        var a = -(c + MathUtils.sqrt(sigma))
        // Is the intersection point on the segment?
        if (0.0f <= a && a <= input.maxFraction * rr) {
            a /= rr
            output.fraction = a
            output.normal.x = rx * a + sx
            output.normal.y = ry * a + sy
            output.normal.normalize()
            return true
        }
        return false
    }

    override fun computeAABB(aabb: AABB, transform: Transform, childIndex: Int) {
        val tq = transform.q
        val tp = transform.p
        val px = tq.c * p.x - tq.s * p.y + tp.x
        val py = tq.s * p.x + tq.c * p.y + tp.y
        aabb.lowerBound.x = px - radius
        aabb.lowerBound.y = py - radius
        aabb.upperBound.x = px + radius
        aabb.upperBound.y = py + radius
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = density * Settings.PI * radius * radius
        massData.center.x = p.x
        massData.center.y = p.y
        // inertia about the local origin
        // massData.I = massData.mass * (0.5f * radius * radius +
        // Vec2.dot(p, p));
        massData.i = massData.mass * (0.5f * radius * radius + (p.x * p.x + p.y * p.y))
    }
}
