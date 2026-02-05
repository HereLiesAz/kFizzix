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
 * A line segment (edge) shape. These can be connected in chains or loops to
 * other edge shapes. The connectivity information is used to ensure correct
 * contact normals.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/collision/b2_edge_shape.cpp
 *
 * @author Daniel Murphy
 */
class EdgeShape : Shape(ShapeType.EDGE) {
    /**
     * edge vertex 1
     */
    val vertex1 = Vec2()

    /**
     * edge vertex 2
     */
    val vertex2 = Vec2()

    /**
     * optional adjacent vertex 1. Used for smooth collision
     */
    val vertex0 = Vec2()

    /**
     * optional adjacent vertex 2. Used for smooth collision
     */
    val vertex3 = Vec2()
    var hasVertex0 = false
    var hasVertex3 = false

    init {
        radius = Settings.polygonRadius
    }

    override val childCount: Int
        get() = 1

    fun set(v1: Vec2, v2: Vec2) {
        vertex1.set(v1)
        vertex2.set(v2)
        hasVertex3 = false
        hasVertex0 = hasVertex3
    }

    override fun testPoint(xf: Transform, p: Vec2): Boolean {
        return false
    }

    // for pooling
    private val normal = Vec2()
    fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val v1 = xf.mul(vertex1)
        val v2 = xf.mul(vertex2)

        var d = p - v1
        val s = v2 - v1
        val ds = d.dot(s)
        if (ds > 0) {
            val s2 = s.dot(s)
            if (ds > s2) {
                d = p - v2
            } else {
                d.subLocal(s.mulLocal(ds / s2))
            }
        }

        val d1 = d.length()
        if (d1 > Settings.EPSILON) {
            normalOut.set(d).mulLocal(1 / d1)
        } else {
            normalOut.setZero()
        }
        return d1
    }

    // p = p1 + t * d
    // v = v1 + s * e
    // p1 + t * d = v1 + s * e
    // s * e - t * d = p1 - v1
    override fun raycast(
        output: RayCastOutput, input: RayCastInput,
        transform: Transform, childIndex: Int
    ): Boolean {
        val xf = transform
        // Put the ray into the edge's frame of reference.
        val p1 = xf.q.mulTrans(input.p1 - xf.p)
        val p2 = xf.q.mulTrans(input.p2 - xf.p)
        val d = p2 - p1

        val v1 = vertex1
        val v2 = vertex2
        val e = v2 - v1
        normal.set(e.y, -e.x)
        normal.normalize()

        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        val numerator = normal.dot(v1 - p1)
        val denominator = normal.dot(d)

        if (denominator == 0.0f) {
            return false
        }

        val t = numerator / denominator
        if (t < 0.0f || t > 1.0f) {
            return false
        }

        val q = p1 + d * t

        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        val r = v2 - v1
        val rr = r.dot(r)
        if (rr == 0.0f) {
            return false
        }

        val s = (q - v1).dot(r) / rr
        if (s < 0.0f || s > 1.0f) {
            return false
        }

        output.fraction = t
        if (numerator > 0.0f) {
            output.normal.set(xf.q.mul(normal)).negateLocal()
        } else {
            output.normal.set(xf.q.mul(normal))
        }
        return true
    }

    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        val v1 = xf.mul(vertex1)
        val v2 = xf.mul(vertex2)

        aabb.lowerBound.x = minOf(v1.x, v2.x) - radius
        aabb.lowerBound.y = minOf(v1.y, v2.y) - radius
        aabb.upperBound.x = maxOf(v1.x, v2.x) + radius
        aabb.upperBound.y = maxOf(v1.y, v2.y) + radius
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = 0.0f
        massData.center.set(vertex1).addLocal(vertex2).mulLocal(0.5f)
        massData.i = 0.0f
    }

    public override fun clone(): Shape {
        val edge = EdgeShape()
        edge.radius = radius
        edge.hasVertex0 = hasVertex0
        edge.hasVertex3 = hasVertex3
        edge.vertex0.set(vertex0)
        edge.vertex1.set(vertex1)
        edge.vertex2.set(vertex2)
        edge.vertex3.set(vertex3)
        return edge
    }
}
