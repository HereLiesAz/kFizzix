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
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2

/**
 * A shape is used for collision detection. You can create a shape however you
 * like. Shapes used for simulation in World are created automatically when a
 * Fixture is created. Shapes may encapsulate a one or more child shapes.
 *
 * @author Daniel Murphy
 */
abstract class Shape(val type: ShapeType) {
    /**
     * The radius of the underlying shape. This can refer to different things
     * depending on the shape implementation
     */
    var radius = 0f

    /**
     * Get the number of child primitives
     */
    abstract val childCount: Int

    /**
     * Test a point for containment in this shape. This only works for convex
     * shapes.
     *
     * @param xf The shape world transform.
     * @param p A point in world coordinates.
     */
    abstract fun testPoint(xf: Transform, p: Vec2): Boolean

    /**
     * Cast a ray against a child shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     * @param transform The transform to be applied to the shape.
     * @param childIndex The child shape index
     *
     * @return if hit
     */
    abstract fun raycast(
        output: RayCastOutput, input: RayCastInput,
        transform: Transform, childIndex: Int
    ): Boolean

    /**
     * Given a transform, compute the associated axis aligned bounding box for a
     * child shape.
     *
     * @param aabb Returns the axis aligned box.
     * @param xf The world transform of the shape.
     */
    abstract fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int)

    /**
     * Compute the mass properties of this shape using its dimensions and
     * density. The inertia tensor is computed about the local origin.
     *
     * @param massData Returns the mass data for this shape.
     * @param density The density in kilograms per meter squared.
     */
    abstract fun computeMass(massData: MassData, density: Float)

    /**
     * Compute the distance from the current shape to the specified point. This
     * only works for convex shapes.
     *
     * @param xf The shape world transform.
     * @param p A point in world coordinates.
     * @param normalOut Returns the direction in which the distance increases.
     *
     * @return distance returns the distance from the current shape.
     */
    abstract fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float
    abstract override fun clone(): Shape
}
