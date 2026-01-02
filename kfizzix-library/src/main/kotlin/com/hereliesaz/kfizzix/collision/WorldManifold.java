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
 * ARISING IN ANY WAY OUT of the use of this SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.collision

import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2

/**
 * This is used to compute the current state of a contact manifold.
 *
 * @author Daniel Murphy
 */
class WorldManifold {
    /**
     * The world vector pointing from A to B.
     */
    val normal: Vec2 = Vec2()

    /**
     * The world contact point (point of intersection).
     */
    val points: Array<Vec2> = Array(Settings.maxManifoldPoints) { Vec2() }

    /**
     * A negative value indicates overlap, in meters.
     */
    val separations: FloatArray = FloatArray(Settings.maxManifoldPoints)


    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/collision/b2_collision.cpp#L26-L90
     */
    fun initialize(
        manifold: Manifold, xfA: Transform,
        radiusA: Float, xfB: Transform, radiusB: Float
    ) {
        if (manifold.pointCount == 0) {
            return
        }
        when (manifold.type) {
            Manifold.ManifoldType.CIRCLES -> {
                normal.set(1f, 0f)
                val pointA = xfA.mul(manifold.localPoint)
                val pointB = xfB.mul(manifold.points[0].localPoint)

                if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
                    normal.set(pointB).subLocal(pointA)
                    normal.normalize()
                }

                val cA = pointA + normal * radiusA
                val cB = pointB - normal * radiusB
                points[0].set(cA).addLocal(cB).mulLocal(0.5f)
                separations[0] = (cB - cA).dot(normal)
            }
            Manifold.ManifoldType.FACE_A -> {
                normal.set(xfA.q.mul(manifold.localNormal))
                val planePoint = xfA.mul(manifold.localPoint)

                for (i in 0 until manifold.pointCount) {
                    val clipPoint = xfB.mul(manifold.points[i].localPoint)
                    val scalar = radiusA - (clipPoint - planePoint).dot(normal)

                    val cA = clipPoint + normal * scalar
                    val cB = clipPoint - normal * radiusB
                    points[i].set(cA).addLocal(cB).mulLocal(0.5f)
                    separations[i] = (cB - cA).dot(normal)
                }
            }
            Manifold.ManifoldType.FACE_B -> {
                normal.set(xfB.q.mul(manifold.localNormal))
                val planePoint = xfB.mul(manifold.localPoint)

                for (i in 0 until manifold.pointCount) {
                    val clipPoint = xfA.mul(manifold.points[i].localPoint)
                    val scalar = radiusB - (clipPoint - planePoint).dot(normal)

                    val cB = clipPoint + normal * scalar
                    val cA = clipPoint - normal * radiusA
                    points[i].set(cA).addLocal(cB).mulLocal(0.5f)
                    separations[i] = (cA - cB).dot(normal)
                }
                // Ensure normal points from A to B.
                normal.negateLocal()
            }
            null -> {

            }
        }
    }
}