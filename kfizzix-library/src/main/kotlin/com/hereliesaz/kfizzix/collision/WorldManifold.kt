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

    private val poolVec1 = Vec2()
    private val poolVec2 = Vec2()
    private val poolVec3 = Vec2()
    private val poolVec4 = Vec2()


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
                Transform.mulToOut(xfA, manifold.localPoint, poolVec1)
                Transform.mulToOut(xfB, manifold.points[0].localPoint, poolVec2)

                if (MathUtils.distanceSquared(poolVec1, poolVec2) > Settings.EPSILON * Settings.EPSILON) {
                    normal.set(poolVec2).subLocal(poolVec1)
                    normal.normalize()
                }

                poolVec3.set(normal).mulLocal(radiusA).addLocal(poolVec1)
                poolVec4.set(normal).mulLocal(-radiusB).addLocal(poolVec2)

                points[0].set(poolVec3).addLocal(poolVec4).mulLocal(0.5f)
                poolVec1.set(poolVec4).subLocal(poolVec3)
                separations[0] = poolVec1.dot(normal)
            }
            Manifold.ManifoldType.FACE_A -> {
                Rot.mulToOut(xfA.q, manifold.localNormal, normal)
                Transform.mulToOut(xfA, manifold.localPoint, poolVec1)

                for (i in 0 until manifold.pointCount) {
                    Transform.mulToOut(xfB, manifold.points[i].localPoint, poolVec2)

                    poolVec3.set(poolVec2).subLocal(poolVec1)
                    val scalar = radiusA - poolVec3.dot(normal)

                    poolVec3.set(normal).mulLocal(scalar).addLocal(poolVec2)
                    poolVec4.set(normal).mulLocal(-radiusB).addLocal(poolVec2)

                    points[i].set(poolVec3).addLocal(poolVec4).mulLocal(0.5f)

                    poolVec1.set(poolVec4).subLocal(poolVec3)
                    separations[i] = poolVec1.dot(normal)
                }
            }
            Manifold.ManifoldType.FACE_B -> {
                Rot.mulToOut(xfB.q, manifold.localNormal, normal)
                Transform.mulToOut(xfB, manifold.localPoint, poolVec1)

                for (i in 0 until manifold.pointCount) {
                    Transform.mulToOut(xfA, manifold.points[i].localPoint, poolVec2)

                    poolVec3.set(poolVec2).subLocal(poolVec1)
                    val scalar = radiusB - poolVec3.dot(normal)

                    poolVec4.set(normal).mulLocal(scalar).addLocal(poolVec2)
                    poolVec3.set(normal).mulLocal(-radiusA).addLocal(poolVec2)

                    points[i].set(poolVec3).addLocal(poolVec4).mulLocal(0.5f)

                    poolVec1.set(poolVec3).subLocal(poolVec4)
                    separations[i] = poolVec1.dot(normal)
                }
                // Ensure normal points from A to B.
                normal.negateLocal()
            }
            null -> {

            }
        }
    }
}