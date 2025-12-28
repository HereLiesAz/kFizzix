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

import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2

/**
 * A manifold for two touching convex shapes. Box2D supports multiple types of
 * contact:
 *
 *  * clip point versus plane with radius
 *  * point versus point with radius (circles)
 *
 * The local point usage depends on the manifold type:
 *
 *  * circles: the local center of circleA
 *  * faceA: the center of faceA
 *  * faceB: the center of faceB
 *
 * Similarly, the local normal usage:
 *
 *  * e_circles: not used
 *  * e_faceA: the normal on polygonA
 *  * e_faceB: the normal on polygonB
 *
 * We store contacts in this way so that position correction can account for
 * movement, which is critical for continuous physics. All contact scenarios

 * must be expressed in one of these types. This structure is stored across time
 * steps, so we keep it small.
 */
class Manifold(
    /** The points of contact. */
    val points: Array<ManifoldPoint> = Array(Settings.maxManifoldPoints) { ManifoldPoint() },
    /** not use for Type::e_points */
    val localNormal: Vec2 = Vec2(),
    /** usage depends on manifold type */
    val localPoint: Vec2 = Vec2(),
    var type: ManifoldType? = null,
    /** The number of manifold points. */
    var pointCount: Int = 0
) {
    enum class ManifoldType {
        CIRCLES, FACE_A, FACE_B
    }

    override fun toString(): String {
        return "Manifold(points=${points.contentToString()}, localNormal=$localNormal, localPoint=$localPoint, type=$type, pointCount=$pointCount)"
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Manifold

        if (!points.contentEquals(other.points)) return false
        if (localNormal != other.localNormal) return false
        if (localPoint != other.localPoint) return false
        if (type != other.type) return false
        if (pointCount != other.pointCount) return false

        return true
    }

    override fun hashCode(): Int {
        var result = points.contentHashCode()
        result = 31 * result + localNormal.hashCode()
        result = 31 * result + localPoint.hashCode()
        result = 31 * result + (type?.hashCode() ?: 0)
        result = 31 * result + pointCount
        return result
    }
}
