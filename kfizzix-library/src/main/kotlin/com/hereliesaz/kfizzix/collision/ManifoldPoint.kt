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

import com.hereliesaz.kfizzix.common.Vec2

/**
 * A manifold point is a contact point belonging to a contact manifold. It holds
 * details related to the geometry and dynamics of the contact points. The local
 * point usage depends on the manifold type:
 *
 *  * circles: the local center of circleB
 *  * faceA: the local center of circleB or the clip point of polygonB
 *  * faceB: the clip point of polygonA
 *
 * This structure is stored across time steps, so we keep it small.<br></br>
 * Note: the impulses are used for internal caching and may not provide reliable
 * contact forces, especially for high speed collisions.
 *
 * @author Daniel Murphy
 */
class ManifoldPoint {
    /**
     * usage depends on manifold type
     */
    val localPoint: Vec2

    /**
     * the non-penetration impulse
     */
    var normalImpulse = 0f

    /**
     * the friction impulse
     */
    var tangentImpulse = 0f

    /**
     * uniquely identifies a contact point between two shapes
     */
    val id: ContactID

    /**
     * Blank manifold point with everything zeroed out.
     */
    constructor() {
        localPoint = Vec2()
        normalImpulse = 0f
        tangentImpulse = 0f
        id = ContactID()
    }

    /**
     * Creates a manifold point as a copy of the given point
     *
     * @param cp The point to copy from.
     */
    constructor(cp: ManifoldPoint) {
        localPoint = cp.localPoint.clone()
        normalImpulse = cp.normalImpulse
        tangentImpulse = cp.tangentImpulse
        id = ContactID(cp.id)
    }

    /**
     * Sets this manifold point form the given one
     *
     * @param cp The point to copy from.
     */
    fun set(cp: ManifoldPoint) {
        localPoint.set(cp.localPoint)
        normalImpulse = cp.normalImpulse
        tangentImpulse = cp.tangentImpulse
        id.set(cp.id)
    }
}
