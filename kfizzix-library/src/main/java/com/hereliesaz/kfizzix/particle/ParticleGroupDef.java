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
package com.hereliesaz.kfizzix.particle

import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.common.Vec2

/**
 * A particle group definition holds all the data needed to construct a particle
 * group. You can safely re-use these definitions.
 */
class ParticleGroupDef {
    /**
     * The particle-behavior flags.
     */
    var flags = 0

    /**
     * The group-construction flags.
     */
    var groupFlags = 0

    /**
     * The world position of the group. Moves the group's shape a distance equal
     * to the value of position.
     */
    val position = Vec2()

    /**
     * The world angle of the group in radians. Rotates the shape by an angle
     * equal to the value of angle.
     */
    var angle = 0f

    /**
     * The linear velocity of the group's origin in world co-ordinates.
     */
    val linearVelocity = Vec2()

    /**
     * The angular velocity of the group.
     */
    var angularVelocity = 0f

    /**
     * The color of all particles in the group.
     */
    var color: ParticleColor? = null

    /**
     * The strength of cohesion among the particles in a group with flag
     * b2_elasticParticle or b2_springParticle.
     */
    var strength = 0f

    /**
     * Shape containing the particle group.
     */
    var shape: Shape? = null

    /**
     * If true, destroy the group automatically after its last particle has been
     * destroyed.
     */
    var destroyAutomatically = false

    /**
     * Use this to store application-specific group data.
     */
    var userData: Any? = null

    init {
        flags = 0
        groupFlags = 0
        angle = 0f
        angularVelocity = 0f
        strength = 1f
        destroyAutomatically = true
    }
}
