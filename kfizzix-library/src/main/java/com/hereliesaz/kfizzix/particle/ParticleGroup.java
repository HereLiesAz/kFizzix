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

import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2

/**
 * A group of particles
 *
 * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L172-L287
 */
class ParticleGroup {
    var system: ParticleSystem? = null
    var firstIndex = 0
    var lastIndex = 0
    var groupFlags = 0
    var strength = 0f
    var prev: ParticleGroup? = null
    var next: ParticleGroup? = null
    var timestamp = 0
    var mass = 0f
    var inertia = 0f
    val center = Vec2()
    val linearVelocity = Vec2()
    var angularVelocity = 0f
    val transform = Transform()
    var destroyAutomatically = false
    var toBeDestroyed = false
    var toBeSplit = false

    /**
     * Use this to store application-specific group data.
     */
    var userData: Any? = null

    init {
        // system = null;
        firstIndex = 0
        lastIndex = 0
        groupFlags = 0
        strength = 1.0f
        timestamp = -1
        mass = 0f
        inertia = 0f
        angularVelocity = 0f
        transform.setIdentity()
        destroyAutomatically = true
        toBeDestroyed = false
        toBeSplit = false
    }

    /**
     * Get the number of particles.
     *
     * @return The number of particles.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L185-L186
     */
    val particleCount: Int
        get() = lastIndex - firstIndex

    /**
     * Get the offset of this group in the global particle buffer.
     *
     * @return The offset of this group in the global particle buffer.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L188-L189
     */
    val bufferIndex: Int
        get() = firstIndex

    /**
     * Get the total mass of the group: the sum of all particles in it.
     *
     * @return The total mass of the group: the sum of all particles in it.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L203-L204
     */
    fun getMass(): Float {
        updateStatistics()
        return mass
    }

    /**
     * Get the moment of inertia for the group.
     *
     * @return The moment of inertia for the group.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L206-L207
     */
    fun getInertia(): Float {
        updateStatistics()
        return inertia
    }

    /**
     * Get the center of gravity for the group.
     *
     * @return The center of gravity for the group.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L209-L210
     */
    fun getCenter(): Vec2 {
        updateStatistics()
        return center
    }

    /**
     * Get the linear velocity of the group.
     *
     * @return The linear velocity of the group.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L212-L213
     */
    fun getLinearVelocity(): Vec2 {
        updateStatistics()
        return linearVelocity
    }

    /**
     * Get the angular velocity of the group.
     *
     * @return The angular velocity of the group.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L215-L216
     */
    fun getAngularVelocity(): Float {
        updateStatistics()
        return angularVelocity
    }

    /**
     * Get the position of the particle group as a whole. Used only with groups
     * of rigid particles.
     *
     * @return The position of the particle group as a whole.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L222-L224
     */
    val position: Vec2
        get() = transform.p

    /**
     * Get the rotational angle of the particle group as a whole. Used only with
     * groups of rigid particles.
     *
     * @return The rotational angle of the particle group as a whole.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleGroup.h#L226-L228
     */
    val angle: Float
        get() = transform.q.angle
    fun updateStatistics() {
        if (timestamp != system!!.timestamp) {
            val m = system!!.particleMass
            mass = 0f
            center.setZero()
            linearVelocity.setZero()
            for (i in firstIndex until lastIndex) {
                mass += m
                val pos = system!!.positionBuffer.data!![i]
                center.x += m * pos.x
                center.y += m * pos.y
                val vel = system!!.velocityBuffer.data!![i]
                linearVelocity.x += m * vel.x
                linearVelocity.y += m * vel.y
            }
            if (mass > 0) {
                center.x *= 1 / mass
                center.y *= 1 / mass
                linearVelocity.x *= 1 / mass
                linearVelocity.y *= 1 / mass
            }
            inertia = 0f
            angularVelocity = 0f
            for (i in firstIndex until lastIndex) {
                val pos = system!!.positionBuffer.data!![i]
                val vel = system!!.velocityBuffer.data!![i]
                val px = pos.x - center.x
                val py = pos.y - center.y
                val vx = vel.x - linearVelocity.x
                val vy = vel.y - linearVelocity.y
                inertia += m * (px * px + py * py)
                angularVelocity += m * (px * vy - py * vx)
            }
            if (inertia > 0) {
                angularVelocity *= 1 / inertia
            }
            timestamp = system!!.timestamp
        }
    }
}
