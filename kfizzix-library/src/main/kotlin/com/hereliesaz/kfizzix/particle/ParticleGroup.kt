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
    private var _mass = 0f
    var mass: Float
        get() {
            updateStatistics()
            return _mass
        }
        set(value) {
            _mass = value
        }
    private var _inertia = 0f
    var inertia: Float
        get() {
            updateStatistics()
            return _inertia
        }
        set(value) {
            _inertia = value
        }

    private val _center = Vec2()
    val center: Vec2
        get() {
            updateStatistics()
            return _center
        }

    private val _linearVelocity = Vec2()
    val linearVelocity: Vec2
        get() {
            updateStatistics()
            return _linearVelocity
        }

    private var _angularVelocity = 0f
    var angularVelocity: Float
        get() {
            updateStatistics()
            return _angularVelocity
        }
        set(value) {
            _angularVelocity = value
        }
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
        _mass = 0f
        _inertia = 0f
        _angularVelocity = 0f
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
            _mass = 0f
            _center.setZero()
            _linearVelocity.setZero()
            for (i in firstIndex until lastIndex) {
                _mass += m
                val pos = system!!.positionBuffer.data!![i]!!
                _center.x += m * pos.x
                _center.y += m * pos.y
                val vel = system!!.velocityBuffer.data!![i]!!
                _linearVelocity.x += m * vel.x
                _linearVelocity.y += m * vel.y
            }
            if (_mass > 0) {
                _center.x *= 1 / _mass
                _center.y *= 1 / _mass
                _linearVelocity.x *= 1 / _mass
                _linearVelocity.y *= 1 / _mass
            }
            _inertia = 0f
            _angularVelocity = 0f
            for (i in firstIndex until lastIndex) {
                val pos = system!!.positionBuffer.data!![i]!!
                val vel = system!!.velocityBuffer.data!![i]!!
                val px = pos.x - _center.x
                val py = pos.y - _center.y
                val vx = vel.x - _linearVelocity.x
                val vy = vel.y - _linearVelocity.y
                _inertia += m * (px * px + py * py)
                _angularVelocity += m * (px * vy - py * vx)
            }
            if (_inertia > 0) {
                _angularVelocity *= 1 / _inertia
            }
            timestamp = system!!.timestamp
        }
    }
}
