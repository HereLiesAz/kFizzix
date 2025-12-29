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
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.dynamics.contacts

import com.hereliesaz.kfizzix.callbacks.ContactListener
import com.hereliesaz.kfizzix.collision.Manifold
import com.hereliesaz.kfizzix.collision.WorldManifold
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.dynamics.Fixture
import com.hereliesaz.kfizzix.pooling.WorldPool
import kotlin.math.max

/**
 * The class manages contact between two shapes. A contact exists for each
 * overlapping AABB in the broad-phase (except if filtered). Therefore, a
 * contact object may exist that has no contact points.
 *
 * @author Daniel Murphy
 */
abstract class Contact(protected val pool: WorldPool) {
    var flags: Int = 0

    // World pool and list pointers.
    var prev: Contact? = null
    var next: Contact? = null

    // Nodes for connecting bodies.
    var nodeA: ContactEdge = ContactEdge()
    var nodeB: ContactEdge = ContactEdge()

    var fixtureA: Fixture? = null
    var fixtureB: Fixture? = null

    var indexA: Int = 0
    var indexB: Int = 0

    val manifold: Manifold = Manifold()

    var toiCount: Float = 0f
    var toi: Float = 0f

    var friction: Float = 0f
    var restitution: Float = 0f
    var tangentSpeed: Float = 0f

    /**
     * initialization for pooling
     */
    open fun init(fA: Fixture, indexA: Int, fB: Fixture, indexB: Int) {
        flags = ENABLED_FLAG
        fixtureA = fA
        fixtureB = fB
        this.indexA = indexA
        this.indexB = indexB
        manifold.pointCount = 0
        prev = null
        next = null
        nodeA.contact = null
        nodeA.prev = null
        nodeA.next = null
        nodeA.other = null
        nodeB.contact = null
        nodeB.prev = null
        nodeB.next = null
        nodeB.other = null
        toiCount = 0f
        friction = mixFriction(fA.friction, fB.friction)
        restitution = mixRestitution(fA.restitution, fB.restitution)
        tangentSpeed = 0f
    }

    /**
     * Get the world manifold.
     */
    fun getWorldManifold(worldManifold: WorldManifold) {
        val bodyA = fixtureA!!.body
        val bodyB = fixtureB!!.body
        val shapeA = fixtureA!!.shape
        val shapeB = fixtureB!!.shape
        worldManifold.initialize(manifold, bodyA!!.transform, shapeA!!.radius,
            bodyB!!.transform, shapeB!!.radius)
    }

    /**
     * Is this contact touching
     */
    val isTouching: Boolean
        get() = (flags and TOUCHING_FLAG) == TOUCHING_FLAG

    /**
     * Enable/disable this contact. This can be used inside the pre-solve
     * contact listener. The contact is only disabled for the current time step
     * (or sub-step in continuous collisions).
     */
    var isEnabled: Boolean
        get() = (flags and ENABLED_FLAG) == ENABLED_FLAG
        set(flag) {
            if (flag) {
                flags = flags or ENABLED_FLAG
            } else {
                flags = flags and ENABLED_FLAG.inv()
            }
        }

    // Mix methods
    fun resetFriction() {
        friction = mixFriction(fixtureA!!.friction, fixtureB!!.friction)
    }

    fun resetRestitution() {
        restitution = mixRestitution(fixtureA!!.restitution, fixtureB!!.restitution)
    }

    abstract fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform)

    /**
     * Flag this contact for filtering. Filtering will occur the next time step.
     */
    fun flagForFiltering() {
        flags = flags or FILTER_FLAG
    }

    // djm pooling
    private val oldManifold = Manifold()

    fun update(listener: ContactListener?) {
        oldManifold.set(manifold)
        // Re-enable this contact.
        flags = flags or ENABLED_FLAG
        var touching = false
        val wasTouching = (flags and TOUCHING_FLAG) == TOUCHING_FLAG
        val sensorA = fixtureA!!.isSensor
        val sensorB = fixtureB!!.isSensor
        val sensor = sensorA || sensorB
        val bodyA = fixtureA!!.body
        val bodyB = fixtureB!!.body
        val xfA = bodyA!!.transform
        val xfB = bodyB!!.transform

        if (sensor) {
            val shapeA = fixtureA!!.shape
            val shapeB = fixtureB!!.shape
            touching = pool.getCollision().testOverlap(shapeA, indexA, shapeB,
                indexB, xfA!!, xfB!!)
            // Sensors don't generate manifolds.
            manifold.pointCount = 0
        } else {
            evaluate(manifold, xfA!!, xfB!!)
            touching = manifold.pointCount > 0
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (i in 0 until manifold.pointCount) {
                val mp2 = manifold.points[i]
                mp2.normalImpulse = 0.0f
                mp2.tangentImpulse = 0.0f
                val id2 = mp2.id
                for (j in 0 until oldManifold.pointCount) {
                    val mp1 = oldManifold.points[j]
                    if (mp1.id.isEqual(id2)) {
                        mp2.normalImpulse = mp1.normalImpulse
                        mp2.tangentImpulse = mp1.tangentImpulse
                        break
                    }
                }
            }
            if (touching != wasTouching) {
                bodyA.isAwake = true
                bodyB.isAwake = true
            }
        }
        if (touching) {
            flags = flags or TOUCHING_FLAG
        } else {
            flags = flags and TOUCHING_FLAG.inv()
        }
        if (listener == null) {
            return
        }
        if (!wasTouching && touching) {
            listener.beginContact(this)
        }
        if (wasTouching && !touching) {
            listener.endContact(this)
        }
        if (!sensor && touching) {
            listener.preSolve(this, oldManifold)
        }
    }

    companion object {
        /**
         * Flags stored in flags Used when crawling contact graph when forming
         * islands.
         */
        const val ISLAND_FLAG = 0x0001

        /**
         * Set when the shapes are touching.
         */
        const val TOUCHING_FLAG = 0x0002

        /**
         * This contact can be disabled (by user)
         */
        const val ENABLED_FLAG = 0x0004

        /**
         * This contact needs filtering because a fixture filter was changed.
         */
        const val FILTER_FLAG = 0x0008

        /**
         * This bullet contact had a TOI event
         */
        const val BULLET_HIT_FLAG = 0x0010
        const val TOI_FLAG = 0x0020

        /**
         * Friction mixing law. The idea is to allow either fixture to drive the
         * restitution to zero. For example, anything slides on ice.
         */
        @JvmStatic
        fun mixFriction(friction1: Float, friction2: Float): Float {
            return MathUtils.sqrt(friction1 * friction2)
        }

        /**
         * Restitution mixing law. The idea is allowed for anything to bounce off an
         * inelastic surface. For example, a superball bounces on anything.
         */
        @JvmStatic
        fun mixRestitution(restitution1: Float, restitution2: Float): Float {
            return max(restitution1, restitution2)
        }
    }
}
