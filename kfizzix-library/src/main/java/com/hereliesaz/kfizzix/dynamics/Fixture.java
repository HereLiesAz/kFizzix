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
package com.hereliesaz.kfizzix.dynamics

import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.collision.broadphase.BroadPhase
import com.hereliesaz.kfizzix.collision.shapes.MassData
import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.collision.shapes.ShapeType
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.contacts.Contact
import com.hereliesaz.kfizzix.dynamics.contacts.ContactEdge
import kotlin.math.max
import kotlin.math.min

/**
 * A fixture is used to attach a shape to a body for collision detection. A
 * fixture inherits its transform from its parent. Fixtures hold additional
 * non-geometric data such as friction, collision filters, etc. Fixtures are
 * created via Body::CreateFixture.
 *
 * @warning you cannot reuse fixtures.
 *
 * @author Daniel Murphy
 */
class Fixture {
    var next: Fixture? = null
    var body: Body? = null

    /**
     * The shape, this must be set. The shape will be cloned, so you can create
     * the shape on the stack.
     */
    var shape: Shape? = null

    /**
     * Use this to store application specific fixture data.
     */
    var userData: Any? = null

    /**
     * The friction coefficient, usually in the range [0,1].
     */
    var friction = 0f

    /**
     * The restitution (elasticity) usually in the range [0,1].
     */
    var restitution = 0f

    /**
     * The density, usually in kg/m^2
     */
    var density = 0f

    /**
     * A sensor shape collects contact information but never generates a
     * collision response.
     */
    var isSensor = false

    /**
     * Contact filtering data;
     */
    val filter: Filter

    var proxies: Array<FixtureProxy?>? = null
    var proxyCount = 0

    init {
        userData = null
        body = null
        next = null
        proxies = null
        proxyCount = 0
        shape = null
        filter = Filter()
    }

    /**
     * Get the type of the child shape. You can use this to downcast to the
     * concrete shape.
     *
     * @return The shape type.
     */
    val type: ShapeType
        get() = shape!!.type

    /**
     * Set the contact filtering data. This is an expensive operation and should
     * not be called frequently. This will not update contacts until the next
     * time step when either parent body is awake. This automatically calls
     * refilter.
     */
    fun setFilterData(filter: Filter) {
        this.filter.set(filter)
        refilter()
    }

    /**
     * Call this if you want to establish a collision that was previously
     * disabled by ContactFilter::ShouldCollide.
     */
    fun refilter() {
        if (body == null) {
            return
        }
        // Flag associated contacts for filtering.
        var edge = body!!.contactList
        while (edge != null) {
            val contact = edge.contact
            val fixtureA = contact!!.fixtureA
            val fixtureB = contact.fixtureB
            if (fixtureA === this || fixtureB === this) {
                contact.flagForFiltering()
            }
            edge = edge.next
        }
        val world = body!!.world
        if (world == null) {
            return
        }
        // Touch each proxy so that new pairs may be created
        val broadPhase = world.contactManager.broadPhase
        for (i in 0 until proxyCount) {
            broadPhase.touchProxy(proxies!![i]!!.proxyId)
        }
    }

    /**
     * Test a point for containment in this fixture. This only works for convex
     * shapes.
     *
     * @param p A point in the world coordinates.
     */
    fun testPoint(p: Vec2): Boolean {
        return shape!!.testPoint(body!!.xf, p)
    }

    /**
     * Cast a ray against this shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     */
    fun raycast(
        output: RayCastOutput, input: RayCastInput,
        childIndex: Int
    ): Boolean {
        return shape!!.raycast(output, input, body!!.xf, childIndex)
    }

    /**
     * Get the mass data for this fixture. The mass data is based on the density
     * and the shape. The rotational inertia is about the shape's origin.
     */
    fun getMassData(massData: MassData) {
        shape!!.computeMass(massData, density)
    }

    /**
     * Get the fixture's AABB. This AABB may be enlarge and/or stale. If you
     * need a more accurate AABB, compute it using the shape and the body
     * transform.
     */
    fun getAABB(childIndex: Int): AABB {
        assert(childIndex >= 0 && childIndex < proxyCount)
        return proxies!![childIndex]!!.aabb
    }

    /**
     * Compute the distance from this fixture.
     *
     * @param p A point in world coordinates.
     *
     * @return distance
     */
    fun computeDistance(p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        return shape!!.computeDistanceToOut(
            body!!.transform, p, childIndex,
            normalOut
        )
    }
    // We need separation create/destroy functions from the
    // constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments
    // allowed by C++).
    fun create(body: Body, def: FixtureDef) {
        userData = def.userData
        friction = def.friction
        restitution = def.restitution
        this.body = body
        next = null
        filter.set(def.filter)
        isSensor = def.isSensor
        shape = def.shape!!.clone()
        // Reserve proxy space
        val childCount = shape!!.childCount
        if (proxies == null) {
            proxies = arrayOfNulls(childCount)
            for (i in 0 until childCount) {
                proxies!![i] = FixtureProxy()
                proxies!![i]!!.fixture = null
                proxies!![i]!!.proxyId = BroadPhase.NULL_PROXY
            }
        }
        if (proxies!!.size < childCount) {
            val old = proxies
            val newLen = MathUtils.max(old!!.size * 2, childCount)
            proxies = arrayOfNulls(newLen)
            System.arraycopy(old, 0, proxies!!, 0, old.size)
            for (i in 0 until newLen) {
                if (i >= old.size) {
                    proxies!![i] = FixtureProxy()
                }
                proxies!![i]!!.fixture = null
                proxies!![i]!!.proxyId = BroadPhase.NULL_PROXY
            }
        }
        proxyCount = 0
        density = def.density
    }

    fun destroy() {
        // The proxies must be destroyed before calling this.
        assert(proxyCount == 0)
        // Free the child shape.
        shape = null
        proxies = null
        next = null
        // TODO pool shapes
        // TODO pool fixtures
    }

    // These support body activation/deactivation.
    fun createProxies(broadPhase: BroadPhase, xf: Transform) {
        assert(proxyCount == 0)
        // Create proxies in the broad-phase.
        proxyCount = shape!!.childCount
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            shape!!.computeAABB(proxy!!.aabb, xf, i)
            proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy)
            proxy.fixture = this
            proxy.childIndex = i
        }
    }

    /**
     * Internal method
     */
    fun destroyProxies(broadPhase: BroadPhase) {
        // Destroy proxies in the broad-phase.
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            broadPhase.destroyProxy(proxy!!.proxyId)
            proxy.proxyId = BroadPhase.NULL_PROXY
        }
        proxyCount = 0
    }

    private val pool1 = AABB()
    private val pool2 = AABB()
    private val displacement = Vec2()

    /**
     * Internal method
     */
    fun synchronize(
        broadPhase: BroadPhase,
        transform1: Transform, transform2: Transform
    ) {
        if (proxyCount == 0) {
            return
        }
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            // Compute an AABB that covers the swept shape (may miss some
            // rotation effect).
            val aabb1 = pool1
            val aab = pool2
            shape!!.computeAABB(aabb1, transform1, proxy!!.childIndex)
            shape!!.computeAABB(aab, transform2, proxy.childIndex)
            proxy.aabb.lowerBound.x = min(aabb1.lowerBound.x, aab.lowerBound.x)
            proxy.aabb.lowerBound.y = min(aabb1.lowerBound.y, aab.lowerBound.y)
            proxy.aabb.upperBound.x = max(aabb1.upperBound.x, aab.upperBound.x)
            proxy.aabb.upperBound.y = max(aabb1.upperBound.y, aab.upperBound.y)
            displacement.x = transform2.p.x - transform1.p.x
            displacement.y = transform2.p.y - transform1.p.y
            broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement)
        }
    }
}
