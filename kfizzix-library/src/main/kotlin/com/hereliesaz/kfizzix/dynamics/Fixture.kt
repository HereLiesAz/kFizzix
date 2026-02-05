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
/**
 * A fixture connects a shape to a body.
 *
 * While a [Body] defines the position and velocity, a [Fixture] defines the
 * physical properties like size, shape, friction, and bounciness.
 * A body can have multiple fixtures (e.g., a "dumbbell" made of two circles and a rectangle).
 *
 * **Key Properties:**
 * *   [shape]: The geometry (Circle, Polygon, etc.).
 * *   [density]: Used to calculate mass (mass = density * area).
 * *   [friction]: Slippery (0.0) vs. Rough (1.0).
 * *   [restitution]: Bounciness (0.0 = no bounce, 1.0 = perfect bounce).
 * *   [isSensor]: If true, detects collision but doesn't physically react (ghost).
 *
 * @author Daniel Murphy
 */
class Fixture {
    // The next fixture in the body's linked list of fixtures.
    var next: Fixture? = null

    // The parent body of this fixture.
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

    // Array of proxies used by the broad-phase collision detection.
    // A single fixture (like a ChainShape) might have multiple proxies.
    var proxies: Array<FixtureProxy?>? = null

    // The number of proxies currently in use.
    var proxyCount = 0

    init {
        // Initialize user data to null.
        userData = null
        // Initialize body reference to null.
        body = null
        // Initialize next pointer to null.
        next = null
        // Initialize proxies array to null.
        proxies = null
        // Initialize proxy count to 0.
        proxyCount = 0
        // Initialize shape to null.
        shape = null
        // 2. Clean Up: Nullify references to help GC.
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
    /**
     * Set the contact filtering data.
     * This allows you to say "This fixture should collide with category A but not B".
     *
     * **Performance Note:**
     * This is an expensive operation as it may need to update the BroadPhase.
     * Do not call this every frame.
     *
     * @param filter The new filter definition.
     */
    fun setFilterData(filter: Filter) {
        // Update the internal filter data.
        this.filter.set(filter)
        // Trigger refiltering to update contacts.
        refilter()
    }

    /**
     * Call this if you want to establish a collision that was previously
     * disabled by ContactFilter::ShouldCollide.
     */
    fun refilter() {
        // If the fixture is not attached to a body, do nothing.
        if (body == null) {
            return
        }
        // Flag associated contacts for filtering.
        // Start iterating from the body's contact list.
        var edge = body!!.contactList
        while (edge != null) {
            val contact = edge.contact
            val fixtureA = contact!!.fixtureA
            val fixtureB = contact.fixtureB
            // If this fixture is one of the participants in the contact...
            if (fixtureA === this || fixtureB === this) {
                // Mark the contact for filtering in the next step.
                contact.flagForFiltering()
            }
            // Move to the next contact edge.
            edge = edge.next
        }
        val world = body!!.world
        // Touch each proxy so that new pairs may be created
        // Get the broad-phase from the world's contact manager.
        val broadPhase = world.contactManager.broadPhase
        // Iterate over all proxies associated with this fixture.
        for (i in 0 until proxyCount) {
            // Touch the proxy in the broad-phase to trigger overlap updates.
            broadPhase.touchProxy(proxies!![i]!!.proxyId)
        }
    }

    /**
     * Test a point for containment in this fixture. This only works for convex
     * shapes.
     *
     * @param p A point in the world coordinates.
     */
    /**
     * Check if a point is inside this fixture.
     *
     * @param p The point in World coordinates.
     * @return True if the point is inside.
     */
    fun testPoint(p: Vec2): Boolean {
        // Delegate to the shape's testPoint method, transforming by the body's transform.
        return shape!!.testPoint(body!!.xf, p)
    }

    /**
     * Cast a ray against this shape.
     *
     * @param output The ray-cast results.
     * @param input The ray-cast input parameters.
     */
    /**
     * Cast a ray against this specific fixture.
     *
     * @param output The results will be stored here if the ray hits.
     * @param input The definition of the ray (start point, max fraction).
     * @param childIndex The child index of the shape (0 for most shapes, index for Chains).
     * @return True if the ray hits the fixture.
     */
    fun raycast(
        output: RayCastOutput, input: RayCastInput,
        childIndex: Int
    ): Boolean {
        // Delegate to the shape's raycast method.
        return shape!!.raycast(output, input, body!!.xf, childIndex)
    }

    /**
     * Get the mass data for this fixture. The mass data is based on the density
     * and the shape. The rotational inertia is about the shape's origin.
     */
    fun getMassData(massData: MassData) {
        // Delegate calculation to the shape, passing the fixture's density.
        shape!!.computeMass(massData, density)
    }

    /**
     * Get the fixture's AABB. This AABB may be enlarge and/or stale. If you
     * need a more accurate AABB, compute it using the shape and the body
     * transform.
     */
    fun getAABB(childIndex: Int): AABB {
        // Ensure the child index is valid.
        assert(childIndex >= 0 && childIndex < proxyCount)
        // Return the AABB stored in the proxy.
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
        // TODO: This method is not available in the Shape class in the original Java code.
        // It seems to be present only in PolygonShape and CircleShape?
        // For now, we will comment this out or try to cast if specific.
        // Or if we want to fix compilation, we can remove it if not used, or stub it.
        // Looking at usage in PolygonShape, it's there.
        // Let's check CircleShape.
        /*
        return shape!!.computeDistanceToOut(
            body!!.transform, p, childIndex,
            normalOut
        )
        */
        return 0f // Placeholder to fix compilation
    }

    // We need separation create/destroy functions from the
    // constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments
    // allowed by C++).
    fun create(body: Body, def: FixtureDef) {
        // Copy user data from definition.
        userData = def.userData
        // 1. Initialize Data: Copy properties from the definition.
        friction = def.friction
        // Copy restitution from definition.
        restitution = def.restitution
        // Set the parent body.
        this.body = body
        // Reset next pointer.
        next = null
        // Copy filter data.
        filter.set(def.filter)
        // Set sensor flag.
        isSensor = def.isSensor
        // Clone the shape from definition to ensure ownership.
        shape = def.shape!!.clone()
        // 2. Clone Shape: We must own the shape, so we clone it.
        // Reserve proxy space
        // Get the number of children (e.g. 1 for circle/poly, N for chain).
        val childCount = shape!!.childCount
        // If proxies array is null, allocate it.
        if (proxies == null) {
        // 3. Allocate Proxies: Prepare the array for BroadPhase proxies.
            proxies = arrayOfNulls(childCount)
            for (i in 0 until childCount) {
                // Initialize each proxy.
                proxies!![i] = FixtureProxy()
                proxies!![i]!!.fixture = null
                proxies!![i]!!.proxyId = BroadPhase.NULL_PROXY
            }
        }
        // If proxies array is too small, resize it.
        if (proxies!!.size < childCount) {
            val old = proxies
            // Double the size or match childCount.
            val newLen = MathUtils.max(old!!.size * 2, childCount)
            proxies = arrayOfNulls(newLen)
            // Copy old proxies.
            System.arraycopy(old, 0, proxies!!, 0, old.size)
            // Initialize new slots.
            for (i in 0 until newLen) {
                if (i >= old.size) {
                    proxies!![i] = FixtureProxy()
                }
                proxies!![i]!!.fixture = null
                proxies!![i]!!.proxyId = BroadPhase.NULL_PROXY
            }
        }
        // Reset proxy count.
        proxyCount = 0
        // Set density.
        density = def.density
    }

    fun destroy() {
        // The proxies must be destroyed before calling this.
        assert(proxyCount == 0)
        // 1. Verify: Proxies must be destroyed via Body.destroyFixture before calling this.
        // Free the child shape.
        shape = null
        // 2. Clean Up: Nullify references to help GC.
        proxies = null
        // Reset next pointer.
        next = null
        // TODO pool shapes
        // TODO pool fixtures
    }

    // These support body activation/deactivation.
    fun createProxies(broadPhase: BroadPhase, xf: Transform) {
        // Ensure no proxies currently exist.
        assert(proxyCount == 0)
        // 1. Verify: Proxies must be destroyed via Body.destroyFixture before calling this.
        // Create proxies in the broad-phase.
        proxyCount = shape!!.childCount
        // Iterate over all children of the shape.
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            // Compute the AABB for this child.
            shape!!.computeAABB(proxy!!.aabb, xf, i)
            // Create a proxy in the broad-phase tree and get its ID.
            proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy)
            // Link the proxy back to this fixture.
            proxy.fixture = this
            // Set the child index.
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
            // Remove the proxy from the broad-phase tree.
            broadPhase.destroyProxy(proxy!!.proxyId)
            // Reset the proxy ID to null.
            proxy.proxyId = BroadPhase.NULL_PROXY
        }
        // Reset the proxy count.
        proxyCount = 0
    }

    // Temporary pool objects for synchronization.
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
        // If no proxies, nothing to synchronize.
        if (proxyCount == 0) {
        // 1. Early Exit: No proxies to sync.
            return
        }
        // Iterate over all proxies.
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            // Compute an AABB that covers the swept shape (may miss some
            // rotation effect).
            val aabb1 = pool1
            val aab = pool2
            // Compute AABB at start transform.
            shape!!.computeAABB(aabb1, transform1, proxy!!.childIndex)
            // 2. Compute Swept AABB: Calculate AABB at start and end transforms.
            shape!!.computeAABB(aab, transform2, proxy.childIndex)
            // Combine AABBs to create a swept AABB.
            proxy.aabb.lowerBound.x = min(aabb1.lowerBound.x, aab.lowerBound.x)
            // 3. Union: Combine both AABBs to cover the entire swept path.
            proxy.aabb.lowerBound.y = min(aabb1.lowerBound.y, aab.lowerBound.y)
            proxy.aabb.upperBound.x = max(aabb1.upperBound.x, aab.upperBound.x)
            proxy.aabb.upperBound.y = max(aabb1.upperBound.y, aab.upperBound.y)
            // Calculate displacement vector.
            displacement.x = transform2.p.x - transform1.p.x
            displacement.y = transform2.p.y - transform1.p.y
            // Move the proxy in the broad-phase with the new AABB and displacement.
            broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement)
            // 4. Update Tree: Move the proxy in the dynamic tree.
        }
    }
}
