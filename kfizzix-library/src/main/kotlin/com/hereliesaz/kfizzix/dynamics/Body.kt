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

import com.hereliesaz.kfizzix.collision.shapes.MassData
import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Sweep
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.contacts.ContactEdge
import com.hereliesaz.kfizzix.dynamics.joints.JointEdge

/**
 * A rigid body. These are created via World.createBody.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_body.cpp
 * @repolink https://github.com/erincatto/box2d/blob/main/include/box2d/b2_body.h
 *
 * @author Daniel Murphy
 */
/**
 * A rigid body. These are created via [World.createBody].
 *
 * **Body Types:**
 * *   [BodyType.STATIC]: Zero mass, zero velocity, may be manually moved. (e.g., Walls, Ground).
 * *   [BodyType.KINEMATIC]: Zero mass, non-zero velocity set by user, moved by solver. (e.g., Moving Platforms).
 * *   [BodyType.DYNAMIC]: Positive mass, non-zero velocity determined by forces & collisions. (e.g., Player, Balls).
 *
 * **Key Properties:**
 * *   [position]: The world position of the body's origin.
 * *   [angle]: The world rotation (radians).
 * *   [linearVelocity]: The speed and direction.
 * *   [angularVelocity]: The rotation speed.
 *
 * @author Daniel Murphy
 */
class Body(bd: BodyDef, @JvmField var world: World) {

    /**
     * The body type: static, kinematic, or dynamic. Note: if a dynamic body
     * would have zero mass, the mass is set to one.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L72-L74
     */
    var type: BodyType

    // Flags for internal state (sleeping, active, bullet, etc.).
    var flags = 0

    // Index used by the Island solver to identify this body in the island array.
    var islandIndex = 0

    /**
     * The body origin transform.
     */
    val xf = Transform()

    /**
     * The previous transform for particle simulation
     */
    val xf0 = Transform()

    /**
     * The swept motion for CCD (Continuous Collision Detection).
     * Tracks the motion from the previous step to the current step.
     */
    val sweep = Sweep()

    // The linear velocity of the center of mass.
    val linearVelocity = Vec2()

    /**
     * The angular velocity of the body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L86-L87
     */
    private var _angularVelocity = 0f

    // Property wrapper for angular velocity to handle side effects.
    var angularVelocity: Float
        get() = _angularVelocity
        set(w) {
            // Static bodies do not rotate.
            if (type == BodyType.STATIC) {
                return
            }
            // If the velocity is non-zero, wake the body up.
            if (w * w > 0f) {
                isAwake = true
            }
            _angularVelocity = w
        }

    // Force accumulator.
    val force = Vec2()

    // Torque accumulator.
    var torque = 0f

    /**
     * The previous body in the world's body list.
     */
    var prev: Body? = null

    /**
     * The next body in the world's body list.
     */
    var next: Body? = null

    // Head of the linked list of fixtures attached to this body.
    var fixtureList: Fixture? = null

    // Number of fixtures attached to this body.
    var fixtureCount = 0

    // Head of the linked list of joints attached to this body.
    var jointList: JointEdge? = null

    // Head of the linked list of contacts involving this body.
    var contactList: ContactEdge? = null

    /**
     * The mass.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L457
     */
    var mass = 0f

    /**
     * The inverse mass ({@code 1 / mass}).
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L457
     */
    var invMass = 0f

    /**
     * The rotational inertia about the center of mass.
     *
     *
     * The moment of inertia, otherwise known as the mass moment of
     * inertia,https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L459-L460
     * angular/rotational mass, second moment of mass, or most accurately,
     * rotational inertia, of a rigid body is a quantity that determines the
     * torque needed for a desired angular acceleration about a rotational axis,
     * akin to how mass determines the force needed for a desired acceleration.
     *
     *
     * [Wikipedia:
 * Moment of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia)
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L459-L460
     */
    var inertia = 0f

    /**
     * The inverse rotational inertia about the center of mass.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L459-L460
     */
    var invI = 0f

    /**
     * Linear damping is used to reduce the linear velocity. The damping
     * parameter can be larger than 1.0f but the damping effect becomes
     * sensitive to the time step when the damping parameter is large. Units are
     * {@code 1 / time}.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L89-L93
     */
    var linearDamping = 0f

    /**
     * Angular damping is used to reduce the angular velocity. The damping
     * parameter can be larger than 1.0f but the damping effect becomes
     * sensitive to the time step when the damping parameter is large. Units are
     * {@code 1 / time}.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L95-L99
     */
    var angularDamping = 0f

    /**
     * Scale the gravity applied to this body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L123-L124
     */
    var gravityScale = 0f

    // Accumulator for sleep time.
    var sleepTime = 0f

    /**
     * Use this to store application specific body data.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L120-L121
     */
    var userData: Any?

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L31-L104
     */
    init {
        // Validate inputs.
        assert(bd.position.isValid())
        assert(bd.linearVelocity.isValid())
        assert(bd.gravityScale >= 0.0f)
        assert(bd.angularDamping >= 0.0f)
        assert(bd.linearDamping >= 0.0f)

        // Initialize flags based on definition.
        flags = 0
        if (bd.bullet) {
            flags = flags or bulletFlag
        }
        if (bd.fixedRotation) {
            flags = flags or fixedRotationFlag
        }
        if (bd.allowSleep) {
            flags = flags or autoSleepFlag
        }
        if (bd.awake) {
            flags = flags or awakeFlag
        }
        if (bd.active) {
            flags = flags or activeFlag
        }

        // Set initial transform.
        xf.p.set(bd.position)
        xf.q.set(bd.angle)

        // Initialize sweep.
        sweep.localCenter.setZero()
        sweep.c0.set(xf.p)
        sweep.c.set(xf.p)
        sweep.a0 = bd.angle
        sweep.a = bd.angle
        sweep.alpha0 = 0.0f

        // Initialize lists.
        jointList = null
        contactList = null
        prev = null
        next = null

        // Set motion properties.
        linearVelocity.set(bd.linearVelocity)
        _angularVelocity = bd.angularVelocity
        linearDamping = bd.linearDamping
        angularDamping = bd.angularDamping
        gravityScale = bd.gravityScale

        // Clear forces.
        force.setZero()
        torque = 0.0f
        sleepTime = 0.0f

        // Set type and mass defaults.
        type = bd.type
        if (type == BodyType.DYNAMIC) {
            mass = 1f
            invMass = 1f
        } else {
            mass = 0f
            invMass = 0f
        }

        // Initialize inertia.
        inertia = 0.0f
        invI = 0.0f

        // Set user data.
        userData = bd.userData

        // Initialize fixture list.
        fixtureList = null
        fixtureCount = 0
    }

    /**
     * Creates a fixture and attach it to this body. Use this function if you
     * need to set some fixture parameters, like friction. Otherwise, you can
     * create the fixture directly from a shape. If the density is non-zero,
     * this function automatically updates the mass of the body. Contacts are
     * not created until the next time step.
     *
     * @param def The fixture definition.
     *
     * @warning This function is locked during callbacks.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L204-L211
     */
    /**
     * Creates a fixture and attaches it to this body. Use this function if you need
     * to set some fixture parameters, like friction. Otherwise you can create the
     * fixture directly from a shape.
     *
     * If the density is non-zero, this function automatically updates the mass of the body.
     * Contacts are not created until the next time step.
     *
     * @param def The fixture definition.
     * @warning This function is locked during callbacks.
     * @return The created fixture.
     */
    fun createFixture(def: FixtureDef): Fixture? {
        // Ensure world is not locked (e.g. during a time step).
        assert(!world.isLocked)
        if (world.isLocked) {
            return null
        }

        // Create the fixture.
        val fixture = Fixture()
        // 1. Create Fixture: Instantiate and initialize the fixture from the definition.
        fixture.create(this, def)

        // If body is active, create proxies in broad-phase.
        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture.createProxies(broadPhase, xf)
        }

        // Add to the front of the fixture list.
        fixture.next = fixtureList
        // 3. Add to List: Prepend the fixture to the body's linked list.
        fixtureList = fixture
        ++fixtureCount

        // Set the body back-pointer.
        fixture.body = this

        // Adjust mass properties if needed.
        if (fixture.density > 0.0f) {
        // 4. Update Mass: If the fixture adds mass, recalculate the body's mass data.
            resetMassData()
        }

        // Note: New contacts will be created at the start of the next time step.
        // We don't set a NEW_FIXTURE flag here because broad-phase updates handle it.
        return fixture
    }

    private val fixDef = FixtureDef()

    /**
     * Creates a fixture from a shape and attach it to this body. This is a
     * convenience function. Use FixtureDef if you need to set parameters like
     * friction, restitution, user data, or filtering. If the density is
     * non-zero, this function automatically updates the mass of the body.
     *
     * @param shape The shape to be cloned.
     * @param density The shape density (set to zero for static bodies).
     *
     * @warning This function is locked during callbacks.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L204-L211
     */
    fun createFixture(shape: Shape, density: Float): Fixture? {
        // Setup a temporary definition.
        fixDef.shape = shape
        fixDef.density = density
        // Delegate to main create method.
        return createFixture(fixDef)
    }

    /**
     * Destroy a fixture. This removes the fixture from the broad-phase and
     * destroys all contacts associated with this fixture. This will
     * automatically adjust the mass of the body if the body is dynamic and the
     * fixture has positive density. All fixtures attached to a body are
     * implicitly destroyed when the body is destroyed.
     *
     * @param fixture The fixture to be removed.
     *
     * @warning This function is locked during callbacks.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L213-L288
     */
    fun destroyFixture(fixture: Fixture?) {
        // Ensure world is not locked.
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }
        // Validate fixture ownership.
        assert(fixture!!.body === this)

        // Remove the fixture from this body's singly linked list.
        assert(fixtureCount > 0)
        var node = fixtureList
        var last: Fixture? = null // Previous node tracker.
        var found = false
        while (node != null) {
            if (node === fixture) {
        // 4. Unlink: Remove the fixture from the body's linked list.
                node = fixture!!.next
                found = true
                break
            }
            last = node
            node = node.next
        }

        // Assert that we actually found the fixture.
        assert(found)

        // Remove it from the list.
        if (last == null) {
            fixtureList = fixture!!.next
        } else {
            last.next = fixture!!.next
        }

        // Destroy any contacts associated with the fixture.
        var edge = contactList
        // 3. Destroy Contacts: Remove any contacts associated with this fixture.
        while (edge != null) {
            val c = edge.contact
            edge = edge.next
            val fixtureA = c!!.fixtureA
            val fixtureB = c.fixtureB
            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                world.contactManager.destroy(c)
            }
        }

        // Remove proxies from broad-phase if body is active.
        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture!!.destroyProxies(broadPhase)
        }

        // Clean up the fixture.
        fixture!!.destroy()
        fixture.body = null
        fixture.next = null
        --fixtureCount

        // Reset the mass data.
        resetMassData()
    }

    /**
     * Set the position of the body's origin and rotation. This breaks any
     * contacts and wakes the other bodies. Manipulating a body's transform may
     * cause non-physical behavior. Note: contacts are updated on the next call
     * to World.step().
     *
     * @param position The world position of the body's local origin.
     * @param angle The world rotation in radians.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L420-L445
     */
    /**
     * Set the position of the body's origin and rotation.
     * This breaks any contacts and wakes the other bodies.
     * Manipulating a body's transform may cause non-physical behavior.
     *
     * @param position The world position of the body's local origin.
     * @param angle The world rotation in radians.
     */
    fun setTransform(position: Vec2, angle: Float) {
        // Ensure world is not locked.
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }

        // Update transform components.
        xf.q.set(angle)
        xf.p.set(position)

        // Update sweep data.
        // sweep.c = Transform * localCenter
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c)
        sweep.a = angle
        sweep.c0.set(sweep.c)
        sweep.a0 = sweep.a

        // Update broad-phase proxies.
        val broadPhase = world.contactManager.broadPhase
        var f = fixtureList
        while (f != null) {
            f.synchronize(broadPhase, xf, xf)
            f = f.next
        }
    }

    /**
     * Get the body transform for the body's origin.
     *
     * @return The world transform of the body's origin.
     */
    val transform: Transform
        get() = xf

    /**
     * Get the world body origin position. Do not modify.
     *
     * @return The world position of the body's origin.
     */
    val position: Vec2
        get() = xf.p

    /**
     * Get the angle in radians.
     *
     * @return The current world rotation angle in radians.
     */
    val angle: Float
        get() = sweep.a

    /**
     * Get the world position of the center of mass. Do not modify.
     */
    val worldCenter: Vec2
        get() = sweep.c

    /**
     * Get the local position of the center of mass. Do not modify.
     */
    val localCenter: Vec2
        get() = sweep.localCenter

    /**
     * Set the linear velocity of the center of mass.
     *
     * @param v The new linear velocity of the center of mass.
     */
    fun setLinearVelocity(v: Vec2) {
        // Static bodies have zero velocity.
        if (type == BodyType.STATIC) {
            return
        }
        // Wake up if velocity is non-zero.
        if (Vec2.dot(v, v) > 0.0f) {
            isAwake = true
        }
        linearVelocity.set(v)
    }

    /**
     * Apply a force at a world point. If the force is not applied at the center
     * of mass, it will generate a torque and affect the angular velocity. This
     * wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     * @param point The world position of the point of application.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L743-L761
     */
    /**
     * Apply a force at a world point. If the force is not
     * applied at the center of mass, it will generate a torque and
     * affect the angular velocity. This wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     * @param point The world position of the point of application.
     */
    fun applyForce(force: Vec2, point: Vec2) {
        // Only dynamic bodies are affected by forces.
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        // Wake the body.
        if (!isAwake) {
            isAwake = true
        }
        // Add linear force.
        this.force.x += force.x
        this.force.y += force.y
        // Add torque (cross product of radius vector and force).
        // Torque += (point - center) x force
        torque += (point.x - sweep.c.x) * force.y - (point.y - sweep.c.y) * force.x
    }

    /**
     * Apply a force to the center of mass. This wakes up the body.
     *
     * @param force The world force vector, usually in Newtons (N).
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L763-L780
     */
    fun applyForceToCenter(force: Vec2) {
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        if (!isAwake) {
            isAwake = true
        }
        this.force.x += force.x
        this.force.y += force.y
    }

    /**
     * Apply a torque. This affects the angular velocity without affecting the
     * linear velocity of the center of mass. This wakes up the body.
     *
     * @param torque About the z-axis (out of the screen), usually in N-m.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L782-L799
     */
    /**
     * Apply a torque. This affects the angular velocity
     * without affecting the linear velocity of the center of mass.
     * This wakes up the body.
     *
     * @param torque The torque (N-m).
     */
    fun applyTorque(torque: Float) {
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        if (!isAwake) {
            isAwake = true
        }
        this.torque += torque
    }

    /**
     * Apply an impulse at a point. This immediately modifies the velocity. It
     * also modifies the angular velocity if the point of application is not at
     * the center of mass. This wakes up the body if 'wake' is set to true. If
     * the body is sleeping and 'wake' is false, then there is no effect.
     *
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     * @param point The world position of the point of application.
     * @param wake Also wake up the body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L801-L819
     */
    /**
     * Apply an impulse at a point. This immediately modifies the velocity.
     * It also modifies the angular velocity if the point of application
     * is not at the center of mass. This wakes up the body.
     *
     * @param impulse The world impulse vector, usually in N-seconds or kg-m/s.
     * @param point The world position of the point of application.
     */
    fun applyLinearImpulse(impulse: Vec2, point: Vec2, wake: Boolean) {
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        if (!isAwake) {
            if (wake) {
                isAwake = true
            } else {
                return
            }
        }
        // Change linear velocity: deltaV = impulse / mass
        linearVelocity.x += impulse.x * invMass
        linearVelocity.y += impulse.y * invMass
        // Change angular velocity: deltaW = invI * (r x impulse)
        angularVelocity += invI * ((point.x - sweep.c.x) * impulse.y
                - (point.y - sweep.c.y) * impulse.x)
    }

    /**
     * Apply an angular impulse.
     *
     * @param impulse The angular impulse in units of kg*m*m/s
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L840-L857
     */
    fun applyAngularImpulse(impulse: Float) {
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        if (!isAwake) {
            isAwake = true
        }
        // Change angular velocity: deltaW = impulse / inertia
        angularVelocity += invI * impulse
    }

    /**
     * Get the mass data of the body. The rotational inertia is relative to the
     * center of mass.
     */
    fun getMassData(data: MassData) {
        data.mass = mass
        // Calculate rotational inertia about the center of mass using the parallel axis theorem if needed?
        // Actually, 'inertia' stores it about center of mass.
        // Wait, the formula here uses sweep.localCenter which implies it's shifting it?
        // Box2D logic:
        // data.I = I + mass * dot(localCenter, localCenter)
        // This effectively moves the inertia back to the body origin.
        data.i = inertia + mass * (sweep.localCenter.x * sweep.localCenter.x
                + sweep.localCenter.y * sweep.localCenter.y)
        data.center.x = sweep.localCenter.x
        data.center.y = sweep.localCenter.y
    }

    /**
     * Set the mass properties to override the mass properties of the fixtures.
     * Note that this changes the center of mass position. Note that creating or
     * destroying fixtures can also alter the mass. This function has no effect
     * if the body isn't dynamic.
     *
     * @param massData The mass properties.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L356-L395
     */
    fun setMassData(massData: MassData) {
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }
        if (type != BodyType.DYNAMIC) {
        // 1. Type Check: Only dynamic bodies can be affected by forces.
            return
        }
        // Initialize inverses.
        invMass = 0.0f
        inertia = 0.0f
        invI = 0.0f

        // Set mass.
        mass = massData.mass
        if (mass <= 0.0f) {
            mass = 1f
        }
        invMass = 1.0f / mass

        // Set inertia.
        if (massData.i > 0.0f && flags and fixedRotationFlag == 0) {
            // Center the inertia about the center of mass.
            // I_center = I_origin - mass * d^2
            inertia = massData.i - mass * Vec2.dot(massData.center, massData.center)
            assert(inertia > 0.0f)
            invI = 1.0f / inertia
        }

        // Get a temp vector from the world pool.
        val oldCenter = world.popVec2()
        // Save the old center of mass.
        oldCenter.set(sweep.c)

        // Update the local center of mass.
        sweep.localCenter.set(massData.center)

        // Update the world center of mass.
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)

        // Update center of mass velocity to preserve momentum.
        // v_new = v_old + w x (c_new - c_old)
        val temp = world.popVec2()
        temp.set(sweep.c).subLocal(oldCenter)
        Vec2.crossToOut(angularVelocity, temp, temp)
        linearVelocity.addLocal(temp)

        // Return vectors to pool.
        world.pushVec2(2)
    }

    private val pmd = MassData()

    /**
     * This resets the mass properties to the sum of the mass properties of the
     * fixtures. This normally does not need to be called unless you called
     * setMassData to override the mass, and you later want to reset the mass.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L290-L354
     */
    fun resetMassData() {
        // Compute mass data from shapes. Each shape has its own density.
        mass = 0.0f
        invMass = 0.0f
        inertia = 0.0f
        invI = 0.0f
        sweep.localCenter.setZero()

        // Static and kinematic bodies have zero mass.
        if (type == BodyType.STATIC || type == BodyType.KINEMATIC) {
            sweep.c0.set(xf.p)
            sweep.c.set(xf.p)
            sweep.a0 = sweep.a
            return
        }
        assert(type == BodyType.DYNAMIC)

        // Accumulate mass over all fixtures.
        val localCenter = world.popVec2()
        localCenter.setZero()
        val temp = world.popVec2()
        val massData = pmd
        var f = fixtureList
        while (f != null) {
            if (f.density == 0.0f) {
                f = f.next
                continue
            }
            f.getMassData(massData)
            mass += massData.mass
            // Weighted center accumulation.
            temp.set(massData.center).mulLocal(massData.mass)
            localCenter.addLocal(temp)
            inertia += massData.i
            f = f.next
        }

        // Compute center of mass.
        if (mass > 0.0f) {
            invMass = 1.0f / mass
            localCenter.mulLocal(invMass)
        } else {
            // Force all dynamic bodies to have a positive mass.
            mass = 1.0f
            invMass = 1.0f
        }

        if (inertia > 0.0f && flags and fixedRotationFlag == 0) {
            // Center the inertia about the center of mass.
            inertia -= mass * Vec2.dot(localCenter, localCenter)
            assert(inertia > 0.0f)
            invI = 1.0f / inertia
        } else {
            inertia = 0.0f
            invI = 0.0f
        }

        val oldCenter = world.popVec2()
        // Save old center.
        oldCenter.set(sweep.c)
        // Set new local center.
        sweep.localCenter.set(localCenter)
        // Update world center.
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)

        // Update velocity.
        temp.set(sweep.c).subLocal(oldCenter)
        Vec2.crossToOutUnsafe(angularVelocity, temp, oldCenter)
        linearVelocity.addLocal(oldCenter)

        // Push 3 vectors back to pool.
        world.pushVec2(3)
    }

    /**
     * Get the world coordinates of a point given the local coordinates.
     *
     * @param localPoint A point on the body measured relative the body's
     *     origin.
     *
     * @return The same point expressed in world coordinates.
     */
    /**
     * Get the world coordinates of a point given the local coordinates.
     *
     * @param localPoint A point on the body measured relative the the body's origin.
     * @return The same point expressed in world coordinates.
     */
    fun getWorldPoint(localPoint: Vec2): Vec2 {
        val v = Vec2()
        getWorldPointToOut(localPoint, v)
        return v
    }

    fun getWorldPointToOut(localPoint: Vec2, out: Vec2) {
        Transform.mulToOut(xf, localPoint, out)
    }

    /**
     * Get the world coordinates of a vector given the local coordinates.
     *
     * @param localVector A vector fixed in the body.
     *
     * @return The same vector expressed in world coordinates.
     */
    fun getWorldVector(localVector: Vec2): Vec2 {
        val out = Vec2()
        getWorldVectorToOut(localVector, out)
        return out
    }

    fun getWorldVectorToOut(localVector: Vec2, out: Vec2) {
        Rot.mulToOut(xf.q, localVector, out)
    }

    fun getWorldVectorToOutUnsafe(localVector: Vec2, out: Vec2) {
        Rot.mulToOutUnsafe(xf.q, localVector, out)
    }

    /**
     * Gets a local point relative to the body's origin given a world point.
     *
     * @param worldPoint Point in world coordinates.
     *
     * @return The corresponding local point relative to the body's origin.
     */
    fun getLocalPoint(worldPoint: Vec2): Vec2 {
        val out = Vec2()
        getLocalPointToOut(worldPoint, out)
        return out
    }

    fun getLocalPointToOut(worldPoint: Vec2, out: Vec2) {
        Transform.mulTransToOut(xf, worldPoint, out)
    }

    /**
     * Gets a local vector given a world vector.
     *
     * @param worldVector A vector in world coordinates.
     *
     * @return The corresponding local vector.
     */
    fun getLocalVector(worldVector: Vec2): Vec2 {
        val out = Vec2()
        getLocalVectorToOut(worldVector, out)
        return out
    }

    fun getLocalVectorToOut(worldVector: Vec2, out: Vec2) {
        Rot.mulTrans(xf.q, worldVector, out)
    }

    fun getLocalVectorToOutUnsafe(worldVector: Vec2, out: Vec2) {
        Rot.mulTransUnsafe(xf.q, worldVector, out)
    }

    /**
     * Get the world linear velocity of a world point attached to this body.
     *
     * @param worldPoint A point in world coordinates.
     *
     * @return The world velocity of a point.
     */
    fun getLinearVelocityFromWorldPoint(worldPoint: Vec2): Vec2 {
        val out = Vec2()
        getLinearVelocityFromWorldPointToOut(worldPoint, out)
        return out
    }

    fun getLinearVelocityFromWorldPointToOut(
        worldPoint: Vec2,
        out: Vec2
    ) {
        // v = v_cm + w x r
        val tempX = worldPoint.x - sweep.c.x
        val tempY = worldPoint.y - sweep.c.y
        out.x = -angularVelocity * tempY + linearVelocity.x
        out.y = angularVelocity * tempX + linearVelocity.y
    }

    /**
     * Get the world velocity of a local point.
     *
     * @param localPoint A point in local coordinates.
     *
     * @return The world velocity of a point.
     */
    fun getLinearVelocityFromLocalPoint(localPoint: Vec2): Vec2 {
        val out = Vec2()
        getLinearVelocityFromLocalPointToOut(localPoint, out)
        return out
    }

    fun getLinearVelocityFromLocalPointToOut(
        localPoint: Vec2,
        out: Vec2
    ) {
        getWorldPointToOut(localPoint, out)
        getLinearVelocityFromWorldPointToOut(out, out)
    }

    /**
     * Is this body treated like a bullet for continuous collision detection?
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L632-L635
     */
    var isBullet: Boolean
        get() = flags and bulletFlag == bulletFlag
        set(flag) {
            if (flag) {
                flags = flags or bulletFlag
            } else {
                flags = flags and bulletFlag.inv()
            }
        }

    /**
     * Is this body allowed to sleep?
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L688-L691
     */
    var isSleepingAllowed: Boolean
        get() = flags and autoSleepFlag == autoSleepFlag
        set(flag) {
            if (flag) {
                flags = flags or autoSleepFlag
            } else {
                flags = flags and autoSleepFlag.inv()
                isAwake = true
            }
        }

    /**
     * Set the sleep state of the body. A sleeping body has very low CPU cost.
     * Note that putting it to sleep will set its velocities and forces to zero.
     *
     * @param flag Set to true to wake the body, false to put it to sleep.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L637-L658
     */
    var isAwake: Boolean
        get() = flags and awakeFlag == awakeFlag
        set(flag) {
            if (flag) {
                if (flags and awakeFlag == 0) {
                    flags = flags or awakeFlag
                    sleepTime = 0.0f
                }
            } else {
                flags = flags and awakeFlag.inv()
                sleepTime = 0.0f
                linearVelocity.setZero()
                angularVelocity = 0.0f
                force.setZero()
                torque = 0.0f
            }
        }

    /**
     * Set the active state of the body. An inactive body is not simulated and
     * cannot be collided with or woken up. If you pass a flag of true, all
     * fixtures will be added to the broad-phase. If you pass a flag of false,
     * all fixtures will be removed from the broad-phase and all contacts will
     * be destroyed. Fixtures and joints are otherwise unaffected. You may
     * continue to create/destroy fixtures and joints on inactive bodies.
     * Fixtures on an inactive body are implicitly inactive and will not
     * participate in collisions, ray-casts, or queries. Joints connected to an
     * inactive body are implicitly inactive. An inactive body is still owned by
     * a World object and remains in the body list.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L471-L515
     */
    var isActive: Boolean
        get() = flags and activeFlag == activeFlag
        set(flag) {
            assert(!world.isLocked)
            if (flag == isActive) {
                return
            }
            if (flag) {
                flags = flags or activeFlag
                // Create all proxies.
                val broadPhase = world.contactManager.broadPhase
                var f = fixtureList
                while (f != null) {
                    f.createProxies(broadPhase, xf)
                    f = f.next
                }
                // Contacts are created the next time step.
            } else {
                flags = flags and activeFlag.inv()
                // Destroy all proxies.
                val broadPhase = world.contactManager.broadPhase
                var f = fixtureList
                while (f != null) {
                    f.destroyProxies(broadPhase)
                    f = f.next
                }
                // Destroy the attached contacts.
                var ce = contactList
                while (ce != null) {
                    val ce0 = ce
                    ce = ce!!.next
                    world.contactManager.destroy(ce0!!.contact)
                }
                contactList = null
            }
        }

    /**
     * Set this body to have fixed rotation. This causes the mass to be reset.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L517-L537
     */
    var isFixedRotation: Boolean
        get() = flags and fixedRotationFlag == fixedRotationFlag
        set(flag) {
            if (flag) {
                flags = flags or fixedRotationFlag
            } else {
                flags = flags and fixedRotationFlag.inv()
            }
            resetMassData()
        }

    // Temporary pool object for synchronization.
    private val pxf = Transform()

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L447-L469
     */
    fun synchronizeFixtures() {
        val xf1 = pxf
        // Calculate the transform at the *start* of the step (alpha0).
        // xf1.q.set(sweep.a0);
        xf1.q.s = MathUtils.sin(sweep.a0)
        xf1.q.c = MathUtils.cos(sweep.a0)
        // xf1.position = sweep.c0 - Mul(xf1.R, sweep.localCenter);
        xf1.p.x = sweep.c0.x - xf1.q.c * sweep.localCenter.x + xf1.q.s * sweep.localCenter.y
        xf1.p.y = sweep.c0.y - xf1.q.s * sweep.localCenter.x - xf1.q.c * sweep.localCenter.y

        var f = fixtureList
        while (f != null) {
            // Synchronize using the start and end transforms (swept AABB).
            f.synchronize(world.contactManager.broadPhase, xf1, xf)
            f = f.next
        }
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L859-L863
     */
    fun synchronizeTransform() {
        // xf.q.set(sweep.a);
        xf.q.s = MathUtils.sin(sweep.a)
        xf.q.c = MathUtils.cos(sweep.a)
        val q = xf.q
        val v = sweep.localCenter
        // xf.position = sweep.c - Mul(xf.R, sweep.localCenter);
        xf.p.x = sweep.c.x - q.c * v.x + q.s * v.y
        xf.p.y = sweep.c.y - q.s * v.x - q.c * v.y
    }

    /**
     * This is used to prevent connected bodies from colliding. It may lie,
     * depending on the collideConnected flag.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L397-L418
     */
    fun shouldCollide(other: Body): Boolean {
        // At least one body should be dynamic.
        if (type != BodyType.DYNAMIC && other.type != BodyType.DYNAMIC) {
            return false
        }
        // Does a joint prevent collision?
        var jn = jointList
        while (jn != null) {
            if (jn.other === other) {
                if (!jn.joint!!.collideConnected) {
                    return false
                }
            }
            jn = jn.next
        }
        return true
    }

    fun advance(t: Float) {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        sweep.advance(t)
        sweep.c.set(sweep.c0)
        sweep.a = sweep.a0
        xf.q.set(sweep.a)
        // xf.position = sweep.c - Mul(xf.R, sweep.localCenter);
        Rot.mulToOutUnsafe(xf.q, sweep.localCenter, xf.p)
        xf.p.mulLocal(-1f).addLocal(sweep.c)
    }

    companion object {
        const val islandFlag = 0x0001
        const val awakeFlag = 0x0002
        const val autoSleepFlag = 0x0004
        const val bulletFlag = 0x0008
        const val fixedRotationFlag = 0x0010
        const val activeFlag = 0x0020
        const val toiFlag = 0x0040
    }
}
