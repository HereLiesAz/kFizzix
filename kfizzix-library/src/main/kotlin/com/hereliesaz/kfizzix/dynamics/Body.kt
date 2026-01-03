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
class Body(bd: BodyDef, @JvmField var world: World) {

    /**
     * The body type: static, kinematic, or dynamic. Note: if a dynamic body
     * would have zero mass, the mass is set to one.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L72-L74
     */
    var type: BodyType
    var flags = 0
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
     * The swept motion for CCD
     */
    val sweep = Sweep()
    val linearVelocity = Vec2()

    /**
     * The angular velocity of the body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L86-L87
     */
    var angularVelocity = 0f
    val force = Vec2()
    var torque = 0f

    /**
     * The previous body in the world's body list.
     */
    var prev: Body? = null

    /**
     * The next body in the world's body list.
     */
    var next: Body? = null
    var fixtureList: Fixture? = null
    var fixtureCount = 0
    var jointList: JointEdge? = null
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
        assert(bd.position.isValid())
        assert(bd.linearVelocity.isValid())
        assert(bd.gravityScale >= 0.0f)
        assert(bd.angularDamping >= 0.0f)
        assert(bd.linearDamping >= 0.0f)
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
        xf.p.set(bd.position)
        xf.q.set(bd.angle)
        sweep.localCenter.setZero()
        sweep.c0.set(xf.p)
        sweep.c.set(xf.p)
        sweep.a0 = bd.angle
        sweep.a = bd.angle
        sweep.alpha0 = 0.0f
        jointList = null
        contactList = null
        prev = null
        next = null
        linearVelocity.set(bd.linearVelocity)
        angularVelocity = bd.angularVelocity
        linearDamping = bd.linearDamping
        angularDamping = bd.angularDamping
        gravityScale = bd.gravityScale
        force.setZero()
        torque = 0.0f
        sleepTime = 0.0f
        type = bd.type
        if (type == BodyType.DYNAMIC) {
            mass = 1f
            invMass = 1f
        } else {
            mass = 0f
            invMass = 0f
        }
        inertia = 0.0f
        invI = 0.0f
        userData = bd.userData
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
    fun createFixture(def: FixtureDef): Fixture? {
        assert(!world.isLocked)
        if (world.isLocked) {
            return null
        }
        val fixture = Fixture()
        fixture.create(this, def)
        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture.createProxies(broadPhase, xf)
        }
        fixture.next = fixtureList
        fixtureList = fixture
        ++fixtureCount
        fixture.body = this
        // Adjust mass properties if needed.
        if (fixture.density > 0.0f) {
            resetMassData()
        }
        // Let the world know we have a new fixture. This will cause new
        // contacts
        // to be created at the beginning of the next time step.
        // TODO: check if NEW_FIXTURE exists in World
        // world.flags = world.flags or World.NEW_FIXTURE
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
        fixDef.shape = shape
        fixDef.density = density
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
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }
        assert(fixture!!.body === this)
        // Remove the fixture from this body's singly linked list.
        assert(fixtureCount > 0)
        var node = fixtureList
        var last: Fixture? = null // java change
        var found = false
        while (node != null) {
            if (node === fixture) {
                node = fixture!!.next
                found = true
                break
            }
            last = node
            node = node.next
        }
        // You tried to remove a shape that is not attached to this body.
        assert(found)
        // java change, remove it from the list
        if (last == null) {
            fixtureList = fixture!!.next
        } else {
            last.next = fixture!!.next
        }
        // Destroy any contacts associated with the fixture.
        var edge = contactList
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
        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture!!.destroyProxies(broadPhase)
        }
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
    fun setTransform(position: Vec2, angle: Float) {
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }
        xf.q.set(angle)
        xf.p.set(position)
        // sweep.c0 = sweep.c = Mul(xf, sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c)
        sweep.a = angle
        sweep.c0.set(sweep.c)
        sweep.a0 = sweep.a
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
        if (type == BodyType.STATIC) {
            return
        }
        if (Vec2.dot(v, v) > 0.0f) {
            isAwake = true
        }
        linearVelocity.set(v)
    }

    /**
     * Set the angular velocity.
     *
     * @param w The new angular velocity in radians/second.
     */
    fun setAngularVelocity(w: Float) {
        if (type == BodyType.STATIC) {
            return
        }
        if (w * w > 0f) {
            isAwake = true
        }
        angularVelocity = w
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
    fun applyForce(force: Vec2, point: Vec2) {
        if (type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake) {
            isAwake = true
        }
        // force.addLocal(force);
        // Vec2 temp = tltemp.get();
        // temp.set(point).subLocal(sweep.c);
        // torque += Vec2.cross(temp, force);
        this.force.x += force.x
        this.force.y += force.y
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
    fun applyTorque(torque: Float) {
        if (type != BodyType.DYNAMIC) {
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
    fun applyLinearImpulse(impulse: Vec2, point: Vec2, wake: Boolean) {
        if (type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake) {
            if (wake) {
                isAwake = true
            } else {
                return
            }
        }
        linearVelocity.x += impulse.x * invMass
        linearVelocity.y += impulse.y * invMass
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
            return
        }
        if (!isAwake) {
            isAwake = true
        }
        angularVelocity += invI * impulse
    }

    /**
     * Get the mass data of the body. The rotational inertia is relative to the
     * center of mass.
     */
    fun getMassData(data: MassData) {
        // data.mass = mass;
        // data.I = I + mass * Vec2.dot(sweep.localCenter,
        // sweep.localCenter);
        // data.center.set(sweep.localCenter);
        data.mass = mass
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
        // TODO_ERIN adjust linear velocity and torque to account for movement
        // of center.
        assert(!world.isLocked)
        if (world.isLocked) {
            return
        }
        if (type != BodyType.DYNAMIC) {
            return
        }
        invMass = 0.0f
        inertia = 0.0f
        invI = 0.0f
        mass = massData.mass
        if (mass <= 0.0f) {
            mass = 1f
        }
        invMass = 1.0f / mass
        if (massData.i > 0.0f && flags and fixedRotationFlag == 0) {
            inertia = massData.i - mass * Vec2.dot(massData.center, massData.center)
            assert(inertia > 0.0f)
            invI = 1.0f / inertia
        }
        val oldCenter = world.popVec2()
        // Move center of mass.
        oldCenter.set(sweep.c)
        sweep.localCenter.set(massData.center)
        // sweep.c0 = sweep.c = Mul(xf, sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)
        // Update center of mass velocity.
        // linearVelocity += Cross(angularVelocity, sweep.c - oldCenter);
        val temp = world.popVec2()
        temp.set(sweep.c).subLocal(oldCenter)
        Vec2.crossToOut(angularVelocity, temp, temp)
        linearVelocity.addLocal(temp)
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
            // sweep.c0 = sweep.c = xf.position;
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
            // center += massData.mass * massData.center;
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
        // Move center of mass.
        oldCenter.set(sweep.c)
        sweep.localCenter.set(localCenter)
        // sweep.c0 = sweep.c = Mul(xf, sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)
        // Update center of mass velocity.
        // linearVelocity += Cross(angularVelocity, sweep.c - oldCenter);
        temp.set(sweep.c).subLocal(oldCenter)
        Vec2.crossToOutUnsafe(angularVelocity, temp, oldCenter)
        linearVelocity.addLocal(oldCenter)
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

    // djm pooling
    private val pxf = Transform()

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_body.cpp#L447-L469
     */
    fun synchronizeFixtures() {
        val xf1 = pxf
        // xf1.position = sweep.c0 - Mul(xf1.R, sweep.localCenter);
        // xf1.q.set(sweep.a0);
        // Rot.mulToOutUnsafe(xf1.q, sweep.localCenter, xf1.p);
        // xf1.p.mulLocal(-1).addLocal(sweep.c0);
        // inlined:
        xf1.q.s = MathUtils.sin(sweep.a0)
        xf1.q.c = MathUtils.cos(sweep.a0)
        xf1.p.x = sweep.c0.x - xf1.q.c * sweep.localCenter.x + xf1.q.s * sweep.localCenter.y
        xf1.p.y = sweep.c0.y - xf1.q.s * sweep.localCenter.x - xf1.q.c * sweep.localCenter.y
        // end inline
        var f = fixtureList
        while (f != null) {
            f.synchronize(world.contactManager.broadPhase, xf1, xf)
            f = f.next
        }
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L859-L863
     */
    fun synchronizeTransform() {
        // xf.q.set(sweep.a);
        //
        // // xf.position = sweep.c - Mul(xf.R, sweep.localCenter);
        // Rot.mulToOutUnsafe(xf.q, sweep.localCenter, xf.p);
        // xf.p.mulLocal(-1).addLocal(sweep.c);
        //
        xf.q.s = MathUtils.sin(sweep.a)
        xf.q.c = MathUtils.cos(sweep.a)
        val q = xf.q
        val v = sweep.localCenter
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
