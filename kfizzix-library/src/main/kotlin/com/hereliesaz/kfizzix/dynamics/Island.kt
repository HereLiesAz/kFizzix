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

import com.hereliesaz.kfizzix.callbacks.ContactImpulse
import com.hereliesaz.kfizzix.callbacks.ContactListener
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Timer
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.contacts.Contact
import com.hereliesaz.kfizzix.dynamics.contacts.ContactSolver
import com.hereliesaz.kfizzix.dynamics.contacts.ContactVelocityConstraint
import com.hereliesaz.kfizzix.dynamics.contacts.Position
import com.hereliesaz.kfizzix.dynamics.contacts.Velocity
import com.hereliesaz.kfizzix.dynamics.joints.Joint

/**
 * This is an internal class.
 *
 * @author Daniel Murphy
 */
class Island {
    var listener: ContactListener? = null
    var bodies: Array<Body?>? = null
    var contacts: Array<Contact?>? = null
    var joints: Array<Joint?>? = null
    var positions: Array<Position?>? = null
    var velocities: Array<Velocity?>? = null
    var bodyCount = 0
    var jointCount = 0
    var contactCount = 0
    var bodyCapacity = 0
    var contactCapacity = 0
    var jointCapacity = 0

    fun init(
        bodyCapacity: Int, contactCapacity: Int, jointCapacity: Int,
        listener: ContactListener?
    ) {
        // System.out.println("Initializing Island");
        this.bodyCapacity = bodyCapacity
        this.contactCapacity = contactCapacity
        this.jointCapacity = jointCapacity
        bodyCount = 0
        contactCount = 0
        jointCount = 0
        this.listener = listener
        if (bodies == null || this.bodyCapacity > bodies!!.size) {
            bodies = arrayOfNulls(this.bodyCapacity)
        }
        if (joints == null || this.jointCapacity > joints!!.size) {
            joints = arrayOfNulls(this.jointCapacity)
        }
        if (contacts == null || this.contactCapacity > contacts!!.size) {
            contacts = arrayOfNulls(this.contactCapacity)
        }
        // dynamic array
        if (velocities == null || this.bodyCapacity > velocities!!.size) {
            val old = if (velocities == null) arrayOfNulls(0) else velocities
            velocities = arrayOfNulls(this.bodyCapacity)
            System.arraycopy(old!!, 0, velocities!!, 0, old.size)
            for (i in old.size until velocities!!.size) {
                velocities!![i] = Velocity()
            }
        }
        // dynamic array
        if (positions == null || this.bodyCapacity > positions!!.size) {
            val old = if (positions == null) arrayOfNulls(0) else positions
            positions = arrayOfNulls(this.bodyCapacity)
            System.arraycopy(old!!, 0, positions!!, 0, old.size)
            for (i in old.size until positions!!.size) {
                positions!![i] = Position()
            }
        }
    }

    fun clear() {
        bodyCount = 0
        contactCount = 0
        jointCount = 0
    }

    private val contactSolver = ContactSolver()
    private val timer = Timer()
    private val solverData = SolverData()
    private val solverDef = ContactSolver.ContactSolverDef()
    fun solve(
        profile: Profile, step: TimeStep, gravity: Vec2,
        allowSleep: Boolean
    ) {
        // System.out.println("Solving Island");
        val h = step.dt
        // Integrate velocities and apply damping. Initialize the body state.
        for (i in 0 until bodyCount) {
            val b = bodies!![i]
            val bmSweep = b!!.sweep
            val c = bmSweep.c
            val a = bmSweep.a
            val v = b.linearVelocity
            var w = b.angularVelocity
            // Store positions for continuous collision.
            bmSweep.c0.set(bmSweep.c)
            bmSweep.a0 = bmSweep.a
            if (b.type == BodyType.DYNAMIC) {
                // Integrate velocities.
                // v += h * (b.gravityScale * gravity + b.invMass *
                // b.force);
                v.x += h * (b.gravityScale * gravity.x + b.invMass * b.force.x)
                v.y += h * (b.gravityScale * gravity.y + b.invMass * b.force.y)
                w += h * b.invI * b.torque
                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c
                // * t) * exp(-c * dt) = v *
                // exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v.x *= 1.0f / (1.0f + h * b.linearDamping)
                v.y *= 1.0f / (1.0f + h * b.linearDamping)
                w *= 1.0f / (1.0f + h * b.angularDamping)
            }
            positions!![i]!!.c.x = c.x
            positions!![i]!!.c.y = c.y
            positions!![i]!!.a = a
            velocities!![i]!!.v.x = v.x
            velocities!![i]!!.v.y = v.y
            velocities!![i]!!.w = w
        }
        timer.reset()
        // Solver data
        solverData.step = step
        @Suppress("UNCHECKED_CAST")
        solverData.positions = positions as Array<Position>
        @Suppress("UNCHECKED_CAST")
        solverData.velocities = velocities as Array<Velocity>
        // Initialize velocity constraints.
        solverDef.step = step
        solverDef.contacts = contacts as Array<Contact>
        solverDef.count = contactCount
        @Suppress("UNCHECKED_CAST")
        solverDef.positions = positions as Array<Position>
        @Suppress("UNCHECKED_CAST")
        solverDef.velocities = velocities as Array<Velocity>
        contactSolver.init(solverDef)
        // System.out.println("island init vel");
        contactSolver.initializeVelocityConstraints()
        if (step.warmStarting) {
            // System.out.println("island warm start");
            contactSolver.warmStart()
        }
        for (i in 0 until jointCount) {
            joints!![i]!!.initVelocityConstraints(solverData)
        }
        profile.solveInit.accum(timer.milliseconds)
        // Solve velocity constraints
        timer.reset()
        // System.out.println("island solving velocities");
        for (i in 0 until step.velocityIterations) {
            for (j in 0 until jointCount) {
                joints!![j]!!.solveVelocityConstraints(solverData)
            }
            contactSolver.solveVelocityConstraints()
        }
        // Store impulses for warm starting
        contactSolver.storeImpulses()
        profile.solveVelocity.accum(timer.milliseconds)
        // Integrate positions
        for (i in 0 until bodyCount) {
            val c = positions!![i]!!.c
            var a = positions!![i]!!.a
            val v = velocities!![i]!!.v
            var w = velocities!![i]!!.w
            // Check for large velocities
            val translationx = v.x * h
            val translationy = v.y * h
            if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
                val ratio = Settings.maxTranslation / MathUtils.sqrt(
                    translationx * translationx
                            + translationy * translationy
                )
                v.x *= ratio
                v.y *= ratio
            }
            val rotation = h * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                val ratio = Settings.maxRotation / MathUtils.abs(rotation)
                w *= ratio
            }
            // Integrate
            c.x += h * v.x
            c.y += h * v.y
            a += h * w
            positions!![i]!!.a = a
            velocities!![i]!!.w = w
        }
        // Solve position constraints
        timer.reset()
        var positionSolved = false
        for (i in 0 until step.positionIterations) {
            val contactsOkay = contactSolver.solvePositionConstraints()
            var jointsOkay = true
            for (j in 0 until jointCount) {
                val jointOkay = joints!![j]!!.solvePositionConstraints(solverData)
                jointsOkay = jointsOkay && jointOkay
            }
            if (contactsOkay && jointsOkay) {
                // Exit early if the position errors are small.
                positionSolved = true
                break
            }
        }
        // Copy state buffers back to the bodies
        for (i in 0 until bodyCount) {
            val body = bodies!![i]
            body!!.sweep.c.x = positions!![i]!!.c.x
            body.sweep.c.y = positions!![i]!!.c.y
            body.sweep.a = positions!![i]!!.a
            body.linearVelocity.x = velocities!![i]!!.v.x
            body.linearVelocity.y = velocities!![i]!!.v.y
            body.angularVelocity = velocities!![i]!!.w
            body.synchronizeTransform()
        }
        profile.solvePosition.accum(timer.milliseconds)
        report(contactSolver.velocityConstraints)
        if (allowSleep) {
            var minSleepTime = Float.MAX_VALUE
            val linTolSqr = Settings.linearSleepTolerance * Settings.linearSleepTolerance
            val angTolSqr = Settings.angularSleepTolerance * Settings.angularSleepTolerance
            for (i in 0 until bodyCount) {
                val b = bodies!![i]
                if (b!!.type == BodyType.STATIC) {
                    continue
                }
                if (b.flags and Body.autoSleepFlag == 0 || b.angularVelocity * b.angularVelocity > angTolSqr || Vec2.dot(
                        b.linearVelocity,
                        b.linearVelocity
                    ) > linTolSqr
                ) {
                    b.sleepTime = 0.0f
                    minSleepTime = 0.0f
                } else {
                    b.sleepTime += h
                    minSleepTime = MathUtils.min(minSleepTime, b.sleepTime)
                }
            }
            if (minSleepTime >= Settings.timeToSleep && positionSolved) {
                for (i in 0 until bodyCount) {
                    val b = bodies!![i]
                    b!!.isAwake = false
                }
            }
        }
    }

    private val toiContactSolver = ContactSolver()
    private val toiSolverDef = ContactSolver.ContactSolverDef()
    fun solveTOI(subStep: TimeStep, toiIndexA: Int, toiIndexB: Int) {
        assert(toiIndexA < bodyCount)
        assert(toiIndexB < bodyCount)
        // Initialize the body state.
        for (i in 0 until bodyCount) {
            positions!![i]!!.c.x = bodies!![i]!!.sweep.c.x
            positions!![i]!!.c.y = bodies!![i]!!.sweep.c.y
            positions!![i]!!.a = bodies!![i]!!.sweep.a
            velocities!![i]!!.v.x = bodies!![i]!!.linearVelocity.x
            velocities!![i]!!.v.y = bodies!![i]!!.linearVelocity.y
            velocities!![i]!!.w = bodies!![i]!!.angularVelocity
        }
        toiSolverDef.contacts = contacts
        toiSolverDef.count = contactCount
        toiSolverDef.step = subStep
        toiSolverDef.positions = positions as Array<Position>
        toiSolverDef.velocities = velocities as Array<Velocity>
        toiContactSolver.init(toiSolverDef)
        // Solve position constraints.
        for (i in 0 until subStep.positionIterations) {
            val contactsOkay = toiContactSolver
                .solveTOIPositionConstraints(toiIndexA, toiIndexB)
            if (contactsOkay) {
                break
            }
        }
        // #if 0
        // // Is the new position really safe?
        // for (int i = 0; i < contactCount; ++i)
        // {
        // Contact* c = contacts[i];
        // Fixture* fA = c.GetFixtureA();
        // Fixture* fB = c.GetFixtureB();
        //
        // Body bA = fA.GetBody();
        // Body bB = fB.GetBody();
        //
        // int indexA = c.GetChildIndexA();
        // int indexB = c.GetChildIndexB();
        //
        // DistanceInput input;
        // input.proxyA.Set(fA.GetShape(), indexA);
        // input.proxyB.Set(fB.GetShape(), indexB);
        // input.transformA = bA.GetTransform();
        // input.transformB = bB.GetTransform();
        // input.useRadii = false;
        //
        // DistanceOutput output;
        // SimplexCache cache;
        // cache.count = 0;
        // Distance(&output, &cache, &input);
        //
        // if (output.distance == 0 || cache.count == 3)
        // {
        // cache.count += 0;
        // }
        // }
        // #endif
        // Leap of faith to new safe state.
        bodies!![toiIndexA]!!.sweep.c0.x = positions!![toiIndexA]!!.c.x
        bodies!![toiIndexA]!!.sweep.c0.y = positions!![toiIndexA]!!.c.y
        bodies!![toiIndexA]!!.sweep.a0 = positions!![toiIndexA]!!.a
        bodies!![toiIndexB]!!.sweep.c0.set(positions!![toiIndexB]!!.c)
        bodies!![toiIndexB]!!.sweep.a0 = positions!![toiIndexB]!!.a
        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        toiContactSolver.initializeVelocityConstraints()
        // Solve velocity constraints.
        for (i in 0 until subStep.velocityIterations) {
            toiContactSolver.solveVelocityConstraints()
        }
        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.
        val h = subStep.dt
        // Integrate positions
        for (i in 0 until bodyCount) {
            val c = positions!![i]!!.c
            var a = positions!![i]!!.a
            val v = velocities!![i]!!.v
            var w = velocities!![i]!!.w
            // Check for large velocities
            val translationx = v.x * h
            val translationy = v.y * h
            if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
                val ratio = Settings.maxTranslation / MathUtils.sqrt(
                    translationx * translationx
                            + translationy * translationy
                )
                v.mulLocal(ratio)
            }
            val rotation = h * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                val ratio = Settings.maxRotation / MathUtils.abs(rotation)
                w *= ratio
            }
            // Integrate
            c.x += v.x * h
            c.y += v.y * h
            a += h * w
            positions!![i]!!.c.x = c.x
            positions!![i]!!.c.y = c.y
            positions!![i]!!.a = a
            velocities!![i]!!.v.x = v.x
            velocities!![i]!!.v.y = v.y
            velocities!![i]!!.w = w
            // Sync bodies
            val body = bodies!![i]
            body!!.sweep.c.x = c.x
            body.sweep.c.y = c.y
            body.sweep.a = a
            body.linearVelocity.x = v.x
            body.linearVelocity.y = v.y
            body.angularVelocity = w
            body.synchronizeTransform()
        }
        report(toiContactSolver.velocityConstraints)
    }

    fun add(body: Body?) {
        assert(bodyCount < bodyCapacity)
        body!!.islandIndex = bodyCount
        bodies!![bodyCount] = body
        ++bodyCount
    }

    fun add(contact: Contact?) {
        assert(contactCount < contactCapacity)
        contacts!![contactCount++] = contact
    }

    fun add(joint: Joint?) {
        assert(jointCount < jointCapacity)
        joints!![jointCount++] = joint
    }

    private val impulse = ContactImpulse()
    fun report(constraints: Array<ContactVelocityConstraint>) {
        if (listener == null) {
            return
        }
        for (i in 0 until contactCount) {
            val c = contacts!![i]
            val vc = constraints[i]
            impulse.count = vc.pointCount
            for (j in 0 until vc.pointCount) {
                impulse.normalImpulses[j] = vc.points[j]!!.normalImpulse
                impulse.tangentImpulses[j] = vc.points[j]!!.tangentImpulse
            }
            listener!!.postSolve(c!!, impulse)
        }
    }
}
