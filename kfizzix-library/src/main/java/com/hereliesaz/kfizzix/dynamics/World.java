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

import com.hereliesaz.kfizzix.callbacks.ContactFilter
import com.hereliesaz.kfizzix.callbacks.ContactListener
import com.hereliesaz.kfizzix.callbacks.DebugDraw
import com.hereliesaz.kfizzix.callbacks.DestructionListener
import com.hereliesaz.kfizzix.callbacks.ParticleDestructionListener
import com.hereliesaz.kfizzix.callbacks.ParticleQueryCallback
import com.hereliesaz.kfizzix.callbacks.ParticleRaycastCallback
import com.hereliesaz.kfizzix.callbacks.QueryCallback
import com.hereliesaz.kfizzix.callbacks.RayCastCallback
import com.hereliesaz.kfizzix.callbacks.TreeCallback
import com.hereliesaz.kfizzix.callbacks.TreeRayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.collision.TimeOfImpact.TOIInput
import com.hereliesaz.kfizzix.collision.TimeOfImpact.TOIOutput
import com.hereliesaz.kfizzix.collision.TimeOfImpact.TOIOutputState
import com.hereliesaz.kfizzix.collision.broadphase.BroadPhase
import com.hereliesaz.kfizzix.collision.broadphase.BroadPhaseStrategy
import com.hereliesaz.kfizzix.collision.broadphase.DefaultBroadPhaseBuffer
import com.hereliesaz.kfizzix.collision.broadphase.DynamicTree
import com.hereliesaz.kfizzix.collision.shapes.ChainShape
import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.collision.shapes.EdgeShape
import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.collision.shapes.ShapeType
import com.hereliesaz.kfizzix.common.Color3f
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Sweep
import com.hereliesaz.kfizzix.common.Timer
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.contacts.Contact
import com.hereliesaz.kfizzix.dynamics.contacts.ContactEdge
import com.hereliesaz.kfizzix.dynamics.contacts.ContactRegister
import com.hereliesaz.kfizzix.dynamics.contacts.ContactVelocityConstraint
import com.hereliesaz.kfizzix.dynamics.joints.Joint
import com.hereliesaz.kfizzix.dynamics.joints.JointDef
import com.hereliesaz.kfizzix.dynamics.joints.JointEdge
import com.hereliesaz.kfizzix.dynamics.joints.PulleyJoint
import com.hereliesaz.kfizzix.particle.ParticleBodyContact
import com.hereliesaz.kfizzix.particle.ParticleColor
import com.hereliesaz.kfizzix.particle.ParticleContact
import com.hereliesaz.kfizzix.particle.ParticleDef
import com.hereliesaz.kfizzix.particle.ParticleGroup
import com.hereliesaz.kfizzix.particle.ParticleGroupDef
import com.hereliesaz.kfizzix.particle.ParticleSystem
import com.hereliesaz.kfizzix.pooling.DynamicStack
import com.hereliesaz.kfizzix.pooling.WorldPool
import com.hereliesaz.kfizzix.pooling.arrays.Vec2Array
import com.hereliesaz.kfizzix.pooling.normal.DefaultWorldPool
import kotlin.math.abs

/**
 * The world-class manages all physics entities, dynamic simulation, and
 * asynchronous queries. The world also contains efficient memory management
 * facilities.
 *
 * @author Daniel Murphy
 *
 * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_world.h#L43-L346
 */
class World(gravity: Vec2, val pool: WorldPool, broadPhase: BroadPhase) {
    // statistics gathering
    var activeContacts = 0
    var contactPoolCount = 0
    var flags: Int
    val contactManager: ContactManager
    var bodyList: Body? = null
        private set
    var jointList: Joint? = null
        private set
    var bodyCount = 0
        private set
    var jointCount = 0
        private set
    var gravity = Vec2()
        private set
    var isSleepingAllowed: Boolean
    // private Body groundBody;
    var destructionListener: DestructionListener? = null
    var particleDestructionListener: ParticleDestructionListener? = null
    var debugDraw: DebugDraw? = null

    /**
     * This is used to compute the time step ratio to support a variable time
     * step.
     */
    private var invDt0: Float

    // these are for debugging the solver
    var isWarmStarting: Boolean
    var isContinuousPhysics: Boolean
    var isSubStepping = false
    private var stepComplete: Boolean
    val profile: Profile
    private val particleSystem: ParticleSystem
    private val contactStacks = Array(ShapeType.values().size) {
        arrayOfNulls<ContactRegister>(ShapeType.values().size)
    }

    /**
     * Construct a world object.
     *
     * @param gravity The world gravity vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_world.h#L49-L51
     */
    constructor(gravity: Vec2) : this(
        gravity, DefaultWorldPool(
            WORLD_POOL_SIZE,
            WORLD_POOL_CONTAINER_SIZE
        )
    )

    /**
     * Construct a world object.
     *
     * @param gravity The world gravity vector.
     * @param pool The world pool that provides pooling for all objects used in
     * the engine.
     */
    constructor(gravity: Vec2, pool: WorldPool) : this(gravity, pool, DynamicTree())

    /**
     * Construct a world object.
     *
     * @param gravity The world gravity vector.
     * @param pool The world pool that provides pooling for all objects used in
     * the engine.
     * @param strategy The broad phase strategy.
     */
    constructor(gravity: Vec2, pool: WorldPool, strategy: BroadPhaseStrategy?) : this(
        gravity,
        pool,
        DefaultBroadPhaseBuffer(strategy)
    )

    init {
        destructionListener = null
        debugDraw = null
        bodyList = null
        jointList = null
        bodyCount = 0
        jointCount = 0
        isWarmStarting = true
        isContinuousPhysics = true
        isSubStepping = false
        stepComplete = true
        isSleepingAllowed = true
        this.gravity.set(gravity)
        flags = CLEAR_FORCES
        invDt0 = 0f
        contactManager = ContactManager(this, broadPhase)
        profile = Profile()
        particleSystem = ParticleSystem(this)
        initializeRegisters()
    }

    fun setAllowSleep(flag: Boolean) {
        if (flag == isSleepingAllowed) {
            return
        }
        isSleepingAllowed = flag
        if (!isSleepingAllowed) {
            var b = bodyList
            while (b != null) {
                b.isAwake = true
                b = b.next
            }
        }
    }

    private fun addType(
        creator: DynamicStack<Contact>, type1: ShapeType,
        type2: ShapeType
    ) {
        val register = ContactRegister()
        register.creator = creator
        register.primary = true
        contactStacks[type1.ordinal][type2.ordinal] = register
        if (type1 != type2) {
            val register2 = ContactRegister()
            register2.creator = creator
            register2.primary = false
            contactStacks[type2.ordinal][type1.ordinal] = register2
        }
    }

    private fun initializeRegisters() {
        addType(
            pool.getCircleContactStack(), ShapeType.CIRCLE,
            ShapeType.CIRCLE
        )
        addType(
            pool.getPolyCircleContactStack(), ShapeType.POLYGON,
            ShapeType.CIRCLE
        )
        addType(
            pool.getPolyContactStack(), ShapeType.POLYGON,
            ShapeType.POLYGON
        )
        addType(
            pool.getEdgeCircleContactStack(), ShapeType.EDGE,
            ShapeType.CIRCLE
        )
        addType(
            pool.getEdgePolyContactStack(), ShapeType.EDGE,
            ShapeType.POLYGON
        )
        addType(
            pool.getChainCircleContactStack(), ShapeType.CHAIN,
            ShapeType.CIRCLE
        )
        addType(
            pool.getChainPolyContactStack(), ShapeType.CHAIN,
            ShapeType.POLYGON
        )
    }

    fun popContact(
        fixtureA: Fixture, indexA: Int, fixtureB: Fixture,
        indexB: Int
    ): Contact? {
        val type1 = fixtureA.type
        val type2 = fixtureB.type
        val reg = contactStacks[type1.ordinal][type2.ordinal]
        if (reg != null) {
            val c = reg.creator!!.pop()
            if (reg.primary) {
                c.init(fixtureA, indexA, fixtureB, indexB)
            } else {
                c.init(fixtureB, indexB, fixtureA, indexA)
            }
            return c
        } else {
            return null
        }
    }

    fun pushContact(contact: Contact) {
        val fixtureA = contact.fixtureA
        val fixtureB = contact.fixtureB
        if (contact.manifold.pointCount > 0 && !fixtureA!!.isSensor
            && !fixtureB!!.isSensor
        ) {
            fixtureA.body!!.isAwake = true
            fixtureB.body!!.isAwake = true
        }
        val type1 = fixtureA!!.type
        val type2 = fixtureB!!.type
        val creator = contactStacks[type1.ordinal][type2.ordinal]!!.creator
        creator!!.push(contact)
    }

    fun createBody(def: BodyDef): Body? {
        assert(!isLocked)
        if (isLocked) {
            return null
        }
        val b = Body(def, this)
        b.prev = null
        b.next = bodyList
        if (bodyList != null) {
            bodyList!!.prev = b
        }
        bodyList = b
        ++bodyCount
        return b
    }

    fun destroyBody(body: Body) {
        assert(bodyCount > 0)
        assert(!isLocked)
        if (isLocked) {
            return
        }
        var je = body.jointList
        while (je != null) {
            val je0 = je
            je = je.next
            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(je0!!.joint!!)
            }
            destroyJoint(je0!!.joint!!)
            body.jointList = je
        }
        var ce = body.contactList
        while (ce != null) {
            val ce0 = ce
            ce = ce.next
            contactManager.destroy(ce0!!.contact)
        }
        body.contactList = null
        var f = body.fixtureList
        while (f != null) {
            val f0 = f
            f = f.next
            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(f0!!)
            }
            f0!!.destroyProxies(contactManager.broadPhase)
            f0.destroy()
            body.fixtureList = f
            body.fixtureCount -= 1
        }
        body.fixtureCount = 0
        if (body.prev != null) {
            body.prev!!.next = body.next
        }
        if (body.next != null) {
            body.next!!.prev = body.prev
        }
        if (body === bodyList) {
            bodyList = body.next
        }
        --bodyCount
    }

    fun createJoint(def: JointDef): Joint? {
        assert(!isLocked)
        if (isLocked) {
            return null
        }
        val j = Joint.create(this, def)
        j!!.prev = null
        j.next = jointList
        if (jointList != null) {
            jointList!!.prev = j
        }
        jointList = j
        ++jointCount
        j.edgeA.joint = j
        j.edgeA.other = j.bodyA
        j.edgeA.prev = null
        j.edgeA.next = j.bodyA.jointList
        if (j.bodyA.jointList != null) {
            j.bodyA.jointList!!.prev = j.edgeA
        }
        j.bodyA.jointList = j.edgeA
        j.edgeB.joint = j
        j.edgeB.other = j.bodyB
        j.edgeB.prev = null
        j.edgeB.next = j.bodyB.jointList
        if (j.bodyB.jointList != null) {
            j.bodyB.jointList!!.prev = j.edgeB
        }
        j.bodyB.jointList = j.edgeB
        val bodyA = def.bodyA
        val bodyB = def.bodyB
        if (!def.collideConnected) {
            var edge = bodyB!!.contactList
            while (edge != null) {
                if (edge.other === bodyA) {
                    edge.contact!!.flagForFiltering()
                }
                edge = edge.next
            }
        }
        return j
    }

    fun destroyJoint(j: Joint) {
        assert(!isLocked)
        if (isLocked) {
            return
        }
        val collideConnected = j.collideConnected
        if (j.prev != null) {
            j.prev!!.next = j.next
        }
        if (j.next != null) {
            j.next!!.prev = j.prev
        }
        if (j === jointList) {
            jointList = j.next
        }
        val bodyA = j.bodyA
        val bodyB = j.bodyB
        bodyA.isAwake = true
        bodyB.isAwake = true
        if (j.edgeA.prev != null) {
            j.edgeA.prev!!.next = j.edgeA.next
        }
        if (j.edgeA.next != null) {
            j.edgeA.next!!.prev = j.edgeA.prev
        }
        if (j.edgeA === bodyA.jointList) {
            bodyA.jointList = j.edgeA.next
        }
        j.edgeA.prev = null
        j.edgeA.next = null
        if (j.edgeB.prev != null) {
            j.edgeB.prev!!.next = j.edgeB.next
        }
        if (j.edgeB.next != null) {
            j.edgeB.next!!.prev = j.edgeB.prev
        }
        if (j.edgeB === bodyB.jointList) {
            bodyB.jointList = j.edgeB.next
        }
        j.edgeB.prev = null
        j.edgeB.next = null
        Joint.destroy(j)
        assert(jointCount > 0)
        --jointCount
        if (!collideConnected) {
            var edge = bodyB.contactList
            while (edge != null) {
                if (edge.other === bodyA) {
                    edge.contact!!.flagForFiltering()
                }
                edge = edge.next
            }
        }
    }

    private val step = TimeStep()
    private val stepTimer = Timer()
    private val tempTimer = Timer()

    fun step(
        timeStep: Float, velocityIterations: Int,
        positionIterations: Int
    ) {
        stepTimer.reset()
        tempTimer.reset()
        if (flags and NEW_FIXTURE == NEW_FIXTURE) {
            contactManager.findNewContacts()
            flags = flags and NEW_FIXTURE.inv()
        }
        flags = flags or LOCKED
        step.dt = timeStep
        step.velocityIterations = velocityIterations
        step.positionIterations = positionIterations
        if (timeStep > 0.0f) {
            step.inverseDt = 1.0f / timeStep
        } else {
            step.inverseDt = 0.0f
        }
        step.dtRatio = invDt0 * timeStep
        step.warmStarting = isWarmStarting
        profile.stepInit.record(tempTimer.milliseconds)
        tempTimer.reset()
        contactManager.collide()
        profile.collide.record(tempTimer.milliseconds)
        if (stepComplete && step.dt > 0.0f) {
            tempTimer.reset()
            particleSystem.solve(step)
            profile.solveParticleSystem.record(tempTimer.milliseconds)
            tempTimer.reset()
            solve(step)
            profile.solve.record(tempTimer.milliseconds)
        }
        if (isContinuousPhysics && step.dt > 0.0f) {
            tempTimer.reset()
            solveTOI(step)
            profile.solveTOI.record(tempTimer.milliseconds)
        }
        if (step.dt > 0.0f) {
            invDt0 = step.inverseDt
        }
        if (flags and CLEAR_FORCES == CLEAR_FORCES) {
            clearForces()
        }
        flags = flags and LOCKED.inv()
        profile.step.record(stepTimer.milliseconds)
    }

    private fun solve(step: TimeStep) {
        profile.solveInit.startAccum()
        profile.solveVelocity.startAccum()
        profile.solvePosition.startAccum()

        var b = bodyList
        while (b != null) {
            b.flags = b.flags and Body.islandFlag.inv()
            b = b.next
        }
        var c = contactManager.contactList
        while (c != null) {
            c.flags = c.flags and Contact.ISLAND_FLAG.inv()
            c = c.next
        }
        var j = jointList
        while (j != null) {
            j.islandFlag = false
            j = j.next
        }

        val island = Island()
        island.init(bodyCount, contactManager.contactCount, jointCount, contactManager.contactListener)
        val stack = java.util.ArrayList<Body>(bodyCount)
        var seed = bodyList
        while (seed != null) {
            if (seed.flags and Body.islandFlag == Body.islandFlag) {
                seed = seed.next
                continue
            }
            if (!seed.isAwake || !seed.isActive) {
                seed = seed.next
                continue
            }
            if (seed.type == BodyType.STATIC) {
                seed = seed.next
                continue
            }
            island.clear()
            stack.add(seed)
            seed.flags = seed.flags or Body.islandFlag
            while (stack.isNotEmpty()) {
                val b = stack.removeAt(stack.size - 1)
                island.add(b)
                if (!b.isActive) {
                    continue
                }
                var ce = b.contactList
                while (ce != null) {
                    val contact = ce.contact!!
                    if (contact.flags and Contact.ISLAND_FLAG == Contact.ISLAND_FLAG) {
                        ce = ce.next
                        continue
                    }
                    if (!contact.isEnabled || !contact.isTouching) {
                        ce = ce.next
                        continue
                    }
                    val sensorA = contact.fixtureA!!.isSensor
                    val sensorB = contact.fixtureB!!.isSensor
                    if (sensorA || sensorB) {
                        ce = ce.next
                        continue
                    }
                    island.add(contact)
                    contact.flags = contact.flags or Contact.ISLAND_FLAG
                    val other = ce.other
                    if (other!!.flags and Body.islandFlag == 0) {
                        stack.add(other)
                        other.flags = other.flags or Body.islandFlag
                    }
                    ce = ce.next
                }
                var je = b.jointList
                while (je != null) {
                    if (je.joint!!.islandFlag) {
                        je = je.next
                        continue
                    }
                    val other = je.other
                    if (!other!!.isActive) {
                        je = je.next
                        continue
                    }
                    island.add(je.joint)
                    je.joint!!.islandFlag = true
                    if (other.flags and Body.islandFlag == 0) {
                        stack.add(other)
                        other.flags = other.flags or Body.islandFlag
                    }
                    je = je.next
                }
            }
            island.solve(profile, step, gravity, isSleepingAllowed)
            for (i in 0 until island.bodyCount) {
                val b = island.bodies!![i]
                if (b!!.type == BodyType.STATIC) {
                    b.flags = b.flags and Body.islandFlag.inv()
                }
            }
            seed = seed.next
        }
        profile.solveInit.endAccum()
        profile.solveVelocity.endAccum()
        profile.solvePosition.endAccum()
    }

    private fun solveTOI(step: TimeStep) {
        val island = Island()
        island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0, contactManager.contactListener)
        if (stepComplete) {
            var b = bodyList
            while (b != null) {
                b.flags = b.flags and Body.islandFlag.inv()
                b.sweep.alpha0 = 0.0f
                b = b.next
            }
            var c = contactManager.contactList
            while (c != null) {
                c.flags = c.flags and (Contact.TOI_FLAG or Contact.ISLAND_FLAG).inv()
                c.toiCount = 0f
                c.toi = 1.0f
                c = c.next
            }
        }
        // TODO: Implement TOI
    }

    fun clearForces() {
        var body = bodyList
        while (body != null) {
            body.force.setZero()
            body.torque = 0.0f
            body = body.next
        }
    }

    private val color = Color3f()
    private val xf = Transform()
    private val cA = Vec2()
    private val cB = Vec2()
    private val avs = Vec2Array()

    fun drawDebugData() {
        if (debugDraw == null) {
            return
        }
        val flags = debugDraw!!.flags
        val wireframe = flags and DebugDraw.wireframeDrawingBit != 0
        if (flags and DebugDraw.shapeBit != 0) {
            var b = bodyList
            while (b != null) {
                xf.set(b.transform)
                var f = b.fixtureList
                while (f != null) {
                    if (!b.isActive) {
                        color.set(0.5f, 0.5f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type == BodyType.STATIC) {
                        color.set(0.5f, 0.9f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type == BodyType.KINEMATIC) {
                        color.set(0.5f, 0.5f, 0.9f)
                        drawShape(f, xf, color, wireframe)
                    } else if (!b.isAwake) {
                        color.set(0.5f, 0.5f, 0.5f)
                        drawShape(f, xf, color, wireframe)
                    } else {
                        color.set(0.9f, 0.7f, 0.7f)
                        drawShape(f, xf, color, wireframe)
                    }
                    f = f.next
                }
                b = b.next
            }
            drawParticleSystem(particleSystem)
        }
        if (flags and DebugDraw.jointBit != 0) {
            var j = jointList
            while (j != null) {
                drawJoint(j)
                j = j.next
            }
        }
        if (flags and DebugDraw.pairBit != 0) {
            color.set(0.3f, 0.9f, 0.9f)
            var c = contactManager.contactList
            while (c != null) {
                val fixtureA = c.fixtureA
                val fixtureB = c.fixtureB
                fixtureA!!.getAABB(c.indexA).getCenterToOut(cA)
                fixtureB!!.getAABB(c.indexB).getCenterToOut(cB)
                debugDraw!!.drawSegment(cA, cB, color)
                c = c.next
            }
        }
        if (flags and DebugDraw.aabbBit != 0) {
            color.set(0.9f, 0.3f, 0.9f)
            var b = bodyList
            while (b != null) {
                if (!b.isActive) {
                    b = b.next
                    continue
                }
                var f = b.fixtureList
                while (f != null) {
                    for (i in 0 until f.proxyCount) {
                        val proxy = f.proxies!![i]
                        val aabb = contactManager.broadPhase
                            .getFatAABB(proxy!!.proxyId)
                        if (aabb != null) {
                            val vs = avs[4]
                            vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y)
                            vs[1].set(aabb.upperBound.x, aabb.lowerBound.y)
                            vs[2].set(aabb.upperBound.x, aabb.upperBound.y)
                            vs[3].set(aabb.lowerBound.x, aabb.upperBound.y)
                            debugDraw!!.drawPolygon(vs, 4, color)
                        }
                    }
                    f = f.next
                }
                b = b.next
            }
        }
        if (flags and DebugDraw.centerOfMassBit != 0) {
            var b = bodyList
            while (b != null) {
                xf.set(b.transform)
                xf.p.set(b.worldCenter)
                debugDraw!!.drawTransform(xf)
                b = b.next
            }
        }
        if (flags and DebugDraw.dynamicTreeBit != 0) {
            contactManager.broadPhase.drawTree(debugDraw!!)
        }
        debugDraw!!.flush()
    }

    private fun drawJoint(joint: Joint) {
        // Placeholder
    }

    private fun drawShape(fixture: Fixture, xf: Transform, color: Color3f, wireframe: Boolean) {
        when (fixture.type) {
            ShapeType.CIRCLE -> {
                val circle = fixture.shape as CircleShape
                val center = xf.mul(circle.p)
                val radius = circle.radius
                val axis = xf.q.getXAxis(Vec2())
                if (wireframe) {
                    debugDraw!!.drawCircle(center, radius, color)
                    debugDraw!!.drawSegment(center, center + axis * radius, color)
                } else {
                    debugDraw!!.drawSolidCircle(center, radius, axis, color)
                }
            }
            ShapeType.POLYGON -> {
                val poly = fixture.shape as PolygonShape
                val vertexCount = poly.count
                val vertices = avs[vertexCount]
                for (i in 0 until vertexCount) {
                    vertices[i] = xf.mul(poly.vertices[i]!!)
                }
                if (wireframe) {
                    debugDraw!!.drawPolygon(vertices, vertexCount, color)
                } else {
                    debugDraw!!.drawSolidPolygon(vertices, vertexCount, color)
                }
            }
            ShapeType.EDGE -> {
                val edge = fixture.shape as EdgeShape
                val v1 = xf.mul(edge.vertex1)
                val v2 = xf.mul(edge.vertex2)
                debugDraw!!.drawSegment(v1, v2, color)
            }
            ShapeType.CHAIN -> {
                val chain = fixture.shape as ChainShape
                val count = chain.count
                val vertices = chain.vertices
                var v1 = xf.mul(vertices!![0]!!)
                for (i in 1 until count) {
                    val v2 = xf.mul(vertices[i]!!)
                    debugDraw!!.drawSegment(v1, v2, color)
                    debugDraw!!.drawCircle(v1, 0.05f, color)
                    v1 = v2
                }
            }
            else -> {}
        }
    }

    private fun drawParticleSystem(system: ParticleSystem) {
        // Placeholder
    }

    private val wqwrapper = WorldQueryWrapper()

    fun queryAABB(callback: QueryCallback, aabb: AABB) {
        wqwrapper.broadPhase = contactManager.broadPhase
        wqwrapper.callback = callback
        contactManager.broadPhase.query(wqwrapper, aabb)
    }

    fun queryAABB(
        callback: QueryCallback?,
        particleCallback: ParticleQueryCallback, aabb: AABB
    ) {
        if (callback != null) {
            wqwrapper.broadPhase = contactManager.broadPhase
            wqwrapper.callback = callback
            contactManager.broadPhase.query(wqwrapper, aabb)
        }
        particleSystem.queryAABB(particleCallback, aabb)
    }

    private val wrcwrapper = WorldRayCastWrapper()
    private val input = RayCastInput()

    fun raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2) {
        wrcwrapper.broadPhase = contactManager.broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        contactManager.broadPhase.raycast(wrcwrapper, input)
    }

    fun raycast(
        callback: RayCastCallback,
        particleCallback: ParticleRaycastCallback, point1: Vec2, point2: Vec2
    ) {
        wrcwrapper.broadPhase = contactManager.broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        contactManager.broadPhase.raycast(wrcwrapper, input)
        particleSystem.raycast(particleCallback, point1, point2)
    }

    val contactList: Contact?
        get() = contactManager.contactList

    val proxyCount: Int
        get() = contactManager.broadPhase.proxyCount

    val treeHeight: Int
        get() = contactManager.broadPhase.treeHeight

    val treeBalance: Int
        get() = contactManager.broadPhase.treeBalance

    val treeQuality: Float
        get() = contactManager.broadPhase.treeQuality

    val isLocked: Boolean
        get() = flags and LOCKED == LOCKED

    var autoClearForces: Boolean
        get() = flags and CLEAR_FORCES == CLEAR_FORCES
        set(flag) {
            if (flag) {
                flags = flags or CLEAR_FORCES
            } else {
                flags = flags and CLEAR_FORCES.inv()
            }
        }

    val particleFlagsBuffer: IntArray
        get() = particleSystem.particleFlagsBuffer!!
    val particlePositionBuffer: Array<Vec2?>?
        get() = particleSystem.particlePositionBuffer
    val particleVelocityBuffer: Array<Vec2?>?
        get() = particleSystem.particleVelocityBuffer
    val particleColorBuffer: Array<ParticleColor?>?
        get() = particleSystem.particleColorBuffer
    val particleGroupBuffer: Array<ParticleGroup?>?
        get() = particleSystem.particleGroupList
    val particleUserDataBuffer: Array<Any?>?
        get() = particleSystem.particleUserDataBuffer

    fun setParticleFlagsBuffer(buffer: IntArray?, capacity: Int) {
        particleSystem.setParticleFlagsBuffer(buffer, capacity)
    }

    fun setParticlePositionBuffer(buffer: Array<Vec2?>?, capacity: Int) {
        particleSystem.setParticlePositionBuffer(buffer, capacity)
    }

    fun setParticleVelocityBuffer(buffer: Array<Vec2?>?, capacity: Int) {
        particleSystem.setParticleVelocityBuffer(buffer, capacity)
    }

    fun setParticleColorBuffer(buffer: Array<ParticleColor?>?, capacity: Int) {
        particleSystem.setParticleColorBuffer(buffer, capacity)
    }

    fun setParticleUserDataBuffer(buffer: Array<Any?>?, capacity: Int) {
        particleSystem.setParticleUserDataBuffer(buffer, capacity)
    }

    val particleContacts: Array<ParticleContact?>?
        get() = particleSystem.contactBuffer
    val particleContactCount: Int
        get() = particleSystem.contactCount

    val particleBodyContacts: Array<ParticleBodyContact?>?
        get() = particleSystem.bodyContactBuffer
    val particleBodyContactCount: Int
        get() = particleSystem.bodyContactCount

    fun computeParticleCollisionEnergy(): Float {
        // return particleSystem.computeParticleCollisionEnergy()
        return 0f
    }

    companion object {
        const val WORLD_POOL_SIZE = 100
        const val WORLD_POOL_CONTAINER_SIZE = 10
        const val NEW_FIXTURE = 0x0001
        const val LOCKED = 0x0002
        const val CLEAR_FORCES = 0x0004
    }

    private class WorldQueryWrapper : TreeCallback {
        override fun treeCallback(proxyId: Int): Boolean {
            val proxy = broadPhase!!.getUserData(proxyId) as FixtureProxy
            return callback!!.reportFixture(proxy.fixture!!)
        }
        var broadPhase: BroadPhase? = null
        var callback: QueryCallback? = null
    }

    private class WorldRayCastWrapper : TreeRayCastCallback {
        override fun raycastCallback(input: RayCastInput, nodeId: Int): Float {
            val userData = broadPhase!!.getUserData(nodeId)
            val proxy = userData as FixtureProxy
            val fixture = proxy.fixture
            val index = proxy.childIndex
            val output = RayCastOutput()
            val hit = fixture!!.raycast(output, input, index)
            if (hit) {
                val fraction = output.fraction
                val point = input.p1 + (input.p2 - input.p1) * fraction
                return callback!!.reportRayFixture(fixture, point, output.normal, fraction)
            }
            return input.maxFraction
        }
        var broadPhase: BroadPhase? = null
        var callback: RayCastCallback? = null
    }
}
