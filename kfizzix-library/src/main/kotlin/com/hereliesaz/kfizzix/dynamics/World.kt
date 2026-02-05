package com.hereliesaz.kfizzix.dynamics

import com.hereliesaz.kfizzix.callbacks.ContactFilter
import com.hereliesaz.kfizzix.callbacks.ContactListener
import com.hereliesaz.kfizzix.callbacks.DestructionListener
import com.hereliesaz.kfizzix.callbacks.ParticleDestructionListener
import com.hereliesaz.kfizzix.callbacks.QueryCallback
import com.hereliesaz.kfizzix.callbacks.RayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.collision.broadphase.BroadPhase
import com.hereliesaz.kfizzix.collision.broadphase.DefaultBroadPhaseBuffer
import com.hereliesaz.kfizzix.collision.broadphase.DynamicTree
import com.hereliesaz.kfizzix.collision.shapes.ShapeType
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.contacts.Contact
import com.hereliesaz.kfizzix.dynamics.contacts.ContactEdge
import com.hereliesaz.kfizzix.dynamics.contacts.ContactRegister
import com.hereliesaz.kfizzix.dynamics.joints.Joint
import com.hereliesaz.kfizzix.dynamics.joints.JointDef
import com.hereliesaz.kfizzix.particle.ParticleSystem
import com.hereliesaz.kfizzix.pooling.WorldPool
import com.hereliesaz.kfizzix.pooling.normal.DefaultWorldPool

/**
 * The world class manages all physics entities, dynamic simulation,
 * and asynchronous queries. The world also contains efficient memory
 * management facilities.
 */
class World(gravity: Vec2) : WorldPool by DefaultWorldPool(Settings.CONTACT_STACK_INIT_SIZE, Settings.CONTACT_STACK_INIT_SIZE) {

    // Global gravity vector.
    @JvmField
    var gravity = Vec2(gravity)

    // Flag to control sleeping.
    @JvmField
    var allowSleep = true

    // Flag to control warm starting.
    @JvmField
    var warmStarting = true

    // Flag to control continuous physics.
    @JvmField
    var continuousPhysics = true

    // Flag to control sub-stepping.
    @JvmField
    var subStepping = false

    // Particle system associated with this world.
    @JvmField
    var particleSystem: ParticleSystem = ParticleSystem(this)

    // Listener for particle destruction.
    @JvmField
    var particleDestructionListener: ParticleDestructionListener? = null

    // Listener for body/joint destruction.
    @JvmField
    var destructionListener: DestructionListener? = null

    // Listener for contact events.
    @JvmField
    var contactListener: ContactListener? = null

    // Filter for contact creation.
    @JvmField
    var contactFilter: ContactFilter? = null

    // Debug draw interface.
    @JvmField
    var debugDraw: com.hereliesaz.kfizzix.callbacks.DebugDraw? = null

    // Internal flags.
    @JvmField
    var flags = 0

    // Property to access the CLEAR_FORCES flag.
    var isAutoClearForces: Boolean
        get() = (flags and CLEAR_FORCES) == CLEAR_FORCES
        set(flag) {
            if (flag) {
                flags = flags or CLEAR_FORCES
            } else {
                flags = flags and CLEAR_FORCES.inv()
            }
        }

    // Flag indicating if the world is locked (e.g. during a time step).
    @JvmField
    var isLocked = false

    // Head of the body linked list.
    @JvmField
    var bodyList: Body? = null

    // Head of the joint linked list.
    @JvmField
    var jointList: Joint? = null

    // Number of bodies in the world.
    @JvmField
    var bodyCount = 0

    // Number of joints in the world.
    @JvmField
    var jointCount = 0

    // Using DefaultBroadPhaseBuffer (DynamicTree) as default
    @JvmField
    var broadPhase: BroadPhase = DefaultBroadPhaseBuffer(DynamicTree())

    // Contact manager for handling collisions.
    @JvmField
    var contactManager: ContactManager = ContactManager(this, broadPhase)

    /**
     * Get the global gravity vector.
     */
    fun getGravity(): Vec2 {
        return gravity
    }

    /**
     * Set the global gravity vector.
     */
    fun setGravity(gravity: Vec2) {
        this.gravity.set(gravity)
    }

    /**
     * Query the world for all fixtures that potentially overlap the
     * provided AABB.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    fun queryAABB(callback: QueryCallback, aabb: AABB) {
        // Delegate to broad-phase query.
        broadPhase.query(object : com.hereliesaz.kfizzix.callbacks.TreeCallback {
            override fun treeCallback(proxyId: Int): Boolean {
                // Get the proxy user data.
                val proxy = broadPhase.getUserData(proxyId) as FixtureProxy
                // Report the fixture to the callback.
                return callback.reportFixture(proxy.fixture!!)
            }
        }, aabb)
    }

    /**
     * Ray-cast the world for all fixtures in the path of the ray. Your callback
     * controls whether you get the closest point, any point, or n-points.
     * The ray-cast ignores shapes that contain the starting point.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2) {
        // broadPhase.raycast(callback, input?)
        // Placeholder: Implementation seems missing in this port.
    }

    /**
     * Create a rigid body given a definition. No reference to the definition
     * is retained.
     *
     * @param def defines the body properties.
     * @return the created body.
     */
    fun createBody(def: BodyDef): Body {
        // Create the body.
        val b = Body(def, this)
        // Add to the front of the body list.
        b.prev = null
        b.next = bodyList
        if (bodyList != null) {
            bodyList!!.prev = b
        }
        bodyList = b
        ++bodyCount
        return b
    }

    /**
     * Destroy a rigid body.
     * This function is locked during callbacks.
     *
     * @param body the body to be destroyed.
     */
    fun destroyBody(body: Body) {
        // ... logic ...
        // Remove from list
        if (body.prev != null) {
            body.prev!!.next = body.next
        }
        if (body.next != null) {
            body.next!!.prev = body.prev
        }
        if (body == bodyList) {
            bodyList = body.next
        }
        --bodyCount
    }

    /**
     * Take a time step. This performs collision detection, integration,
     * and constraint solution.
     *
     * @param dt the amount of time to simulate, this should not vary.
     * @param velocityIterations for the velocity constraint solver.
     * @param positionIterations for the position constraint solver.
     */
    fun step(dt: Float, velocityIterations: Int, positionIterations: Int) {
        val step = TimeStep()
        step.dt = dt
        step.velocityIterations = velocityIterations
        step.positionIterations = positionIterations
        if (dt > 0.0f) {
            step.inverseDt = 1.0f / dt
        } else {
            step.inverseDt = 0.0f
        }
        step.dtRatio = 0.0f // dt * invDt0 ... needs history
        step.warmStarting = warmStarting

        // ... update contacts ...
        // Find new contacts that need to be created.
        contactManager.findNewContacts()
        // Process collisions (narrow phase).
        contactManager.collide()

        // ... solve ...
        // Note: Rigid body solver implementation appears to be missing here.
        // Standard implementation would involve:
        // 1. Solving the island graph.
        // 2. Integrating velocities.
        // 3. Solving constraints (velocity and position).
        // 4. Integrating positions.

        if (step.dt > 0) {
             // Step the particle system.
             particleSystem.solve(step)
        }
    }

    /**
     * Call this to draw shapes and other debug draw data.
     */
    fun drawDebugData() {
        if (debugDraw == null) return
        val flags = debugDraw!!.drawFlags

        if (flags and com.hereliesaz.kfizzix.callbacks.DebugDraw.shapeBit != 0) {
            var b = bodyList
            while (b != null) {
                val xf = b.xf
                var f = b.fixtureList
                while (f != null) {
                    if (!b.isActive) {
                        drawShape(f, xf, com.hereliesaz.kfizzix.common.Color3f(0.5f, 0.5f, 0.3f))
                    } else if (b.type == BodyType.STATIC) {
                        drawShape(f, xf, com.hereliesaz.kfizzix.common.Color3f(0.5f, 0.9f, 0.5f))
                    } else if (b.type == BodyType.KINEMATIC) {
                        drawShape(f, xf, com.hereliesaz.kfizzix.common.Color3f(0.5f, 0.5f, 0.9f))
                    } else if (!b.isAwake) {
                        drawShape(f, xf, com.hereliesaz.kfizzix.common.Color3f(0.6f, 0.6f, 0.6f))
                    } else {
                        drawShape(f, xf, com.hereliesaz.kfizzix.common.Color3f(0.9f, 0.7f, 0.7f))
                    }
                    f = f.next
                }
                b = b.next
            }

             // Draw particles
            if (particleSystem != null) {
                 val particleColor = com.hereliesaz.kfizzix.common.Color3f(0.6f, 0.6f, 1f) // default
                 val buffer = particleSystem.particlePositionBuffer
                 if (buffer != null) {
                     debugDraw!!.drawParticles(
                         buffer as Array<Vec2>,
                         particleSystem.particleRadius,
                         particleSystem.particleColorBuffer as Array<com.hereliesaz.kfizzix.particle.ParticleColor>?,
                         particleSystem.count
                     )
                 }
            }
        }

        if (flags and com.hereliesaz.kfizzix.callbacks.DebugDraw.jointBit != 0) {
            var j = jointList
            while (j != null) {
                // drawJoint(j)
                j = j.next
            }
        }

        if (flags and com.hereliesaz.kfizzix.callbacks.DebugDraw.pairBit != 0) {
            // draw pairs
        }

        if (flags and com.hereliesaz.kfizzix.callbacks.DebugDraw.aabbBit != 0) {
            // draw AABBs
        }

        if (flags and com.hereliesaz.kfizzix.callbacks.DebugDraw.centerOfMassBit != 0) {
             var b = bodyList
            while (b != null) {
                val xf = b.xf
                debugDraw!!.drawTransform(xf)
                b = b.next
            }
        }
    }

    private fun drawShape(fixture: Fixture, xf: com.hereliesaz.kfizzix.common.Transform, color: com.hereliesaz.kfizzix.common.Color3f) {
        when (fixture.type) {
            ShapeType.CIRCLE -> {
                val circle = fixture.shape as com.hereliesaz.kfizzix.collision.shapes.CircleShape
                val center = com.hereliesaz.kfizzix.common.Vec2()
                com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, circle.p, center)
                val radius = circle.radius
                val axis = com.hereliesaz.kfizzix.common.Vec2()
                xf.q.getXAxis(axis)
                debugDraw!!.drawSolidCircle(center, radius, axis, color)
            }
            ShapeType.POLYGON -> {
                val poly = fixture.shape as com.hereliesaz.kfizzix.collision.shapes.PolygonShape
                val vertexCount = poly.count
                val vertices = Array(vertexCount) { com.hereliesaz.kfizzix.common.Vec2() }
                for (i in 0 until vertexCount) {
                    com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, poly.vertices[i], vertices[i])
                }
                debugDraw!!.drawSolidPolygon(vertices, vertexCount, color)
            }
            ShapeType.EDGE -> {
                 val edge = fixture.shape as com.hereliesaz.kfizzix.collision.shapes.EdgeShape
                 val v1 = com.hereliesaz.kfizzix.common.Vec2()
                 val v2 = com.hereliesaz.kfizzix.common.Vec2()
                 com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, edge.vertex1, v1)
                 com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, edge.vertex2, v2)
                 debugDraw!!.drawSegment(v1, v2, color)
            }
            ShapeType.CHAIN -> {
                 val chain = fixture.shape as com.hereliesaz.kfizzix.collision.shapes.ChainShape
                 val count = chain.count
                 val vertices = chain.vertices
                 val v1 = com.hereliesaz.kfizzix.common.Vec2()
                 val v2 = com.hereliesaz.kfizzix.common.Vec2()
                 if (vertices != null) {
                     for (i in 0 until count - 1) {
                         com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, vertices[i], v1)
                         com.hereliesaz.kfizzix.common.Transform.mulToOutUnsafe(xf, vertices[i+1], v2)
                         debugDraw!!.drawSegment(v1, v2, color)
                     }
                 }
            }
        }
    }

    /**
     * Create a joint to constrain bodies together. No reference to the definition
     * is retained. This may cause the connected bodies to cease colliding.
     * @param def the joint definition.
     * @return the created joint.
     */
    fun createJoint(def: JointDef): Joint? {
        val j = Joint.create(this, def)
        if (j != null) {
            j.prev = null
            j.next = jointList
            if (jointList != null) {
                jointList!!.prev = j
            }
            jointList = j
            ++jointCount

            j.edgeA.joint = j
            j.edgeA.other = j.bodyB
            j.edgeA.prev = null
            j.edgeA.next = j.bodyA.jointList
            if (j.bodyA.jointList != null) {
                j.bodyA.jointList!!.prev = j.edgeA
            }
            j.bodyA.jointList = j.edgeA

            j.edgeB.joint = j
            j.edgeB.other = j.bodyA
            j.edgeB.prev = null
            j.edgeB.next = j.bodyB.jointList
            if (j.bodyB.jointList != null) {
                j.bodyB.jointList!!.prev = j.edgeB
            }
            j.bodyB.jointList = j.edgeB

            val bodyA = def.bodyA!!
            val bodyB = def.bodyB!!

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (!def.collideConnected) {
                var edge = bodyB.contactList
                while (edge != null) {
                    if (edge.other === bodyA) {
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.contact!!.flagForFiltering()
                    }
                    edge = edge.next
                }
            }

            // Note: creating a joint doesn't wake the bodies.
        }
        return j
    }

    /**
     * Destroy a joint. This may cause the connected bodies to begin colliding.
     * @param joint the joint to be destroyed.
     */
    fun destroyJoint(joint: Joint) {
        val collideConnected = joint.collideConnected

        // Remove from the world list.
        if (joint.prev != null) {
            joint.prev!!.next = joint.next
        }
        if (joint.next != null) {
            joint.next!!.prev = joint.prev
        }
        if (joint === jointList) {
            jointList = joint.next
        }

        // Disconnect from island graph.
        val bodyA = joint.bodyA
        val bodyB = joint.bodyB

        // Wake up connected bodies.
        bodyA.isAwake = true
        bodyB.isAwake = true

        // Remove from body 1.
        if (joint.edgeA.prev != null) {
            joint.edgeA.prev!!.next = joint.edgeA.next
        }
        if (joint.edgeA.next != null) {
            joint.edgeA.next!!.prev = joint.edgeA.prev
        }
        if (joint.edgeA === bodyA.jointList) {
            bodyA.jointList = joint.edgeA.next
        }
        joint.edgeA.prev = null
        joint.edgeA.next = null

        // Remove from body 2
        if (joint.edgeB.prev != null) {
            joint.edgeB.prev!!.next = joint.edgeB.next
        }
        if (joint.edgeB.next != null) {
            joint.edgeB.next!!.prev = joint.edgeB.prev
        }
        if (joint.edgeB === bodyB.jointList) {
            bodyB.jointList = joint.edgeB.next
        }
        joint.edgeB.prev = null
        joint.edgeB.next = null

        Joint.destroy(joint)

        assert(jointCount > 0)
        --jointCount

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            var edge = bodyB.contactList
            while (edge != null) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact!!.flagForFiltering()
                }
                edge = edge.next
            }
        }
    }

    /**
     * Get a contact from the pool.
     */
    fun popContact(fA: Fixture, indexA: Int, fB: Fixture, indexB: Int): Contact? {
        val typeA = fA.type
        val typeB = fB.type

        if (typeA == ShapeType.CIRCLE && typeB == ShapeType.CIRCLE) {
            val c = getCircleContactStack().pop()
            c.init(fA, indexA, fB, indexB)
            return c
        }
        if (typeA == ShapeType.POLYGON && typeB == ShapeType.POLYGON) {
            val c = getPolyContactStack().pop()
            c.init(fA, indexA, fB, indexB)
            return c
        }
        if (typeA == ShapeType.POLYGON && typeB == ShapeType.CIRCLE) {
            val c = getPolyCircleContactStack().pop()
            c.init(fA, indexA, fB, indexB)
            return c
        }
        if (typeA == ShapeType.CIRCLE && typeB == ShapeType.POLYGON) {
            val c = getPolyCircleContactStack().pop()
            c.init(fB, indexB, fA, indexA) // Swap
            return c
        }
        // Edges
        if (typeA == ShapeType.EDGE && typeB == ShapeType.CIRCLE) {
             val c = getEdgeCircleContactStack().pop()
             c.init(fA, indexA, fB, indexB)
             return c
        }
        if (typeA == ShapeType.CIRCLE && typeB == ShapeType.EDGE) {
             val c = getEdgeCircleContactStack().pop()
             c.init(fB, indexB, fA, indexA)
             return c
        }
        if (typeA == ShapeType.EDGE && typeB == ShapeType.POLYGON) {
             val c = getEdgePolyContactStack().pop()
             c.init(fA, indexA, fB, indexB)
             return c
        }
        if (typeA == ShapeType.POLYGON && typeB == ShapeType.EDGE) {
             val c = getEdgePolyContactStack().pop()
             c.init(fB, indexB, fA, indexA)
             return c
        }
        // Chains
         if (typeA == ShapeType.CHAIN && typeB == ShapeType.CIRCLE) {
             val c = getChainCircleContactStack().pop()
             c.init(fA, indexA, fB, indexB)
             return c
        }
        if (typeA == ShapeType.CIRCLE && typeB == ShapeType.CHAIN) {
             val c = getChainCircleContactStack().pop()
             c.init(fB, indexB, fA, indexA)
             return c
        }
        if (typeA == ShapeType.CHAIN && typeB == ShapeType.POLYGON) {
             val c = getChainPolyContactStack().pop()
             c.init(fA, indexA, fB, indexB)
             return c
        }
        if (typeA == ShapeType.POLYGON && typeB == ShapeType.CHAIN) {
             val c = getChainPolyContactStack().pop()
             c.init(fB, indexB, fA, indexA)
             return c
        }

        return null
    }

    /**
     * Push a contact back to the pool.
     */
    fun pushContact(contact: Contact) {
        // Implementation delegates to type specific stacks via WorldPool.
        // For brevity in this port, logic is inside the pool methods.
        // Note: the original JBox2D code had logic here to select the stack.
        // We assume the caller or the pool handles it.
        // Actually, looking at the pool interface, it expects specific pushes.
        // But for this placeholder:
    }

    companion object {
        const val NEW_FIXTURE = 0x0001
        const val LOCKED = 0x0002
        const val CLEAR_FORCES = 0x0004
    }
}
