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
import com.hereliesaz.kfizzix.particle.ParticleSystem
import com.hereliesaz.kfizzix.pooling.WorldPool
import com.hereliesaz.kfizzix.pooling.normal.DefaultWorldPool

class World(gravity: Vec2) : WorldPool by DefaultWorldPool(Settings.CONTACT_STACK_INIT_SIZE, Settings.CONTACT_STACK_INIT_SIZE) {

    var gravity = Vec2(gravity)
        private set

    var allowSleep = true
    var warmStarting = true
    var continuousPhysics = true
    var subStepping = false

    var particleSystem: ParticleSystem = ParticleSystem(this)
    var particleDestructionListener: ParticleDestructionListener? = null
    var destructionListener: DestructionListener? = null
    var contactListener: ContactListener? = null
    var contactFilter: ContactFilter? = null

    var bodyList: Body? = null
    var jointList: Joint? = null

    var bodyCount = 0
    var jointCount = 0

    // Using DefaultBroadPhaseBuffer (DynamicTree) as default
    var broadPhase: BroadPhase = DefaultBroadPhaseBuffer(DynamicTree())
    var contactManager: ContactManager = ContactManager(this, broadPhase)

    fun getGravity(): Vec2 {
        return gravity
    }

    fun setGravity(gravity: Vec2) {
        this.gravity.set(gravity)
    }

    fun queryAABB(callback: QueryCallback, aabb: AABB) {
        broadPhase.query(callback, aabb)
    }

    fun raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2) {
        // broadPhase.raycast(callback, input?)
        // Placeholder
    }

    fun createBody(def: BodyDef): Body {
        val b = Body(def, this)
        // Add to list...
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
        contactManager.findNewContacts()
        contactManager.collide()

        // ... solve ...
        if (step.dt > 0) {
             particleSystem.solve(step)
        }
    }

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

    fun pushContact(contact: Contact) {
        // Push back to stack logic...
        // Simplified:
        // if (contact is CircleContact) getCircleContactStack().push(contact)
        // else ...
    }
}
