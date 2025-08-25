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
package com.hereliesaz.kbox2d.serialization.pb

import com.hereliesaz.kbox2d.collision.shapes.*
import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.*
import com.hereliesaz.kbox2d.dynamics.joints.*
import com.hereliesaz.kbox2d.serialization.*
import com.hereliesaz.kbox2d.serialization.UnsupportedObjectException.Type
import org.box2d.proto.Box2D.*
import java.io.IOException
import java.io.OutputStream

/**
 * Protobuffer serializer implementation.
 *
 * @author Daniel Murphy
 */
class PbSerializer : JbSerializer {
    private var signer: JbSerializer.ObjectSigner? = null
    private var listener: UnsupportedListener? = null

    constructor()
    constructor(argListener: UnsupportedListener?) {
        listener = argListener
    }

    constructor(argSigner: JbSerializer.ObjectSigner?) {
        signer = argSigner
    }

    constructor(argListener: UnsupportedListener?, argSigner: JbSerializer.ObjectSigner?) {
        listener = argListener
        signer = argSigner
    }

    override fun setObjectSigner(argSigner: JbSerializer.ObjectSigner) {
        signer = argSigner
    }

    override fun setUnsupportedListener(argListener: UnsupportedListener) {
        listener = argListener
    }

    override fun serialize(argWorld: World): SerializationResult {
        val world = serializeWorld(argWorld).build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(argOutputStream: OutputStream) {
                world.writeTo(argOutputStream)
            }

            override fun getValue(): Any {
                return world
            }
        }
    }

    fun serializeWorld(argWorld: World): PbWorld.Builder {
        val builder = PbWorld.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(argWorld)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.setGravity(vecToPb(argWorld.gravity))
        builder.autoClearForces = argWorld.isAutoClearForces
        builder.allowSleep = argWorld.isAllowSleep
        builder.isContinuousPhysics = argWorld.isContinuousPhysics
        builder.isWarmStarting = argWorld.isWarmStarting
        builder.isSubStepping = argWorld.isSubStepping
        var cbody = argWorld.bodyList
        var cnt = 0
        val bodies = HashMap<Body, Int>()
        while (cbody != null) {
            builder.addBodies(serializeBody(cbody))
            bodies[cbody] = cnt
            cnt++
            cbody = cbody.next
        }
        cnt = 0
        val joints = HashMap<Joint, Int>()
        var cjoint = argWorld.jointList
        // first pass
        while (cjoint != null) {
            if (SerializationHelper.isIndependentJoint(cjoint.type)) {
                builder.addJoints(serializeJoint(cjoint, bodies, joints))
                joints[cjoint] = cnt
                cnt++
            }
            cjoint = cjoint.next
        }
        // second pass for dependent joints
        cjoint = argWorld.jointList
        while (cjoint != null) {
            if (!SerializationHelper.isIndependentJoint(cjoint.type)) {
                builder.addJoints(serializeJoint(cjoint, bodies, joints))
                joints[cjoint] = cnt
                cnt++
            }
            cjoint = cjoint.next
        }
        return builder
    }

    override fun serialize(argBody: Body): SerializationResult {
        val builder = serializeBody(argBody)
        val body = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(argOutputStream: OutputStream) {
                body.writeTo(argOutputStream)
            }

            override fun getValue(): Any {
                return body
            }
        }
    }

    fun serializeBody(argBody: Body): PbBody.Builder {
        val builder = PbBody.newBuilder()
        if (signer != null) {
            val id = signer!!.getTag(argBody)
            if (id != null) {
                builder.tag = id
            }
        }
        when (argBody.type) {
            BodyType.DYNAMIC -> builder.type = PbBodyType.DYNAMIC
            BodyType.KINEMATIC -> builder.type = PbBodyType.KINEMATIC
            BodyType.STATIC -> builder.type = PbBodyType.STATIC
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown body type: " + argBody.type, Type.BODY)
                if (listener == null || listener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        builder.setPosition(vecToPb(argBody.position))
        builder.angle = argBody.angle
        builder.setLinearVelocity(vecToPb(argBody.linearVelocity))
        builder.angularVelocity = argBody.angularVelocity
        builder.linearDamping = argBody.linearDamping
        builder.angularDamping = argBody.angularDamping
        builder.gravityScale = argBody.gravityScale
        builder.bullet = argBody.isBullet
        builder.allowSleep = argBody.isSleepingAllowed
        builder.awake = argBody.isAwake
        builder.isActive = argBody.isActive
        builder.fixedRotation = argBody.isFixedRotation
        var curr = argBody.fixtureList
        while (curr != null) {
            builder.addFixtures(serializeFixture(curr))
            curr = curr.next
        }
        return builder
    }

    override fun serialize(argFixture: Fixture): SerializationResult {
        val fixture = serializeFixture(argFixture).build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(argOutputStream: OutputStream) {
                fixture.writeTo(argOutputStream)
            }

            override fun getValue(): Any {
                return fixture
            }
        }
    }

    fun serializeFixture(argFixture: Fixture): PbFixture.Builder {
        val builder = PbFixture.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(argFixture)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.density = argFixture.density
        builder.friction = argFixture.friction
        builder.restitution = argFixture.restitution
        builder.sensor = argFixture.isSensor
        builder.setShape(serializeShape(argFixture.shape))
        builder.setFilter(serializeFilter(argFixture.filter))
        return builder
    }

    override fun serialize(argShape: Shape): SerializationResult {
        val builder = serializeShape(argShape)
// should we do lazy building?
        val shape = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(argOutputStream: OutputStream) {
                shape.writeTo(argOutputStream)
            }

            override fun getValue(): Any {
                return shape
            }
        }
    }

    fun serializeShape(argShape: Shape): PbShape.Builder {
        val builder = PbShape.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(argShape)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.radius = argShape.radius
        when (argShape.type) {
            ShapeType.CIRCLE -> {
                val c = argShape as CircleShape
                builder.type = PbShapeType.CIRCLE
                builder.setCenter(vecToPb(c.p))
            }
            ShapeType.POLYGON -> {
                val p = argShape as PolygonShape
                builder.type = PbShapeType.POLYGON
                builder.setCentroid(vecToPb(p.centroid))
                for (i in 0 until p.count) {
                    builder.addPoints(vecToPb(p.vertices[i]))
                    builder.addNormals(vecToPb(p.normals[i]))
                }
            }
            ShapeType.EDGE -> {
                val e = argShape as EdgeShape
                builder.type = PbShapeType.EDGE
                builder.setV0(vecToPb(e.vertex0))
                builder.setV1(vecToPb(e.vertex1))
                builder.setV2(vecToPb(e.vertex2))
                builder.setV3(vecToPb(e.vertex3))
                builder.setHas0(e.m_hasVertex0)
                builder.setHas3(e.m_hasVertex3)
            }
            ShapeType.CHAIN -> {
                val h = argShape as ChainShape
                builder.type = PbShapeType.CHAIN
                for (i in 0 until h.m_count) {
                    builder.addPoints(vecToPb(h.m_vertices!![i]))
                }
                builder.setPrev(vecToPb(h.m_prevVertex))
                builder.setNext(vecToPb(h.m_nextVertex))
                builder.setHas0(h.m_hasPrevVertex)
                builder.setHas3(h.m_hasNextVertex)
            }
            else -> {
                val ex = UnsupportedObjectException(
                        "Currently only encodes circle and polygon shapes",
                        Type.SHAPE)
                if (listener == null || listener!!.isUnsupported(ex)) {
                    throw ex
                }
                throw ex
            }
        }
        return builder
    }

    override fun serialize(argJoint: Joint, argBodyIndexMap: Map<Body, Int>,
                           argJointIndexMap: Map<Joint, Int>): SerializationResult {
        val builder = serializeJoint(argJoint, argBodyIndexMap,
                argJointIndexMap)
        val joint = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(argOutputStream: OutputStream) {
                joint.writeTo(argOutputStream)
            }

            override fun getValue(): Any {
                return joint
            }
        }
    }

    fun serializeJoint(joint: Joint,
                       argBodyIndexMap: Map<Body, Int>,
                       argJointIndexMap: Map<Joint, Int>): PbJoint.Builder {
        val builder = PbJoint.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(joint)
            if (tag != null) {
                builder.tag = tag
            } else {
                builder.clearTag()
            }
        }
        val bA = joint.bodyA
        val bB = joint.bodyB
        if (!argBodyIndexMap.containsKey(bA)) {
            throw IllegalArgumentException(
                    "Body $bA is not present in the index map")
        }
        builder.setBodyA(argBodyIndexMap[bA]!!)
        if (!argBodyIndexMap.containsKey(bB)) {
            throw IllegalArgumentException(
                    "Body $bB is not present in the index map")
        }
        builder.setBodyB(argBodyIndexMap[bB]!!)
        builder.collideConnected = joint.collideConnected
        when (joint.type) {
            JointType.REVOLUTE -> {
                val j = joint as RevoluteJoint
                builder.type = PbJointType.REVOLUTE
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.refAngle = j.referenceAngle
                builder.enableLimit = j.isLimitEnabled
                builder.lowerLimit = j.lowerLimit
                builder.upperLimit = j.upperLimit
                builder.enableMotor = j.isMotorEnabled
                builder.motorSpeed = j.motorSpeed
                builder.maxMotorTorque = j.maxMotorTorque
            }
            JointType.PRISMATIC -> {
                val j = joint as PrismaticJoint
                builder.type = PbJointType.PRISMATIC
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.setLocalAxisA(vecToPb(j.localAxisA))
                builder.refAngle = j.referenceAngle
                builder.enableLimit = j.isLimitEnabled
                builder.lowerLimit = j.lowerLimit
                builder.upperLimit = j.upperLimit
                builder.enableMotor = j.isMotorEnabled
                builder.maxMotorForce = j.maxMotorForce
                builder.motorSpeed = j.motorSpeed
            }
            JointType.DISTANCE -> {
                val j = joint as DistanceJoint
                builder.type = PbJointType.DISTANCE
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.length = j.length
                builder.frequency = j.frequency
                builder.dampingRatio = j.dampingRatio
            }
            JointType.PULLEY -> {
                val j = joint as PulleyJoint
                builder.type = PbJointType.PULLEY
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.setGroundAnchorA(vecToPb(j.groundAnchorA))
                builder.setGroundAnchorB(vecToPb(j.groundAnchorB))
                builder.setLengthA(j.lengthA)
                builder.setLengthB(j.lengthB)
                builder.ratio = j.ratio
            }
            JointType.MOUSE -> {
                val j = joint as MouseJoint
                builder.type = PbJointType.MOUSE
                builder.setTarget(vecToPb(j.target))
                builder.maxForce = j.maxForce
                builder.frequency = j.frequency
                builder.dampingRatio = j.dampingRatio
            }
            JointType.GEAR -> {
                val j = joint as GearJoint
                builder.type = PbJointType.GEAR
                builder.ratio = j.ratio
                if (!argJointIndexMap.containsKey(j.joint1)) {
                    throw IllegalArgumentException("Joint 1 not in map")
                }
                val j1 = argJointIndexMap[j.joint1]!!
                if (!argJointIndexMap.containsKey(j.joint2)) {
                    throw IllegalArgumentException("Joint 2 not in map")
                }
                val j2 = argJointIndexMap[j.joint2]!!
                builder.setJoint1(j1)
                builder.setJoint2(j2)
            }
            JointType.FRICTION -> {
                val j = joint as FrictionJoint
                builder.type = PbJointType.FRICTION
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.maxForce = j.maxForce
                builder.maxTorque = j.maxTorque
            }
            JointType.CONSTANT_VOLUME -> {
                val j = joint as ConstantVolumeJoint
                builder.type = PbJointType.CONSTANT_VOLUME
                for (i in j.bodies.indices) {
                    val b = j.bodies[i]
                    val distanceJoint = j.joints[i]
                    if (!argBodyIndexMap.containsKey(b)) {
                        throw IllegalArgumentException(
                                "Body $b is not present in the index map")
                    }
                    builder.addBodies(argBodyIndexMap[b]!!)
                    if (!argJointIndexMap.containsKey(distanceJoint)) {
                        throw IllegalArgumentException("Joint $distanceJoint is not present in the index map")
                    }
                    builder.addJoints(argJointIndexMap[distanceJoint]!!)
                }
            }
            JointType.WHEEL -> {
                val j = joint as WheelJoint
                builder.type = PbJointType.WHEEL
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.setLocalAxisA(vecToPb(j.localAxisA))
                builder.enableMotor = j.isMotorEnabled
                builder.maxMotorTorque = j.maxMotorTorque
                builder.motorSpeed = j.motorSpeed
                builder.frequency = j.springFrequencyHz
                builder.dampingRatio = j.springDampingRatio
            }
            JointType.ROPE -> {
                val j = joint as RopeJoint
                builder.type = PbJointType.ROPE
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.maxLength = j.maxLength
            }
            JointType.WELD -> {
                val j = joint as WeldJoint
                builder.type = PbJointType.WELD
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.refAngle = j.referenceAngle
                builder.frequency = j.frequency
                builder.dampingRatio = j.dampingRatio
            }
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown joint type: " + joint.type, Type.JOINT)
                if (listener == null || listener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        return builder
    }

    fun serializeFilter(argFilter: Filter): PbFilter.Builder {
        val builder = PbFilter.newBuilder()
        builder.categoryBits = argFilter.categoryBits
        builder.groupIndex = argFilter.groupIndex
        builder.maskBits = argFilter.maskBits
        return builder
    }

    private fun vecToPb(argVec: Vec2?): PbVec2? {
        if (argVec == null) {
            return null
        }
        return PbVec2.newBuilder().setX(argVec.x).setY(argVec.y).build()
    }
}
