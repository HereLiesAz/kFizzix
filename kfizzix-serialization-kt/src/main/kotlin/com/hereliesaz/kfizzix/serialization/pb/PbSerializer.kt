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
package com.hereliesaz.kfizzix.serialization.pb

import com.hereliesaz.kfizzix.collision.shapes.*
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.*
import com.hereliesaz.kfizzix.dynamics.joints.*
import com.hereliesaz.kfizzix.serialization.*
import com.hereliesaz.kfizzix.serialization.UnsupportedObjectException.Type
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
    constructor(listener: UnsupportedListener?) {
        this.listener = listener
    }

    constructor(signer: JbSerializer.ObjectSigner?) {
        this.signer = signer
    }

    constructor(listener: UnsupportedListener?, signer: JbSerializer.ObjectSigner?) {
        this.listener = listener
        this.signer = signer
    }

    override fun setObjectSigner(signer: JbSerializer.ObjectSigner) {
        this.signer = signer
    }

    override fun setUnsupportedListener(listener: UnsupportedListener) {
        this.listener = listener
    }

    override fun serialize(world: World): SerializationResult {
        val pbWorld = serializeWorld(world).build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(outputStream: OutputStream) {
                pbWorld.writeTo(outputStream)
            }

            override fun getValue(): Any {
                return pbWorld
            }
        }
    }

    fun serializeWorld(world: World): PbWorld.Builder {
        val builder = PbWorld.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(world)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.setGravity(vecToPb(world.gravity))
        builder.autoClearForces = world.isAutoClearForces
        builder.allowSleep = world.allowSleep
        builder.continuousPhysics = world.continuousPhysics
        builder.warmStarting = world.warmStarting
        builder.subStepping = world.subStepping
        var cbody = world.bodyList
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
        var cjoint = world.jointList
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
        cjoint = world.jointList
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

    override fun serialize(body: Body): SerializationResult {
        val builder = serializeBody(body)
        val pbBody = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(outputStream: OutputStream) {
                pbBody.writeTo(outputStream)
            }

            override fun getValue(): Any {
                return pbBody
            }
        }
    }

    fun serializeBody(body: Body): PbBody.Builder {
        val builder = PbBody.newBuilder()
        if (signer != null) {
            val id = signer!!.getTag(body)
            if (id != null) {
                builder.tag = id
            }
        }
        when (body.type) {
            BodyType.DYNAMIC -> builder.type = PbBodyType.DYNAMIC
            BodyType.KINEMATIC -> builder.type = PbBodyType.KINEMATIC
            BodyType.STATIC -> builder.type = PbBodyType.STATIC
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown body type: " + body.type, Type.BODY)
                if (listener == null || listener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        builder.setPosition(vecToPb(body.position))
        builder.angle = body.angle
        builder.setLinearVelocity(vecToPb(body.linearVelocity))
        builder.angularVelocity = body.angularVelocity
        builder.linearDamping = body.linearDamping
        builder.angularDamping = body.angularDamping
        builder.gravityScale = body.gravityScale
        builder.bullet = body.isBullet
        builder.allowSleep = body.isSleepingAllowed
        builder.awake = body.isAwake
        builder.active = body.isActive
        builder.fixedRotation = body.isFixedRotation
        var curr = body.fixtureList
        while (curr != null) {
            builder.addFixtures(serializeFixture(curr))
            curr = curr.next
        }
        return builder
    }

    override fun serialize(fixture: Fixture): SerializationResult {
        val pbFixture = serializeFixture(fixture).build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(outputStream: OutputStream) {
                pbFixture.writeTo(outputStream)
            }

            override fun getValue(): Any {
                return pbFixture
            }
        }
    }

    fun serializeFixture(fixture: Fixture): PbFixture.Builder {
        val builder = PbFixture.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(fixture)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.density = fixture.density
        builder.friction = fixture.friction
        builder.restitution = fixture.restitution
        builder.sensor = fixture.isSensor
        builder.setShape(serializeShape(fixture.shape!!))
        builder.setFilter(serializeFilter(fixture.filter!!))
        return builder
    }

    override fun serialize(shape: Shape): SerializationResult {
        val builder = serializeShape(shape)
// should we do lazy building?
        val pbShape = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(outputStream: OutputStream) {
                pbShape.writeTo(outputStream)
            }

            override fun getValue(): Any {
                return pbShape
            }
        }
    }

    fun serializeShape(shape: Shape): PbShape.Builder {
        val builder = PbShape.newBuilder()
        if (signer != null) {
            val tag = signer!!.getTag(shape)
            if (tag != null) {
                builder.tag = tag
            }
        }
        builder.radius = shape.radius
        when (shape.type) {
            ShapeType.CIRCLE -> {
                val c = shape as CircleShape
                builder.type = PbShapeType.CIRCLE
                builder.setCenter(vecToPb(c.p))
            }
            ShapeType.POLYGON -> {
                val p = shape as PolygonShape
                builder.type = PbShapeType.POLYGON
                builder.setCentroid(vecToPb(p.centroid))
                for (i in 0 until p.count) {
                    builder.addPoints(vecToPb(p.vertices[i]))
                    builder.addNormals(vecToPb(p.normals[i]))
                }
            }
            ShapeType.EDGE -> {
                val e = shape as EdgeShape
                builder.type = PbShapeType.EDGE
                builder.setV0(vecToPb(e.vertex0))
                builder.setV1(vecToPb(e.vertex1))
                builder.setV2(vecToPb(e.vertex2))
                builder.setV3(vecToPb(e.vertex3))
                builder.setHas0(e.hasVertex0)
                builder.setHas3(e.hasVertex3)
            }
            ShapeType.CHAIN -> {
                val h = shape as ChainShape
                builder.type = PbShapeType.CHAIN
                for (i in 0 until h.count) {
                    builder.addPoints(vecToPb(h.vertices!![i]))
                }
                builder.setPrev(vecToPb(h.prevVertex))
                builder.setNext(vecToPb(h.nextVertex))
                builder.setHas0(h.hasPrevVertex)
                builder.setHas3(h.hasNextVertex)
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

    override fun serialize(joint: Joint, bodyIndexMap: Map<Body, Int>,
                           jointIndexMap: Map<Joint, Int>): SerializationResult {
        val builder = serializeJoint(joint, bodyIndexMap,
                jointIndexMap)
        val pbJoint = builder.build()
        return object : SerializationResult {
            @Throws(IOException::class)
            override fun writeTo(outputStream: OutputStream) {
                pbJoint.writeTo(outputStream)
            }

            override fun getValue(): Any {
                return pbJoint
            }
        }
    }

    fun serializeJoint(joint: Joint,
                       bodyIndexMap: Map<Body, Int>,
                       jointIndexMap: Map<Joint, Int>): PbJoint.Builder {
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
        if (!bodyIndexMap.containsKey(bA)) {
            throw IllegalArgumentException(
                    "Body $bA is not present in the index map")
        }
        builder.setBodyA(bodyIndexMap[bA]!!)
        if (!bodyIndexMap.containsKey(bB)) {
            throw IllegalArgumentException(
                    "Body $bB is not present in the index map")
        }
        builder.setBodyB(bodyIndexMap[bB]!!)
        builder.collideConnected = joint.collideConnected
        when (joint.type) {
            JointType.REVOLUTE -> {
                val j = joint as RevoluteJoint
                builder.type = PbJointType.REVOLUTE
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.refAngle = j.referenceAngle
                builder.enableLimit = j.isLimitEnabled()
                builder.lowerLimit = j.getLowerLimit()
                builder.upperLimit = j.getUpperLimit()
                builder.enableMotor = j.isMotorEnabled()
                builder.motorSpeed = j.motorSpeed
                builder.maxMotorTorque = j.maxMotorTorque
            }
            JointType.PRISMATIC -> {
                val j = joint as PrismaticJoint
                builder.type = PbJointType.PRISMATIC
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.setLocalAxisA(vecToPb(j.localXAxisA))
                builder.refAngle = j.referenceAngle
                builder.enableLimit = j.isLimitEnabled()
                builder.lowerLimit = j.getLowerLimit()
                builder.upperLimit = j.getUpperLimit()
                builder.enableMotor = j.isMotorEnabled()
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
                builder.setTarget(vecToPb(j.getTarget()))
                builder.maxForce = j.getMaxForce()
                builder.frequency = j.getFrequency()
                builder.dampingRatio = j.getDampingRatio()
            }
            JointType.GEAR -> {
                val j = joint as GearJoint
                builder.type = PbJointType.GEAR
                builder.ratio = j.ratio
                if (!jointIndexMap.containsKey(j.joint1)) {
                    throw IllegalArgumentException("Joint 1 not in map")
                }
                val j1 = jointIndexMap[j.joint1]!!
                if (!jointIndexMap.containsKey(j.joint2)) {
                    throw IllegalArgumentException("Joint 2 not in map")
                }
                val j2 = jointIndexMap[j.joint2]!!
                builder.setJoint1(j1)
                builder.setJoint2(j2)
            }
            JointType.FRICTION -> {
                val j = joint as FrictionJoint
                builder.type = PbJointType.FRICTION
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.maxForce = j.getMaxForce()
                builder.maxTorque = j.getMaxTorque()
            }
            JointType.CONSTANT_VOLUME -> {
                val j = joint as ConstantVolumeJoint
                builder.type = PbJointType.CONSTANT_VOLUME
                for (i in j.bodies.indices) {
                    val b = j.bodies[i]
                    val distanceJoint = j.joints[i]
                    if (!bodyIndexMap.containsKey(b)) {
                        throw IllegalArgumentException(
                                "Body $b is not present in the index map")
                    }
                    builder.addBodies(bodyIndexMap[b]!!)
                    if (!jointIndexMap.containsKey(distanceJoint)) {
                        throw IllegalArgumentException("Joint $distanceJoint is not present in the index map")
                    }
                    builder.addJoints(jointIndexMap[distanceJoint]!!)
                }
            }
            JointType.WHEEL -> {
                val j = joint as WheelJoint
                builder.type = PbJointType.WHEEL
                builder.setLocalAnchorA(vecToPb(j.localAnchorA))
                builder.setLocalAnchorB(vecToPb(j.localAnchorB))
                builder.setLocalAxisA(vecToPb(j.localXAxisA))
                builder.enableMotor = j.isMotorEnabled()
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

    fun serializeFilter(filter: Filter): PbFilter.Builder {
        val builder = PbFilter.newBuilder()
        builder.categoryBits = filter.categoryBits
        builder.groupIndex = filter.groupIndex
        builder.maskBits = filter.maskBits
        return builder
    }

    private fun vecToPb(vec: Vec2?): PbVec2? {
        if (vec == null) {
            return null
        }
        return PbVec2.newBuilder().setX(vec.x).setY(vec.y).build()
    }
}
