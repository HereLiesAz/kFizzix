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
import com.hereliesaz.kfizzix.serialization.JbDeserializer
import com.hereliesaz.kfizzix.serialization.UnsupportedListener
import com.hereliesaz.kfizzix.serialization.UnsupportedObjectException
import com.hereliesaz.kfizzix.serialization.UnsupportedObjectException.Type
import org.box2d.proto.Box2D.*
import java.io.IOException
import java.io.InputStream

class PbDeserializer : JbDeserializer {
    private var listener: JbDeserializer.ObjectListener? = null
    private var unsupportedlistener: UnsupportedListener? = null

    constructor()
    constructor(listener: UnsupportedListener?) {
        unsupportedlistener = listener
    }

    constructor(objectListener: JbDeserializer.ObjectListener?) {
        listener = objectListener
    }

    constructor(listener: UnsupportedListener?, objectListener: JbDeserializer.ObjectListener?) {
        unsupportedlistener = listener
        this.listener = objectListener
    }

    override fun setObjectListener(listener: JbDeserializer.ObjectListener) {
        this.listener = listener
    }

    override fun setUnsupportedListener(unsupportedListener: UnsupportedListener) {
        this.unsupportedlistener = unsupportedListener
    }

    private fun isIndependentJoint(argType: PbJointType): Boolean {
        return argType != PbJointType.GEAR && argType != PbJointType.CONSTANT_VOLUME
    }

    @Throws(IOException::class)
    override fun deserializeWorld(input: InputStream): World {
        val world = PbWorld.parseFrom(input)
        return deserializeWorld(world)
    }

    fun deserializeWorld(pbWorld: PbWorld): World {
        val world = World(pbToVec(pbWorld.gravity))
        world.isAutoClearForces = pbWorld.autoClearForces
        world.continuousPhysics = pbWorld.continuousPhysics
        world.warmStarting = pbWorld.warmStarting
        world.subStepping = pbWorld.subStepping
        val bodyMap = HashMap<Int, Body>()
        val jointMap = HashMap<Int, Joint>()
        for (i in 0 until pbWorld.bodiesCount) {
            val pbBody = pbWorld.getBodies(i)
            val body = deserializeBody(world, pbBody)
            bodyMap[i] = body
        }
        // first pass, independent joints
        var cnt = 0
        for (i in 0 until pbWorld.jointsCount) {
            val pbJoint = pbWorld.getJoints(i)
            if (isIndependentJoint(pbJoint.type)) {
                val joint = deserializeJoint(world, pbJoint, bodyMap,
                        jointMap)
                jointMap[cnt] = joint
                cnt++
            }
        }
        // second pass, dependent joints
        for (i in 0 until pbWorld.jointsCount) {
            val pbJoint = pbWorld.getJoints(i)
            if (!isIndependentJoint(pbJoint.type)) {
                val joint = deserializeJoint(world, pbJoint, bodyMap,
                        jointMap)
                jointMap[cnt] = joint
                cnt++
            }
        }
        if (listener != null && pbWorld.hasTag()) {
            listener!!.processWorld(world, pbWorld.tag)
        }
        return world
    }

    @Throws(IOException::class)
    override fun deserializeBody(world: World, input: InputStream): Body {
        val body = PbBody.parseFrom(input)
        return deserializeBody(world, body)
    }

    fun deserializeBody(world: World, pbBody: PbBody): Body {
        val bd = BodyDef()
        bd.position.set(pbToVec(pbBody.position))
        bd.angle = pbBody.angle
        bd.linearDamping = pbBody.linearDamping
        bd.angularDamping = pbBody.angularDamping
        bd.gravityScale = pbBody.gravityScale
        // velocities are populated after fixture addition
        bd.bullet = pbBody.bullet
        bd.allowSleep = pbBody.allowSleep
        bd.awake = pbBody.awake
        bd.active = pbBody.active
        bd.fixedRotation = pbBody.fixedRotation
        when (pbBody.type) {
            PbBodyType.DYNAMIC -> bd.type = BodyType.DYNAMIC
            PbBodyType.KINEMATIC -> bd.type = BodyType.KINEMATIC
            PbBodyType.STATIC -> bd.type = BodyType.STATIC
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown body type: " + pbBody.type, Type.BODY)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        val body = world.createBody(bd)
        for (i in 0 until pbBody.fixturesCount) {
            deserializeFixture(body, pbBody.getFixtures(i))
        }
        // adding fixtures can change this, so we put this here and set it
// directly in the body
        body.linearVelocity.set(pbToVec(pbBody.linearVelocity))
        body.angularVelocity = pbBody.angularVelocity
        if (listener != null && pbBody.hasTag()) {
            listener!!.processBody(body, pbBody.tag)
        }
        return body
    }

    @Throws(IOException::class)
    override fun deserializeFixture(body: Body, input: InputStream): Fixture {
        val fixture = PbFixture.parseFrom(input)
        return deserializeFixture(body, fixture)
    }

    fun deserializeFixture(body: Body, pbFixture: PbFixture): Fixture {
        val fd = FixtureDef()
        fd.density = pbFixture.density
        fd.filter.categoryBits = pbFixture.filter.categoryBits
        fd.filter.groupIndex = pbFixture.filter.groupIndex
        fd.filter.maskBits = pbFixture.filter.maskBits
        fd.friction = pbFixture.friction
        fd.isSensor = pbFixture.sensor
        fd.restitution = pbFixture.restitution
        fd.shape = deserializeShape(pbFixture.shape)
        val fixture = body.createFixture(fd)
        if (fixture != null && listener != null && pbFixture.hasTag()) {
            listener!!.processFixture(fixture, pbFixture.tag)
        }
        return fixture!!
    }

    @Throws(IOException::class)
    override fun deserializeShape(input: InputStream): Shape {
        val s = PbShape.parseFrom(input)
        return deserializeShape(s)
    }

    fun deserializeShape(pbShape: PbShape): Shape {
        val shape: Shape
        when (pbShape.type) {
            PbShapeType.CIRCLE -> {
                val c = CircleShape()
                c.p.set(pbToVec(pbShape.center))
                shape = c
            }
            PbShapeType.POLYGON -> {
                val p = PolygonShape()
                p.centroid.set(pbToVec(pbShape.centroid))
                p.count = pbShape.pointsCount
                for (i in 0 until p.count) {
                    p.vertices[i].set(pbToVec(pbShape.getPoints(i)))
                    p.normals[i].set(pbToVec(pbShape.getNormals(i)))
                }
                shape = p
            }
            PbShapeType.EDGE -> {
                val edge = EdgeShape()
                edge.vertex0.set(pbToVec(pbShape.v0))
                edge.vertex1.set(pbToVec(pbShape.v1))
                edge.vertex2.set(pbToVec(pbShape.v2))
                edge.vertex3.set(pbToVec(pbShape.v3))
                edge.hasVertex0 = pbShape.has0
                edge.hasVertex3 = pbShape.has3
                shape = edge
            }
            PbShapeType.CHAIN -> {
                val chain = ChainShape()
                chain.count = pbShape.pointsCount
                chain.vertices = Array(chain.count) { Vec2() }
                for (i in 0 until chain.count) {

                    chain.vertices!![i] = Vec2(pbToVec(pbShape.getPoints(i)))

                }
                chain.hasPrevVertex = pbShape.has0
                chain.hasNextVertex = pbShape.has3
                chain.prevVertex.set(pbToVec(pbShape.prev))
                chain.nextVertex.set(pbToVec(pbShape.next))
                shape = chain
            }
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown shape type: " + pbShape.type, Type.SHAPE)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        shape.radius = pbShape.radius
        if (listener != null && pbShape.hasTag()) {
            listener!!.processShape(shape, pbShape.tag)
        }
        return shape
    }

    @Throws(IOException::class)
    override fun deserializeJoint(world: World, input: InputStream,
                                  bodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>): Joint {
        val joint = PbJoint.parseFrom(input)
        return deserializeJoint(world, joint, bodyMap, jointMap)
    }

    fun deserializeJoint(world: World, joint: PbJoint,
                         bodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>): Joint {
        val jd: JointDef
        when (joint.type) {
            PbJointType.PRISMATIC -> {
                val def = PrismaticJointDef()
                jd = def
                def.enableLimit = joint.enableLimit
                def.enableMotor = joint.enableMotor
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.localAxisA.set(pbToVec(joint.localAxisA))
                def.lowerTranslation = joint.lowerLimit
                def.maxMotorForce = joint.maxMotorForce
                def.motorSpeed = joint.motorSpeed
                def.referenceAngle = joint.refAngle
                def.upperTranslation = joint.upperLimit
            }
            PbJointType.REVOLUTE -> {
                val def = RevoluteJointDef()
                jd = def
                def.enableLimit = joint.enableLimit
                def.enableMotor = joint.enableMotor
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.lowerAngle = joint.lowerLimit
                def.maxMotorTorque = joint.maxMotorTorque
                def.motorSpeed = joint.motorSpeed
                def.referenceAngle = joint.refAngle
                def.upperAngle = joint.upperLimit
            }
            PbJointType.DISTANCE -> {
                val def = DistanceJointDef()
                jd = def
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.dampingRatio = joint.dampingRatio
                def.frequencyHz = joint.frequency
                def.length = joint.length
            }
            PbJointType.PULLEY -> {
                val def = PulleyJointDef()
                jd = def
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.groundAnchorA.set(pbToVec(joint.groundAnchorA))
                def.groundAnchorB.set(pbToVec(joint.groundAnchorB))
                def.lengthA = joint.lengthA
                def.lengthB = joint.lengthB
                def.ratio = joint.ratio
            }
            PbJointType.MOUSE -> {
                val def = MouseJointDef()
                jd = def
                def.dampingRatio = joint.dampingRatio
                def.frequencyHz = joint.frequency
                def.maxForce = joint.maxForce
                def.target.set(pbToVec(joint.target))
            }
            PbJointType.GEAR -> {
                val def = GearJointDef()
                jd = def
                if (!jointMap.containsKey(joint.joint1)) {
                    throw IllegalArgumentException("Index " + joint.joint1
                            + " is not present in the joint map.")
                }
                def.joint1 = jointMap[joint.joint1]
                if (!jointMap.containsKey(joint.joint2)) {
                    throw IllegalArgumentException("Index " + joint.joint2
                            + " is not present in the joint map.")
                }
                def.joint2 = jointMap[joint.joint2]
                def.ratio = joint.ratio
            }
            PbJointType.WHEEL -> {
                val def = WheelJointDef()
                jd = def
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.localAxisA.set(pbToVec(joint.localAxisA))
                def.enableMotor = joint.enableMotor
                def.maxMotorTorque = joint.maxMotorTorque
                def.motorSpeed = joint.motorSpeed
                def.frequencyHz = joint.frequency
                def.dampingRatio = joint.dampingRatio
            }
            PbJointType.WELD -> {
                val def = WeldJointDef()
                jd = def
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.referenceAngle = joint.refAngle
                def.frequencyHz = joint.frequency
                def.dampingRatio = joint.dampingRatio
            }
            PbJointType.FRICTION -> {
                val def = FrictionJointDef()
                jd = def
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.maxForce = joint.maxForce
                def.maxTorque = joint.maxTorque
            }
            PbJointType.ROPE -> {
                val def = RopeJointDef()
                def.localAnchorA.set(pbToVec(joint.localAnchorA))
                def.localAnchorB.set(pbToVec(joint.localAnchorB))
                def.maxLength = joint.maxLength
                throw UnsupportedObjectException("Rope joint not supported yet", Type.JOINT)
            }
            PbJointType.CONSTANT_VOLUME -> {
                val def = ConstantVolumeJointDef()
                jd = def
                def.dampingRatio = joint.dampingRatio
                def.frequencyHz = joint.frequency
                if (joint.bodiesCount != joint.jointsCount) {
                    throw IllegalArgumentException(
                            "Constant volume joint must have bodies and joints defined")
                }
                for (i in 0 until joint.bodiesCount) {
                    val body = joint.getBodies(i)
                    if (!bodyMap.containsKey(body)) {
                        throw IllegalArgumentException("Index " + body
                                + " is not present in the body map")
                    }
                    val jointIndex = joint.getJoints(i)
                    if (!jointMap.containsKey(jointIndex)) {
                        throw IllegalArgumentException("Index " + jointIndex
                                + " is not present in the joint map")
                    }
                    val djoint = jointMap[jointIndex]
                    if (djoint !is DistanceJoint) {
                        throw IllegalArgumentException(
                                "Joints for constant volume joint must be distance joints")
                    }
                    def.addBodyAndJoint(bodyMap[body]!!,
                            djoint as DistanceJoint)
                }
            }
            PbJointType.LINE -> {
                val e = UnsupportedObjectException(
                        "Line joint no longer supported.", Type.JOINT)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown joint type: " + joint.type, Type.JOINT)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        jd.collideConnected = joint.collideConnected
        if (!bodyMap.containsKey(joint.bodyA)) {
            throw IllegalArgumentException("Index " + joint.bodyA
                    + " is not present in the body map")
        }
        jd.bodyA = bodyMap[joint.bodyA]
        if (!bodyMap.containsKey(joint.bodyB)) {
            throw IllegalArgumentException("Index " + joint.bodyB
                    + " is not present in the body map")
        }
        jd.bodyB = bodyMap[joint.bodyB]
        val realJoint = world.createJoint(jd)
        if (realJoint != null && listener != null && joint.hasTag()) {
            listener!!.processJoint(realJoint, joint.tag)
        }
        return realJoint!!
    }

    private fun pbToVec(v: PbVec2): Vec2 {
        return Vec2(v.x, v.y)
    }
}
