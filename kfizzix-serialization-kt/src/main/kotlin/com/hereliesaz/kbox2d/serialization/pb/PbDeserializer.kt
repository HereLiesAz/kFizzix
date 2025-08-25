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
import com.hereliesaz.kbox2d.serialization.JbDeserializer
import com.hereliesaz.kbox2d.serialization.UnsupportedListener
import com.hereliesaz.kbox2d.serialization.UnsupportedObjectException
import com.hereliesaz.kbox2d.serialization.UnsupportedObjectException.Type
import org.box2d.proto.Box2D.*
import java.io.IOException
import java.io.InputStream

class PbDeserializer : JbDeserializer {
    private var listener: JbDeserializer.ObjectListener? = null
    private var unsupportedlistener: UnsupportedListener? = null

    constructor()
    constructor(argListener: UnsupportedListener?) {
        unsupportedlistener = argListener
    }

    constructor(argObjectListener: JbDeserializer.ObjectListener?) {
        listener = argObjectListener
    }

    constructor(argListener: UnsupportedListener?, argObjectListener: JbDeserializer.ObjectListener?) {
        unsupportedlistener = argListener
        listener = argObjectListener
    }

    override fun setObjectListener(argListener: JbDeserializer.ObjectListener) {
        listener = argListener
    }

    override fun setUnsupportedListener(argListener: UnsupportedListener) {
        unsupportedlistener = argListener
    }

    private fun isIndependentJoint(argType: PbJointType): Boolean {
        return argType != PbJointType.GEAR && argType != PbJointType.CONSTANT_VOLUME
    }

    @Throws(IOException::class)
    override fun deserializeWorld(argInput: InputStream): World {
        val world = PbWorld.parseFrom(argInput)
        return deserializeWorld(world)
    }

    fun deserializeWorld(pbWorld: PbWorld): World {
        val world = World(pbToVec(pbWorld.gravity))
        world.isAutoClearForces = pbWorld.autoClearForces
        world.isContinuousPhysics = pbWorld.continuousPhysics
        world.isWarmStarting = pbWorld.warmStarting
        world.isSubStepping = pbWorld.subStepping
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
    override fun deserializeBody(argWorld: World, argInput: InputStream): Body {
        val body = PbBody.parseFrom(argInput)
        return deserializeBody(argWorld, body)
    }

    fun deserializeBody(argWorld: World, argBody: PbBody): Body {
        val bd = BodyDef()
        bd.position.set(pbToVec(argBody.position))
        bd.angle = argBody.angle
        bd.linearDamping = argBody.linearDamping
        bd.angularDamping = argBody.angularDamping
        bd.gravityScale = argBody.gravityScale
        // velocities are populated after fixture addition
        bd.bullet = argBody.bullet
        bd.allowSleep = argBody.allowSleep
        bd.awake = argBody.awake
        bd.active = argBody.active
        bd.fixedRotation = argBody.fixedRotation
        when (argBody.type) {
            PbBodyType.DYNAMIC -> bd.type = BodyType.DYNAMIC
            PbBodyType.KINEMATIC -> bd.type = BodyType.KINEMATIC
            PbBodyType.STATIC -> bd.type = BodyType.STATIC
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown body type: " + argBody.type, Type.BODY)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        val body = argWorld.createBody(bd)
        for (i in 0 until argBody.fixturesCount) {
            deserializeFixture(body, argBody.getFixtures(i))
        }
        // adding fixtures can change this, so we put this here and set it
// directly in the body
        body.linearVelocity.set(pbToVec(argBody.linearVelocity))
        body.angularVelocity = argBody.angularVelocity
        if (listener != null && argBody.hasTag()) {
            listener!!.processBody(body, argBody.tag)
        }
        return body
    }

    @Throws(IOException::class)
    override fun deserializeFixture(argBody: Body, argInput: InputStream): Fixture {
        val fixture = PbFixture.parseFrom(argInput)
        return deserializeFixture(argBody, fixture)
    }

    fun deserializeFixture(argBody: Body, argFixture: PbFixture): Fixture {
        val fd = FixtureDef()
        fd.density = argFixture.density
        fd.filter.categoryBits = argFixture.filter.categoryBits
        fd.filter.groupIndex = argFixture.filter.groupIndex
        fd.filter.maskBits = argFixture.filter.maskBits
        fd.friction = argFixture.friction
        fd.isSensor = argFixture.sensor
        fd.restitution = argFixture.restitution
        fd.shape = deserializeShape(argFixture.shape)
        val fixture = argBody.createFixture(fd)
        if (listener != null && argFixture.hasTag()) {
            listener!!.processFixture(fixture, argFixture.tag)
        }
        return fixture
    }

    @Throws(IOException::class)
    override fun deserializeShape(argInput: InputStream): Shape {
        val s = PbShape.parseFrom(argInput)
        return deserializeShape(s)
    }

    fun deserializeShape(argShape: PbShape): Shape {
        val shape: Shape
        when (argShape.type) {
            PbShapeType.CIRCLE -> {
                val c = CircleShape()
                c.p.set(pbToVec(argShape.center))
                shape = c
            }
            PbShapeType.POLYGON -> {
                val p = PolygonShape()
                p.centroid.set(pbToVec(argShape.centroid))
                p.count = argShape.pointsCount
                for (i in 0 until p.count) {
                    p.vertices[i].set(pbToVec(argShape.getPoints(i)))
                    p.normals[i].set(pbToVec(argShape.getNormals(i)))
                }
                shape = p
            }
            PbShapeType.EDGE -> {
                val edge = EdgeShape()
                edge.vertex0.set(pbToVec(argShape.v0))
                edge.vertex1.set(pbToVec(argShape.v1))
                edge.vertex2.set(pbToVec(argShape.v2))
                edge.vertex3.set(pbToVec(argShape.v3))
                edge.m_hasVertex0 = argShape.has0
                edge.m_hasVertex3 = argShape.has3
                shape = edge
            }
            PbShapeType.CHAIN -> {
                val chain = ChainShape()
                chain.m_count = argShape.pointsCount
                chain.m_vertices = Array(chain.m_count) { Vec2() }
                for (i in 0 until chain.m_count) {

                    chain.m_vertices!![i] = Vec2(pbToVec(argShape.getPoints(i)))

                }
                chain.m_hasPrevVertex = argShape.has0
                chain.m_hasNextVertex = argShape.has3
                chain.m_prevVertex.set(pbToVec(argShape.prev))
                chain.m_nextVertex.set(pbToVec(argShape.next))
                shape = chain
            }
            else -> {
                val e = UnsupportedObjectException(
                        "Unknown shape type: " + argShape.type, Type.SHAPE)
                if (unsupportedlistener == null
                        || unsupportedlistener!!.isUnsupported(e)) {
                    throw e
                }
                throw e
            }
        }
        shape.radius = argShape.radius
        if (listener != null && argShape.hasTag()) {
            listener!!.processShape(shape, argShape.tag)
        }
        return shape
    }

    @Throws(IOException::class)
    override fun deserializeJoint(argWorld: World, argInput: InputStream,
                                  argBodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>): Joint {
        val joint = PbJoint.parseFrom(argInput)
        return deserializeJoint(argWorld, joint, argBodyMap, jointMap)
    }

    fun deserializeJoint(argWorld: World, joint: PbJoint,
                         argBodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>): Joint {
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
                    if (!argBodyMap.containsKey(body)) {
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
                    def.addBodyAndJoint(argBodyMap[body],
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
        if (!argBodyMap.containsKey(joint.bodyA)) {
            throw IllegalArgumentException("Index " + joint.bodyA
                    + " is not present in the body map")
        }
        jd.bodyA = argBodyMap[joint.bodyA]
        if (!argBodyMap.containsKey(joint.bodyB)) {
            throw IllegalArgumentException("Index " + joint.bodyB
                    + " is not present in the body map")
        }
        jd.bodyB = argBodyMap[joint.bodyB]
        val realJoint = argWorld.createJoint(jd)
        if (listener != null && joint.hasTag()) {
            listener!!.processJoint(realJoint, joint.tag)
        }
        return realJoint
    }

    private fun pbToVec(argVec: PbVec2): Vec2 {
        return Vec2(argVec.x, argVec.y)
    }
}
