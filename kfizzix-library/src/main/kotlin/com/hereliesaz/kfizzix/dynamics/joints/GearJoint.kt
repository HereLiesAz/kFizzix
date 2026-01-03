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
package com.hereliesaz.kfizzix.dynamics.joints

import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.Body
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * A gear joint is used to connect two joints together. Either joint can be a
 * revolute or prismatic joint. You specify a gear ratio to bind the motions
 * together: coordinate1 + ratio * coordinate2 = constant The ratio can be
 * negative or positive. If one joint is a revolute joint and the other joint is
 * a prismatic joint, then the ratio will have units of length or units of
 * 1/length.
 *
 * <p>
 * <img src=
 * "https://github.com/engine-pi/jbox2d/blob/main/misc/images/joints/gear_joint.gif"
 * alt="gear joint">
 * </p>
 *
 * @warning The revolute and prismatic joints must be attached to fixed bodies
 *     (which must be body1 on those joints).
 * @warning You have to manually destroy the gear joint if joint1 or joint2 is
 *     destroyed.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_gear_joint.cpp
 *
 * @author Daniel Murphy
 */
class GearJoint(argWorldPool: WorldPool, def: GearJointDef) : Joint(argWorldPool, def) {
    /**
     * The first revolute/prismatic joint attached to the gear joint.
     */
    val joint1: Joint

    /**
     * The second revolute/prismatic joint attached to the gear joint.
     */
    val joint2: Joint

    /**
     * Gear ratio.
     *
     * @see GearJoint
     */
    var ratio: Float = def.ratio

    private val typeA: JointType
    private val typeB: JointType

    // Body A is connected to body C
    // Body B is connected to body D
    private val bodyC: Body?
    private val bodyD: Body?

    // Solver shared
    private val localAnchorA = Vec2()
    private val localAnchorB = Vec2()
    private val localAnchorC = Vec2()
    private val localAnchorD = Vec2()
    private val localAxisC = Vec2()
    private val localAxisD = Vec2()
    private val referenceAngleA: Float
    private val referenceAngleB: Float
    private val constant: Float
    private var impulse: Float = 0.0f

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private var indexC: Int = 0
    private var indexD: Int = 0
    private val lcA = Vec2()
    private val lcB = Vec2()
    private val lcC = Vec2()
    private val lcD = Vec2()
    private var mA: Float = 0.0f
    private var mB: Float = 0.0f
    private var mC: Float = 0.0f
    private var mD: Float = 0.0f
    private var iA: Float = 0.0f
    private var iB: Float = 0.0f
    private var iC: Float = 0.0f
    private var iD: Float = 0.0f
    private val JvAC = Vec2()
    private val JvBD = Vec2()
    private var JwA: Float = 0.0f
    private var JwB: Float = 0.0f
    private var JwC: Float = 0.0f
    private var JwD: Float = 0.0f
    private var mass: Float = 0.0f

    init {
        joint1 = def.joint1!!
        joint2 = def.joint2!!
        typeA = joint1.type
        typeB = joint2.type
        assert(typeA == JointType.REVOLUTE || typeA == JointType.PRISMATIC)
        assert(typeB == JointType.REVOLUTE || typeB == JointType.PRISMATIC)
        var coordinateA: Float
        var coordinateB: Float
        // TODO_ERIN there might be some problem with the joint edges in Joint.
        bodyC = joint1.bodyA
        bodyA = joint1.bodyB
        // Get geometry of joint1
        val xfA = bodyA!!.xf
        val aA = bodyA!!.sweep.a
        val xfC = bodyC!!.xf
        val aC = bodyC!!.sweep.a
        if (typeA == JointType.REVOLUTE) {
            val revolute = def.joint1 as RevoluteJoint
            localAnchorC.set(revolute.localAnchorA)
            localAnchorA.set(revolute.localAnchorB)
            referenceAngleA = revolute.referenceAngle
            localAxisC.setZero()
            coordinateA = aA - aC - referenceAngleA
        } else {
            val pA = pool.popVec2()
            val temp = pool.popVec2()
            val prismatic = def.joint1 as PrismaticJoint
            localAnchorC.set(prismatic.localAnchorA)
            localAnchorA.set(prismatic.localAnchorB)
            referenceAngleA = prismatic.referenceAngle
            localAxisC.set(prismatic.localXAxisA)
            Rot.mulToOutUnsafe(xfA.q, localAnchorA, temp)
            temp.addLocal(xfA.p).subLocal(xfC.p)
            Rot.mulTransUnsafe(xfC.q, temp, pA)
            coordinateA = Vec2.dot(pA.subLocal(localAnchorC), localAxisC)
            pool.pushVec2(2)
        }
        bodyD = joint2.bodyA
        bodyB = joint2.bodyB
        // Get geometry of joint2
        val xfB = bodyB!!.xf
        val aB = bodyB!!.sweep.a
        val xfD = bodyD!!.xf
        val aD = bodyD!!.sweep.a
        if (typeB == JointType.REVOLUTE) {
            val revolute = def.joint2 as RevoluteJoint
            localAnchorD.set(revolute.localAnchorA)
            localAnchorB.set(revolute.localAnchorB)
            referenceAngleB = revolute.referenceAngle
            localAxisD.setZero()
            coordinateB = aB - aD - referenceAngleB
        } else {
            val pB = pool.popVec2()
            val temp = pool.popVec2()
            val prismatic = def.joint2 as PrismaticJoint
            localAnchorD.set(prismatic.localAnchorA)
            localAnchorB.set(prismatic.localAnchorB)
            referenceAngleB = prismatic.referenceAngle
            localAxisD.set(prismatic.localXAxisA)
            Rot.mulToOutUnsafe(xfB.q, localAnchorB, temp)
            temp.addLocal(xfB.p).subLocal(xfD.p)
            Rot.mulTransUnsafe(xfD.q, temp, pB)
            coordinateB = Vec2.dot(pB.subLocal(localAnchorD), localAxisD)
            pool.pushVec2(2)
        }
        constant = coordinateA + ratio * coordinateB
        impulse = 0.0f
    }

    override fun getAnchorA(argOut: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, argOut)
    }

    override fun getAnchorB(argOut: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, argOut)
    }

    override fun getReactionForce(invDt: Float, argOut: Vec2) {
        argOut.set(JvAC).mulLocal(impulse)
        argOut.mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        val L = impulse * JwA
        return invDt * L
    }

    override fun initVelocityConstraints(data: SolverData) {
        indexA = bodyA!!.islandIndex
        indexB = bodyB!!.islandIndex
        indexC = bodyC!!.islandIndex
        indexD = bodyD!!.islandIndex
        lcA.set(bodyA!!.sweep.localCenter)
        lcB.set(bodyB!!.sweep.localCenter)
        lcC.set(bodyC!!.sweep.localCenter)
        lcD.set(bodyD!!.sweep.localCenter)
        mA = bodyA!!.invMass
        mB = bodyB!!.invMass
        mC = bodyC!!.invMass
        mD = bodyD!!.invMass
        iA = bodyA!!.invI
        iB = bodyB!!.invI
        iC = bodyC!!.invI
        iD = bodyD!!.invI
        // Vec2 cA = data.positions[indexA]!!.c
        val aA = data.positions!![indexA].a
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        // Vec2 cB = data.positions[indexB]!!.c
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        // Vec2 cC = data.positions[indexC]!!.c
        val aC = data.positions!![indexC].a
        val vC = data.velocities!![indexC].v
        var wC = data.velocities!![indexC].w
        // Vec2 cD = data.positions[indexD]!!.c
        val aD = data.positions!![indexD].a
        val vD = data.velocities!![indexD].v
        var wD = data.velocities!![indexD].w
        val qA = pool.popRot()
        val qB = pool.popRot()
        val qC = pool.popRot()
        val qD = pool.popRot()
        qA.set(aA)
        qB.set(aB)
        qC.set(aC)
        qD.set(aD)
        mass = 0.0f
        val temp = pool.popVec2()
        if (typeA == JointType.REVOLUTE) {
            JvAC.setZero()
            JwA = 1.0f
            JwC = 1.0f
            mass += iA + iC
        } else {
            val rC = pool.popVec2()
            val rA = pool.popVec2()
            Rot.mulToOutUnsafe(qC, localAxisC, JvAC)
            Rot.mulToOutUnsafe(qC, temp.set(localAnchorC).subLocal(lcC), rC)
            Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(lcA), rA)
            JwC = Vec2.cross(rC, JvAC)
            JwA = Vec2.cross(rA, JvAC)
            mass += mC + mA + iC * JwC * JwC + iA * JwA * JwA
            pool.pushVec2(2)
        }
        if (typeB == JointType.REVOLUTE) {
            JvBD.setZero()
            JwB = ratio
            JwD = ratio
            mass += ratio * ratio * (iB + iD)
        } else {
            val u = pool.popVec2()
            val rD = pool.popVec2()
            val rB = pool.popVec2()
            Rot.mulToOutUnsafe(qD, localAxisD, u)
            Rot.mulToOutUnsafe(qD, temp.set(localAnchorD).subLocal(lcD), rD)
            Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(lcB), rB)
            JvBD.set(u).mulLocal(ratio)
            JwD = ratio * Vec2.cross(rD, u)
            JwB = ratio * Vec2.cross(rB, u)
            mass += ratio * ratio * (mD + mB) + iD * JwD * JwD + iB * JwB * JwB
            pool.pushVec2(3)
        }
        // Compute effective mass.
        mass = if (mass > 0.0f) 1.0f / mass else 0.0f
        if (data.step!!.warmStarting) {
            vA.x += (mA * impulse) * JvAC.x
            vA.y += (mA * impulse) * JvAC.y
            wA += iA * impulse * JwA
            vB.x += (mB * impulse) * JvBD.x
            vB.y += (mB * impulse) * JvBD.y
            wB += iB * impulse * JwB
            vC.x -= (mC * impulse) * JvAC.x
            vC.y -= (mC * impulse) * JvAC.y
            wC -= iC * impulse * JwC
            vD.x -= (mD * impulse) * JvBD.x
            vD.y -= (mD * impulse) * JvBD.y
            wD -= iD * impulse * JwD
        } else {
            impulse = 0.0f
        }
        pool.pushVec2(1)
        pool.pushRot(4)
        // data.velocities[indexA].v = vA
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v = vB
        data.velocities!![indexB].w = wB
        // data.velocities[indexC].v = vC
        data.velocities!![indexC].w = wC
        // data.velocities[indexD].v = vD
        data.velocities!![indexD].w = wD
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val vC = data.velocities!![indexC].v
        var wC = data.velocities!![indexC].w
        val vD = data.velocities!![indexD].v
        var wD = data.velocities!![indexD].w
        val temp1 = pool.popVec2()
        val temp2 = pool.popVec2()
        var Cdot = Vec2.dot(JvAC, temp1.set(vA).subLocal(vC)) +
                Vec2.dot(JvBD, temp2.set(vB).subLocal(vD))
        Cdot += (JwA * wA - JwC * wC) + (JwB * wB - JwD * wD)
        pool.pushVec2(2)
        val impulse = -mass * Cdot
        this.impulse += impulse
        vA.x += (mA * impulse) * JvAC.x
        vA.y += (mA * impulse) * JvAC.y
        wA += iA * impulse * JwA
        vB.x += (mB * impulse) * JvBD.x
        vB.y += (mB * impulse) * JvBD.y
        wB += iB * impulse * JwB
        vC.x -= (mC * impulse) * JvAC.x
        vC.y -= (mC * impulse) * JvAC.y
        wC -= iC * impulse * JwC
        vD.x -= (mD * impulse) * JvBD.x
        vD.y -= (mD * impulse) * JvBD.y
        wD -= iD * impulse * JwD
        // data.velocities[indexA].v = vA
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v = vB
        data.velocities!![indexB].w = wB
        // data.velocities[indexC].v = vC
        data.velocities!![indexC].w = wC
        // data.velocities[indexD].v = vD
        data.velocities!![indexD].w = wD
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val cA = data.positions!![indexA].c
        var aA = data.positions!![indexA].a
        val cB = data.positions!![indexB].c
        var aB = data.positions!![indexB].a
        val cC = data.positions!![indexC].c
        var aC = data.positions!![indexC].a
        val cD = data.positions!![indexD].c
        var aD = data.positions!![indexD].a
        val qA = pool.popRot()
        val qB = pool.popRot()
        val qC = pool.popRot()
        val qD = pool.popRot()
        qA.set(aA)
        qB.set(aB)
        qC.set(aC)
        qD.set(aD)
        val linearError = 0.0f
        var coordinateA: Float
        var coordinateB: Float
        val temp = pool.popVec2()
        val JvAC = pool.popVec2()
        val JvBD = pool.popVec2()
        var JwA: Float
        var JwB: Float
        var JwC: Float
        var JwD: Float
        var mass = 0.0f
        if (typeA == JointType.REVOLUTE) {
            JvAC.setZero()
            JwA = 1.0f
            JwC = 1.0f
            mass += iA + iC
            coordinateA = aA - aC - referenceAngleA
        } else {
            val rC = pool.popVec2()
            val rA = pool.popVec2()
            val pC = pool.popVec2()
            val pA = pool.popVec2()
            Rot.mulToOutUnsafe(qC, localAxisC, JvAC)
            Rot.mulToOutUnsafe(qC, temp.set(localAnchorC).subLocal(lcC), rC)
            Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(lcA), rA)
            JwC = Vec2.cross(rC, JvAC)
            JwA = Vec2.cross(rA, JvAC)
            mass += mC + mA + iC * JwC * JwC + iA * JwA * JwA
            pC.set(localAnchorC).subLocal(lcC)
            Rot.mulTransUnsafe(qC, temp.set(rA).addLocal(cA).subLocal(cC), pA)
            coordinateA = Vec2.dot(pA.subLocal(pC), localAxisC)
            pool.pushVec2(4)
        }
        if (typeB == JointType.REVOLUTE) {
            JvBD.setZero()
            JwB = ratio
            JwD = ratio
            mass += ratio * ratio * (iB + iD)
            coordinateB = aB - aD - referenceAngleB
        } else {
            val u = pool.popVec2()
            val rD = pool.popVec2()
            val rB = pool.popVec2()
            val pD = pool.popVec2()
            val pB = pool.popVec2()
            Rot.mulToOutUnsafe(qD, localAxisD, u)
            Rot.mulToOutUnsafe(qD, temp.set(localAnchorD).subLocal(lcD), rD)
            Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(lcB), rB)
            JvBD.set(u).mulLocal(ratio)
            JwD = Vec2.cross(rD, u)
            JwB = Vec2.cross(rB, u)
            mass += ratio * ratio * (mD + mB) + iD * JwD * JwD + iB * JwB * JwB
            pD.set(localAnchorD).subLocal(lcD)
            Rot.mulTransUnsafe(qD, temp.set(rB).addLocal(cB).subLocal(cD), pB)
            coordinateB = Vec2.dot(pB.subLocal(pD), localAxisD)
            pool.pushVec2(5)
        }
        val C = (coordinateA + ratio * coordinateB) - constant
        var impulse = 0.0f
        if (mass > 0.0f) {
            impulse = -C / mass
        }
        pool.pushVec2(3)
        pool.pushRot(4)
        cA.x += (mA * impulse) * JvAC.x
        cA.y += (mA * impulse) * JvAC.y
        aA += iA * impulse * JwA
        cB.x += (mB * impulse) * JvBD.x
        cB.y += (mB * impulse) * JvBD.y
        aB += iB * impulse * JwB
        cC.x -= (mC * impulse) * JvAC.x
        cC.y -= (mC * impulse) * JvAC.y
        aC -= iC * impulse * JwC
        cD.x -= (mD * impulse) * JvBD.x
        cD.y -= (mD * impulse) * JvBD.y
        aD -= iD * impulse * JwD
        // data.positions[indexA].c = cA
        data.positions!![indexA].a = aA
        // data.positions[indexB].c = cB
        data.positions!![indexB].a = aB
        // data.positions[indexC].c = cC
        data.positions!![indexC].a = aC
        // data.positions[indexD].c = cD
        data.positions!![indexD].a = aD
        // TODO_ERIN not implemented
        return linearError < Settings.linearSlop
    }
}
