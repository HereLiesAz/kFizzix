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

import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * The pulley joint is connected to two bodies and two fixed ground points. The
 * pulley supports a ratio such that:
 * {@code length1 + ratio * length2 <= constant} Yes, the force transmitted is
 * scaled by the ratio. Warning: the pulley joint can get a bit squirrelly by
 * itself. They often work better when combined with prismatic joints. You
 * should also cover the anchor points with static shapes to prevent one side
 * from going to zero length.
 *
 * <p>
 * <img src=
 * "https://github.com/engine-pi/jbox2d/blob/main/misc/images/joints/pulley_joint.gif"
 * alt="pulley joint">
 * </p>
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_pulley_joint.cpp
 *
 * @author Daniel Murphy
 */
class PulleyJoint(argWorldPool: WorldPool, def: PulleyJointDef) : Joint(argWorldPool, def) {
    /**
     * The first ground anchor in the world coordinates. This point never moves.
     */
    val groundAnchorA: Vec2 = Vec2(def.groundAnchorA)

    /**
     * The second ground anchor in the world coordinates. This point never
     * moves.
     */
    val groundAnchorB: Vec2 = Vec2(def.groundAnchorB)

    /**
     * The local anchor point relative to bodyA's origin.
     */
    val localAnchorA: Vec2 = Vec2(def.localAnchorA)

    /**
     * The local anchor point relative to bodyB's origin.
     */
    val localAnchorB: Vec2 = Vec2(def.localAnchorB)

    /**
     * The reference length for the segment attached to bodyA.
     */
    val lengthA: Float

    /**
     * The reference length for the segment attached to bodyB.
     */
    val lengthB: Float

    /**
     * The pulley ratio, used to simulate a block-and-tackle.
     */
    val ratio: Float

    private val constant: Float
    private var impulse: Float = 0.0f

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val uA = Vec2()
    private val uB = Vec2()
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private var invMassA: Float = 0.0f
    private var invMassB: Float = 0.0f
    private var invIA: Float = 0.0f
    private var invIB: Float = 0.0f
    private var mass: Float = 0.0f

    companion object {
        const val MIN_PULLEY_LENGTH = 2.0f
    }

    init {
        assert(def.ratio != 0.0f)
        ratio = def.ratio
        lengthA = def.lengthA
        lengthB = def.lengthB
        constant = def.lengthA + ratio * def.lengthB
        impulse = 0.0f
    }

    fun getCurrentLengthA(): Float {
        val p = pool.popVec2()
        bodyA!!.getWorldPointToOut(localAnchorA, p)
        p.subLocal(groundAnchorA)
        val length = p.length()
        pool.pushVec2(1)
        return length
    }

    fun getCurrentLengthB(): Float {
        val p = pool.popVec2()
        bodyB!!.getWorldPointToOut(localAnchorB, p)
        p.subLocal(groundAnchorB)
        val length = p.length()
        pool.pushVec2(1)
        return length
    }

    override fun getAnchorA(argOut: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, argOut)
    }

    override fun getAnchorB(argOut: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, argOut)
    }

    override fun getReactionForce(invDt: Float, argOut: Vec2) {
        argOut.set(uB).mulLocal(impulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return 0f
    }

    fun getLength1(): Float {
        val p = pool.popVec2()
        bodyA!!.getWorldPointToOut(localAnchorA, p)
        p.subLocal(groundAnchorA)
        val len = p.length()
        pool.pushVec2(1)
        return len
    }

    fun getLength2(): Float {
        val p = pool.popVec2()
        bodyB!!.getWorldPointToOut(localAnchorB, p)
        p.subLocal(groundAnchorB)
        val len = p.length()
        pool.pushVec2(1)
        return len
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_pulley_joint.cpp#L77-L165
     */
    override fun initVelocityConstraints(data: SolverData) {
        indexA = bodyA!!.islandIndex
        indexB = bodyB!!.islandIndex
        localCenterA.set(bodyA!!.sweep.localCenter)
        localCenterB.set(bodyB!!.sweep.localCenter)
        invMassA = bodyA!!.invMass
        invMassB = bodyB!!.invMass
        invIA = bodyA!!.invI
        invIB = bodyB!!.invI
        val cA = data.positions[indexA].c
        val aA = data.positions[indexA].a
        val vA = data.velocities[indexA].v
        var wA = data.velocities[indexA].w
        val cB = data.positions[indexB].c
        val aB = data.positions[indexB].a
        val vB = data.velocities[indexB].v
        var wB = data.velocities[indexB].w
        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()
        qA.set(aA)
        qB.set(aB)
        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
        uA.set(cA).addLocal(rA).subLocal(groundAnchorA)
        uB.set(cB).addLocal(rB).subLocal(groundAnchorB)
        val lengthA = uA.length()
        val lengthB = uB.length()
        if (lengthA > 10f * Settings.linearSlop) {
            uA.mulLocal(1.0f / lengthA)
        } else {
            uA.setZero()
        }
        if (lengthB > 10f * Settings.linearSlop) {
            uB.mulLocal(1.0f / lengthB)
        } else {
            uB.setZero()
        }
        // Compute effective mass.
        val ruA = Vec2.cross(rA, uA)
        val ruB = Vec2.cross(rB, uB)
        val mA = invMassA + invIA * ruA * ruA
        val mB = invMassB + invIB * ruB * ruB
        mass = mA + ratio * ratio * mB
        if (mass > 0.0f) {
            mass = 1.0f / mass
        }
        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            impulse *= data.step.dtRatio
            // Warm starting.
            val PA = pool.popVec2()
            val PB = pool.popVec2()
            PA.set(uA).mulLocal(-impulse)
            PB.set(uB).mulLocal(-ratio * impulse)
            vA.x += invMassA * PA.x
            vA.y += invMassA * PA.y
            wA += invIA * Vec2.cross(rA, PA)
            vB.x += invMassB * PB.x;
            vB.y += invMassB * PB.y;
            wB += invIB * Vec2.cross(rB, PB);
            pool.pushVec2(2);
        } else {
            impulse = 0.0f
        }
        // data.velocities[indexA].v.set(vA);
        data.velocities[indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities[indexB].w = wB
        pool.pushVec2(1)
        pool.pushRot(2)
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_pulley_joint.cpp#L167-L192
     */
    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[indexA].v
        var wA = data.velocities[indexA].w
        val vB = data.velocities[indexB].v
        var wB = data.velocities[indexB].w
        val vpA = pool.popVec2()
        val vpB = pool.popVec2()
        val PA = pool.popVec2()
        val PB = pool.popVec2()
        Vec2.crossToOutUnsafe(wA, rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, rB, vpB)
        vpB.addLocal(vB)
        val Cdot = -Vec2.dot(uA, vpA) - ratio * Vec2.dot(uB, vpB)
        val impulse = -mass * Cdot
        this.impulse += impulse
        PA.set(uA).mulLocal(-impulse)
        PB.set(uB).mulLocal(-ratio * impulse)
        vA.x += invMassA * PA.x
        vA.y += invMassA * PA.y
        wA += invIA * Vec2.cross(rA, PA)
        vB.x += invMassB * PB.x
        vB.y += invMassB * PB.y
        wB += invIB * Vec2.cross(rB, PB)
        // data.velocities[indexA].v.set(vA);
        data.velocities[indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities[indexB].w = wB
        pool.pushVec2(4)
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_pulley_joint.cpp#L194-L264
     */
    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val uA = pool.popVec2()
        val uB = pool.popVec2()
        val temp = pool.popVec2()
        val PA = pool.popVec2()
        val PB = pool.popVec2()
        val cA = data.positions[indexA].c
        var aA = data.positions[indexA].a
        val cB = data.positions[indexB].c
        var aB = data.positions[indexB].a
        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
        uA.set(cA).addLocal(rA).subLocal(groundAnchorA)
        uB.set(cB).addLocal(rB).subLocal(groundAnchorB)
        val lengthA = uA.length()
        val lengthB = uB.length()
        if (lengthA > 10.0f * Settings.linearSlop) {
            uA.mulLocal(1.0f / lengthA)
        } else {
            uA.setZero()
        }
        if (lengthB > 10.0f * Settings.linearSlop) {
            uB.mulLocal(1.0f / lengthB)
        } else {
            uB.setZero()
        }
        // Compute effective mass.
        val ruA = Vec2.cross(rA, uA)
        val ruB = Vec2.cross(rB, uB)
        val mA = invMassA + invIA * ruA * ruA
        val mB = invMassB + invIB * ruB * ruB
        var mass = mA + ratio * ratio * mB
        if (mass > 0.0f) {
            mass = 1.0f / mass
        }
        val C = constant - lengthA - ratio * lengthB
        val linearError = MathUtils.abs(C)
        val impulse = -mass * C
        PA.set(uA).mulLocal(-impulse)
        PB.set(uB).mulLocal(-ratio * impulse)
        cA.x += invMassA * PA.x
        cA.y += invMassA * PA.y
        aA += invIA * Vec2.cross(rA, PA)
        cB.x += invMassB * PB.x
        cB.y += invMassB * PB.y
        aB += invIB * Vec2.cross(rB, PB)
        // data.positions[indexA].c.set(cA);
        data.positions[indexA].a = aA
        // data.positions[indexB].c.set(cB);
        data.positions[indexB].a = aB
        pool.pushRot(2)
        pool.pushVec2(7)
        return linearError < Settings.linearSlop
    }
}
