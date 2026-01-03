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

import com.hereliesaz.kfizzix.common.Mat22
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * A motor joint is used to control the relative motion between two bodies. A
 * typical usage is to control the movement of a dynamic body with respect to
 * the ground.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_motor_joint.cpp
 *
 * @author Daniel Murphy
 */
class MotorJoint(pool: WorldPool, def: MotorJointDef) : Joint(pool, def) {
    /**
     * Position of bodyB minus the position of bodyA, in bodyA's frame, in
     * meters.
     */
    private val linearOffset = Vec2()

    /**
     * The bodyB angle minus bodyA angle in radians.
     */
    var angularOffset: Float

    /**
     * The maximum motor force in N.
     */
    var maxForce: Float

    /**
     * The maximum motor torque in N-m.
     */
    var maxTorque: Float

    /**
     * Position correction factor in the range [0,1].
     */
    var correctionFactor: Float

    private val linearImpulse = Vec2()
    private var angularImpulse: Float = 0.0f

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private val linearError = Vec2()
    private var angularError: Float = 0.0f
    private var invMassA: Float = 0.0f
    private var invMassB: Float = 0.0f
    private var invIA: Float = 0.0f
    private var invIB: Float = 0.0f
    private val linearMass = Mat22()
    private var angularMass: Float = 0.0f

    init {
        linearOffset.set(def.linearOffset)
        angularOffset = def.angularOffset
        angularImpulse = 0.0f
        maxForce = def.maxForce
        maxTorque = def.maxTorque
        correctionFactor = def.correctionFactor
    }

    override fun getAnchorA(argOut: Vec2) {
        argOut.set(bodyA!!.position)
    }

    override fun getAnchorB(argOut: Vec2) {
        argOut.set(bodyB!!.position)
    }

    override fun getReactionForce(invDt: Float, argOut: Vec2) {
        argOut.set(linearImpulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return angularImpulse * invDt
    }

    /**
     * Set the target linear offset, in frame A, in meters.
     */
    fun setLinearOffset(linearOffset: Vec2) {
        if (linearOffset.x != this.linearOffset.x || linearOffset.y != this.linearOffset.y) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            this.linearOffset.set(linearOffset)
        }
    }

    /**
     * Get the target linear offset, in frame A, in meters.
     */
    fun getLinearOffset(argOut: Vec2) {
        argOut.set(linearOffset)
    }

    /**
     * Get the target linear offset, in frame A, in meters. Do not modify.
     */
    fun getLinearOffset(): Vec2 {
        return linearOffset
    }

    override fun initVelocityConstraints(data: SolverData) {
        indexA = bodyA!!.islandIndex
        indexB = bodyB!!.islandIndex
        localCenterA.set(bodyA!!.sweep.localCenter)
        localCenterB.set(bodyB!!.sweep.localCenter)
        invMassA = bodyA!!.invMass
        invMassB = bodyB!!.invMass
        invIA = bodyA!!.invI
        invIB = bodyB!!.invI
        val cA = data.positions!![indexA].c
        val aA = data.positions!![indexA].a
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val cB = data.positions!![indexB].c
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()
        val K = pool.popMat22()
        qA.set(aA)
        qB.set(aB)
        // Compute the effective mass matrix.
        // rA = b2Mul(qA, -localCenterA);
        // rB = b2Mul(qB, -localCenterB);
        rA.x = qA.c * -localCenterA.x - qA.s * -localCenterA.y
        rA.y = qA.s * -localCenterA.x + qA.c * -localCenterA.y
        rB.x = qB.c * -localCenterB.x - qB.s * -localCenterB.y
        rB.y = qB.s * -localCenterB.x + qB.c * -localCenterB.y
        // J = [-I -r1_skew I r2_skew]
        // [ 0 -1 0 1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x,
        // -r1y*iA-r2y*iB]
        // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
        // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
        K.ey.x = K.ex.y
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x
        K.invertToOut(linearMass)
        angularMass = iA + iB
        if (angularMass > 0.0f) {
            angularMass = 1.0f / angularMass
        }
        // linearError = cB + rB - cA - rA - b2Mul(qA, linearOffset);
        Rot.mulToOutUnsafe(qA, linearOffset, temp)
        linearError.x = cB.x + rB.x - cA.x - rA.x - temp.x
        linearError.y = cB.y + rB.y - cA.y - rA.y - temp.y
        angularError = aB - aA - angularOffset
        if (data.step!!.warmStarting) {
            // Scale impulses to support a variable time step.
            linearImpulse.x *= data.step!!.dtRatio
            linearImpulse.y *= data.step!!.dtRatio
            angularImpulse *= data.step!!.dtRatio
            val P = linearImpulse
            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * (rA.x * P.y - rA.y * P.x + angularImpulse)
            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * (rB.x * P.y - rB.y * P.x + angularImpulse)
        } else {
            linearImpulse.setZero()
            angularImpulse = 0.0f
        }
        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(2)
        // data.velocities[indexA].v = vA;
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v = vB;
        data.velocities!![indexB].w = wB
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB
        val h = data.step!!.dt
        val inv_h = data.step!!.inverseDt
        val temp = pool.popVec2()
        // Solve angular friction
        run {
            val Cdot = wB - wA + inv_h * correctionFactor * angularError
            val impulse = -angularMass * Cdot
            val oldImpulse = angularImpulse
            val maxImpulse = h * maxTorque
            angularImpulse = MathUtils.clamp(angularImpulse + impulse,
                -maxImpulse, maxImpulse)
            val incImpulse = angularImpulse - oldImpulse
            wA -= iA * incImpulse
            wB += iB * incImpulse
        }
        val Cdot = pool.popVec2()
        // Solve linear friction
        run {
            // Cdot = vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA) + inv_h *
            // correctionFactor *
            // linearError;
            Cdot.x = vB.x + -wB * rB.y - vA.x - -wA * rA.y + inv_h * correctionFactor * linearError.x
            Cdot.y = vB.y + wB * rB.x - vA.y - wA * rA.x + inv_h * correctionFactor * linearError.y
            Mat22.mulToOutUnsafe(linearMass, Cdot, temp)
            temp.negateLocal()
            val oldImpulse = pool.popVec2()
            oldImpulse.set(linearImpulse)
            linearImpulse.addLocal(temp)
            val maxImpulse = h * maxForce
            if (linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                linearImpulse.normalize()
                linearImpulse.mulLocal(maxImpulse)
            }
            temp.x = linearImpulse.x - oldImpulse.x
            temp.y = linearImpulse.y - oldImpulse.y
            vA.x -= mA * temp.x
            vA.y -= mA * temp.y
            wA -= iA * (rA.x * temp.y - rA.y * temp.x)
            vB.x += mB * temp.x
            vB.y += mB * temp.y
            wB += iB * (rB.x * temp.y - rB.y * temp.x)
        }
        pool.pushVec2(3)
        // data.velocities[indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        return true
    }
}
