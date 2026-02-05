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
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * A mouse joint is used to make a point on a body track a specified world point.
 * This a soft constraint with a maximum force. This allows the constraint to
 * stretch and without applying huge forces.
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_mouse_joint.cpp
 *
 * @author Daniel Murphy
 */
class MouseJoint(argWorld: WorldPool, def: MouseJointDef) : Joint(argWorld, def) {
    private val localAnchorB = Vec2()
    private val targetA = Vec2()
    private var frequency = 0.0f
    private var dampingRatio = 0.0f
    private var beta = 0.0f

    // Solver shared
    private val impulse = Vec2()
    private var maxForce = 0.0f
    private var gamma = 0.0f

    // Solver temp
    private var indexB = 0
    private val rB = Vec2()
    private val localCenterB = Vec2()
    private var invMassB = 0.0f
    private var invIB = 0.0f
    private val mass = Mat22()
    private val C = Vec2()

    init {
        assert(def.target.isValid())
        assert(def.maxForce >= 0)
        assert(def.frequencyHz >= 0)
        assert(def.dampingRatio >= 0)
        targetA.set(def.target)
        Transform.mulTransToOutUnsafe(bodyB!!.xf, targetA, localAnchorB)
        maxForce = def.maxForce
        impulse.setZero()
        frequency = def.frequencyHz
        dampingRatio = def.dampingRatio
        beta = 0.0f
        gamma = 0.0f
    }

    override fun getAnchorA(out: Vec2) {
        out.set(targetA)
    }

    override fun getAnchorB(out: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(impulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return invDt * 0.0f
    }

    fun getMaxForce(): Float {
        return maxForce
    }

    fun getFrequency(): Float {
        return frequency
    }

    fun getDampingRatio(): Float {
        return dampingRatio
    }

    fun setTarget(target: Vec2) {
        if (bodyB!!.isAwake == false) {
            bodyB!!.isAwake = true
        }
        targetA.set(target)
    }

    fun getTarget(): Vec2 {
        return targetA
    }

    override fun initVelocityConstraints(data: SolverData) {
        indexB = bodyB!!.islandIndex
        localCenterB.set(bodyB!!.sweep.localCenter)
        invMassB = bodyB!!.invMass
        invIB = bodyB!!.invI
        val cB = data.positions!![indexB].c
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val qB = pool.popRot()
        qB.set(aB)
        val mass = bodyB!!.mass
        // Frequency
        val omega = 2.0f * MathUtils.PI * frequency
        // Damping coefficient
        val d = 2.0f * mass * dampingRatio * omega
        // Spring stiffness
        val k = mass * (omega * omega)
        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        val h = data.step!!.dt
        assert(d + h * k > Settings.EPSILON)
        gamma = h * (d + h * k)
        if (gamma != 0.0f) {
            gamma = 1.0f / gamma
        }
        beta = h * k * gamma
        val temp = pool.popVec2()
        // Compute the effective mass matrix.
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
        val K = this.mass
        K.ex.x = invMassB + invIB * rB.y * rB.y + gamma
        K.ex.y = -invIB * rB.x * rB.y
        K.ey.x = K.ex.y
        K.ey.y = invMassB + invIB * rB.x * rB.x + gamma
        K.invertLocal()
        C.set(cB).addLocal(rB).subLocal(targetA)
        C.mulLocal(beta)
        // Cheat with some damping
        wB *= 0.98f
        if (data.step!!.warmStarting) {
            impulse.mulLocal(data.step!!.dtRatio)
            vB.x += invMassB * impulse.x
            vB.y += invMassB * impulse.y
            wB += invIB * Vec2.cross(rB, impulse)
        } else {
            impulse.setZero()
        }
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(1)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        return true
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        // Cdot = v + cross(w, r)
        val Cdot = pool.popVec2()
        Vec2.crossToOutUnsafe(wB, rB, Cdot)
        Cdot.addLocal(vB)
        val impulse = pool.popVec2()
        val temp = pool.popVec2()
        temp.set(this.impulse).mulLocal(gamma).addLocal(C).addLocal(Cdot).negateLocal()
        Mat22.mulToOutUnsafe(mass, temp, impulse)
        temp.set(this.impulse)
        this.impulse.addLocal(impulse)
        val maxImpulse = data.step!!.dt * maxForce
        if (this.impulse.lengthSquared() > maxImpulse * maxImpulse) {
            this.impulse.mulLocal(maxImpulse / this.impulse.length())
        }
        impulse.set(this.impulse).subLocal(temp)
        vB.x += invMassB * impulse.x
        vB.y += invMassB * impulse.y
        wB += invIB * Vec2.cross(rB, impulse)
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
        pool.pushVec2(3)
    }
}
