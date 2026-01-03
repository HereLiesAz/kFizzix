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

import com.hereliesaz.kfizzix.common.Mat33
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.common.Vec3
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * A revolute joint constrains two bodies to share a common point while they are
 * free to rotate about the point. The relative rotation about the shared point
 * is the joint angle. You can limit the relative rotation with a joint limit
 * that specifies a lower and upper angle. You can use a motor to drive the
 * relative rotation about the shared point. A maximum motor torque is provided
 * so that infinite forces are not generated.
 *
 * <p>
 * <img src=
 * "https://github.com/engine-pi/jbox2d/blob/main/misc/images/joints/revolute_joint.svg"
 * alt="revolute joint">
 * </p>
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_revolute_joint.cpp
 *
 * @author Daniel Murphy
 */
class RevoluteJoint(argWorld: WorldPool, def: RevoluteJointDef) : Joint(argWorld, def) {
    /**
     * The local anchor point relative to body1's origin.
     */
    val localAnchorA: Vec2 = Vec2(def.localAnchorA)

    /**
     * The local anchor point relative to body2's origin.
     */
    val localAnchorB: Vec2 = Vec2(def.localAnchorB)

    /**
     * The body2 angle minus body1 angle in the reference state (radians).
     */
    val referenceAngle: Float

    /**
     * A flag to enable joint limits.
     */
    private var enableLimit: Boolean

    /**
     * The lower angle for the joint limit (radians).
     */
    private var lowerAngle: Float

    /**
     * The upper angle for the joint limit (radians).
     */
    private var upperAngle: Float

    /**
     * A flag to enable the joint motor.
     */
    private var enableMotor: Boolean

    /**
     * The desired motor speed. Usually in radians per second.
     */
    var motorSpeed: Float

    /**
     * The maximum motor torque used to achieve the desired motor speed. Usually
     * in N-m.
     */
    var maxMotorTorque: Float

    private var motorImpulse: Float
    private val impulse = Vec3()

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private var invMassA: Float = 0.0f
    private var invMassB: Float = 0.0f
    private var invIA: Float = 0.0f
    private var invIB: Float = 0.0f

    /**
     * effective mass for point-to-point constraint.
     */
    private val mass = Mat33()

    /**
     * effective mass for motor/limit angular constraint.
     */
    private var motorMass: Float = 0.0f
    private var limitState: LimitState

    init {
        referenceAngle = def.referenceAngle
        motorImpulse = 0f
        lowerAngle = def.lowerAngle
        upperAngle = def.upperAngle
        maxMotorTorque = def.maxMotorTorque
        motorSpeed = def.motorSpeed
        enableLimit = def.enableLimit
        enableMotor = def.enableMotor
        limitState = LimitState.INACTIVE
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
        val aA = data.positions!![indexA].a
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()
        qA.set(aA)
        qB.set(aB)
        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
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
        val fixedRotation = iA + iB == 0.0f
        mass.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB
        mass.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB
        mass.ez.x = -rA.y * iA - rB.y * iB
        mass.ex.y = mass.ey.x
        mass.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB
        mass.ez.y = rA.x * iA + rB.x * iB
        mass.ex.z = mass.ez.x
        mass.ey.z = mass.ez.y
        mass.ez.z = iA + iB
        motorMass = iA + iB
        if (motorMass > 0.0f) {
            motorMass = 1.0f / motorMass
        }
        if (!enableMotor || fixedRotation) {
            motorImpulse = 0.0f
        }
        if (enableLimit && !fixedRotation) {
            val jointAngle = aB - aA - referenceAngle
            if (MathUtils.abs(upperAngle - lowerAngle) < 2.0f * Settings.angularSlop) {
                limitState = LimitState.EQUAL
            } else if (jointAngle <= lowerAngle) {
                if (limitState != LimitState.AT_LOWER) {
                    impulse.z = 0.0f
                }
                limitState = LimitState.AT_LOWER
            } else if (jointAngle >= upperAngle) {
                if (limitState != LimitState.AT_UPPER) {
                    impulse.z = 0.0f
                }
                limitState = LimitState.AT_UPPER
            } else {
                limitState = LimitState.INACTIVE
                impulse.z = 0.0f
            }
        } else {
            limitState = LimitState.INACTIVE
        }
        if (data.step!!.warmStarting) {
            val P = pool.popVec2()
            // Scale impulses to support a variable time step.
            impulse.x *= data.step!!.dtRatio
            impulse.y *= data.step!!.dtRatio
            motorImpulse *= data.step!!.dtRatio
            P.x = impulse.x
            P.y = impulse.y
            vA.x = vA.x - mA * P.x
            vA.y = vA.y - mA * P.y
            wA = wA - iA * (Vec2.cross(rA, P) + motorImpulse + impulse.z)
            vB.x = vB.x + mB * P.x
            vB.y = vB.y + mB * P.y
            wB = wB + iB * (Vec2.cross(rB, P) + motorImpulse + impulse.z)
            pool.pushVec2(1)
        } else {
            impulse.setZero()
            motorImpulse = 0.0f
        }
        // data.velocities[indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
        pool.pushVec2(1)
        pool.pushRot(2)
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
        val fixedRotation = iA + iB == 0.0f
        // Solve motor constraint.
        if (enableMotor && limitState != LimitState.EQUAL && !fixedRotation) {
            val Cdot = wB - wA - motorSpeed
            val impulse = -motorMass * Cdot
            val oldImpulse = motorImpulse
            val maxImpulse = data.step!!.dt * maxMotorTorque
            motorImpulse = MathUtils.clamp(motorImpulse + impulse, -maxImpulse, maxImpulse)
            val incImpulse = motorImpulse - oldImpulse
            wA = wA - iA * incImpulse
            wB = wB + iB * incImpulse
        }
        val temp = pool.popVec2()
        // Solve limit constraint.
        if (enableLimit && limitState != LimitState.INACTIVE && !fixedRotation) {
            val Cdot1 = pool.popVec2()
            val Cdot = pool.popVec3()
            // Solve point-to-point constraint
            Vec2.crossToOutUnsafe(wA, rA, temp)
            Vec2.crossToOutUnsafe(wB, rB, Cdot1)
            Cdot1.addLocal(vB).subLocal(vA).subLocal(temp)
            val Cdot2 = wB - wA
            Cdot.set(Cdot1.x, Cdot1.y, Cdot2)
            val impulse = pool.popVec3()
            mass.solve33ToOut(Cdot, impulse)
            impulse.negateLocal()
            if (limitState == LimitState.EQUAL) {
                this.impulse.addLocal(impulse)
            } else if (limitState == LimitState.AT_LOWER) {
                val newImpulse = this.impulse.z + impulse.z
                if (newImpulse < 0.0f) {
                    val rhs = pool.popVec2()
                    rhs.set(mass.ez.x, mass.ez.y).mulLocal(this.impulse.z).subLocal(Cdot1)
                    mass.solve22ToOut(rhs, temp)
                    impulse.x = temp.x
                    impulse.y = temp.y
                    impulse.z = -this.impulse.z
                    this.impulse.x += temp.x
                    this.impulse.y += temp.y
                    this.impulse.z = 0.0f
                    pool.pushVec2(1)
                } else {
                    this.impulse.addLocal(impulse)
                }
            } else if (limitState == LimitState.AT_UPPER) {
                val newImpulse = this.impulse.z + impulse.z
                if (newImpulse > 0.0f) {
                    val rhs = pool.popVec2()
                    rhs.set(mass.ez.x, mass.ez.y).mulLocal(this.impulse.z).subLocal(Cdot1)
                    mass.solve22ToOut(rhs, temp)
                    impulse.x = temp.x
                    impulse.y = temp.y
                    impulse.z = -this.impulse.z
                    this.impulse.x += temp.x
                    this.impulse.y += temp.y
                    this.impulse.z = 0.0f
                    pool.pushVec2(1)
                } else {
                    this.impulse.addLocal(impulse)
                }
            }
            val P = pool.popVec2()
            P.set(impulse.x, impulse.y)
            vA.x = vA.x - mA * P.x
            vA.y = vA.y - mA * P.y
            wA = wA - iA * (Vec2.cross(rA, P) + impulse.z)
            vB.x = vB.x + mB * P.x
            vB.y = vB.y + mB * P.y
            wB = wB + iB * (Vec2.cross(rB, P) + impulse.z)
            pool.pushVec2(2)
            pool.pushVec3(2)
        } else {
            // Solve point-to-point constraint
            val Cdot = pool.popVec2()
            val impulse = pool.popVec2()
            Vec2.crossToOutUnsafe(wA, rA, temp)
            Vec2.crossToOutUnsafe(wB, rB, Cdot)
            Cdot.addLocal(vB).subLocal(vA).subLocal(temp)
            mass.solve22ToOut(Cdot.negateLocal(), impulse) // just leave negated
            this.impulse.x += impulse.x
            this.impulse.y += impulse.y
            vA.x = vA.x - mA * impulse.x
            vA.y = vA.y - mA * impulse.y
            wA = wA - iA * Vec2.cross(rA, impulse)
            vB.x = vB.x + mB * impulse.x
            vB.y = vB.y + mB * impulse.y
            wB = wB + iB * Vec2.cross(rB, impulse)
            pool.pushVec2(2)
        }
        // data.velocities[indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
        pool.pushVec2(1)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val cA = data.positions!![indexA].c
        var aA = data.positions!![indexA].a
        val cB = data.positions!![indexB].c
        var aB = data.positions!![indexB].a
        qA.set(aA)
        qB.set(aB)
        var angularError = 0.0f
        val positionError: Float
        val fixedRotation = invIA + invIB == 0.0f
        // Solve angular limit constraint.
        if (enableLimit && limitState != LimitState.INACTIVE && !fixedRotation) {
            val angle = aB - aA - referenceAngle
            var limitImpulse = 0.0f
            if (limitState == LimitState.EQUAL) {
                // Prevent large angular corrections
                val C = MathUtils.clamp(angle - lowerAngle, -Settings.maxAngularCorrection,
                    Settings.maxAngularCorrection)
                limitImpulse = -motorMass * C
                angularError = MathUtils.abs(C)
            } else if (limitState == LimitState.AT_LOWER) {
                var C = angle - lowerAngle
                angularError = -C
                // Prevent large angular corrections and allow some slop.
                C = MathUtils.clamp(C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0f)
                limitImpulse = -motorMass * C
            } else if (limitState == LimitState.AT_UPPER) {
                var C = angle - upperAngle
                angularError = C
                // Prevent large angular corrections and allow some slop.
                C = MathUtils.clamp(C - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection)
                limitImpulse = -motorMass * C
            }
            aA -= invIA * limitImpulse
            aB += invIB * limitImpulse
        }
        // Solve point-to-point constraint.
        run {
            qA.set(aA)
            qB.set(aB)
            val rA = pool.popVec2()
            val rB = pool.popVec2()
            val C = pool.popVec2()
            val impulse = pool.popVec2()
            Rot.mulToOutUnsafe(qA, C.set(localAnchorA).subLocal(localCenterA), rA)
            Rot.mulToOutUnsafe(qB, C.set(localAnchorB).subLocal(localCenterB), rB)
            C.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
            positionError = C.length()
            val mA = invMassA
            val mB = invMassB
            val iA = invIA
            val iB = invIB
            val K = pool.popMat22()
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
            K.ey.x = K.ex.y
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x
            K.solveToOut(C, impulse)
            impulse.negateLocal()
            cA.x = cA.x - mA * impulse.x
            cA.y = cA.y - mA * impulse.y
            aA = aA - iA * Vec2.cross(rA, impulse)
            cB.x = cB.x + mB * impulse.x
            cB.y = cB.y + mB * impulse.y
            aB = aB + iB * Vec2.cross(rB, impulse)
            pool.pushVec2(4)
            pool.pushMat22(1)
        }
        // data.positions[indexA].c.set(cA);
        data.positions!![indexA].a = aA
        // data.positions[indexB].c.set(cB);
        data.positions!![indexB].a = aB
        pool.pushRot(2)
        return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }

    override fun getAnchorA(argOut: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, argOut)
    }

    override fun getAnchorB(argOut: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, argOut)
    }

    override fun getReactionForce(invDt: Float, argOut: Vec2) {
        argOut.set(impulse.x, impulse.y).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return invDt * impulse.z
    }

    fun getJointAngle(): Float {
        val b1 = bodyA!!
        val b2 = bodyB!!
        return b2.sweep.a - b1.sweep.a - referenceAngle
    }

    fun getJointSpeed(): Float {
        val b1 = bodyA!!
        val b2 = bodyB!!
        return b2.angularVelocity - b1.angularVelocity
    }

    fun isMotorEnabled(): Boolean {
        return enableMotor
    }

    fun enableMotor(flag: Boolean) {
        bodyA!!.isAwake = true
        bodyB!!.isAwake = true
        enableMotor = flag
    }

    fun getMotorTorque(inv_dt: Float): Float {
        return motorImpulse * inv_dt
    }

    fun setMotorSpeed(speed: Float) {
        bodyA!!.isAwake = true
        bodyB!!.isAwake = true
        motorSpeed = speed
    }

    fun setMaxMotorTorque(torque: Float) {
        bodyA!!.isAwake = true
        bodyB!!.isAwake = true
        maxMotorTorque = torque
    }

    fun isLimitEnabled(): Boolean {
        return enableLimit
    }

    fun enableLimit(flag: Boolean) {
        if (flag != enableLimit) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            enableLimit = flag
            impulse.z = 0.0f
        }
    }

    fun getLowerLimit(): Float {
        return lowerAngle
    }

    fun getUpperLimit(): Float {
        return upperAngle
    }

    fun setLimits(lower: Float, upper: Float) {
        assert(lower <= upper)
        if (lower != lowerAngle || upper != upperAngle) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            impulse.z = 0.0f
            lowerAngle = lower
            upperAngle = upper
        }
    }
}
