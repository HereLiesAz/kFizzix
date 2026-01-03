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
 * A distance joint constrains two points on two bodies to remain at a fixed
 * distance from each other. You can view this as a massless, rigid rod.
 *
 * <p>
 * One of the simplest joint is a distance joint which says that the distance
 * between two points on two bodies must be constant. When you specify a
 * distance joint the two bodies should already be in place. Then you specify
 * the two anchor points in world coordinates. The first anchor point is
 * connected to body 1, and the second anchor point is connected to body 2.
 * These points imply the length of the distance constraint.
 * </p>
 *
 * <p>
 * <img src=
 * "https://github.com/engine-pi/jbox2d/blob/main/misc/images/joints/distance_joint.svg"
 * alt="distance joint">
 * </p>
 *
 * <p>
 * Here is an example of a distance joint definition. In this case we decide to
 * allow the bodies to collide.
 * </p>
 *
 * <p>
 * The distance joint can also be made soft, like a spring-damper connection.
 * </p>
 *
 * <p>
 * Softness is achieved by tuning two constants in the definition: stiffness and
 * damping. It can be non-intuitive setting these values directly since they
 * have units in terms on Newtons.
 * </p>
 *
 * <p>
 * Think of the frequency as the frequency of a harmonic oscillator (like a
 * guitar string). The frequency is specified in Hertz. Typically the frequency
 * should be less than a half the frequency of the time step. So if you are
 * using a 60Hz time step, the frequency of the distance joint should be less
 * than 30Hz. The reason is related to the Nyquist frequency.
 * </p>
 *
 * <p>
 * The damping ratio is non-dimensional and is typically between 0 and 1, but
 * can be larger. At 1, the damping is critical (all oscillations should
 * vanish).
 * </p>
 *
 * <p>
 * It is also possible to define a minimum and maximum length for the distance
 * joint.
 * </p>
 *
 * @repolink https://github.com/erincatto/box2d/blob/main/src/dynamics/b2_distance_joint.cpp
 *
 * @author Daniel Murphy
 */
class DistanceJoint(argWorld: WorldPool, def: DistanceJointDef) : Joint(argWorld, def) {
    /**
     * The local anchor point relative to bodyA's origin.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_distance_joint.h#L52-L53
     */
    val localAnchorA: Vec2 = def.localAnchorA.clone()

    /**
     * The local anchor point relative to bodyB's origin.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_distance_joint.h#L55-L56
     */
    val localAnchorB: Vec2 = def.localAnchorB.clone()

    /**
     * The equilibrium length between the anchor points.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_distance_joint.h#L58-L59
     */
    var length: Float = def.length

    /**
     * The mass-spring-damper frequency in Hertz.
     */
    var frequency: Float = def.frequencyHz

    /**
     * The damping ratio. 0 = no damping, 1 = critical damping.
     */
    var dampingRatio: Float = def.dampingRatio

    private var bias: Float = 0.0f
    private var gamma: Float = 0.0f
    private var impulse: Float = 0.0f

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val u = Vec2()
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private var invMassA: Float = 0.0f
    private var invMassB: Float = 0.0f
    private var invIA: Float = 0.0f
    private var invIB: Float = 0.0f
    private var mass: Float = 0.0f

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L76-L173
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
        qA.set(aA)
        qB.set(aB)
        // use u as temporary variable
        Rot.mulToOutUnsafe(qA, u.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, u.set(localAnchorB).subLocal(localCenterB), rB)
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        pool.pushRot(2)
        // Handle singularity.
        val length = u.length()
        if (length > Settings.linearSlop) {
            u.x *= 1.0f / length
            u.y *= 1.0f / length
        } else {
            u.set(0.0f, 0.0f)
        }
        val crAu = Vec2.cross(rA, u)
        val crBu = Vec2.cross(rB, u)
        var invMass = invMassA + invIA * crAu * crAu + invMassB + invIB * crBu * crBu
        // Compute the effective mass matrix.
        mass = if (invMass != 0.0f) 1.0f / invMass else 0.0f
        if (frequency > 0.0f) {
            val C = length - this.length
            // Frequency
            val omega = 2.0f * MathUtils.PI * frequency
            // Damping coefficient
            val d = 2.0f * mass * dampingRatio * omega
            // Spring stiffness
            val k = mass * omega * omega
            // magic formulas
            val h = data.step!!.dt
            gamma = h * (d + h * k)
            gamma = if (gamma != 0.0f) 1.0f / gamma else 0.0f
            bias = C * h * k * gamma
            invMass += gamma
            mass = if (invMass != 0.0f) 1.0f / invMass else 0.0f
        } else {
            gamma = 0.0f
            bias = 0.0f
        }
        if (data.step!!.warmStarting) {
            // Scale the impulse to support a variable time step.
            impulse *= data.step!!.dtRatio
            val P = pool.popVec2()
            P.set(u).mulLocal(impulse)
            vA.x = vA.x - invMassA * P.x
            vA.y = vA.y - invMassA * P.y
            wA = wA - invIA * Vec2.cross(rA, P)
            vB.x = vB.x + invMassB * P.x
            vB.y = vB.y + invMassB * P.y
            wB = wB + invIB * Vec2.cross(rB, P)
            pool.pushVec2(1)
        } else {
            impulse = 0.0f
        }
        // data.velocities[indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_wheel_joint.cpp#L237-L342
     */
    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w
        val vpA = pool.popVec2()
        val vpB = pool.popVec2()
        // Cdot = dot(u, v + cross(w, r))
        Vec2.crossToOutUnsafe(wA, rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, rB, vpB)
        vpB.addLocal(vB)
        val Cdot = Vec2.dot(u, vpB.subLocal(vpA))
        val impulse = -mass * (Cdot + bias + gamma * this.impulse)
        this.impulse += impulse
        val Px = impulse * u.x
        val Py = impulse * u.y
        vA.x -= invMassA * Px
        vA.y -= invMassA * Py
        wA -= invIA * (rA.x * Py - rA.y * Px)
        vB.x += invMassB * Px
        vB.y += invMassB * Py
        wB += invIB * (rB.x * Py - rB.y * Px)
        // data.velocities[indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[indexB].v.set(vB);
        data.velocities!![indexB].w = wB
        pool.pushVec2(2)
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L268-L314
     */
    override fun solvePositionConstraints(data: SolverData): Boolean {
        if (frequency > 0.0f) {
            return true
        }
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val u = pool.popVec2()
        val cA = data.positions!![indexA].c
        var aA = data.positions!![indexA].a
        val cB = data.positions!![indexB].c
        var aB = data.positions!![indexB].a
        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, u.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, u.set(localAnchorB).subLocal(localCenterB), rB)
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        val length = u.normalize()
        var C = length - this.length
        C = MathUtils.clamp(C, -Settings.maxLinearCorrection,
            Settings.maxLinearCorrection)
        val impulse = -mass * C
        val Px = impulse * u.x
        val Py = impulse * u.y
        cA.x = cA.x - invMassA * Px
        cA.y = cA.y - invMassA * Py
        aA = aA - invIA * (rA.x * Py - rA.y * Px)
        cB.x = cB.x + invMassB * Px
        cB.y = cB.y + invMassB * Py
        aB = aB + invIB * (rB.x * Py - rB.y * Px)
        // data.positions[indexA].c.set(cA);
        data.positions!![indexA].a = aA
        // data.positions[indexB].c.set(cB);
        data.positions!![indexB].a = aB
        pool.pushVec2(3)
        pool.pushRot(2)
        return MathUtils.abs(C) < Settings.linearSlop
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L316-L319
     */
    override fun getAnchorA(argOut: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, argOut)
    }

    /**
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L321-L324
     */
    override fun getAnchorB(argOut: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, argOut)
    }

    /**
     * Get the reaction force given the inverse time step. Unit is N.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L326-L330
     */
    override fun getReactionForce(invDt: Float, argOut: Vec2) {
        argOut.x = impulse * u.x * invDt
        argOut.y = impulse * u.y * invDt
    }

    /**
     * Get the reaction torque given the inverse time step. Unit is N*m. This is
     * always zero for a distance joint.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/src/dynamics/b2_distance_joint.cpp#L332-L336
     */
    override fun getReactionTorque(invDt: Float): Float {
        return 0.0f
    }
}
