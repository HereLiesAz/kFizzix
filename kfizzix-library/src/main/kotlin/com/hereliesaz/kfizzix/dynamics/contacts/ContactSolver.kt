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
package com.hereliesaz.kfizzix.dynamics.contacts

import com.hereliesaz.kfizzix.collision.Manifold.ManifoldType
import com.hereliesaz.kfizzix.collision.WorldManifold
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.TimeStep

/**
 * @author Daniel Murphy
 */
class ContactSolver {
    var step: TimeStep? = null
    var positions: Array<Position>? = null
    var velocities: Array<Velocity>? = null
    var positionConstraints: Array<ContactPositionConstraint>
    var velocityConstraints: Array<ContactVelocityConstraint>
    var contacts: Array<Contact?>? = null
    var count: Int = 0

    init {
        positionConstraints = Array(INITIAL_NUM_CONSTRAINTS) { ContactPositionConstraint() }
        velocityConstraints = Array(INITIAL_NUM_CONSTRAINTS) { ContactVelocityConstraint() }
    }

    fun init(def: ContactSolverDef) {
        step = def.step
        count = def.count
        if (positionConstraints.size < count) {
            val old = positionConstraints
            positionConstraints = Array(MathUtils.max(old.size * 2, count)) { i ->
                if (i < old.size) old[i] else ContactPositionConstraint()
            }
        }
        if (velocityConstraints.size < count) {
            val old = velocityConstraints
            velocityConstraints = Array(MathUtils.max(old.size * 2, count)) { i ->
                if (i < old.size) old[i] else ContactVelocityConstraint()
            }
        }
        positions = def.positions
        velocities = def.velocities
        contacts = def.contacts
        for (i in 0 until count) {
            val contact = contacts!![i]!!
            val fixtureA = contact.fixtureA
            val fixtureB = contact.fixtureB
            val shapeA = fixtureA!!.shape
            val shapeB = fixtureB!!.shape
            val radiusA = shapeA!!.radius
            val radiusB = shapeB!!.radius
            val bodyA = fixtureA.body
            val bodyB = fixtureB.body
            val manifold = contact.manifold
            val pointCount = manifold!!.pointCount
            assert(pointCount > 0)
            val vc = velocityConstraints[i]
            vc.friction = contact.friction
            vc.restitution = contact.restitution
            vc.tangentSpeed = contact.tangentSpeed
            vc.indexA = bodyA!!.islandIndex
            vc.indexB = bodyB!!.islandIndex
            vc.invMassA = bodyA.invMass
            vc.invMassB = bodyB.invMass
            vc.invIA = bodyA.invI
            vc.invIB = bodyB.invI
            vc.contactIndex = i
            vc.pointCount = pointCount
            vc.K.setZero()
            vc.normalMass.setZero()
            val pc = positionConstraints[i]
            pc.indexA = bodyA.islandIndex
            pc.indexB = bodyB.islandIndex
            pc.invMassA = bodyA.invMass
            pc.invMassB = bodyB.invMass
            pc.localCenterA.set(bodyA.sweep.localCenter)
            pc.localCenterB.set(bodyB.sweep.localCenter)
            pc.invIA = bodyA.invI
            pc.invIB = bodyB.invI
            pc.localNormal.set(manifold.localNormal)
            pc.localPoint.set(manifold.localPoint)
            pc.pointCount = pointCount
            pc.radiusA = radiusA
            pc.radiusB = radiusB
            pc.type = manifold.type
            for (j in 0 until pointCount) {
                val cp = manifold.points[j]
                val vcp = vc.points[j]
                if (step!!.warmStarting) {
                    vcp.normalImpulse = step!!.dtRatio * cp.normalImpulse
                    vcp.tangentImpulse = step!!.dtRatio * cp.tangentImpulse
                } else {
                    vcp.normalImpulse = 0f
                    vcp.tangentImpulse = 0f
                }
                vcp.rA.setZero()
                vcp.rB.setZero()
                vcp.normalMass = 0f
                vcp.tangentMass = 0f
                vcp.velocityBias = 0f
                pc.localPoints[j].x = cp.localPoint.x
                pc.localPoints[j].y = cp.localPoint.y
            }
        }
    }

    fun warmStart() {
        for (i in 0 until count) {
            val vc = velocityConstraints[i]
            val indexA = vc.indexA
            val indexB = vc.indexB
            val mA = vc.invMassA
            val iA = vc.invIA
            val mB = vc.invMassB
            val iB = vc.invIB
            val pointCount = vc.pointCount
            val vA = velocities!![indexA].v
            var wA = velocities!![indexA].w
            val vB = velocities!![indexB].v
            var wB = velocities!![indexB].w
            val normal = vc.normal
            val tangentX = normal.y
            val tangentY = -1.0f * normal.x
            for (j in 0 until pointCount) {
                val vcp = vc.points[j]
                val Px = tangentX * vcp.tangentImpulse + normal.x * vcp.normalImpulse
                val Py = tangentY * vcp.tangentImpulse + normal.y * vcp.normalImpulse
                wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px)
                vA.x -= Px * mA
                vA.y -= Py * mA
                wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px)
                vB.x += Px * mB
                vB.y += Py * mB
            }
            velocities!![indexA].w = wA
            velocities!![indexB].w = wB
        }
    }

    private val xfA = Transform()
    private val xfB = Transform()
    private val worldManifold = WorldManifold()

    fun initializeVelocityConstraints() {
        for (i in 0 until count) {
            val vc = velocityConstraints[i]
            val pc = positionConstraints[i]
            val radiusA = pc.radiusA
            val radiusB = pc.radiusB
            val manifold = contacts!![vc.contactIndex]!!.manifold
            val indexA = vc.indexA
            val indexB = vc.indexB
            val mA = vc.invMassA
            val mB = vc.invMassB
            val iA = vc.invIA
            val iB = vc.invIB
            val localCenterA = pc.localCenterA
            val localCenterB = pc.localCenterB
            val cA = positions!![indexA].c
            val aA = positions!![indexA].a
            val vA = velocities!![indexA].v
            val wA = velocities!![indexA].w
            val cB = positions!![indexB].c
            val aB = positions!![indexB].a
            val vB = velocities!![indexB].v
            val wB = velocities!![indexB].w
            assert(manifold!!.pointCount > 0)
            val xfAq = xfA.q
            val xfBq = xfB.q
            xfAq.set(aA)
            xfBq.set(aB)
            xfA.p.x = cA.x - (xfAq.c * localCenterA.x - xfAq.s * localCenterA.y)
            xfA.p.y = cA.y - (xfAq.s * localCenterA.x + xfAq.c * localCenterA.y)
            xfB.p.x = cB.x - (xfBq.c * localCenterB.x - xfBq.s * localCenterB.y)
            xfB.p.y = cB.y - (xfBq.s * localCenterB.x + xfBq.c * localCenterB.y)
            worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB)
            val vcNormal = vc.normal
            vcNormal.x = worldManifold.normal.x
            vcNormal.y = worldManifold.normal.y
            val pointCount = vc.pointCount
            for (j in 0 until pointCount) {
                val vcp = vc.points[j]
                val wmPj = worldManifold.points[j]
                val vcprA = vcp.rA
                val vcprB = vcp.rB
                vcprA.x = wmPj.x - cA.x
                vcprA.y = wmPj.y - cA.y
                vcprB.x = wmPj.x - cB.x
                vcprB.y = wmPj.y - cB.y
                val rnA = vcprA.x * vcNormal.y - vcprA.y * vcNormal.x
                val rnB = vcprB.x * vcNormal.y - vcprB.y * vcNormal.x
                val kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB
                vcp.normalMass = if (kNormal > 0.0f) 1.0f / kNormal else 0.0f
                val tangentX = vcNormal.y
                val tangentY = -1.0f * vcNormal.x
                val rtA = vcprA.x * tangentY - vcprA.y * tangentX
                val rtB = vcprB.x * tangentY - vcprB.y * tangentX
                val kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB
                vcp.tangentMass = if (kTangent > 0.0f) 1.0f / kTangent else 0.0f
                vcp.velocityBias = 0.0f
                val tempX = vB.x + -wB * vcprB.y - vA.x - (-wA * vcprA.y)
                val tempY = vB.y + wB * vcprB.x - vA.y - (wA * vcprA.x)
                val vRel = vcNormal.x * tempX + vcNormal.y * tempY
                if (vRel < -Settings.velocityThreshold) {
                    vcp.velocityBias = -vc.restitution * vRel
                }
            }
            if (vc.pointCount == 2) {
                val vcp1 = vc.points[0]
                val vcp2 = vc.points[1]
                val rn1A = vcp1.rA.x * vcNormal.y - vcp1.rA.y * vcNormal.x
                val rn1B = vcp1.rB.x * vcNormal.y - vcp1.rB.y * vcNormal.x
                val rn2A = vcp2.rA.x * vcNormal.y - vcp2.rA.y * vcNormal.x
                val rn2B = vcp2.rB.x * vcNormal.y - vcp2.rB.y * vcNormal.x
                val k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B
                val k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B
                val k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B
                if (k11 * k11 < maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    vc.K.ex.x = k11
                    vc.K.ex.y = k12
                    vc.K.ey.x = k12
                    vc.K.ey.y = k22
                    vc.K.invertToOut(vc.normalMass)
                } else {
                    vc.pointCount = 1
                }
            }
        }
    }

    fun solveVelocityConstraints() {
        for (i in 0 until count) {
            val vc = velocityConstraints[i]
            val indexA = vc.indexA
            val indexB = vc.indexB
            val mA = vc.invMassA
            val mB = vc.invMassB
            val iA = vc.invIA
            val iB = vc.invIB
            val pointCount = vc.pointCount
            val vA = velocities!![indexA].v
            var wA = velocities!![indexA].w
            val vB = velocities!![indexB].v
            var wB = velocities!![indexB].w
            val normal = vc.normal
            val normalX = normal.x
            val normalY = normal.y
            val tangentX = vc.normal.y
            val tangentY = -1.0f * vc.normal.x
            val friction = vc.friction
            assert(pointCount == 1 || pointCount == 2)
            for (j in 0 until pointCount) {
                val vcp = vc.points[j]
                val a = vcp.rA
                val dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * a.y
                val dvy = wB * vcp.rB.x + vB.y - vA.y - wA * a.x
                val vt = dvx * tangentX + dvy * tangentY - vc.tangentSpeed
                var lambda = vcp.tangentMass * (-vt)
                val maxFriction = friction * vcp.normalImpulse
                val newImpulse = MathUtils.clamp(
                    vcp.tangentImpulse + lambda, -maxFriction, maxFriction
                )
                lambda = newImpulse - vcp.tangentImpulse
                vcp.tangentImpulse = newImpulse
                val Px = tangentX * lambda
                val Py = tangentY * lambda
                vA.x -= Px * mA
                vA.y -= Py * mA
                wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px)
                vB.x += Px * mB
                vB.y += Py * mB
                wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px)
            }
            if (vc.pointCount == 1) {
                val vcp = vc.points[0]
                val dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * vcp.rA.y
                val dvy = wB * vcp.rB.x + vB.y - vA.y - wA * vcp.rA.x
                val vn = dvx * normalX + dvy * normalY
                var lambda = -vcp.normalMass * (vn - vcp.velocityBias)
                val a = vcp.normalImpulse + lambda
                val newImpulse = MathUtils.max(a, 0.0f)
                lambda = newImpulse - vcp.normalImpulse
                vcp.normalImpulse = newImpulse
                val Px = normalX * lambda
                val Py = normalY * lambda
                vA.x -= Px * mA
                vA.y -= Py * mA
                wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px)
                vB.x += Px * mB
                vB.y += Py * mB
                wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px)
            } else {
                val cp1 = vc.points[0]
                val cp2 = vc.points[1]
                val cp1rA = cp1.rA
                val cp1rB = cp1.rB
                val cp2rA = cp2.rA
                val cp2rB = cp2.rB
                val ax = cp1.normalImpulse
                val ay = cp2.normalImpulse
                assert(ax >= 0.0f && ay >= 0.0f)
                val dv1x = -wB * cp1rB.y + vB.x - vA.x + wA * cp1rA.y
                val dv1y = wB * cp1rB.x + vB.y - vA.y - wA * cp1rA.x
                val dv2x = -wB * cp2rB.y + vB.x - vA.x + wA * cp2rA.y
                val dv2y = wB * cp2rB.x + vB.y - vA.y - wA * cp2rA.x
                var vn1 = dv1x * normalX + dv1y * normalY
                var vn2 = dv2x * normalX + dv2y * normalY
                var bx = vn1 - cp1.velocityBias
                var by = vn2 - cp2.velocityBias
                val R = vc.K
                bx -= R.ex.x * ax + R.ey.x * ay
                by -= R.ex.y * ax + R.ey.y * ay
                loop@ while (true) {
                    val R1 = vc.normalMass
                    var xx = R1.ex.x * bx + R1.ey.x * by
                    var xy = R1.ex.y * bx + R1.ey.y * by
                    xx *= -1f
                    xy *= -1f
                    if (xx >= 0.0f && xy >= 0.0f) {
                        val dx = xx - ax
                        val dy = xy - ay
                        val P1x = dx * normalX
                        val P1y = dx * normalY
                        val P2x = dy * normalX
                        val P2y = dy * normalY
                        vA.x -= mA * (P1x + P2x)
                        vA.y -= mA * (P1y + P2y)
                        vB.x += mB * (P1x + P2x)
                        vB.y += mB * (P1y + P2y)
                        wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x))
                        wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x))
                        cp1.normalImpulse = xx
                        cp2.normalImpulse = xy
                        if (DEBUG_SOLVER) {
                            val dv1 = vB + (Vec2.cross(wB, cp1rB) - vA - Vec2.cross(wA, cp1rA))
                            val dv2 = vB + (Vec2.cross(wB, cp2rB) - vA - Vec2.cross(wA, cp2rA))
                            vn1 = Vec2.dot(dv1, normal)
                            vn2 = Vec2.dot(dv2, normal)
                            assert(MathUtils.abs(vn1 - cp1.velocityBias) < errorTol)
                            assert(MathUtils.abs(vn2 - cp2.velocityBias) < errorTol)
                        }
                        break@loop
                    }
                    xx = -cp1.normalMass * bx
                    xy = 0.0f
                    vn2 = vc.K.ex.y * xx + by
                    if (xx >= 0.0f && vn2 >= 0.0f) {
                        val dx = xx - ax
                        val dy = xy - ay
                        val P1x = normalX * dx
                        val P1y = normalY * dx
                        val P2x = normalX * dy
                        val P2y = normalY * dy
                        vA.x -= mA * (P1x + P2x)
                        vA.y -= mA * (P1y + P2y)
                        vB.x += mB * (P1x + P2x)
                        vB.y += mB * (P1y + P2y)
                        wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x))
                        wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x))
                        cp1.normalImpulse = xx
                        cp2.normalImpulse = xy
                        if (DEBUG_SOLVER) {
                            val dv1 = vB + (Vec2.cross(wB, cp1rB) - vA - Vec2.cross(wA, cp1rA))
                            vn1 = Vec2.dot(dv1, normal)
                            assert(MathUtils.abs(vn1 - cp1.velocityBias) < errorTol)
                        }
                        break@loop
                    }
                    xx = 0.0f
                    xy = -cp2.normalMass * by
                    vn1 = vc.K.ey.x * xy + bx
                    if (xy >= 0.0f && vn1 >= 0.0f) {
                        val dx = xx - ax
                        val dy = xy - ay
                        val P1x = normalX * dx
                        val P1y = normalY * dx
                        val P2x = normalX * dy
                        val P2y = normalY * dy
                        vA.x -= mA * (P1x + P2x)
                        vA.y -= mA * (P1y + P2y)
                        vB.x += mB * (P1x + P2x)
                        vB.y += mB * (P1y + P2y)
                        wA -= iA * (cp1rA.x * P1y - cp1rA.y * P1x + (cp2rA.x * P2y - cp2rA.y * P2x))
                        wB += iB * (cp1rB.x * P1y - cp1rB.y * P1x + (cp2rB.x * P2y - cp2rB.y * P2x))
                        cp1.normalImpulse = xx
                        cp2.normalImpulse = xy
                        if (DEBUG_SOLVER) {
                            val dv2 = vB + (Vec2.cross(wB, cp2rB) - vA - Vec2.cross(wA, cp2rA))
                            vn2 = Vec2.dot(dv2, normal)
                            assert(MathUtils.abs(vn2 - cp2.velocityBias) < errorTol)
                        }
                        break@loop
                    }
                    xx = 0.0f
                    xy = 0.0f
                    vn1 = bx
                    vn2 = by
                    if (vn1 >= 0.0f && vn2 >= 0.0f) {
                        cp1.normalImpulse = xx
                        cp2.normalImpulse = xy
                        break@loop
                    }
                    break@loop
                }
            }
            velocities!![indexA].w = wA
            velocities!![indexB].w = wB
        }
    }

    fun storeImpulses() {
        for (i in 0 until count) {
            val vc = velocityConstraints[i]
            val manifold = contacts!![vc.contactIndex]!!.manifold
            for (j in 0 until vc.pointCount) {
                manifold!!.points[j].normalImpulse = vc.points[j].normalImpulse
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse
            }
        }
    }

    private val psolver = PositionSolverManifold()

    fun solvePositionConstraints(): Boolean {
        var minSeparation = 0.0f
        for (i in 0 until count) {
            val pc = positionConstraints[i]
            val indexA = pc.indexA
            val indexB = pc.indexB
            val mA = pc.invMassA
            val iA = pc.invIA
            val localCenterAx = pc.localCenterA.x
            val localCenterAy = pc.localCenterA.y
            val mB = pc.invMassB
            val iB = pc.invIB
            val localCenterBx = pc.localCenterB.x
            val localCenterBy = pc.localCenterB.y
            val pointCount = pc.pointCount
            val cA = positions!![indexA].c
            var aA = positions!![indexA].a
            val cB = positions!![indexB].c
            var aB = positions!![indexB].a
            for (j in 0 until pointCount) {
                val xfAq = xfA.q
                val xfBq = xfB.q
                xfAq.set(aA)
                xfBq.set(aB)
                xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy
                xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy
                xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy
                xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy
                val psm = psolver
                psm.initialize(pc, xfA, xfB, j)
                val normal = psm.normal
                val point = psm.point
                val separation = psm.separation
                val rAx = point.x - cA.x
                val rAy = point.y - cA.y
                val rBx = point.x - cB.x
                val rBy = point.y - cB.y
                minSeparation = MathUtils.min(minSeparation, separation)
                val C = MathUtils.clamp(
                    Settings.baumgarte * (separation + Settings.linearSlop),
                    -Settings.maxLinearCorrection, 0.0f
                )
                val rnA = rAx * normal.y - rAy * normal.x
                val rnB = rBx * normal.y - rBy * normal.x
                val K = mA + mB + iA * rnA * rnA + iB * rnB * rnB
                val impulse = if (K > 0.0f) -C / K else 0.0f
                val Px = normal.x * impulse
                val Py = normal.y * impulse
                cA.x -= Px * mA
                cA.y -= Py * mA
                aA -= iA * (rAx * Py - rAy * Px)
                cB.x += Px * mB
                cB.y += Py * mB
                aB += iB * (rBx * Py - rBy * Px)
            }
            positions!![indexA].a = aA
            positions!![indexB].a = aB
        }
        return minSeparation >= -3.0f * Settings.linearSlop
    }

    fun solveTOIPositionConstraints(toiIndexA: Int, toiIndexB: Int): Boolean {
        var minSeparation = 0.0f
        for (i in 0 until count) {
            val pc = positionConstraints[i]
            val indexA = pc.indexA
            val indexB = pc.indexB
            val localCenterA = pc.localCenterA
            val localCenterB = pc.localCenterB
            val localCenterAx = localCenterA.x
            val localCenterAy = localCenterA.y
            val localCenterBx = localCenterB.x
            val localCenterBy = localCenterB.y
            val pointCount = pc.pointCount
            var mA = 0.0f
            var iA = 0.0f
            if (indexA == toiIndexA || indexA == toiIndexB) {
                mA = pc.invMassA
                iA = pc.invIA
            }
            var mB = 0f
            var iB = 0f
            if (indexB == toiIndexA || indexB == toiIndexB) {
                mB = pc.invMassB
                iB = pc.invIB
            }
            val cA = positions!![indexA].c
            var aA = positions!![indexA].a
            val cB = positions!![indexB].c
            var aB = positions!![indexB].a
            for (j in 0 until pointCount) {
                val xfAq = xfA.q
                val xfBq = xfB.q
                xfAq.set(aA)
                xfBq.set(aB)
                xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy
                xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy
                xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy
                xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy
                val psm = psolver
                psm.initialize(pc, xfA, xfB, j)
                val normal = psm.normal
                val point = psm.point
                val separation = psm.separation
                val rAx = point.x - cA.x
                val rAy = point.y - cA.y
                val rBx = point.x - cB.x
                val rBy = point.y - cB.y
                minSeparation = MathUtils.min(minSeparation, separation)
                val C = MathUtils.clamp(
                    Settings.toiBaugarte * (separation + Settings.linearSlop),
                    -Settings.maxLinearCorrection, 0.0f
                )
                val rnA = rAx * normal.y - rAy * normal.x
                val rnB = rBx * normal.y - rBy * normal.x
                val K = mA + mB + iA * rnA * rnA + iB * rnB * rnB
                val impulse = if (K > 0.0f) -C / K else 0.0f
                val Px = normal.x * impulse
                val Py = normal.y * impulse
                cA.x -= Px * mA
                cA.y -= Py * mA
                aA -= iA * (rAx * Py - rAy * Px)
                cB.x += Px * mB
                cB.y += Py * mB
                aB += iB * (rBx * Py - rBy * Px)
            }
            positions!![indexA].a = aA
            positions!![indexB].a = aB
        }
        return minSeparation >= -1.5f * Settings.linearSlop
    }

    class ContactSolverDef {
        var step: TimeStep? = null
        var contacts: Array<Contact?>? = null
        var count: Int = 0
        var positions: Array<Position>? = null
        var velocities: Array<Velocity>? = null
    }

    companion object {
        const val DEBUG_SOLVER = false
        const val errorTol = 1e-3f
        const val INITIAL_NUM_CONSTRAINTS = 256
        const val maxConditionNumber = 100.0f
    }
}

class PositionSolverManifold {
    val normal = Vec2()
    val point = Vec2()
    var separation = 0f

    fun initialize(pc: ContactPositionConstraint, xfA: Transform, xfB: Transform, index: Int) {
        assert(pc.pointCount > 0)
        val xfAq = xfA.q
        val xfBq = xfB.q
        val pcLocalPointsI = pc.localPoints[index]
        when (pc.type) {
            ManifoldType.CIRCLES -> {
                val plocalPoint = pc.localPoint
                val pLocalPoints0 = pc.localPoints[0]
                val pointAx = xfAq.c * plocalPoint.x - xfAq.s * plocalPoint.y + xfA.p.x
                val pointAy = xfAq.s * plocalPoint.x + xfAq.c * plocalPoint.y + xfA.p.y
                val pointBx = xfBq.c * pLocalPoints0.x - xfBq.s * pLocalPoints0.y + xfB.p.x
                val pointBy = xfBq.s * pLocalPoints0.x + xfBq.c * pLocalPoints0.y + xfB.p.y
                normal.x = pointBx - pointAx
                normal.y = pointBy - pointAy
                normal.normalize()
                point.x = (pointAx + pointBx) * .5f
                point.y = (pointAy + pointBy) * .5f
                val tempx = pointBx - pointAx
                val tempy = pointBy - pointAy
                separation = tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB
            }
            ManifoldType.FACE_A -> {
                val pcLocalNormal = pc.localNormal
                val pcLocalPoint = pc.localPoint
                normal.x = xfAq.c * pcLocalNormal.x - xfAq.s * pcLocalNormal.y
                normal.y = xfAq.s * pcLocalNormal.x + xfAq.c * pcLocalNormal.y
                val planePointX = xfAq.c * pcLocalPoint.x - xfAq.s * pcLocalPoint.y + xfA.p.x
                val planePointY = xfAq.s * pcLocalPoint.x + xfAq.c * pcLocalPoint.y + xfA.p.y
                val clipPointX = xfBq.c * pcLocalPointsI.x - xfBq.s * pcLocalPointsI.y + xfB.p.x
                val clipPointy = xfBq.s * pcLocalPointsI.x + xfBq.c * pcLocalPointsI.y + xfB.p.y
                val tempX = clipPointX - planePointX
                val tempY = clipPointy - planePointY
                separation = tempX * normal.x + tempY * normal.y - pc.radiusA - pc.radiusB
                point.x = clipPointX
                point.y = clipPointy
            }
            ManifoldType.FACE_B -> {
                val pcLocalNormal = pc.localNormal
                val pcLocalPoint = pc.localPoint
                normal.x = xfBq.c * pcLocalNormal.x - xfBq.s * pcLocalNormal.y
                normal.y = xfBq.s * pcLocalNormal.x + xfBq.c * pcLocalNormal.y
                val planePointX = xfBq.c * pcLocalPoint.x - xfBq.s * pcLocalPoint.y + xfB.p.x
                val planePointY = xfBq.s * pcLocalPoint.x + xfBq.c * pcLocalPoint.y + xfB.p.y
                val clipPointX = xfAq.c * pcLocalPointsI.x - xfAq.s * pcLocalPointsI.y + xfA.p.x
                val clipPointy = xfAq.s * pcLocalPointsI.x + xfAq.c * pcLocalPointsI.y + xfA.p.y
                val tempX = clipPointX - planePointX
                val tempY = clipPointy - planePointY
                separation = tempX * normal.x + tempY * normal.y - pc.radiusA - pc.radiusB
                point.x = clipPointX
                point.y = clipPointy
                normal.x *= -1f
                normal.y *= -1f
            }
            else -> {}
        }
    }
}
