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

import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.Body
import com.hereliesaz.kfizzix.dynamics.SolverData
import com.hereliesaz.kfizzix.dynamics.World
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * The base joint class. Joints are used to constrain two bodies together in
 * various fashions. Some joints also feature limits and motors.
 *
 *
 * Joints are used to constrain bodies to the world or to each other. Typical
 * examples in games include ragdolls, teeters, and pulleys. Joints can be
 * combined in many different ways to create interesting motions.
 *
 *
 *
 * Some joints provide limits so you can control the range of motion. Some joint
 * provide motors which can be used to drive the joint at a prescribed speed
 * until a prescribed force/torque is exceeded.
 *
 *
 *
 * Joint motors can be used in many ways. You can use motors to control position
 * by specifying a joint velocity that is proportional to the difference between
 * the actual and desired position. You can also use motors to simulate joint
 * friction: set the joint velocity to zero and provide a small, but significant
 * maximum motor force/torque. Then the motor will attempt to keep the joint
 * from moving until the load becomes too strong.
 *
 *
 * https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md81
 *
 * @author Daniel Murphy
 */
abstract class Joint protected constructor(protected var pool: WorldPool, def: JointDef) {
    /**
     * Get the type of the concrete joint.
     */
    val type: JointType
    var prev: Joint? = null
    var next: Joint? = null
    var edgeA: JointEdge
    var edgeB: JointEdge

    /**
     * Get the first body attached to this joint.
     */
    var bodyA: Body

    /**
     * Get the second body attached to this joint.
     */
    var bodyB: Body
    var islandFlag = false

    /**
     * Get collide connected. Note: modifying the collide connect flag won't
     * work correctly because the flag is only checked when fixture AABBs begin
     * to overlap.
     */
    val collideConnected: Boolean

    /**
     * Set the user data pointer.
     */
    /**
     * Get the user data pointer.
     */
    var userData: Any?

    // Cache here per time step to reduce cache misses.
    // final Vec2 localCenterA, localCenterB;
    // float invMassA, invIA;
    // float invMassB, invIB;
    init {
        assert(def.bodyA !== def.bodyB)
        type = def.type
        prev = null
        next = null
        bodyA = def.bodyA!!
        bodyB = def.bodyB!!
        collideConnected = def.collideConnected
        userData = def.userData
        edgeA = JointEdge()
        edgeA.joint = null
        edgeA.other = null
        edgeA.prev = null
        edgeA.next = null
        edgeB = JointEdge()
        edgeB.joint = null
        edgeB.other = null
        edgeB.prev = null
        edgeB.next = null
        // localCenterA = new Vec2();
        // localCenterB = new Vec2();
    }

    /**
     * Get the anchor point on bodyA in world coordinates.
     */
    abstract fun getAnchorA(out: Vec2)

    /**
     * Get the anchor point on bodyB in world coordinates.
     */
    abstract fun getAnchorB(out: Vec2)

    /**
     * Get the reaction force on body2 at the joint anchor in Newtons.
     */
    abstract fun getReactionForce(invDt: Float, out: Vec2)

    /**
     * Get the reaction torque on body2 in N*m.
     */
    abstract fun getReactionTorque(invDt: Float): Float

    /**
     * Short-cut function to determine if either body is inactive.
     */
    val isActive: Boolean
        get() = bodyA.isActive && bodyB.isActive

    /**
     * Internal
     */
    abstract fun initVelocityConstraints(data: SolverData)

    /**
     * Internal
     */
    abstract fun solveVelocityConstraints(data: SolverData)

    /**
     * This returns true if the position errors are within tolerance. Internal.
     */
    abstract fun solvePositionConstraints(data: SolverData): Boolean

    /**
     * Override to handle destruction of joint
     */
    open fun destructor() {}

    companion object {
        fun create(world: World, def: JointDef): Joint? {
            // Joint joint = null;
            return when (def.type) {
                JointType.MOUSE -> MouseJoint(world, def as MouseJointDef)
                JointType.DISTANCE -> DistanceJoint(world, def as DistanceJointDef)
                JointType.PRISMATIC -> PrismaticJoint(world, def as PrismaticJointDef)
                JointType.REVOLUTE -> RevoluteJoint(world, def as RevoluteJointDef)
                JointType.WELD -> WeldJoint(world, def as WeldJointDef)
                JointType.FRICTION -> FrictionJoint(world, def as FrictionJointDef)
                JointType.WHEEL -> WheelJoint(world, def as WheelJointDef)
                JointType.GEAR -> GearJoint(world, def as GearJointDef)
                JointType.PULLEY -> PulleyJoint(world, def as PulleyJointDef)
                JointType.CONSTANT_VOLUME -> ConstantVolumeJoint(
                    world,
                    def as ConstantVolumeJointDef
                )

                JointType.ROPE -> RopeJoint(world, def as RopeJointDef)
                JointType.MOTOR -> MotorJoint(world, def as MotorJointDef)
                else -> null
            }
        }

        fun destroy(joint: Joint) {
            joint.destructor()
        }
    }
}
