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

import com.hereliesaz.kfizzix.dynamics.joints.RevoluteJoint

/**
 * Revolute joint definition. This requires defining an anchor point where the
 * bodies are joined. The definition uses local anchor points so that the
 * initial configuration can violate the constraint slightly. You also need to
 * specify the initial relative angle for joint limits. This helps when saving
 * and loading a game.
 *
 * The local anchor points are measured from the body's origin rather than the
 * center of mass because: 1. you might not know where the center of mass will
 * be. 2. if you add/remove shapes from a body and recompute the mass, the
 * joints will be broken.
 *
 * @author Daniel Murphy
 */
class RevoluteJointDef : JointDef(JointType.REVOLUTE) {
    /**
     * The local anchor point relative to body1's origin.
     */
    val localAnchorA: com.hereliesaz.kfizzix.common.Vec2 = com.hereliesaz.kfizzix.common.Vec2(0.0f, 0.0f)

    /**
     * The local anchor point relative to body2's origin.
     */
    val localAnchorB: com.hereliesaz.kfizzix.common.Vec2 = com.hereliesaz.kfizzix.common.Vec2(0.0f, 0.0f)

    /**
     * The body2 angle minus body1 angle in the reference state (radians).
     */
    var referenceAngle = 0.0f

    /**
     * A flag to enable joint limits.
     */
    var enableLimit = false

    /**
     * The lower angle for the joint limit (radians).
     */
    var lowerAngle = 0.0f

    /**
     * The upper angle for the joint limit (radians).
     */
    var upperAngle = 0.0f

    /**
     * A flag to enable the joint motor.
     */
    var enableMotor = false

    /**
     * The desired motor speed. Usually in radians per second.
     */
    var motorSpeed = 0.0f

    /**
     * The maximum motor torque used to achieve the desired motor speed. Usually
     * in N-m.
     */
    var maxMotorTorque = 0.0f

    /**
     * Initialize the bodies, anchors, and reference angle using the world
     * anchor.
     */
    fun initialize(b1: com.hereliesaz.kfizzix.dynamics.Body, b2: com.hereliesaz.kfizzix.dynamics.Body, anchor: com.hereliesaz.kfizzix.common.Vec2) {
        bodyA = b1
        bodyB = b2
        bodyA!!.getLocalPointToOut(anchor, localAnchorA)
        bodyB!!.getLocalPointToOut(anchor, localAnchorB)
        referenceAngle = bodyB!!.angle - bodyA!!.angle
    }
}
