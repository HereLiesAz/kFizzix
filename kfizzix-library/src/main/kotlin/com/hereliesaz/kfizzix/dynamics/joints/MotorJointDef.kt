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

/**
 * Motor joint definition.
 *
 * @author Daniel Murphy
 */
class MotorJointDef : JointDef(JointType.MOTOR) {
    /**
     * Position of bodyB minus the position of bodyA, in bodyA's frame, in
     * meters.
     */
    val linearOffset: Vec2 = Vec2()

    /**
     * The bodyB angle minus bodyA angle in radians.
     */
    var angularOffset: Float = 0f

    /**
     * The maximum motor force in N.
     */
    var maxForce: Float = 1f

    /**
     * The maximum motor torque in N-m.
     */
    var maxTorque: Float = 1f

    /**
     * Position correction factor in the range [0,1].
     */
    var correctionFactor: Float = 0.3f

    fun initialize(bA: Body, bB: Body) {
        bodyA = bA
        bodyB = bB
        val xB = bB.position
        bA.getLocalPointToOut(xB, linearOffset)
        val angleA = bA.angle
        val angleB = bB.angle
        angularOffset = angleB - angleA
    }
}
