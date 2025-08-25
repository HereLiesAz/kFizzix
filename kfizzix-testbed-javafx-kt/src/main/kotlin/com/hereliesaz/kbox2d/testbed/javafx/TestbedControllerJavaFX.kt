/*
 * Copyright (c) 2013, Daniel Murphy All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kbox2d.testbed.javafx

import com.hereliesaz.kbox2d.testbed.framework.AbstractTestbedController
import com.hereliesaz.kbox2d.testbed.framework.TestbedErrorHandler
import com.hereliesaz.kbox2d.testbed.framework.TestbedModel
import javafx.animation.AnimationTimer

/**
 * This class contains most control logic for the testbed and the update loop. It also watches the
 * model to switch tests and populates the model with some loop statistics.
 *
 * @author Daniel Murphy
 */
class TestbedControllerJavaFX(argModel: TestbedModel,
                              behavior: UpdateBehavior, mouseBehavior: MouseBehavior,
                              errorHandler: TestbedErrorHandler) : AbstractTestbedController(argModel, behavior, mouseBehavior, errorHandler) {
    private val animator: AnimationTimer = object : AnimationTimer() {
        var last: Long = -1
        override fun handle(now: Long) {
            if (last >= 0) {
                val dt = (now - last) / 1000000 // convert til millis
                sleepTime -= dt
                if (sleepTime < 0) {
                    stepAndRender()
                }
            }
            last = now
        }
    }

    override fun startAnimator() {
        animator.start()
    }

    override fun stopAnimator() {
        animator.stop()
    }
}
