/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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
package de.pirckheimer_gymnasium.jbox2d.testbed.framework.jogl

import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestList
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedController
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedErrorHandler
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedModel
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.AbstractTestbedController.MouseBehavior
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.AbstractTestbedController.UpdateBehavior
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.j2d.TestbedSidePanel
import java.awt.BorderLayout
import javax.swing.JFrame
import javax.swing.JOptionPane
import javax.swing.JScrollPane
import javax.swing.SwingUtilities

object JoglTestbedMain {
    @JvmStatic
    fun main(args: Array<String>) {
        val model = TestbedModel()
        val controller = TestbedController(model,
            UpdateBehavior.UPDATE_IGNORED, MouseBehavior.FORCE_Y_FLIP,
            object : TestbedErrorHandler {
                override fun serializationError(e: Exception, message: String) {
                    JOptionPane.showMessageDialog(null, message,
                        "Serialization Error",
                        JOptionPane.ERROR_MESSAGE)
                }
            })
        val panel = JoglPanel(model, controller)
        model.debugDraw = JoglDebugDraw(panel)
        model.panel = panel
        TestList.populateModel(model)
        model.settings
            .getSetting(TestbedSettings.DrawWireframe).enabled = false
        val testbed = JFrame("JBox2D Testbed")
        testbed.layout = BorderLayout()
        val side = TestbedSidePanel(model, controller)
        testbed.add(panel, BorderLayout.CENTER)
        testbed.add(JScrollPane(side), BorderLayout.EAST)
        testbed.pack()
        testbed.isVisible = true
        testbed.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
        SwingUtilities.invokeLater {
            controller.playTest(0)
            controller.start()
        }
    }
}
