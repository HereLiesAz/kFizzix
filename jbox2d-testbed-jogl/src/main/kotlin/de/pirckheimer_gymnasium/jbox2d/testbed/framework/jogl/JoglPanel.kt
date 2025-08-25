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
package de.pirckheimer_gymnasium.jbox2d.testbed.framework.jogl

import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedController
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedModel
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedPanel
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.j2d.AWTPanelHelper
import java.awt.Dimension
import javax.media.opengl.GL2
import javax.media.opengl.GLAutoDrawable
import javax.media.opengl.GLCapabilities
import javax.media.opengl.GLEventListener
import javax.media.opengl.GLProfile
import javax.media.opengl.awt.GLJPanel
import javax.media.opengl.glu.GLU

class JoglPanel(model: TestbedModel, private val controller: TestbedController) :
    GLJPanel(GLCapabilities(GLProfile.getDefault())), TestbedPanel, GLEventListener {

    init {
        setSize(INIT_WIDTH, INIT_HEIGHT)
        preferredSize = Dimension(INIT_WIDTH, INIT_HEIGHT)
        autoSwapBufferMode = true
        addGLEventListener(this)
        AWTPanelHelper.addHelpAndPanelListeners(
            this, model, controller,
            SCREEN_DRAG_BUTTON
        )
    }

    override fun render(): Boolean {
        return true
    }

    override fun paintScreen() {
        display()
    }

    override fun display(drawable: GLAutoDrawable) {
        gl.gL2.glClear(GL2.GL_COLOR_BUFFER_BIT)
        controller.updateTest()
        gl.glFlush()
    }

    override fun dispose(drawable: GLAutoDrawable) {}

    override fun init(drawable: GLAutoDrawable) {
        gl.gL2.glLineWidth(1f)
        gl.gL2.glEnable(GL2.GL_BLEND)
        gl.gL2.glBlendFunc(
            GL2.GL_SRC_ALPHA,
            GL2.GL_ONE_MINUS_SRC_ALPHA
        )
    }

    override fun reshape(drawable: GLAutoDrawable, x: Int, y: Int, width: Int, height: Int) {
        val gl2 = drawable.gl.gL2
        gl2.glMatrixMode(GL2.GL_PROJECTION)
        gl2.glLoadIdentity()
        // coordinate system origin at lower left with width and height same as
        // the window
        val glu = GLU()
        glu.gluOrtho2D(0.0f, width.toFloat(), 0.0f, height.toFloat())
        gl2.glMatrixMode(GL2.GL_MODELVIEW)
        gl2.glLoadIdentity()
        gl2.glViewport(0, 0, width, height)
        controller.updateExtents(width / 2, height / 2)
    }

    companion object {
        private const val serialVersionUID = 1L
        const val SCREEN_DRAG_BUTTON = 3
        const val INIT_WIDTH = 600
        const val INIT_HEIGHT = 600
    }
}
