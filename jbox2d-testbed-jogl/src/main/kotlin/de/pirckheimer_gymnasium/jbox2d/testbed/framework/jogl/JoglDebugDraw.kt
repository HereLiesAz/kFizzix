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

import com.jogamp.opengl.util.awt.TextRenderer
import de.pirckheimer_gymnasium.jbox2d.callbacks.DebugDraw
import de.pirckheimer_gymnasium.jbox2d.common.*
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleColor
import java.awt.Font
import javax.media.opengl.GL2

class JoglDebugDraw(private val panel: JoglPanel) : DebugDraw() {
    private val text: TextRenderer
    private val mat = FloatArray(16)
    private val zero = Vec2()
    private val temp = Vec2()
    private val temp2 = Vec2()

    init {
        text = TextRenderer(Font("Courier New", Font.PLAIN, 12))
        mat[8] = 0f
        mat[9] = 0f
        mat[2] = 0f
        mat[6] = 0f
        mat[10] = 1f
        mat[14] = 0f
        mat[3] = 0f
        mat[7] = 0f
        mat[11] = 0f
        mat[15] = 1f
    }

    override fun setViewportTransform(viewportTransform: IViewportTransform) {
        viewportTransform.isYFlip = false
        super.setViewportTransform(viewportTransform)
    }

    fun transformViewport(gl: GL2, center: Vec2) {
        val e = viewportTransform.extents
        val vc = viewportTransform.center
        val vt = viewportTransform.mat22Representation
        val f = if (viewportTransform.isYFlip) -1 else 1
        mat[0] = vt.ex.x
        mat[4] = vt.ey.x
        mat[12] = e.x
        mat[1] = f * vt.ex.y
        mat[5] = f * vt.ey.y
        mat[13] = e.y
        gl.glMultMatrixf(mat, 0)
        gl.glTranslatef(center.x - vc.x, center.y - vc.y, 0f)
    }

    override fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f) {
        val vec = worldToScreen(argPoint)
        val gl = panel.gl.gL2
        gl.glPointSize(argRadiusOnScreen)
        gl.glBegin(GL2.GL_POINTS)
        gl.glVertex2f(vec.x, vec.y)
        gl.glEnd()
    }

    override fun drawPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        gl.glBegin(GL2.GL_LINE_LOOP)
        gl.glColor4f(color.x, color.y, color.z, 1f)
        for (i in 0 until vertexCount) {
            val v = vertices[i]
            gl.glVertex2f(v.x, v.y)
        }
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        gl.glBegin(GL2.GL_TRIANGLE_FAN)
        gl.glColor4f(color.x, color.y, color.z, .4f)
        for (i in 0 until vertexCount) {
            val v = vertices[i]
            gl.glVertex2f(v.x, v.y)
        }
        gl.glEnd()
        gl.glBegin(GL2.GL_LINE_LOOP)
        gl.glColor4f(color.x, color.y, color.z, 1f)
        for (i in 0 until vertexCount) {
            val v = vertices[i]
            gl.glVertex2f(v.x, v.y)
        }
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawCircle(center: Vec2, radius: Float, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        val theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS
        val c = MathUtils.cos(theta)
        val s = MathUtils.sin(theta)
        var x = radius
        var y = 0f
        val cx = center.x
        val cy = center.y
        gl.glBegin(GL2.GL_LINE_LOOP)
        gl.glColor4f(color.x, color.y, color.z, 1f)
        for (i in 0 until NUM_CIRCLE_POINTS) {
            gl.glVertex3f(x + cx, y + cy, 0f)
            val temp = x
            x = c * x - s * y
            y = s * temp + c * y
        }
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        val theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS
        val c = MathUtils.cos(theta)
        val s = MathUtils.sin(theta)
        var x = radius
        var y = 0f
        val cx = center.x
        val cy = center.y
        gl.glBegin(GL2.GL_LINE_LOOP)
        gl.glColor4f(color.x, color.y, color.z, 1f)
        for (i in 0 until NUM_CIRCLE_POINTS) {
            gl.glVertex3f(x + cx, y + cy, 0f)
            val temp = x
            x = c * x - s * y
            y = s * temp + c * y
        }
        gl.glEnd()
        gl.glBegin(GL2.GL_LINES)
        gl.glVertex3f(cx, cy, 0f)
        gl.glVertex3f(cx + axis.x * radius, cy + axis.y * radius, 0f)
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        val theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS
        val c = MathUtils.cos(theta)
        val s = MathUtils.sin(theta)
        var x = radius
        var y = 0f
        val cx = center.x
        val cy = center.y
        gl.glBegin(GL2.GL_TRIANGLE_FAN)
        gl.glColor4f(color.x, color.y, color.z, .4f)
        for (i in 0 until NUM_CIRCLE_POINTS) {
            gl.glVertex3f(x + cx, y + cy, 0f)
            val temp = x
            x = c * x - s * y
            y = s * temp + c * y
        }
        gl.glEnd()
        gl.glBegin(GL2.GL_LINE_LOOP)
        gl.glColor4f(color.x, color.y, color.z, 1f)
        x = radius
        y = 0f
        for (i in 0 until NUM_CIRCLE_POINTS) {
            gl.glVertex3f(x + cx, y + cy, 0f)
            val temp = x
            x = c * x - s * y
            y = s * temp + c * y
        }
        gl.glEnd()
        gl.glBegin(GL2.GL_LINES)
        gl.glVertex3f(cx, cy, 0f)
        gl.glVertex3f(cx + axis.x * radius, cy + axis.y * radius, 0f)
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        gl.glBegin(GL2.GL_LINES)
        gl.glColor3f(color.x, color.y, color.z)
        gl.glVertex3f(p1.x, p1.y, 0f)
        gl.glVertex3f(p2.x, p2.y, 0f)
        gl.glEnd()
        gl.glPopMatrix()
    }

    override fun drawParticles(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        val theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS
        val c = MathUtils.cos(theta)
        val s = MathUtils.sin(theta)
        for (i in 0 until count) {
            var x = radius
            var y = 0f
            val center = centers[i]
            val cx = center.x
            val cy = center.y
            gl.glBegin(GL2.GL_TRIANGLE_FAN)
            if (colors == null) {
                gl.glColor4f(1f, 1f, 1f, .4f)
            } else {
                val color = colors[i]
                gl.glColor4b(color.r, color.g, color.b, color.a)
            }
            for (j in 0 until NUM_CIRCLE_POINTS) {
                gl.glVertex3f(x + cx, y + cy, 0f)
                val temp = x
                x = c * x - s * y
                y = s * temp + c * y
            }
            gl.glEnd()
        }
        gl.glPopMatrix()
    }

    override fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int) {
        val gl = panel.gl.gL2
        gl.glPushMatrix()
        transformViewport(gl, zero)
        val theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS
        val c = MathUtils.cos(theta)
        val s = MathUtils.sin(theta)
        for (i in 0 until count) {
            var x = radius
            var y = 0f
            val center = centers[i]
            val cx = center.x
            val cy = center.y
            gl.glBegin(GL2.GL_LINE_LOOP)
            if (colors == null) {
                gl.glColor4f(1f, 1f, 1f, 1f)
            } else {
                val color = colors[i]
                gl.glColor4b(color.r, color.g, color.b, 127.toByte())
            }
            for (j in 0 until NUM_CIRCLE_POINTS) {
                gl.glVertex3f(x + cx, y + cy, 0f)
                val temp = x
                x = c * x - s * y
                y = s * temp + c * y
            }
            gl.glEnd()
        }
        gl.glPopMatrix()
    }

    override fun drawTransform(xf: Transform) {
        val gl = panel.gl.gL2
        worldToScreenToOut(xf.p, temp)
        temp2.setZero()
        val axisScale = 0.4f
        gl.glBegin(GL2.GL_LINES)
        gl.glColor3f(1f, 0f, 0f)
        temp2.x = xf.p.x + axisScale * xf.q.c
        temp2.y = xf.p.y + axisScale * xf.q.s
        worldToScreenToOut(temp2, temp2)
        gl.glVertex2f(temp.x, temp.y)
        gl.glVertex2f(temp2.x, temp2.y)
        gl.glColor3f(0f, 1f, 0f)
        temp2.x = xf.p.x + -axisScale * xf.q.s
        temp2.y = xf.p.y + axisScale * xf.q.c
        worldToScreenToOut(temp2, temp2)
        gl.glVertex2f(temp.x, temp.y)
        gl.glVertex2f(temp2.x, temp2.y)
        gl.glEnd()
    }

    override fun drawString(x: Float, y: Float, s: String, color: Color3f) {
        text.beginRendering(panel.width, panel.height)
        text.setColor(color.x, color.y, color.z, 1f)
        text.draw(s, x.toInt(), panel.height - y.toInt())
        text.endRendering()
    }

    companion object {
        private const val NUM_CIRCLE_POINTS = 13
    }
}
