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

import com.hereliesaz.kbox2d.callbacks.DebugDraw
import com.hereliesaz.kbox2d.collision.AABB
import com.hereliesaz.kbox2d.common.*
import com.hereliesaz.kbox2d.particle.ParticleColor
import com.hereliesaz.kbox2d.pooling.arrays.Vec2Array
import com.hereliesaz.kbox2d.testbed.pooling.ColorPool
import javafx.geometry.Rectangle2D
import javafx.scene.canvas.GraphicsContext
import javafx.scene.paint.Color
import javafx.scene.transform.Affine

/**
 * Implementation of [DebugDraw] that uses JavaFX! Hooray!<br>
 *
 * @author Daniel Murphy Initial Java 2D implementation
 * @author Hallvard Traetteberg JavaFX port
 */
class DebugDrawJavaFX(private val panel: TestPanelJavaFX, private val yFlip: Boolean) : DebugDraw() {
    private val cpool: ColorPool<Color> = object : ColorPool<Color>() {
        override fun newColor(r: Float, g: Float, b: Float, alpha: Float): Color {
            return Color(r.toDouble(), g.toDouble(), b.toDouble(), alpha.toDouble())
        }
    }
    private var circle: Rectangle2D
    override fun setViewportTransform(viewportTransform: IViewportTransform) {
        super.setViewportTransform(viewportTransform)
        viewportTransform.isYFlip = yFlip
    }

    private val vec2Array = Vec2Array()
    override fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f) {
        getWorldToScreenToOut(argPoint, sp1)
        val g = graphics
        val c = cpool.getColor(argColor.x, argColor.y, argColor.z)
        g.stroke = c
        sp1.x -= argRadiusOnScreen
        sp1.y -= argRadiusOnScreen
        g.fillOval(sp1.x.toDouble(), sp1.y.toDouble(), (argRadiusOnScreen * 2).toDouble(),
                (argRadiusOnScreen * 2).toDouble())
    }

    private val sp1 = Vec2()
    private val sp2 = Vec2()
    override fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f) {
        getWorldToScreenToOut(p1, sp1)
        getWorldToScreenToOut(p2, sp2)
        val c = cpool.getColor(color.x, color.y, color.z)
        val g = graphics
        g.stroke = c
        g.lineWidth = 0.0
        g.strokeLine(sp1.x.toDouble(), sp1.y.toDouble(), sp2.x.toDouble(), sp2.y.toDouble())
    }

    fun drawAABB(argAABB: AABB, color: Color3f) {
        val vecs = vec2Array[4]
        argAABB.getVertices(vecs)
        drawPolygon(vecs, 4, color)
    }

    private val tr = Affine()
    private val oldTrans = Affine()
    private var stroke: Double
    private var oldStroke = 0.0
    private fun saveState(g: GraphicsContext) {
        oldTrans.setToTransform(g.transform)
        oldStroke = g.lineWidth
    }

    private fun restoreState(g: GraphicsContext) {
        g.transform = oldTrans
        g.lineWidth = oldStroke
    }

    private fun transformGraphics(g: GraphicsContext, center: Vec2): Double {
        val e = viewportTransform.extents
        val vc = viewportTransform.center
        val vt = viewportTransform.mat22Representation
        val flip = if (yFlip) -1 else 1
        tr.setToTransform(vt.ex.x.toDouble(), (flip * vt.ex.y).toDouble(), e.x.toDouble(), vt.ey.x.toDouble(), (flip * vt.ey.y).toDouble(),
                e.y.toDouble())
        tr.appendTranslation(-vc.x.toDouble(), -vc.y.toDouble())
        tr.appendTranslation(center.x.toDouble(), center.y.toDouble())
        g.transform = tr
        return vt.ex.x.toDouble()
    }

    override fun drawCircle(center: Vec2, radius: Float, color: Color3f) {
        val g = graphics
        val s = cpool.getColor(color.x, color.y, color.z, 1.0f)
        saveState(g)
        val scaling = transformGraphics(g, center) * radius
        g.lineWidth = stroke / scaling
        g.scale(radius.toDouble(), radius.toDouble())
        g.stroke = s
        g.strokeOval(circle.minX, circle.minX, circle.width,
                circle.height)
        restoreState(g)
    }

    override fun drawCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        val g = graphics
        val s = cpool.getColor(color.x, color.y, color.z, 1f)
        saveState(g)
        val scaling = transformGraphics(g, center) * radius
        g.lineWidth = stroke / scaling
        g.scale(radius.toDouble(), radius.toDouble())
        g.stroke = s
        g.strokeOval(circle.minX, circle.minX, circle.width,
                circle.height)
        g.rotate(MathUtils.atan2(axis.y, axis.x).toDouble())
        g.strokeLine(0.0, 0.0, 1.0, 0.0)
        restoreState(g)
    }

    override fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2,
                                 color: Color3f) {
        val g = graphics
        val f = cpool.getColor(color.x, color.y, color.z, .4f)
        val s = cpool.getColor(color.x, color.y, color.z, 1f)
        saveState(g)
        val scaling = transformGraphics(g, center) * radius
        g.lineWidth = stroke / scaling
        g.scale(radius.toDouble(), radius.toDouble())
        g.fill = f
        g.fillOval(circle.minX, circle.minX, circle.width,
                circle.height)
        g.stroke = s
        g.strokeOval(circle.minX, circle.minX, circle.width,
                circle.height)
        g.rotate(MathUtils.atan2(axis.y, axis.x).toDouble())
        g.strokeLine(0.0, 0.0, 1.0, 0.0)
        restoreState(g)
    }

    private val zero = Vec2()
    private val pcolorA = Color(1.0, 1.0, 1.0, .4)
    override fun drawParticles(centers: Array<Vec2>, radius: Float,
                               colors: Array<ParticleColor>?, count: Int) {
        val g = graphics
        saveState(g)
        val scaling = transformGraphics(g, zero) * radius
        g.lineWidth = stroke / scaling
        for (i in 0 until count) {
            val center = centers[i]
            val color: Color = if (colors == null) {
                pcolorA
            } else {
                val c = colors[i]
                cpool.getColor(c.r * 1f / 127, c.g * 1f / 127,
                        c.b * 1f / 127, c.a * 1f / 127)
            }
            val old = g.transform
            g.translate(center.x.toDouble(), center.y.toDouble())
            g.scale(radius.toDouble(), radius.toDouble())
            g.fill = color
            g.fillOval(circle.minX, circle.minX, circle.width,
                    circle.height)
            g.transform = old
        }
        restoreState(g)
    }

    private val pcolor = Color(1.0, 1.0, 1.0, 1.0)
    override fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float,
                                        colors: Array<ParticleColor>?, count: Int) {
        val g = graphics
        saveState(g)
        val scaling = transformGraphics(g, zero) * radius
        g.lineWidth = stroke / scaling
        for (i in 0 until count) {
            val center = centers[i]
            val color: Color
            // No alpha channel, it slows everything down way too much.
            color = if (colors == null) {
                pcolor
            } else {
                val c = colors[i]
                Color(c.r * 1f / 127.toDouble(), c.g * 1f / 127.toDouble(),
                        c.b * 1f / 127.toDouble(), 1.0)
            }
            val old = g.transform
            g.translate(center.x.toDouble(), center.y.toDouble())
            g.scale(radius.toDouble(), radius.toDouble())
            g.stroke = color
            g.strokeOval(circle.minX, circle.minX, circle.width,
                    circle.height)
            g.transform = old
        }
        restoreState(g)
    }

    private val temp = Vec2()
    override fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int,
                                  color: Color3f) {
        val f = cpool.getColor(color.x, color.y, color.z, .4f)
        val s = cpool.getColor(color.x, color.y, color.z, 1f)
        val g = graphics
        saveState(g)
        val xs = xDoublePool[vertexCount]
        val ys = yDoublePool[vertexCount]
        for (i in 0 until vertexCount) {
            getWorldToScreenToOut(vertices[i], temp)
            xs[i] = temp.x.toDouble()
            ys[i] = temp.y.toDouble()
        }
        g.lineWidth = stroke
        g.fill = f
        g.fillPolygon(xs, ys, vertexCount)
        g.stroke = s
        g.strokePolygon(xs, ys, vertexCount)
        restoreState(g)
    }

    override fun drawPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        val s = cpool.getColor(color.x, color.y, color.z, 1f)
        val g = graphics
        saveState(g)
        val xs = xDoublePool[vertexCount]
        val ys = yDoublePool[vertexCount]
        for (i in 0 until vertexCount) {
            getWorldToScreenToOut(vertices[i], temp)
            xs[i] = temp.x.toDouble()
            ys[i] = temp.y.toDouble()
        }
        g.lineWidth = stroke
        g.stroke = s
        g.strokePolygon(xs, ys, vertexCount)
        restoreState(g)
    }

    override fun drawString(x: Float, y: Float, s: String, color: Color3f) {
        val g = graphics
        val c = cpool.getColor(color.x, color.y, color.z)
        g.fill = c
        g.fillText(s, x.toDouble(), y.toDouble())
    }

    private val graphics: GraphicsContext
        private get() = panel.dbGraphics
    private val temp2 = Vec2()
    override fun drawTransform(xf: Transform) {
        val g = graphics
        getWorldToScreenToOut(xf.p, temp)
        temp2.setZero()
        val axisScale = 0.4f
        var c = cpool.getColor(1f, 0f, 0f)
        g.stroke = c
        temp2.x = xf.p.x + axisScale * xf.q.c
        temp2.y = xf.p.y + axisScale * xf.q.s
        getWorldToScreenToOut(temp2, temp2)
        g.strokeLine(temp.x.toDouble(), temp.y.toDouble(), temp2.x.toDouble(), temp2.y.toDouble())
        c = cpool.getColor(0f, 1f, 0f)
        g.stroke = c
        temp2.x = xf.p.x + -axisScale * xf.q.s
        temp2.y = xf.p.y + axisScale * xf.q.c
        getWorldToScreenToOut(temp2, temp2)
        g.strokeLine(temp.x.toDouble(), temp.y.toDouble(), temp2.x.toDouble(), temp2.y.toDouble())
    }

    companion object {
        var circlePoints = 13
        const val edgeWidth = 0.02f
        private val xDoublePool = DoubleArray()
        private val yDoublePool = DoubleArray()
    }

    init {
        stroke = 1.0
        circle = Rectangle2D(-1.0, -1.0, 2.0, 2.0)
    }
}
