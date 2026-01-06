package com.hereliesaz.kfizzix.showcase

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import com.hereliesaz.kfizzix.callbacks.DebugDraw
import com.hereliesaz.kfizzix.common.Color3f
import com.hereliesaz.kfizzix.common.Mat22
import com.hereliesaz.kfizzix.common.OBBViewportTransform
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.particle.ParticleColor

class AndroidDebugDraw : DebugDraw(OBBViewportTransform()) {

    private val paint = Paint().apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }
    private val filledPaint = Paint().apply {
        style = Paint.Style.FILL
    }
    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = 30f
        isAntiAlias = true
    }

    var canvas: Canvas? = null
    var height: Int = 0

    init {
        val tr = viewportTransform as OBBViewportTransform
        tr.isYFlip = true
        // tr.isCameraEnabled = true // Not in IViewportTransform
    }

    // DebugDraw.setCamera is NOT open, so we cannot override it.
    // Instead, we just use the base implementation which delegates to viewportTransform

    // Helper to access scale since OBBViewportTransform stores it in the matrix R
    private val scale: Float
        get() {
             val tr = viewportTransform as OBBViewportTransform
             return tr.box.R.ex.x // Assuming uniform scale
        }

    override fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f) {
        val p = getWorldToScreen(argPoint)
        filledPaint.color = toAndroidColor(argColor)
        canvas?.drawCircle(p.x, p.y, argRadiusOnScreen, filledPaint)
    }

    override fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        val path = Path()
        val v0 = getWorldToScreen(vertices[0])
        path.moveTo(v0.x, v0.y)
        for (i in 1 until vertexCount) {
            val v = getWorldToScreen(vertices[i])
            path.lineTo(v.x, v.y)
        }
        path.close()

        filledPaint.color = toAndroidColor(color, 128)
        canvas?.drawPath(path, filledPaint)

        paint.color = toAndroidColor(color)
        canvas?.drawPath(path, paint)
    }

    override fun drawCircle(center: Vec2, radius: Float, color: Color3f) {
        val p = getWorldToScreen(center)
        val r = radius * scale
        paint.color = toAndroidColor(color)
        canvas?.drawCircle(p.x, p.y, r, paint)
    }

    override fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        val p = getWorldToScreen(center)
        val r = radius * scale

        filledPaint.color = toAndroidColor(color, 128)
        canvas?.drawCircle(p.x, p.y, r, filledPaint)

        paint.color = toAndroidColor(color)
        canvas?.drawCircle(p.x, p.y, r, paint)

        val sa = getWorldToScreen(center)
        val pAxis = Vec2(center.x + axis.x * radius, center.y + axis.y * radius)
        val ea = getWorldToScreen(pAxis)
        canvas?.drawLine(sa.x, sa.y, ea.x, ea.y, paint)
    }

    override fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f) {
        val sp1 = getWorldToScreen(p1)
        val sp2 = getWorldToScreen(p2)
        paint.color = toAndroidColor(color)
        canvas?.drawLine(sp1.x, sp1.y, sp2.x, sp2.y, paint)
    }

    override fun drawTransform(xf: Transform) {
        val k_axisScale = 0.4f
        val p1 = xf.p

        val p2 = Vec2()
        p2.setZero()
        p2.x = p1.x + k_axisScale * xf.q.c
        p2.y = p1.y + k_axisScale * xf.q.s
        drawSegment(p1, p2, Color3f(1f, 0f, 0f))

        p2.x = p1.x + k_axisScale * -xf.q.s
        p2.y = p1.y + k_axisScale * xf.q.c
        drawSegment(p1, p2, Color3f(0f, 1f, 0f))
    }

    override fun drawString(x: Float, y: Float, s: String, color: Color3f) {
        // debug draw string uses screen coordinates, but we need to verify coordinate system
        textPaint.color = toAndroidColor(color)
        canvas?.drawText(s, x, y, textPaint)
    }

    override fun drawParticles(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int) {
         val r = radius * scale
         for(i in 0 until count) {
             val p = getWorldToScreen(centers[i])
             if(colors != null) {
                 val c = colors[i]
                 filledPaint.setARGB(c.a.toInt(), c.r.toInt(), c.g.toInt(), c.b.toInt())
             } else {
                 filledPaint.color = Color.CYAN
             }
             canvas?.drawCircle(p.x, p.y, r, filledPaint)
         }
    }

    override fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int) {
         val r = radius * scale
         for(i in 0 until count) {
             val p = getWorldToScreen(centers[i])
             paint.color = Color.CYAN
             canvas?.drawCircle(p.x, p.y, r, paint)
         }
    }

    private fun toAndroidColor(color: Color3f, alpha: Int = 255): Int {
        return Color.argb(alpha, (color.x * 255).toInt(), (color.y * 255).toInt(), (color.z * 255).toInt())
    }
}
