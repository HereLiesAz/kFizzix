package com.hereliesaz.kfizzix.showcase

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.util.Log
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.SurfaceView
import com.hereliesaz.kfizzix.common.Vec2

class TestbedView(context: Context) : SurfaceView(context), SurfaceHolder.Callback {

    private var renderThread: RenderThread? = null
    var currentTest: Test? = null
        set(value) {
            field = value
            value?.initialize()
        }

    init {
        holder.addCallback(this)
    }

    override fun surfaceCreated(holder: SurfaceHolder) {
        renderThread = RenderThread(holder, this)
        renderThread?.running = true
        renderThread?.start()
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        currentTest?.debugDraw?.height = height
        // Reset view?
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        var retry = true
        renderThread?.running = false
        while (retry) {
            try {
                renderThread?.join()
                retry = false
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        currentTest?.onTouch(event)
        return true
    }

    class RenderThread(private val surfaceHolder: SurfaceHolder, private val view: TestbedView) : Thread() {
        var running = false

        override fun run() {
            while (running) {
                val canvas: Canvas? = surfaceHolder.lockCanvas()
                if (canvas != null) {
                    try {
                        synchronized(surfaceHolder) {
                            view.updateAndDraw(canvas)
                        }
                    } catch (e: Exception) {
                        e.printStackTrace()
                    } finally {
                        surfaceHolder.unlockCanvasAndPost(canvas)
                    }
                }
            }
        }
    }

    fun updateAndDraw(canvas: Canvas) {
        canvas.drawColor(Color.BLACK)
        currentTest?.let { test ->
            test.debugDraw.canvas = canvas
            test.step()
            test.render()
        }
    }
}
