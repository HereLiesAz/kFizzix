package com.hereliesaz.kfizzix.dambreaker

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.os.Bundle
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import com.hereliesaz.kfizzix.callbacks.QueryCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.*
import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.particle.ParticleGroupDef
import com.hereliesaz.kfizzix.particle.ParticleSystem
import com.hereliesaz.kfizzix.particle.ParticleSystemDef

class MainActivity : AppCompatActivity(), SurfaceHolder.Callback, View.OnTouchListener {

    private lateinit var surfaceView: SurfaceView
    private var renderThread: RenderThread? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        surfaceView = findViewById(R.id.surface_view)
        surfaceView.holder.addCallback(this)
        surfaceView.setOnTouchListener(this)
    }

    override fun onTouch(v: View?, event: MotionEvent?): Boolean {
        if (event?.action == MotionEvent.ACTION_DOWN) {
            val worldX = event.x / renderThread!!.scale
            val worldY = (surfaceView.height - event.y) / renderThread!!.scale
            renderThread?.destroyBlockAt(worldX, worldY)
            return true
        }
        return false
    }

    override fun surfaceCreated(holder: SurfaceHolder) {
        renderThread = RenderThread(holder)
        renderThread?.setRunning(true)
        renderThread?.start()
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        renderThread?.setSurfaceSize(width, height)
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        var retry = true
        renderThread?.setRunning(false)
        while (retry) {
            try {
                renderThread?.join()
                retry = false
            } catch (e: InterruptedException) {
                // try again
            }
        }
    }

    inner class RenderThread(private val surfaceHolder: SurfaceHolder) : Thread() {

        @Volatile
        private var running = false
        private lateinit var world: World
        private lateinit var particleSystem: ParticleSystem
        private val damBlocks = mutableListOf<Body>()
        private val bodiesToDestroy = mutableListOf<Body>()

        var surfaceWidth = 0
        var surfaceHeight = 0
        val scale = 20.0f

        private val particlePaint = Paint().apply { color = Color.CYAN }
        private val blockPaint = Paint().apply { color = Color.DKGRAY }

        fun setRunning(isRunning: Boolean) {
            running = isRunning
        }

        fun setSurfaceSize(width: Int, height: Int) {
            surfaceWidth = width
            surfaceHeight = height
        }

        fun destroyBlockAt(worldX: Float, worldY: Float) {
            val aabb = AABB()
            val d = 0.001f
            aabb.lowerBound.set(worldX - d, worldY - d)
            aabb.upperBound.set(worldX + d, worldY + d)

            world.queryAABB(object : QueryCallback {
                override fun reportFixture(fixture: Fixture): Boolean {
                    val body = fixture.body
                    if (body.type == BodyType.DYNAMIC) {
                        if (!bodiesToDestroy.contains(body)) {
                            bodiesToDestroy.add(body)
                        }
                    }
                    return true
                }
            }, aabb)
        }

        override fun run() {
            initPhysics()

            while (running) {
                val canvas = surfaceHolder.lockCanvas()
                if (canvas != null) {
                    try {
                        synchronized(surfaceHolder) {
                            world.step(1.0f / 60.0f, 8, 3)

                            for (body in bodiesToDestroy) {
                                world.destroyBody(body)
                                damBlocks.remove(body)
                            }
                            bodiesToDestroy.clear()

                            render(canvas)
                        }
                    } finally {
                        surfaceHolder.unlockCanvasAndPost(canvas)
                    }
                }
            }
        }

        private fun initPhysics() {
            world = World(Vec2(0.0f, -10.0f))
            if (surfaceWidth > 0 && surfaceHeight > 0) {
                createContainer()
                createWater()
                createDam()
            }
        }

        private fun createContainer() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale

            val groundBodyDef = BodyDef().apply { type = BodyType.STATIC; position.set(worldWidth / 2, 0f) }
            val groundBody = world.createBody(groundBodyDef)
            val groundBox = PolygonShape().apply { setAsBox(worldWidth / 2, 0.5f) }
            groundBody.createFixture(groundBox, 0.0f)

            val leftWallDef = BodyDef().apply { type = BodyType.STATIC; position.set(0f, worldHeight / 2) }
            val leftWallBody = world.createBody(leftWallDef)
            val leftWallBox = PolygonShape().apply { setAsBox(0.5f, worldHeight / 2) }
            leftWallBody.createFixture(leftWallBox, 0.0f)

            val rightWallDef = BodyDef().apply { type = BodyType.STATIC; position.set(worldWidth, worldHeight / 2) }
            val rightWallBody = world.createBody(rightWallDef)
            val rightWallBox = PolygonShape().apply { setAsBox(0.5f, worldHeight / 2) }
            rightWallBody.createFixture(rightWallBox, 0.0f)
        }

        private fun createWater() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale
            val particleSystemDef = ParticleSystemDef().apply {
                density = 1.2f
                gravityScale = 0.4f
                radius = 0.3f
            }
            particleSystem = world.createParticleSystem(particleSystemDef)
            val particleGroupDef = ParticleGroupDef().apply {
                shape = PolygonShape().apply { setAsBox(worldWidth / 3, worldHeight * 0.8f) }
                position.set(worldWidth * 0.3f, worldHeight * 0.8f)
            }
            particleSystem.createParticleGroup(particleGroupDef)
        }

        private fun createDam() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale
            val boxWidth = 0.5f
            val boxHeight = 0.5f
            val numRows = 10
            val numCols = 3

            for (i in 0 until numRows) {
                for (j in 0 until numCols) {
                    val blockDef = BodyDef().apply {
                        type = BodyType.DYNAMIC
                        position.set(worldWidth * 0.6f + j * boxWidth * 2, i * boxHeight * 2 + boxHeight)
                    }
                    val blockBody = world.createBody(blockDef)
                    val blockShape = PolygonShape().apply { setAsBox(boxWidth, boxHeight) }
                    blockBody.createFixture(blockShape, 5.0f)
                    damBlocks.add(blockBody)
                }
            }
        }

        private fun render(canvas: Canvas) {
            canvas.drawColor(Color.WHITE)

            if (!::particleSystem.isInitialized) return

            // Render particles
            val positions = particleSystem.particlePositionBuffer
            val radius = particleSystem.radius * scale
            for (i in 0 until particleSystem.particleCount) {
                val pos = positions[i]
                canvas.drawCircle(pos.x * scale, surfaceHeight - pos.y * scale, radius, particlePaint)
            }

            // Render dam blocks
            for (block in damBlocks) {
                val pos = block.position
                val angle = block.angle
                canvas.save()
                canvas.translate(pos.x * scale, surfaceHeight - pos.y * scale)
                canvas.rotate(-Math.toDegrees(angle.toDouble()).toFloat())
                val shape = block.fixtureList.shape as PolygonShape
                val vertices = shape.vertices
                val path = android.graphics.Path()
                path.moveTo(vertices[0].x * scale, -vertices[0].y * scale)
                for (k in 1 until vertices.size) {
                    path.lineTo(vertices[k].x * scale, -vertices[k].y * scale)
                }
                path.close()
                canvas.drawPath(path, blockPaint)
                canvas.restore()
            }
        }
    }
}
