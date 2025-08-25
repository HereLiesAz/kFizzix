package com.hereliesaz.kfizzix.androidapp

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.os.Bundle
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.*
import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.particle.ParticleGroupDef
import com.hereliesaz.kfizzix.particle.ParticleSystem
import com.hereliesaz.kfizzix.particle.ParticleSystemDef
import kotlin.random.Random

class MainActivity : AppCompatActivity(), SurfaceHolder.Callback {

    private lateinit var surfaceView: SurfaceView
    private var renderThread: RenderThread? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        surfaceView = findViewById(R.id.surface_view)
        surfaceView.holder.addCallback(this)

        val shakeButton: Button = findViewById(R.id.shake_button)
        shakeButton.setOnClickListener {
            renderThread?.shake()
        }
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
        private lateinit var die: Body

        private var surfaceWidth = 0
        private var surfaceHeight = 0
        private val scale = 20.0f

        private val particlePaint = Paint().apply { color = Color.parseColor("#202020") }
        private val diePaint = Paint().apply { color = Color.DODGERBLUE }
        private val textPaint = Paint().apply {
            color = Color.WHITE
            textSize = 40f
            textAlign = Paint.Align.CENTER
        }

        private val answers = listOf(
            "It is certain.", "It is decidedly so.", "Without a doubt.", "Yes – definitely.", "You may rely on it.",
            "As I see it, yes.", "Most likely.", "Outlook good.", "Yes.", "Signs point to yes.",
            "Reply hazy, try again.", "Ask again later.", "Better not tell you now.", "Cannot predict now.", "Concentrate and ask again.",
            "Don't count on it.", "My reply is no.", "My sources say no.", "Outlook not so good.", "Very doubtful."
        )
        private var currentAnswer: String? = null

        fun setRunning(isRunning: Boolean) {
            running = isRunning
        }

        fun setSurfaceSize(width: Int, height: Int) {
            surfaceWidth = width
            surfaceHeight = height
        }

        fun shake() {
            val force = Vec2(Random.nextFloat() * 1000 - 500, Random.nextFloat() * 1000 - 500)
            die.applyLinearImpulse(force, die.worldCenter)

            currentAnswer = answers.random()
        }

        override fun run() {
            initPhysics()

            while (running) {
                val canvas = surfaceHolder.lockCanvas()
                if (canvas != null) {
                    try {
                        synchronized(surfaceHolder) {
                            world.step(1.0f / 60.0f, 8, 3)
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
                createLiquid()
                createDie()
            }
            diePaint.setShadowLayer(15f, 0f, 0f, Color.CYAN)
        }

        private fun createContainer() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale

            // ground
            val groundBodyDef = BodyDef().apply { type = BodyType.STATIC; position.set(worldWidth / 2, 0f) }
            val groundBody = world.createBody(groundBodyDef)
            val groundBox = PolygonShape().apply { setAsBox(worldWidth / 2, 0.5f) }
            groundBody.createFixture(groundBox, 0.0f)

            // left wall
            val leftWallDef = BodyDef().apply { type = BodyType.STATIC; position.set(0f, worldHeight / 2) }
            val leftWallBody = world.createBody(leftWallDef)
            val leftWallBox = PolygonShape().apply { setAsBox(0.5f, worldHeight / 2) }
            leftWallBody.createFixture(leftWallBox, 0.0f)

            // right wall
            val rightWallDef = BodyDef().apply { type = BodyType.STATIC; position.set(worldWidth, worldHeight / 2) }
            val rightWallBody = world.createBody(rightWallDef)
            val rightWallBox = PolygonShape().apply { setAsBox(0.5f, worldHeight / 2) }
            rightWallBody.createFixture(rightWallBox, 0.0f)
        }

        private fun createLiquid() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale
            val particleSystemDef = ParticleSystemDef().apply {
                density = 1.2f
                gravityScale = 0.4f
                radius = 0.3f
                viscousStrength = 0.9f
            }
            particleSystem = world.createParticleSystem(particleSystemDef)
            val particleGroupDef = ParticleGroupDef().apply {
                shape = PolygonShape().apply { setAsBox(worldWidth / 2 - 2, worldHeight / 2) }
                position.set(worldWidth / 2, worldHeight / 2)
            }
            particleSystem.createParticleGroup(particleGroupDef)
        }

        private fun createDie() {
            val worldWidth = surfaceWidth / scale
            val worldHeight = surfaceHeight / scale
            val dieBodyDef = BodyDef().apply {
                type = BodyType.DYNAMIC
                position.set(worldWidth / 2, worldHeight / 4)
            }
            die = world.createBody(dieBodyDef)
            val dieShape = PolygonShape().apply {
                val vertices = arrayOf(Vec2(0.0f, 1.5f), Vec2(-1.5f, -1.0f), Vec2(1.5f, -1.0f))
                set(vertices, vertices.size)
            }
            val fixtureDef = FixtureDef().apply {
                shape = dieShape
                density = 0.5f
            }
            die.createFixture(fixtureDef)
        }

        private fun render(canvas: Canvas) {
            canvas.drawColor(Color.BLACK)

            if (!::particleSystem.isInitialized) return

            // Render particles
            val positions = particleSystem.particlePositionBuffer
            val radius = particleSystem.radius * scale
            for (i in 0 until particleSystem.particleCount) {
                val pos = positions[i]
                canvas.drawCircle(pos.x * scale, surfaceHeight - pos.y * scale, radius, particlePaint)
            }

            // Render die
            val diePos = die.position
            val dieAngle = die.angle
            canvas.save()
            canvas.translate(diePos.x * scale, surfaceHeight - diePos.y * scale)
            canvas.rotate(-Math.toDegrees(dieAngle.toDouble()).toFloat())

            val fixture = die.fixtureList
            val shape = fixture.shape as PolygonShape
            val vertices = shape.vertices
            val path = Path()
            path.moveTo(vertices[0].x * scale, -vertices[0].y * scale)
            for (i in 1 until vertices.size) {
                path.lineTo(vertices[i].x * scale, -vertices[i].y * scale)
            }
            path.close()
            canvas.drawPath(path, diePaint)

            currentAnswer?.let {
                val textBounds = android.graphics.Rect()
                textPaint.getTextBounds(it, 0, it.length, textBounds)
                canvas.drawText(it, 0f, textBounds.height() / 2f, textPaint)
            }

            canvas.restore()
        }
    }
}
