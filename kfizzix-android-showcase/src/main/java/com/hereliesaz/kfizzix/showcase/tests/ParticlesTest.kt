package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.particle.ParticleColor
import com.hereliesaz.kfizzix.particle.ParticleGroupDef
import com.hereliesaz.kfizzix.particle.ParticleType
import com.hereliesaz.kfizzix.showcase.Test

class ParticlesTest : Test() {

    override fun initialize() {
        super.initialize()

        // Ground container
        val groundDef = BodyDef()
        val ground = world.createBody(groundDef)
        val shape = PolygonShape()
        shape.setAsBox(40f, 1f, Vec2(0f, -5f), 0f)
        ground.createFixture(shape, 0f)
        shape.setAsBox(1f, 20f, Vec2(-40f, 10f), 0f)
        ground.createFixture(shape, 0f)
        shape.setAsBox(1f, 20f, Vec2(40f, 10f), 0f)
        ground.createFixture(shape, 0f)

        // There is no ParticleSystemDef or createParticleSystem(Def) in the current API.
        // World creates a default one. We configure it directly.
        val system = world.particleSystem
        system.particleRadius = 0.5f

        // 1. Water
        run {
            val pgd = ParticleGroupDef()
            pgd.shape = PolygonShape().apply { setAsBox(5f, 5f) }
            pgd.position.set(-30f, 10f)
            pgd.flags = ParticleType.waterParticle
            pgd.color = ParticleColor(0.toByte(), 0.toByte(), 255.toByte(), 255.toByte())
            system.createParticleGroup(pgd)
        }

        // 2. Elastic
        run {
            val pgd = ParticleGroupDef()
            pgd.shape = PolygonShape().apply { setAsBox(5f, 5f) }
            pgd.position.set(-15f, 10f)
            pgd.flags = ParticleType.elasticParticle
            pgd.color = ParticleColor(0.toByte(), 255.toByte(), 0.toByte(), 255.toByte())
            system.createParticleGroup(pgd)
        }

        // 3. Viscous
        run {
            val pgd = ParticleGroupDef()
            pgd.shape = PolygonShape().apply { setAsBox(5f, 5f) }
            pgd.position.set(0f, 10f)
            pgd.flags = ParticleType.viscousParticle
            pgd.color = ParticleColor(128.toByte(), 128.toByte(), 128.toByte(), 255.toByte())
            system.createParticleGroup(pgd)
        }

        // 4. Powder
        run {
            val pgd = ParticleGroupDef()
            pgd.shape = PolygonShape().apply { setAsBox(5f, 5f) }
            pgd.position.set(15f, 10f)
            pgd.flags = ParticleType.powderParticle
            pgd.color = ParticleColor(200.toByte(), 200.toByte(), 200.toByte(), 255.toByte())
            system.createParticleGroup(pgd)
        }

        // 5. Tensile
        run {
            val pgd = ParticleGroupDef()
            pgd.shape = PolygonShape().apply { setAsBox(5f, 5f) }
            pgd.position.set(30f, 10f)
            pgd.flags = ParticleType.tensileParticle
            pgd.color = ParticleColor(255.toByte(), 0.toByte(), 0.toByte(), 255.toByte())
            system.createParticleGroup(pgd)
        }
    }

    override fun getTestName(): String {
        return "Particles"
    }
}
