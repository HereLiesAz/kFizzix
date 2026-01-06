package com.hereliesaz.kfizzix.showcase.tests

import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.dynamics.BodyDef
import com.hereliesaz.kfizzix.dynamics.BodyType
import com.hereliesaz.kfizzix.dynamics.joints.*
import com.hereliesaz.kfizzix.showcase.Test

class JointsTest : Test() {

    override fun initialize() {
        super.initialize()

        val ground = world.createBody(BodyDef())

        // 1. Revolute Joint
        run {
            val bodyDef = BodyDef()
            bodyDef.type = BodyType.DYNAMIC
            bodyDef.position.set(-10f, 10f)
            val body = world.createBody(bodyDef)
            body.createFixture(PolygonShape().apply { setAsBox(0.5f, 2f) }, 10f)

            val rjd = RevoluteJointDef()
            rjd.initialize(ground, body, Vec2(-10f, 12f))
            world.createJoint(rjd)
        }

        // 2. Prismatic Joint
        run {
            val bodyDef = BodyDef()
            bodyDef.type = BodyType.DYNAMIC
            bodyDef.position.set(-5f, 10f)
            val body = world.createBody(bodyDef)
            body.createFixture(PolygonShape().apply { setAsBox(0.5f, 2f) }, 10f)

            val pjd = PrismaticJointDef()
            pjd.initialize(ground, body, Vec2(-5f, 10f), Vec2(0f, 1f))
            pjd.lowerTranslation = -5f
            pjd.upperTranslation = 5f
            pjd.enableLimit = true
            world.createJoint(pjd)
        }

        // 3. Distance Joint
        run {
            val b1Def = BodyDef().apply { type = BodyType.DYNAMIC; position.set(0f, 10f) }
            val b1 = world.createBody(b1Def)
            b1.createFixture(PolygonShape().apply { setAsBox(0.5f, 0.5f) }, 10f)

            val b2Def = BodyDef().apply { type = BodyType.DYNAMIC; position.set(0f, 5f) }
            val b2 = world.createBody(b2Def)
            b2.createFixture(PolygonShape().apply { setAsBox(0.5f, 0.5f) }, 10f)

            val djd = DistanceJointDef()
            djd.initialize(b1, b2, b1.position, b2.position)
            djd.frequencyHz = 2f
            djd.dampingRatio = 0.5f
            world.createJoint(djd)
        }

        // 4. Pulley Joint
        run {
            val y = 5f
            val b1Def = BodyDef().apply { type = BodyType.DYNAMIC; position.set(5f, y) }
            val b1 = world.createBody(b1Def)
            b1.createFixture(PolygonShape().apply { setAsBox(0.5f, 0.5f) }, 10f)

            val b2Def = BodyDef().apply { type = BodyType.DYNAMIC; position.set(10f, y) }
            val b2 = world.createBody(b2Def)
            b2.createFixture(PolygonShape().apply { setAsBox(0.5f, 0.5f) }, 10f)

            val pulleyDef = PulleyJointDef()
            val anchor1 = Vec2(5f, y)
            val anchor2 = Vec2(10f, y)
            val groundAnchor1 = Vec2(5f, y + 5f)
            val groundAnchor2 = Vec2(10f, y + 5f)
            pulleyDef.initialize(b1, b2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1f)
            world.createJoint(pulleyDef)
        }

        // 5. Gear Joint (needs two other joints)
        run {
             // Setup revolute and prismatic
             val x = 15f
             val b1 = world.createBody(BodyDef().apply { type = BodyType.DYNAMIC; position.set(x, 10f) })
             b1.createFixture(CircleShape().apply { radius = 1f }, 5f)
             val jd1 = RevoluteJointDef()
             jd1.initialize(ground, b1, b1.position)
             val joint1 = world.createJoint(jd1)

             val b2 = world.createBody(BodyDef().apply { type = BodyType.DYNAMIC; position.set(x, 10f) })
             b2.createFixture(PolygonShape().apply { setAsBox(0.5f, 5f) }, 5f)
             val jd2 = PrismaticJointDef()
             jd2.initialize(ground, b2, b2.position, Vec2(0f, 1f))
             val joint2 = world.createJoint(jd2)

             val gearDef = GearJointDef()
             gearDef.bodyA = b1
             gearDef.bodyB = b2
             gearDef.joint1 = joint1
             gearDef.joint2 = joint2
             gearDef.ratio = 1f
             world.createJoint(gearDef)
        }

        // 6. Wheel Joint
        run {
            val bDef = BodyDef().apply { type = BodyType.DYNAMIC; position.set(20f, 10f) }
            val body = world.createBody(bDef)
            body.createFixture(CircleShape().apply { radius = 1f }, 1f)

            val wjd = WheelJointDef()
            wjd.initialize(ground, body, body.position, Vec2(0f, 1f))
            wjd.motorSpeed = 10f
            wjd.maxMotorTorque = 1000f
            wjd.enableMotor = true
            world.createJoint(wjd)
        }

        // 7. Weld Joint
        run {
            val bDef = BodyDef().apply { type = BodyType.DYNAMIC; position.set(25f, 10f) }
            val body = world.createBody(bDef)
            body.createFixture(PolygonShape().apply { setAsBox(1f, 1f) }, 1f)

            val wjd = WeldJointDef()
            wjd.initialize(ground, body, Vec2(25f, 11f))
            world.createJoint(wjd)
        }

        // 8. Friction Joint
        run {
            val bDef = BodyDef().apply { type = BodyType.DYNAMIC; position.set(30f, 10f) }
            val body = world.createBody(bDef)
            body.createFixture(PolygonShape().apply { setAsBox(1f, 1f) }, 1f)

            val fjd = FrictionJointDef()
            fjd.initialize(ground, body, body.position)
            fjd.maxForce = 10f
            world.createJoint(fjd)
        }

        // 9. Motor Joint
        run {
             val bDef = BodyDef().apply { type = BodyType.DYNAMIC; position.set(35f, 10f) }
             val body = world.createBody(bDef)
             body.createFixture(PolygonShape().apply { setAsBox(1f, 1f) }, 1f)

             val mjd = MotorJointDef()
             mjd.initialize(ground, body)
             mjd.maxForce = 1000f
             mjd.maxTorque = 1000f
             world.createJoint(mjd)
        }

        // 10. Rope Joint
        run {
            val bDef = BodyDef().apply { type = BodyType.DYNAMIC; position.set(40f, 10f) }
            val body = world.createBody(bDef)
            body.createFixture(PolygonShape().apply { setAsBox(0.5f, 0.5f) }, 1f)

            val rjd = RopeJointDef()
            rjd.localAnchorA.set(40f, 15f)
            rjd.localAnchorB.setZero()
            rjd.bodyA = ground
            rjd.bodyB = body
            rjd.maxLength = 6f
            world.createJoint(rjd)
        }
    }

    override fun getTestName(): String {
        return "Joints"
    }
}
