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
package com.hereliesaz.kfizzix.common

/**
 * Global tuning constants based on MKS (Meters-Kilograms-Seconds) units.
 *
 * This file controls the "physics laws" of the engine. Adjusting these values
 * can significantly change the stability and behavior of the simulation.
 *
 * **Note on Units:**
 * Box2D/kfizzix is tuned for meters, kilograms, and seconds.
 * *   Objects between 0.1 and 10 meters work best.
 * *   Do not use pixels as units! (e.g., a 100x100 pixel box is 100 meters tall -> a skyscraper).
 * *   Scale your graphics to match the physics world, not the other way around.
 *
 * @author Daniel Murphy
 */
object Settings {
    /**
     * A "close to zero" float epsilon value.
     * Used for floating point comparisons to handle precision errors.
     */
    const val EPSILON = 1.1920928955078125E-7f

    /** Pi. (3.14159...) */
    const val PI = Math.PI.toFloat()

    // --- Optimization Settings ---

    /** Use a faster but less accurate absolute value function? */
    var FAST_ABS = true
    /** Use a faster but less accurate floor function? */
    var FAST_FLOOR = true
    /** Use a faster but less accurate ceil function? */
    var FAST_CEIL = true
    /** Use a faster but less accurate round function? */
    var FAST_ROUND = true
    /** Use a faster approximation for atan2? (Significantly affects accuracy) */
    var FAST_ATAN2 = false
    /** Use a faster approximation for power functions? */
    var FAST_POW = true
    /** Initial size of the contact stack. */
    var CONTACT_STACK_INIT_SIZE = 10
    /** Enable Sin/Cos Lookup Table? (Faster trig, less memory efficient) */
    var SINCOS_LUT_ENABLED = false

    /**
     * Precision for the Sin/Cos Lookup Table.
     * Smaller precision = larger table.
     *
     * Good lerp precision values:
     * * .0092
     * * .002807
     * * .0001098
     */
    const val SINCOS_LUT_PRECISION = .00011f
    // Length of the sin/cos lookup table based on precision.
    const val SINCOS_LUT_LENGTH = (Math.PI * 2 / SINCOS_LUT_PRECISION).toInt()

    /**
     * Use linear interpolation for the Sin/Cos LUT.
     * More expensive than raw lookup, but much more accurate for large precision steps.
     */
    var SINCOS_LUT_LERP = false

    // --- Collision Settings ---

    /**
     * The maximum number of contact points between two convex shapes.
     * Box2D supports up to 2 contact points for 2D convex polygons.
     */
    var maxManifoldPoints = 2

    /**
     * The maximum number of vertices on a convex polygon.
     * You cannot create a polygon with more than this many vertices.
     * If you need more, use a ChainShape or multiple PolygonShapes.
     */
    var maxPolygonVertices = 8

    /**
     * "Fattening" factor for AABBs (Axis Aligned Bounding Boxes) in the broad-phase.
     * We add this buffer (in meters) to the AABB so that objects can move slightly
     * without triggering an expensive tree re-insertion.
     */
    var aabbExtension = 0.1f

    /**
     * Multiplier for AABB extension based on velocity.
     * Used to predict where the object will be next frame.
     */
    var aabbMultiplier = 2.0f

    /**
     * **Linear Slop**
     *
     * A small length (meters) used as a collision and constraint tolerance.
     * Usually it is chosen to be numerically significant, but visually insignificant.
     *
     * *Why do we need this?*
     * Ideally, objects would touch exactly at the surface (distance = 0).
     * However, numerical errors cause objects to slightly penetrate or float.
     * The "slop" allows shapes to penetrate slightly (by this amount) without
     * the physics engine trying to push them apart violently. This improves stability.
     */
    var linearSlop = 0.005f

    /**
     * **Angular Slop**
     *
     * Similar to Linear Slop, but for angles (radians).
     * Allows small angular errors in joints without correction.
     */
    var angularSlop = 2.0f / 180.0f * PI

    /**
     * The "Skin" of a polygon.
     * Polygons are actually slightly smaller than defined, with a "skin" of this radius
     * added around them. This creates rounded corners which prevents objects from
     * getting snagged on sharp edges.
     */
    var polygonRadius = 2.0f * linearSlop

    /**
     * Maximum number of sub-steps per contact in continuous physics simulation (TOI).
     */
    var maxSubSteps = 8

    // --- Dynamics Settings ---

    /**
     * Maximum number of contacts to be handled to solve a TOI (Time of Impact) island.
     */
    var maxTOIContacts = 32

    /**
     * Velocity threshold for elastic collisions (m/s).
     * Any collision with a relative linear velocity below this threshold will be
     * treated as inelastic (bounciness = 0).
     *
     * *Why?*
     * To prevent "jitter" when objects are resting on the ground. Without this,
     * a ball would bounce infinitely with tiny micro-bounces.
     */
    var velocityThreshold = 1.0f

    /**
     * The maximum linear position correction used when solving constraints.
     * This helps to prevent overshoot when pushing overlapping objects apart.
     */
    var maxLinearCorrection = 0.2f

    /**
     * The maximum angular position correction used when solving constraints.
     */
    var maxAngularCorrection = 8.0f / 180.0f * PI

    /**
     * The maximum linear velocity of a body (m/s).
     * This limit is very large and is used to prevent numerical problems (instability)
     * if a body is blasted with huge force.
     */
    var maxTranslation = 2.0f

    // Squared value for optimization.
    var maxTranslationSquared = maxTranslation * maxTranslation

    /**
     * The maximum angular velocity of a body (rad/s).
     */
    var maxRotation = 0.5f * PI

    // Squared value for optimization.
    var maxRotationSquared = maxRotation * maxRotation

    /**
     * **Baumgarte Stabilization Factor**
     *
     * This scale factor controls how fast overlap is resolved.
     * Range: [0, 1].
     *
     * *   0.0: No correction (objects stay overlapping).
     * *   1.0: Remove all overlap in a single time step (can cause violent instability/overshoot).
     * *   0.2: Remove 20% of the overlap per step. This is a safe default (smooth, sponge-like correction).
     */
    var baumgarte = 0.2f

    // Baumgarte factor for Time of Impact (TOI) solving.
    var toiBaugarte = 0.75f

    // --- Sleep Settings ---

    /**
     * The time (seconds) that a body must be still before it will go to sleep.
     * Sleeping bodies are removed from simulation to save CPU.
     */
    var timeToSleep = 0.5f

    /**
     * A body cannot sleep if its linear velocity is above this tolerance (m/s).
     */
    var linearSleepTolerance = 0.01f

    /**
     * A body cannot sleep if its angular velocity is above this tolerance (rad/s).
     */
    var angularSleepTolerance = 2.0f / 180.0f * PI

    // --- Particle Settings (LiquidFun) ---

    /** A symbolic constant that stands for particle allocation error. */
    const val invalidParticleIndex = -1

    /** The standard distance between particles, divided by the particle radius. */
    const val particleStride = 0.75f

    /** The minimum particle weight that produces pressure. */
    const val minParticleWeight = 1.0f

    /** The upper limit for particle weight used in pressure calculation. */
    const val maxParticleWeight = 5.0f

    /** The maximum distance between particles in a triad, divided by the particle radius. */
    const val maxTriadDistance = 2

    // Squared value.
    const val maxTriadDistanceSquared = maxTriadDistance * maxTriadDistance

    /** The initial size of particle data buffers. */
    const val minParticleBufferCapacity = 256

    /**
     * Friction mixing law.
     * Determines how friction combines when two fixtures touch.
     * Default: sqrt(f1 * f2).
     */
    fun mixFriction(friction1: Float, friction2: Float): Float {
        // Return geometric mean.
        return MathUtils.sqrt(friction1 * friction2)
    }

    /**
     * Restitution (Bounciness) mixing law.
     * Determines how bounciness combines.
     * Default: max(r1, r2). If one object is bouncy, the collision is bouncy.
     */
    fun mixRestitution(restitution1: Float, restitution2: Float): Float {
        // Return max.
        return Math.max(restitution1, restitution2)
    }
}
