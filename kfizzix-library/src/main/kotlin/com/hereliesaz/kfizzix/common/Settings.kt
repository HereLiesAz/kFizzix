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

    // JBox2D specific settings

    // Uses a lookup table for MathUtils.abs, might be faster on some JVMs.
    var FAST_ABS = true
    // Uses a lookup table or optimized logic for MathUtils.floor.
    var FAST_FLOOR = true
    // Uses a lookup table or optimized logic for MathUtils.ceil.
    var FAST_CEIL = true
    // Uses a lookup table or optimized logic for MathUtils.round.
    var FAST_ROUND = true
    // Uses a fast approximation for atan2. Defaults to false for precision.
    var FAST_ATAN2 = false
    // Uses a fast approximation for power functions.
    var FAST_POW = true
    // Initial size of contact listener stacks.
    var CONTACT_STACK_INIT_SIZE = 10
    // Enable/Disable Sin/Cos Lookup Table. Defaults to false.
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

    // Collision

    /**
     * The maximum number of contact points between two convex shapes.
     * Box2D supports up to 2 points for polygon-polygon (edge-edge) collisions.
     */
    var maxManifoldPoints = 2

    /**
     * The maximum number of vertices on a convex polygon.
     * Changing this requires recompiling the library.
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
     * A small length used as a collision and constraint tolerance. Usually it
     * is chosen to be numerically significant, but visually insignificant.
     * If objects overlap by less than this, the solver considers them "touching" but not "penetrating".
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
     * The radius of the polygon/edge shape skin. This should not be modified.
     * Making this smaller means polygons will have and insufficient for
     * continuous collision. Making it larger may create artifacts for vertex
     * collision.
     *
     * Polygons are actually treated as a core polygon + a radius (skin).
     */
    var polygonRadius = 2.0f * linearSlop

    /**
     * Maximum number of sub-steps per contact in continuous physics simulation.
     * Higher values are more expensive but handle fast moving objects better.
     */
    var maxSubSteps = 8

    // Dynamics

    /**
     * Maximum number of contacts to be handled to solve a TOI (Time of Impact) island.
     */
    var maxTOIContacts = 32

    /**
     * A velocity threshold for elastic collisions. Any collision with a
     * relative linear velocity below this threshold will be treated as
     * inelastic. This prevents "jittering" for objects resting on the floor.
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

    // Sleep

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

    // Particle

    /**
     * A symbolic constant that stands for particle allocation error.
     */
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
     * Friction mixing law. Feel free to customize this.
     * By default, it uses the geometric mean: sqrt(f1 * f2).
     */
    fun mixFriction(friction1: Float, friction2: Float): Float {
        // Return geometric mean.
        return MathUtils.sqrt(friction1 * friction2)
    }

    /**
     * Restitution mixing law. Feel free to customize this.
     * By default, it uses the maximum of the two values.
     */
    fun mixRestitution(restitution1: Float, restitution2: Float): Float {
        // Return max.
        return Math.max(restitution1, restitution2)
    }
}
