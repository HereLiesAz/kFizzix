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

import java.io.Serializable

/**
 * This describes the motion of a body/shape for TOI (Time of Impact) computation.
 * Shapes are defined with respect to the body origin, which may not coincide with the
 * center of mass. However, to support dynamics we must interpolate the center
 * of mass position.
 *
 * @author Daniel Murphy
 */
class Sweep : Serializable {

    /**
     * The local center of mass position.
     * This is the center of mass relative to the body origin.
     */
    @JvmField
    val localCenter: Vec2 = Vec2()

    /**
     * The world position of the center of mass at the beginning of the sweep.
     */
    @JvmField
    val c0: Vec2 = Vec2()

    /**
     * The world position of the center of mass at the end of the sweep.
     */
    @JvmField
    val c: Vec2 = Vec2()

    /**
     * The world angle at the beginning of the sweep.
     */
    @JvmField
    var a0: Float = 0f

    /**
     * The world angle at the end of the sweep.
     */
    @JvmField
    var a: Float = 0f

    /**
     * Fraction of the current time step in the range [0,1].
     * `c0` and `a0` are the positions at `alpha0`.
     */
    @JvmField
    var alpha0: Float = 0f

    override fun toString(): String {
        var s = "Sweep:\nlocalCenter: $localCenter\n"
        s += "c0: $c0, c: $c\n"
        s += "a0: $a0, a: $a\n"
        s += "alpha0: $alpha0"
        return s
    }

    /**
     * Normalizes the angles to be in the range [0, 2*PI).
     */
    fun normalize() {
        val d = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI)
        a0 -= d
        a -= d
    }

    /**
     * Sets this sweep to be a copy of another sweep.
     *
     * @param other the sweep to copy from
     * @return this sweep for chaining
     */
    fun set(other: Sweep): Sweep {
        localCenter.set(other.localCenter)
        c0.set(other.c0)
        c.set(other.c)
        a0 = other.a0
        a = other.a
        alpha0 = other.alpha0
        return this
    }

    /**
     * Get the interpolated transform at a specific time.
     * This is used to get the transform of a body at a specific point in time within the sweep.
     *
     * @param xf the transform to be populated with the result. Must not be null.
     * @param beta the normalized time in [0,1]. `0` corresponds to the start of the sweep (`c0`, `a0`) and `1` to the end (`c`, `a`).
     *
     * @see [Box2D b2Math.h](https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L688-L697)
     */
    fun getTransform(xf: Transform, beta: Float) {
        xf.p.x = (1.0f - beta) * c0.x + beta * c.x
        xf.p.y = (1.0f - beta) * c0.y + beta * c.y
        val angle = (1.0f - beta) * a0 + beta * a
        xf.q.set(angle)
        // Shift to origin
        val q = xf.q
        xf.p.x -= q.c * localCenter.x - q.s * localCenter.y
        xf.p.y -= q.s * localCenter.x + q.c * localCenter.y
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     * This is used to update the sweep after a time step has been processed.
     *
     * @param alpha the new initial time.
     *
     * @see [Box2D b2Math.h](https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L699-L706)
     */
    fun advance(alpha: Float) {
        assert(alpha0 < 1.0f)
        val beta = (alpha - alpha0) / (1.0f - alpha0)
        c0.x += beta * (c.x - c0.x)
        c0.y += beta * (c.y - c0.y)
        a0 += beta * (a - a0)
        alpha0 = alpha
    }

    companion object {
        private const val serialVersionUID = 1L
    }
}
