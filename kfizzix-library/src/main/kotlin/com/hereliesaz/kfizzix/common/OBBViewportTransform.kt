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
 * Orientated Bounding Box Viewport Transform
 *
 * @author Daniel Murphy
 */
class OBBViewportTransform : IViewportTransform {
    val box = OBBViewportTransform.OBB()
    override var isYFlip = false
    private val yFlipMat = Mat22(1f, 0f, 0f, -1f)
    private val yFlipMatInv = yFlipMat.invert()

    init {
        box.R.setIdentity()
    }

    override fun setCamera(x: Float, y: Float, scale: Float) {
        box.center.set(x, y)
        Mat22.createScaleTransform(scale, box.R)
    }

    override val extents: Vec2
        get() = box.extents

    override val mat22Representation: Mat22
        get() = box.R

    override fun setExtents(extents: Vec2) {
        box.extents.set(extents)
    }

    override fun setExtents(halfWidth: Float, halfHeight: Float) {
        box.extents.set(halfWidth, halfHeight)
    }

    override var center: Vec2
        get() = box.center
        set(value) {
            box.center.set(value)
        }

    override fun setCenter(x: Float, y: Float) {
        box.center.set(x, y)
    }

    override fun getWorldVectorToScreen(world: Vec2, screen: Vec2) {
        box.R.mulToOut(world, screen)
        if (isYFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
    }

    override fun getScreenVectorToWorld(screen: Vec2, world: Vec2) {
        if (isYFlip) {
            yFlipMatInv.mulToOut(screen, world)
        } else {
            world.set(screen)
        }
        box.R.solveToOut(world, world)
    }

    override fun getWorldToScreen(world: Vec2, screen: Vec2) {
        screen.x = world.x - box.center.x
        screen.y = world.y - box.center.y
        box.R.mulToOut(screen, screen)
        if (isYFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
        screen.x += box.extents.x
        screen.y += box.extents.y
    }

    override fun getScreenToWorld(screen: Vec2, world: Vec2) {
        world.x = screen.x - box.extents.x
        world.y = screen.y - box.extents.y
        if (isYFlip) {
            yFlipMatInv.mulToOut(world, world)
        }
        box.R.solveToOut(world, world)
        world.x += box.center.x
        world.y += box.center.y
    }

    override fun mulByTransform(transform: Mat22) {
        box.R.mulLocal(transform)
    }

    class OBB {
        val R = Mat22()
        val center = Vec2()
        val extents = Vec2()
    }
}
