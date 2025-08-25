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
 * ARISING IN ANY WAY OUT OF THE USE OF this SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.callbacks

import com.hereliesaz.kfizzix.common.Color3f
import com.hereliesaz.kfizzix.common.IViewportTransform
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.particle.ParticleColor

/**
 * Implement this abstract class to allow JBox2d to automatically draw your
 * physics for debugging purposes. Not intended to replace your own custom
 * rendering routines!
 *
 * @author Daniel Murphy
 */
abstract class DebugDraw(var viewportTransform: IViewportTransform?) {
    var drawFlags = 0

    fun setFlags(flags: Int) {
        drawFlags = flags
    }

    fun appendFlags(flags: Int) {
        drawFlags = drawFlags or flags
    }

    fun clearFlags(flags: Int) {
        drawFlags = drawFlags and flags.inv()
    }

    /**
     * Draw a closed polygon provided in CCW order. This implementation uses
     * [.drawSegment] to draw each side of the
     * polygon.
     */
    fun drawPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        if (vertexCount == 1) {
            drawSegment(vertices[0], vertices[0], color)
            return
        }
        for (i in 0 until vertexCount - 1) {
            drawSegment(vertices[i], vertices[i + 1], color)
        }
        if (vertexCount > 2) {
            drawSegment(vertices[vertexCount - 1], vertices[0], color)
        }
    }

    abstract fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f)

    /**
     * Draw a solid closed polygon provided in CCW order.
     */
    abstract fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f)

    /**
     * Draw a circle.
     */
    abstract fun drawCircle(center: Vec2, radius: Float, color: Color3f)

    /** Draws a circle with an axis  */
    fun drawCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        drawCircle(center, radius, color)
    }

    /**
     * Draw a solid circle.
     */
    abstract fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f)

    /**
     * Draw a line segment.
     */
    abstract fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f)

    /**
     * Draw a transform. Choose your own length scale
     */
    abstract fun drawTransform(xf: Transform)

    /**
     * Draw a string.
     */
    abstract fun drawString(x: Float, y: Float, s: String, color: Color3f)

    /**
     * Draw a particle array
     *
     * @param colors Can be null
     */
    abstract fun drawParticles(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int)

    /**
     * Draw a particle array
     *
     * @param colors Can be null
     */
    abstract fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int)

    /** Called at the end of drawing a world  */
    open fun flush() {}

    fun drawString(pos: Vec2, s: String, color: Color3f) {
        drawString(pos.x, pos.y, s, color)
    }

    /**
     * @deprecated use the viewport transform in [.getViewportTransform]
     */
    fun setCamera(x: Float, y: Float, scale: Float) {
        viewportTransform!!.setCamera(x, y, scale)
    }

    fun getScreenToWorldToOut(argScreen: Vec2, argWorld: Vec2) {
        viewportTransform!!.getScreenToWorld(argScreen, argWorld)
    }

    fun getWorldToScreenToOut(argWorld: Vec2, argScreen: Vec2) {
        viewportTransform!!.getWorldToScreen(argWorld, argScreen)
    }

    /**
     * Takes the world coordinates and puts the corresponding screen coordinates
     * in argScreen.
     */
    fun getWorldToScreenToOut(worldX: Float, worldY: Float, argScreen: Vec2) {
        argScreen.set(worldX, worldY)
        viewportTransform!!.getWorldToScreen(argScreen, argScreen)
    }

    /**
     * takes the world coordinate (argWorld) and returns the screen coordinates.
     */
    fun getWorldToScreen(argWorld: Vec2): Vec2 {
        val screen = Vec2()
        viewportTransform!!.getWorldToScreen(argWorld, screen)
        return screen
    }

    /**
     * Takes the world coordinates and returns the screen coordinates.
     */
    fun getWorldToScreen(worldX: Float, worldY: Float): Vec2 {
        val argScreen = Vec2(worldX, worldY)
        viewportTransform!!.getWorldToScreen(argScreen, argScreen)
        return argScreen
    }

    /**
     * takes the screen coordinates and puts the corresponding world coordinates
     * in argWorld.
     */
    fun getScreenToWorldToOut(screenX: Float, screenY: Float, argWorld: Vec2) {
        argWorld.set(screenX, screenY)
        viewportTransform!!.getScreenToWorld(argWorld, argWorld)
    }

    /**
     * takes the screen coordinates (argScreen) and returns the world
     * coordinates
     */
    fun getScreenToWorld(argScreen: Vec2): Vec2 {
        val world = Vec2()
        viewportTransform!!.getScreenToWorld(argScreen, world)
        return world
    }

    /**
     * takes the screen coordinates and returns the world coordinates.
     */
    fun getScreenToWorld(screenX: Float, screenY: Float): Vec2 {
        val screen = Vec2(screenX, screenY)
        viewportTransform!!.getScreenToWorld(screen, screen)
        return screen
    }

    companion object {
        /**
         * Draw shapes
         */
        const val shapeBit = 1 shl 1

        /**
         * Draw joint connections
         */
        const val jointBit = 1 shl 2

        /**
         * Draw axis aligned bounding boxes
         */
        const val aabbBit = 1 shl 3

        /**
         * Draw pairs of connected objects
         */
        const val pairBit = 1 shl 4

        /**
         * Draw center of mass frame
         */
        const val centerOfMassBit = 1 shl 5

        /**
         * Draw dynamic tree
         */
        const val dynamicTreeBit = 1 shl 6

        /**
         * Draw only the wireframe for drawing performance
         */
        const val wireframeDrawingBit = 1 shl 7
    }

}
