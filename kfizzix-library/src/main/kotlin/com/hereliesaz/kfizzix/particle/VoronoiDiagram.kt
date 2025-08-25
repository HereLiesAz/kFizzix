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
package com.hereliesaz.kfizzix.particle

import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.pooling.normal.MutableStack

/**
 * A field representing the nearest generator from each point.
 *
 * @repolink https://github.com/google/liquidfun/blob/master/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h
 */
class VoronoiDiagram(generatorCapacity: Int) {
    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h#L62-L67
     */
    class Generator {
        val center = Vec2()
        var tag = 0
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h#L69-L82
     */
    class VoronoiDiagramTask {
        var x = 0
        var y = 0
        var i = 0
        var generator: Generator? = null

        constructor() {}
        constructor(x: Int, y: Int, i: Int, g: Generator?) {
            this.x = x
            this.y = y
            this.i = i
            generator = g
        }

        fun set(x: Int, y: Int, i: Int, g: Generator?): VoronoiDiagramTask {
            this.x = x
            this.y = y
            this.i = i
            generator = g
            return this
        }
    }

    fun interface VoronoiDiagramCallback {
        fun callback(aTag: Int, bTag: Int, cTag: Int)
    }

    private val generatorBuffer: Array<Generator>
    private var generatorCount: Int
    private var countX = 0
    private var countY = 0

    // The diagram is an array of "pointers".
    private var diagram: Array<Generator?>? = null

    init {
        generatorBuffer = Array(generatorCapacity) { Generator() }
        generatorCount = 0
    }

    /**
     * List all nodes that contain at least one necessary generator.
     *
     * @param callback A callback function object called for each node.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h#L56-L58
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.cpp#L195-L221
     */
    fun getNodes(callback: VoronoiDiagramCallback) {
        for (y in 0 until countY - 1) {
            for (x in 0 until countX - 1) {
                val i = x + y * countX
                val a = diagram!![i]
                val b = diagram!![i + 1]
                val c = diagram!![i + countX]
                val d = diagram!![i + 1 + countX]
                if (b !== c) {
                    if (a !== b && a !== c) {
                        callback.callback(a!!.tag, b!!.tag, c!!.tag)
                    }
                    if (d !== b && d !== c) {
                        callback.callback(b!!.tag, d!!.tag, c!!.tag)
                    }
                }
            }
        }
    }

    /**
     * Add a generator.
     *
     * @param center The position of the generator.
     * @param tag A tag used to identify the generator in callback functions.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.cpp#L45-L53
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h#L35-L39
     */
    fun addGenerator(center: Vec2, tag: Int) {
        val g = generatorBuffer[generatorCount++]
        g.center.x = center.x
        g.center.y = center.y
        g.tag = tag
    }

    private val lower = Vec2()
    private val upper = Vec2()
    private val taskPool: MutableStack<VoronoiDiagramTask> = object : MutableStack<VoronoiDiagramTask>(50) {
        override fun newInstance(): VoronoiDiagramTask {
            return VoronoiDiagramTask()
        }

        override fun newArray(size: Int): Array<VoronoiDiagramTask?> {
            return arrayOfNulls(size)
        }
    }
    private val queue = StackQueue<VoronoiDiagramTask>()

    /**
     * Generate the Voronoi diagram. It is rasterized with a given interval in
     * the same range as the necessary generators exist.
     *
     * @param radius The interval of the diagram.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.cpp#L55-L193
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2VoronoiDiagram.h#L41-L45
     */
    fun generate(radius: Float) {
        assert(diagram == null)
        val inverseRadius = 1 / radius
        lower.x = Float.MAX_VALUE
        lower.y = Float.MAX_VALUE
        upper.x = -Float.MAX_VALUE
        upper.y = -Float.MAX_VALUE
        for (k in 0 until generatorCount) {
            val g = generatorBuffer[k]
            Vec2.minToOut(lower, g.center, lower)
            Vec2.maxToOut(upper, g.center, upper)
        }
        countX = 1 + (inverseRadius * (upper.x - lower.x)).toInt()
        countY = 1 + (inverseRadius * (upper.y - lower.y)).toInt()
        diagram = arrayOfNulls(countX * countY)
        queue.reset(arrayOfNulls<VoronoiDiagramTask>(4 * countX * countX))
        for (k in 0 until generatorCount) {
            val g = generatorBuffer[k]
            g.center.x = inverseRadius * (g.center.x - lower.x)
            g.center.y = inverseRadius * (g.center.y - lower.y)
            val x = MathUtils.max(
                0,
                MathUtils.min(g.center.x.toInt(), countX - 1)
            )
            val y = MathUtils.max(
                0,
                MathUtils.min(g.center.y.toInt(), countY - 1)
            )
            queue.push(taskPool.pop()!!.set(x, y, x + y * countX, g))
        }
        while (!queue.empty()) {
            val front = queue.pop()
            val x = front!!.x
            val y = front.y
            val i = front.i
            val g = front.generator
            if (diagram!![i] == null) {
                diagram!![i] = g
                if (x > 0) {
                    queue.push(taskPool.pop()!!.set(x - 1, y, i - 1, g))
                }
                if (y > 0) {
                    queue.push(taskPool.pop()!!.set(x, y - 1, i - countX, g))
                }
                if (x < countX - 1) {
                    queue.push(taskPool.pop()!!.set(x + 1, y, i + 1, g))
                }
                if (y < countY - 1) {
                    queue.push(taskPool.pop()!!.set(x, y + 1, i + countX, g))
                }
            }
            taskPool.push(front)
        }
        val maxIteration = countX + countY
        for (iteration in 0 until maxIteration) {
            for (y in 0 until countY) {
                for (x in 0 until countX - 1) {
                    val i = x + y * countX
                    val a = diagram!![i]
                    val b = diagram!![i + 1]
                    if (a !== b) {
                        queue.push(taskPool.pop()!!.set(x, y, i, b))
                        queue.push(taskPool.pop()!!.set(x + 1, y, i + 1, a))
                    }
                }
            }
            for (y in 0 until countY - 1) {
                for (x in 0 until countX) {
                    val i = x + y * countX
                    val a = diagram!![i]
                    val b = diagram!![i + countX]
                    if (a !== b) {
                        queue.push(taskPool.pop()!!.set(x, y, i, b))
                        queue.push(taskPool.pop()!!.set(x, y + 1, i + countX, a))
                    }
                }
            }
            var updated = false
            while (!queue.empty()) {
                val front = queue.pop()
                val x = front!!.x
                val y = front.y
                val i = front.i
                val k = front.generator
                val a = diagram!![i]
                if (a !== k) {
                    val ax = a!!.center.x - x
                    val ay = a.center.y - y
                    val bx = k!!.center.x - x
                    val by = k.center.y - y
                    val a2 = ax * ax + ay * ay
                    val b2 = bx * bx + by * by
                    if (a2 > b2) {
                        diagram!![i] = k
                        if (x > 0) {
                            queue.push(taskPool.pop()!!.set(x - 1, y, i - 1, k))
                        }
                        if (y > 0) {
                            queue.push(taskPool.pop()!!.set(x, y - 1, i - countX, k))
                        }
                        if (x < countX - 1) {
                            queue.push(taskPool.pop()!!.set(x + 1, y, i + 1, k))
                        }
                        if (y < countY - 1) {
                            queue.push(taskPool.pop()!!.set(x, y + 1, i + countX, k))
                        }
                        updated = true
                    }
                }
                taskPool.push(front)
            }
            if (!updated) {
                break
            }
        }
    }
}
