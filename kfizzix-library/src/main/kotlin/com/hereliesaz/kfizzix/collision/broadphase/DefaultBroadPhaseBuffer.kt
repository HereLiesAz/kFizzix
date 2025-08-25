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
package com.hereliesaz.kfizzix.collision.broadphase

import com.hereliesaz.kfizzix.callbacks.DebugDraw
import com.hereliesaz.kfizzix.callbacks.PairCallback
import com.hereliesaz.kfizzix.callbacks.TreeCallback
import com.hereliesaz.kfizzix.callbacks.TreeRayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.common.Vec2
import java.util.*

/**
 * The broad-phase is used for computing pairs and performing volume queries and
 * ray casts. This broad-phase does not persist pairs. Instead, this reports
 * potentially new pairs. It is up to the client to consume the new pairs and to
 * track subsequent overlap.
 *
 * @author Daniel Murphy
 */
class DefaultBroadPhaseBuffer(private val tree: BroadPhaseStrategy) : TreeCallback, BroadPhase {
    override var proxyCount: Int
    private var moveBuffer: IntArray
    private var moveCapacity: Int
    private var moveCount: Int
    private var pairBuffer: LongArray
    private var pairCapacity: Int
    private var pairCount: Int
    private var queryProxyId: Int
    override fun createProxy(aabb: AABB, userData: Any): Int {
        val proxyId = tree.createProxy(aabb, userData)
        ++proxyCount
        bufferMove(proxyId)
        return proxyId
    }

    override fun destroyProxy(proxyId: Int) {
        unbufferMove(proxyId)
        --proxyCount
        tree.destroyProxy(proxyId)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2) {
        val buffer = tree.moveProxy(proxyId, aabb, displacement)
        if (buffer) {
            bufferMove(proxyId)
        }
    }

    override fun touchProxy(proxyId: Int) {
        bufferMove(proxyId)
    }

    override fun getUserData(proxyId: Int): Any {
        return tree.getUserData(proxyId)
    }

    override fun getFatAABB(proxyId: Int): AABB {
        return tree.getFatAABB(proxyId)
    }

    override fun testOverlap(proxyIdA: Int, proxyIdB: Int): Boolean {
        // return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
        // return tree.overlap(proxyIdA, proxyIdB);
        val a = tree.getFatAABB(proxyIdA)
        val b = tree.getFatAABB(proxyIdB)
        if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
            return false
        }
        return !(a.lowerBound.x - b.upperBound.x > 0.0f) && !(a.lowerBound.y - b.upperBound.y > 0.0f)
    }

    override fun drawTree(argDraw: DebugDraw) {
        tree.drawTree(argDraw)
    }

    override fun updatePairs(callback: PairCallback) {
        // Reset pair buffer
        pairCount = 0
        // Perform tree queries for all moving proxies.
        for (i in 0 until moveCount) {
            queryProxyId = moveBuffer[i]
            if (queryProxyId == BroadPhase.NULL_PROXY) {
                continue
            }
            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            val fatAABB = tree.getFatAABB(queryProxyId)
            // Query tree, create pairs and add them pair buffer.
            // log.debug("quering aabb: "+queryProxy.aabb);
            tree.query(this, fatAABB)
        }
        // log.debug("Number of pairs found: "+pairCount);
        // Reset move buffer
        moveCount = 0
        // Sort the pair buffer to expose duplicates.
        Arrays.sort(pairBuffer, 0, pairCount)
        // Send the pairs back to the client.
        var i = 0
        while (i < pairCount) {
            val primaryPair = pairBuffer[i]
            val userDataA = tree.getUserData((primaryPair shr 32).toInt())
            val userDataB = tree.getUserData(primaryPair.toInt())
            // log.debug("returning pair: "+userDataA+", "+userDataB);
            callback.addPair(userDataA, userDataB)
            ++i
            // Skip any duplicate pairs.
            while (i < pairCount) {
                val pair = pairBuffer[i]
                if (pair != primaryPair) {
                    break
                }
                ++i
            }
        }
    }

    override fun query(callback: TreeCallback, aabb: AABB) {
        tree.query(callback, aabb)
    }

    override fun raycast(callback: TreeRayCastCallback, input: RayCastInput) {
        tree.raycast(callback, input)
    }

    override val treeHeight: Int
        get() = tree.height
    override val treeBalance: Int
        get() = tree.maxBalance
    override val treeQuality: Float
        get() = tree.areaRatio

    fun bufferMove(proxyId: Int) {
        if (moveCount == moveCapacity) {
            val old = moveBuffer
            moveCapacity *= 2
            moveBuffer = IntArray(moveCapacity)
            System.arraycopy(old, 0, moveBuffer, 0, old.size)
        }
        moveBuffer[moveCount] = proxyId
        ++moveCount
    }

    fun unbufferMove(proxyId: Int) {
        for (i in 0 until moveCount) {
            if (moveBuffer[i] == proxyId) {
                moveBuffer[i] = BroadPhase.NULL_PROXY
            }
        }
    }

    /**
     * This is called from DynamicTree::query when we are gathering pairs.
     */
    override fun treeCallback(proxyId: Int): Boolean {
        // A proxy cannot form a pair with itself.
        if (proxyId == queryProxyId) {
            return true
        }
        // Grow the pair buffer as needed.
        if (pairCount == pairCapacity) {
            val oldBuffer = pairBuffer
            pairCapacity *= 2
            pairBuffer = LongArray(pairCapacity)
            System.arraycopy(oldBuffer, 0, pairBuffer, 0, oldBuffer.size)
            for (i in oldBuffer.size until pairCapacity) {
                pairBuffer[i] = 0
            }
        }
        pairBuffer[pairCount] = if (proxyId < queryProxyId) {
            proxyId.toLong() shl 32 or queryProxyId.toLong()
        } else {
            queryProxyId.toLong() shl 32 or proxyId.toLong()
        }
        ++pairCount
        return true
    }

    init {
        proxyCount = 0
        pairCapacity = 16
        pairCount = 0
        pairBuffer = LongArray(pairCapacity)
        for (i in 0 until pairCapacity) {
            pairBuffer[i] = 0
        }
        moveCapacity = 16
        moveCount = 0
        moveBuffer = IntArray(moveCapacity)
        queryProxyId = BroadPhase.NULL_PROXY
    }
}
