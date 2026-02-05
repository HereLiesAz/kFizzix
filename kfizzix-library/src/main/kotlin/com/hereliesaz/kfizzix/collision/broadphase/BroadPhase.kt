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
package com.hereliesaz.kfizzix.collision.broadphase

import com.hereliesaz.kfizzix.callbacks.DebugDraw
import com.hereliesaz.kfizzix.callbacks.PairCallback
import com.hereliesaz.kfizzix.callbacks.TreeCallback
import com.hereliesaz.kfizzix.callbacks.TreeRayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.common.Vec2

/**
 * The broad-phase is used for computing pairs and performing volume queries and
 * ray casts. This broad-phase does not persist pairs. Instead, this reports
 * potentially new pairs. It is up to the client to consume the new pairs and to
 * track subsequent overlap.
 *
 *
 *
 * Collision processing in a physics step can be divided into narrow-phase and
 * broad-phase. In the narrow-phase we compute contact points between pairs of
 * shapes. Imagine we have N shapes. Using brute force, we would need to perform
 * the narrow-phase for N*N/2 pairs.
 *
 *
 *
 * The BroadPhase class reduces this load by using a dynamic tree for pair
 * management. This greatly reduces the number of narrow-phase calls.
 *
 *
 *
 * Normally you do not interact with the broad-phase directly. Instead, the engine
 * creates and manages a broad-phase internally. Also, BroadPhase is designed
 * with the simulation loop in mind, so it is likely not suited for other
 * use cases.
 *
 *
 * https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_collision.html
 *
 * @author Daniel Murphy
 */
interface BroadPhase {
    /**
     * Get the number of proxies.
     */
    val proxyCount: Int

    /**
     * Get the height of the embedded tree.
     */
    val treeHeight: Int

    /**
     * Get the balance of the embedded tree.
     */
    val treeBalance: Int

    /**
     * Get the quality metric of the embedded tree.
     */
    val treeQuality: Float

    /**
     * Create a proxy with an initial AABB. Pairs are not reported until
     * updatePairs is called.
     *
     * @param aabb The axis aligned bounding box.
     * @param userData User data (usually a FixtureProxy).
     * @return The proxy ID.
     */
    fun createProxy(aabb: AABB, userData: Any): Int

    /**
     * Destroy a proxy. It is up to the client to remove any pairs.
     *
     * @param proxyId The proxy ID to destroy.
     */
    fun destroyProxy(proxyId: Int)

    /**
     * Call MoveProxy as many times as you like, then when you are done call
     * UpdatePairs to finalize the proxy pairs (for your time step).
     *
     * @param proxyId The proxy ID.
     * @param aabb The new AABB.
     * @param displacement The displacement since the last move.
     */
    fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2)

    /**
     * Call to trigger a re-processing of its pairs on the next call to
     * UpdatePairs.
     *
     * @param proxyId The proxy ID to touch.
     */
    fun touchProxy(proxyId: Int)

    /**
     * Get the user data associated with a proxy.
     *
     * @param proxyId The proxy ID.
     * @return The user data.
     */
    fun getUserData(proxyId: Int): Any

    /**
     * Get the "fat" AABB (expanded AABB) of a proxy.
     *
     * @param proxyId The proxy ID.
     * @return The fat AABB.
     */
    fun getFatAABB(proxyId: Int): AABB

    /**
     * Test overlap of two proxies.
     *
     * @param proxyIdA The first proxy ID.
     * @param proxyIdB The second proxy ID.
     * @return True if they overlap.
     */
    fun testOverlap(proxyIdA: Int, proxyIdB: Int): Boolean

    /**
     * Draw the broad-phase tree for debugging.
     *
     * @param argDraw The debug draw interface.
     */
    fun drawTree(argDraw: DebugDraw)

    /**
     * Update the pairs. This results in pair callbacks. This can only add
     * pairs.
     *
     * @param callback The callback to report new pairs.
     */
    fun updatePairs(callback: PairCallback)

    /**
     * Query an AABB for overlapping proxies. The callback class is called for
     * each proxy that overlaps the supplied AABB.
     *
     * @param callback The callback for reporting overlaps.
     * @param aabb The query AABB.
     */
    fun query(callback: TreeCallback, aabb: AABB)

    /**
     * Ray-cast against the proxies in the tree. This relies on the callback to
     * perform an exact ray-cast in the case were the proxy contains a shape.
     * The callback also performs any collision filtering. This has performance
     * roughly equal to k * log(n), where k is the number of collisions and n is
     * the number of proxies in the tree.
     *
     * @param input The ray-cast input data. The ray extends from p1 to p1 +
     * maxFraction * (p2 - p1).
     * @param callback A callback class that is called for each proxy that is
     * hit by the ray.
     */
    fun raycast(callback: TreeRayCastCallback, input: RayCastInput)

    companion object {
        const val NULL_PROXY = -1
    }
}
