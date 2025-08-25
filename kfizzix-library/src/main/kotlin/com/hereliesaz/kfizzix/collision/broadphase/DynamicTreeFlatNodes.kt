/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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
import com.hereliesaz.kfizzix.callbacks.TreeCallback
import com.hereliesaz.kfizzix.callbacks.TreeRayCastCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.common.BufferUtils
import com.hereliesaz.kfizzix.common.Color3f
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2

/**
 * @author Daniel Murphy
 */
class DynamicTreeFlatNodes : BroadPhaseStrategy {
    var root: Int
    var memberAabb: Array<AABB>
    var userData: Array<Any?>
    var parent: IntArray
    var child1: IntArray
    var child2: IntArray
    var height: IntArray
    private var nodeCount: Int
    private var nodeCapacity: Int
    private var freeList: Int
    private val drawVecs = Array(4) { Vec2() }

    init {
        root = NULL_NODE
        nodeCount = 0
        nodeCapacity = 16
        memberAabb = emptyArray()
        userData = emptyArray()
        parent = IntArray(0)
        child1 = IntArray(0)
        child2 = IntArray(0)
        height = IntArray(0)
        expandBuffers(0, nodeCapacity)
    }

    private fun expandBuffers(oldSize: Int, newSize: Int) {
        memberAabb = BufferUtils.reallocateBuffer(AABB::class.java, memberAabb, oldSize, newSize)
        userData = BufferUtils.reallocateBuffer(Any::class.java, userData, oldSize, newSize)
        parent = BufferUtils.reallocateBuffer(parent, oldSize, newSize)
        child1 = BufferUtils.reallocateBuffer(child1, oldSize, newSize)
        child2 = BufferUtils.reallocateBuffer(child2, oldSize, newSize)
        height = BufferUtils.reallocateBuffer(height, oldSize, newSize)
        // Build a linked list for the free list.
        for (i in oldSize until newSize) {
            memberAabb[i] = AABB()
            parent[i] = if (i == newSize - 1) NULL_NODE else i + 1
            height[i] = -1
            child1[i] = -1
            child2[i] = -1
        }
        freeList = oldSize
    }

    override fun createProxy(aabb: AABB, userData: Any): Int {
        val node = allocateNode()
        // Fatten the aabb
        val nodeAABB = memberAabb[node]
        nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension
        this.userData[node] = userData
        insertLeaf(node)
        return node
    }

    override fun destroyProxy(proxyId: Int) {
        assert(0 <= proxyId && proxyId < nodeCapacity)
        assert(child1[proxyId] == NULL_NODE)
        removeLeaf(proxyId)
        freeNode(proxyId)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2): Boolean {
        assert(0 <= proxyId && proxyId < nodeCapacity)
        assert(child1[proxyId] == NULL_NODE)
        val nodeAABB = memberAabb[proxyId]
        // if (nodeAABB.contains(aabb)) {
        if (nodeAABB.lowerBound.x <= aabb.lowerBound.x && nodeAABB.lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= nodeAABB.upperBound.x && aabb.upperBound.y <= nodeAABB.upperBound.y) {
            return false
        }
        removeLeaf(proxyId)
        // Extend AABB
        val lowerBound = nodeAABB.lowerBound
        val upperBound = nodeAABB.upperBound
        lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        upperBound.y = aabb.upperBound.y + Settings.aabbExtension
        // Predict AABB displacement.
        val dx = displacement.x * Settings.aabbMultiplier
        val dy = displacement.y * Settings.aabbMultiplier
        if (dx < 0.0f) {
            lowerBound.x += dx
        } else {
            upperBound.x += dx
        }
        if (dy < 0.0f) {
            lowerBound.y += dy
        } else {
            upperBound.y += dy
        }
        insertLeaf(proxyId)
        return true
    }

    override fun getUserData(proxyId: Int): Any {
        assert(0 <= proxyId && proxyId < nodeCount)
        return userData[proxyId]!!
    }

    override fun getFatAABB(proxyId: Int): AABB {
        assert(0 <= proxyId && proxyId < nodeCount)
        return memberAabb[proxyId]
    }

    private var nodeStack = IntArray(20)
    private var nodeStackIndex = 0
    override fun query(callback: TreeCallback, aabb: AABB) {
        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = root
        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex]
            if (node == NULL_NODE) {
                continue
            }
            if (AABB.testOverlap(memberAabb[node], aabb)) {
                val child1 = child1[node]
                if (child1 == NULL_NODE) {
                    val proceed = callback.treeCallback(node)
                    if (!proceed) {
                        return
                    }
                } else {
                    if (nodeStack.size - nodeStackIndex - 2 <= 0) {
                        nodeStack = BufferUtils.reallocateBuffer(
                            nodeStack,
                            nodeStack.size, nodeStack.size * 2
                        )
                    }
                    nodeStack[nodeStackIndex++] = child1
                    nodeStack[nodeStackIndex++] = child2[node]
                }
            }
        }
    }

    private val r = Vec2()
    private val aabb = AABB()
    private val subInput = RayCastInput()
    override fun raycast(callback: TreeRayCastCallback, input: RayCastInput) {
        val p1 = input.p1
        val p2 = input.p2
        val p1x = p1.x
        val p2x = p2.x
        val p1y = p1.y
        val p2y = p2.y
        var vx: Float
        var vy: Float
        val rx: Float
        val ry: Float
        var absVx: Float
        var absVy: Float
        var cx: Float
        var cy: Float
        var hx: Float
        var hy: Float
        var tempX: Float
        var tempY: Float
        r.x = p2x - p1x
        r.y = p2y - p1y
        assert(r.x * r.x + r.y * r.y > 0f)
        r.normalize()
        rx = r.x
        ry = r.y
        // v is perpendicular to the segment.
        vx = -1f * ry
        vy = rx
        absVx = MathUtils.abs(vx)
        absVy = MathUtils.abs(vy)
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        var maxFraction = input.maxFraction
        // Build a bounding box for the segment.
        val segAABB = aabb
        // Vec2 t = p1 + maxFraction * (p2 - p1);
        // before inline
        // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
        // Vec2.minToOut(p1, temp, segAABB.lowerBound);
        // Vec2.maxToOut(p1, temp, segAABB.upperBound);
        tempX = (p2x - p1x) * maxFraction + p1x
        tempY = (p2y - p1y) * maxFraction + p1y
        segAABB.lowerBound.x = Math.min(p1x, tempX)
        segAABB.lowerBound.y = Math.min(p1y, tempY)
        segAABB.upperBound.x = Math.max(p1x, tempX)
        segAABB.upperBound.y = Math.max(p1y, tempY)
        // end inline
        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = root
        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex]
            if (node == NULL_NODE) {
                continue
            }
            val nodeAABB = memberAabb[node]
            if (!AABB.testOverlap(nodeAABB, segAABB)) {
                continue
            }
            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            // node.aabb.getCenterToOut(c);
            // node.aabb.getExtentsToOut(h);
            cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5f
            cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5f
            hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5f
            hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5f
            tempX = p1x - cx
            tempY = p1y - cy
            val separation = MathUtils.abs(vx * tempX + vy * tempY) - (absVx * hx + absVy * hy)
            if (separation > 0.0f) {
                continue
            }
            val child1 = child1[node]
            if (child1 == NULL_NODE) {
                subInput.p1.x = p1x
                subInput.p1.y = p1y
                subInput.p2.x = p2x
                subInput.p2.y = p2y
                subInput.maxFraction = maxFraction
                val value = callback.raycastCallback(subInput, node)
                if (value == 0.0f) {
                    // The client has terminated the ray cast.
                    return
                }
                if (value > 0.0f) {
                    // Update segment bounding box.
                    maxFraction = value
                    // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
                    // Vec2.minToOut(p1, temp, segAABB.lowerBound);
                    // Vec2.maxToOut(p1, temp, segAABB.upperBound);
                    tempX = (p2x - p1x) * maxFraction + p1x
                    tempY = (p2y - p1y) * maxFraction + p1y
                    segAABB.lowerBound.x = Math.min(p1x, tempX)
                    segAABB.lowerBound.y = Math.min(p1y, tempY)
                    segAABB.upperBound.x = Math.max(p1x, tempX)
                    segAABB.upperBound.y = Math.max(p1y, tempY)
                }
            } else {
                nodeStack[nodeStackIndex++] = child1
                nodeStack[nodeStackIndex++] = child2[node]
            }
        }
    }

    override fun computeHeight(): Int {
        return computeHeight(root)
    }

    private fun computeHeight(node: Int): Int {
        assert(0 <= node && node < nodeCapacity)
        if (child1[node] == NULL_NODE) {
            return 0
        }
        val height1 = computeHeight(child1[node])
        val height2 = computeHeight(child2[node])
        return 1 + MathUtils.max(height1, height2)
    }

    /**
     * Validate this tree. For testing.
     */
    fun validate() {
        validateStructure(root)
        validateMetrics(root)
        var freeCount = 0
        var freeNode = freeList
        while (freeNode != NULL_NODE) {
            assert(0 <= freeNode && freeNode < nodeCapacity)
            freeNode = parent[freeNode]
            ++freeCount
        }
        assert(height == computeHeight())
        assert(nodeCount + freeCount == nodeCapacity)
    }

    override val height: Int
        get() = if (root == NULL_NODE) {
            0
        } else height[root]
    override val maxBalance: Int
        get() {
            var maxBalance = 0
            for (i in 0 until nodeCapacity) {
                if (height[i] <= 1) {
                    continue
                }
                assert(child1[i] != NULL_NODE)
                val child1 = child1[i]
                val child2 = child2[i]
                val balance = MathUtils.abs(height[child2] - height[child1])
                maxBalance = MathUtils.max(maxBalance, balance)
            }
            return maxBalance
        }
    override val areaRatio: Float
        get() {
            if (root == NULL_NODE) {
                return 0.0f
            }
            val root = root
            val rootArea = memberAabb[root].perimeter
            var totalArea = 0.0f
            for (i in 0 until nodeCapacity) {
                if (height[i] < 0) {
                    // Free node in pool
                    continue
                }
                totalArea += memberAabb[i].perimeter
            }
            return totalArea / rootArea
        }
    // /**
    // * Build an optimal tree. Very expensive. For testing.
    // */
    // public void rebuildBottomUp() {
    // int[] nodes = new int[nodeCount];
    // int count = 0;
    //
    // // Build array of leaves. Free the rest.
    // for (int i = 0; i < nodeCapacity; ++i) {
    // if (nodes[i].height < 0) {
    // // free node in pool
    // continue;
    // }
    //
    // DynamicTreeNode node = nodes[i];
    // if (node.isLeaf()) {
    // node.parent = null;
    // nodes[count] = i;
    // ++count;
    // } else {
    // freeNode(node);
    // }
    // }
    //
    // AABB b = new AABB();
    // while (count > 1) {
    // float minCost = Float.MAX_VALUE;
    // int iMin = -1, jMin = -1;
    // for (int i = 0; i < count; ++i) {
    // AABB aabbi = nodes[nodes[i]].aabb;
    //
    // for (int j = i + 1; j < count; ++j) {
    // AABB aabbj = nodes[nodes[j]].aabb;
    // b.combine(aabbi, aabbj);
    // float cost = b.getPerimeter();
    // if (cost < minCost) {
    // iMin = i;
    // jMin = j;
    // minCost = cost;
    // }
    // }
    // }
    //
    // int index1 = nodes[iMin];
    // int index2 = nodes[jMin];
    // DynamicTreeNode child1 = nodes[index1];
    // DynamicTreeNode child2 = nodes[index2];
    //
    // DynamicTreeNode parent = allocateNode();
    // parent.child1 = child1;
    // parent.child2 = child2;
    // parent.height = 1 + MathUtils.max(child1.height, child2.height);
    // parent.aabb.combine(child1.aabb, child2.aabb);
    // parent.parent = null;
    //
    // child1.parent = parent;
    // child2.parent = parent;
    //
    // nodes[jMin] = nodes[count - 1];
    // nodes[iMin] = parent.id;
    // --count;
    // }
    //
    // root = nodes[nodes[0]];
    //
    // validate();
    // }
    private fun allocateNode(): Int {
        if (freeList == NULL_NODE) {
            assert(nodeCount == nodeCapacity)
            nodeCapacity *= 2
            expandBuffers(nodeCount, nodeCapacity)
        }
        assert(freeList != NULL_NODE)
        val node = freeList
        freeList = parent[node]
        parent[node] = NULL_NODE
        child1[node] = NULL_NODE
        height[node] = 0
        ++nodeCount
        return node
    }

    /**
     * returns a node to the pool
     */
    private fun freeNode(node: Int) {
        assert(node != NULL_NODE)
        assert(0 < nodeCount)
        parent[node] = if (freeList != NULL_NODE) freeList else NULL_NODE
        height[node] = -1
        freeList = node
        nodeCount--
    }

    private val combinedAABB = AABB()
    private fun insertLeaf(leaf: Int) {
        if (root == NULL_NODE) {
            root = leaf
            parent[root] = NULL_NODE
            return
        }
        // find the best sibling
        val leafAABB = memberAabb[leaf]
        var index = root
        while (child1[index] != NULL_NODE) {
            val node = index
            val child1 = child1[node]
            val child2 = child2[node]
            val nodeAABB = memberAabb[node]
            val area = nodeAABB.perimeter
            combinedAABB.combine(nodeAABB, leafAABB)
            val combinedArea = combinedAABB.perimeter
            // Cost of creating a new parent for this node and the new leaf
            val cost = 2.0f * combinedArea
            // Minimum cost of pushing the leaf further down the tree
            val inheritanceCost = 2.0f * (combinedArea - area)
            // Cost of descending into child1
            val cost1: Float
            val child1AABB = memberAabb[child1]
            cost1 = if (this.child1[child1] == NULL_NODE) {
                combinedAABB.combine(leafAABB, child1AABB)
                combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child1AABB)
                val oldArea = child1AABB.perimeter
                val newArea = combinedAABB.perimeter
                newArea - oldArea + inheritanceCost
            }
            // Cost of descending into child2
            val cost2: Float
            val child2AABB = memberAabb[child2]
            cost2 = if (this.child1[child2] == NULL_NODE) {
                combinedAABB.combine(leafAABB, child2AABB)
                combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child2AABB)
                val oldArea = child2AABB.perimeter
                val newArea = combinedAABB.perimeter
                newArea - oldArea + inheritanceCost
            }
            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2) {
                break
            }
            // Descend
            index = if (cost1 < cost2) {
                child1
            } else {
                child2
            }
        }
        val sibling = index
        val oldParent = parent[sibling]
        val newParent = allocateNode()
        parent[newParent] = oldParent
        userData[newParent] = null
        memberAabb[newParent].combine(leafAABB, memberAabb[sibling])
        height[newParent] = height[sibling] + 1
        if (oldParent != NULL_NODE) {
            // The sibling was not the root.
            if (child1[oldParent] == sibling) {
                child1[oldParent] = newParent
            } else {
                child2[oldParent] = newParent
            }
            child1[newParent] = sibling
            child2[newParent] = leaf
            parent[sibling] = newParent
            parent[leaf] = newParent
        } else {
            // The sibling was the root.
            child1[newParent] = sibling
            child2[newParent] = leaf
            parent[sibling] = newParent
            parent[leaf] = newParent
            root = newParent
        }
        // Walk back up the tree fixing heights and AABBs
        index = parent[leaf]
        while (index != NULL_NODE) {
            index = balance(index)
            val child1 = child1[index]
            val child2 = child2[index]
            assert(child1 != NULL_NODE)
            assert(child2 != NULL_NODE)
            height[index] = 1 + MathUtils.max(height[child1], height[child2])
            memberAabb[index].combine(memberAabb[child1], memberAabb[child2])
            index = parent[index]
        }
        // validate();
    }

    private fun removeLeaf(leaf: Int) {
        if (leaf == root) {
            root = NULL_NODE
            return
        }
        val parent = parent[leaf]
        val grandParent = this.parent[parent]
        val parentChild1 = child1[parent]
        val parentChild2 = child2[parent]
        val sibling: Int
        sibling = if (parentChild1 == leaf) {
            parentChild2
        } else {
            parentChild1
        }
        if (grandParent != NULL_NODE) {
            // Destroy parent and connect sibling to grandParent.
            if (child1[grandParent] == parent) {
                child1[grandParent] = sibling
            } else {
                child2[grandParent] = sibling
            }
            this.parent[sibling] = grandParent
            freeNode(parent)
            // Adjust ancestor bounds.
            var index = grandParent
            while (index != NULL_NODE) {
                index = balance(index)
                val child1 = child1[index]
                val child2 = child2[index]
                memberAabb[index].combine(memberAabb[child1], memberAabb[child2])
                height[index] = 1 + MathUtils.max(height[child1], height[child2])
                index = this.parent[index]
            }
        } else {
            root = sibling
            this.parent[sibling] = NULL_NODE
            freeNode(parent)
        }
        // validate();
    }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    private fun balance(iA: Int): Int {
        assert(iA != NULL_NODE)
        if (child1[iA] == NULL_NODE || height[iA] < 2) {
            return iA
        }
        val iB = child1[iA]
        val iC = child2[iA]
        assert(0 <= iB && iB < nodeCapacity)
        assert(0 <= iC && iC < nodeCapacity)
        val balance = height[iC] - height[iB]
        // Rotate C up
        if (balance > 1) {
            val iF = child1[iC]
            val iG = child2[iC]
            // assert (F != null);
            // assert (G != null);
            assert(0 <= iF && iF < nodeCapacity)
            assert(0 <= iG && iG < nodeCapacity)
            // Swap A and C
            child1[iC] = iA
            val cParent = parent[iA].also { parent[iC] = it }
            parent[iA] = iC
            // A's old parent should point to C
            if (cParent != NULL_NODE) {
                if (child1[cParent] == iA) {
                    child1[cParent] = iC
                } else {
                    assert(child2[cParent] == iA)
                    child2[cParent] = iC
                }
            } else {
                root = iC
            }
            // Rotate
            if (height[iF] > height[iG]) {
                child2[iC] = iF
                child2[iA] = iG
                parent[iG] = iA
                memberAabb[iA].combine(memberAabb[iB], memberAabb[iG])
                memberAabb[iC].combine(memberAabb[iA], memberAabb[iF])
                height[iA] = 1 + MathUtils.max(height[iB], height[iG])
                height[iC] = 1 + MathUtils.max(height[iA], height[iF])
            } else {
                child2[iC] = iG
                child2[iA] = iF
                parent[iF] = iA
                memberAabb[iA].combine(memberAabb[iB], memberAabb[iF])
                memberAabb[iC].combine(memberAabb[iA], memberAabb[iG])
                height[iA] = 1 + MathUtils.max(height[iB], height[iF])
                height[iC] = 1 + MathUtils.max(height[iA], height[iG])
            }
            return iC
        }
        // Rotate B up
        if (balance < -1) {
            val iD = child1[iB]
            val iE = child2[iB]
            assert(0 <= iD && iD < nodeCapacity)
            assert(0 <= iE && iE < nodeCapacity)
            // Swap A and B
            child1[iB] = iA
            val bParent = parent[iA].also { parent[iB] = it }
            parent[iA] = iB
            // A's old parent should point to B
            if (bParent != NULL_NODE) {
                if (child1[bParent] == iA) {
                    child1[bParent] = iB
                } else {
                    assert(child2[bParent] == iA)
                    child2[bParent] = iB
                }
            } else {
                root = iB
            }
            // Rotate
            if (height[iD] > height[iE]) {
                child2[iB] = iD
                child1[iA] = iE
                parent[iE] = iA
                memberAabb[iA].combine(memberAabb[iC], memberAabb[iE])
                memberAabb[iB].combine(memberAabb[iA], memberAabb[iD])
                height[iA] = 1 + MathUtils.max(height[iC], height[iE])
                height[iB] = 1 + MathUtils.max(height[iA], height[iD])
            } else {
                child2[iB] = iE
                child1[iA] = iD
                parent[iD] = iA
                memberAabb[iA].combine(memberAabb[iC], memberAabb[iD])
                memberAabb[iB].combine(memberAabb[iA], memberAabb[iE])
                height[iA] = 1 + MathUtils.max(height[iC], height[iD])
                height[iB] = 1 + MathUtils.max(height[iA], height[iE])
            }
            return iB
        }
        return iA
    }

    private fun validateStructure(node: Int) {
        if (node == NULL_NODE) {
            return
        }
        assert(node != root || parent[node] == NULL_NODE)
        val child1 = child1[node]
        val child2 = child2[node]
        if (child1 == NULL_NODE) {
            assert(child2 == NULL_NODE)
            assert(height[node] == 0)
            return
        }
        assert(0 <= child1 && child1 < nodeCapacity)
        assert(0 <= child2 && child2 < nodeCapacity)
        assert(parent[child1] == node)
        assert(parent[child2] == node)
        validateStructure(child1)
        validateStructure(child2)
    }

    private fun validateMetrics(node: Int) {
        if (node == NULL_NODE) {
            return
        }
        val child1 = child1[node]
        val child2 = child2[node]
        if (child1 == NULL_NODE) {
            assert(child2 == NULL_NODE)
            assert(height[node] == 0)
            return
        }
        assert(0 <= child1 && child1 < nodeCapacity)
        assert(child2 != child1 && 0 <= child2 && child2 < nodeCapacity)
        val height1 = height[child1]
        val height2 = height[child2]
        val height: Int
        height = 1 + MathUtils.max(height1, height2)
        assert(this.height[node] == height)
        val aabb = AABB()
        aabb.combine(memberAabb[child1], memberAabb[child2])
        assert(aabb.lowerBound == memberAabb[node].lowerBound)
        assert(aabb.upperBound == memberAabb[node].upperBound)
        validateMetrics(child1)
        validateMetrics(child2)
    }

    override fun drawTree(argDraw: DebugDraw) {
        if (root == NULL_NODE) {
            return
        }
        val height = computeHeight()
        drawTree(argDraw, root, 0, height)
    }

    private val color = Color3f()
    private val textVec = Vec2()
    fun drawTree(argDraw: DebugDraw, node: Int, spot: Int, height: Int) {
        val a = memberAabb[node]
        a.getVertices(drawVecs)
        color.set(1f, (height - spot) * 1f / height, (height - spot) * 1f / height)
        argDraw.drawPolygon(drawVecs, 4, color)
        argDraw.viewportTransform!!.getWorldToScreen(a.upperBound, textVec)
        argDraw.drawString(textVec.x, textVec.y, "$node-$spot+1/$height", color)
        val c1 = child1[node]
        val c2 = child2[node]
        if (c1 != NULL_NODE) {
            drawTree(argDraw, c1, spot + 1, height)
        }
        if (c2 != NULL_NODE) {
            drawTree(argDraw, c2, spot + 1, height)
        }
    }

    companion object {
        const val MAX_STACK_SIZE = 64
        const val NULL_NODE = -1
        const val INITIAL_BUFFER_LENGTH = 16
    }
}
