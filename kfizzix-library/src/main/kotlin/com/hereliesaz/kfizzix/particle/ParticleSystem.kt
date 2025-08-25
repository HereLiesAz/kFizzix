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

import com.hereliesaz.kfizzix.callbacks.ParticleDestructionListener
import com.hereliesaz.kfizzix.callbacks.ParticleQueryCallback
import com.hereliesaz.kfizzix.callbacks.ParticleRaycastCallback
import com.hereliesaz.kfizzix.callbacks.QueryCallback
import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.RayCastInput
import com.hereliesaz.kfizzix.collision.RayCastOutput
import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.common.*
import com.hereliesaz.kfizzix.dynamics.Body
import com.hereliesaz.kfizzix.dynamics.Fixture
import com.hereliesaz.kfizzix.dynamics.TimeStep
import com.hereliesaz.kfizzix.dynamics.World
import com.hereliesaz.kfizzix.particle.VoronoiDiagram.VoronoiDiagramCallback
import java.lang.reflect.Array
import java.util.*

class ParticleSystem(var world: World) {
    var timestamp = 0
    var allParticleFlags = 0
    var allGroupFlags = 0
    var density = 0f
    var inverseDensity = 0f
    var gravityScale = 0f
    var particleDiameter = 0f
    var inverseDiameter = 0f
    var squaredDiameter = 0f
    var count = 0
    var internalAllocatedCapacity = 0
    var maxCount = 0
    var flagsBuffer: ParticleBufferInt
    var positionBuffer: ParticleBuffer<Vec2>
    var velocityBuffer: ParticleBuffer<Vec2>
    var accumulationBuffer: FloatArray? = null // temporary values
    var accumulation2Buffer: Array<Vec2>? = null // temporary vector values
    var depthBuffer: FloatArray? = null // distance from the surface
    var colorBuffer: ParticleBuffer<ParticleColor>
    var groupBuffer: Array<ParticleGroup?>? = null
    var userDataBuffer: ParticleBuffer<Any>
    var proxyCount = 0
    var proxyCapacity = 0
    var proxyBuffer: Array<Proxy?>? = null
    var contactCount = 0
    var contactCapacity = 0
    var contactBuffer: Array<ParticleContact?>? = null
    var bodyContactCount = 0
    var bodyContactCapacity = 0
    var bodyContactBuffer: Array<ParticleBodyContact?>? = null
    var pairCount = 0
    var pairCapacity = 0
    var pairBuffer: Array<Pair?>? = null
    var triadCount = 0
    var triadCapacity = 0
    var triadBuffer: Array<Triad?>? = null
    var groupCount = 0
    var groupList: ParticleGroup? = null
    var pressureStrength = 0f
    var dampingStrength = 0f
    var elasticStrength = 0f
    var springStrength = 0f
    var viscousStrength = 0f
    var surfaceTensionStrengthA = 0f
    var surfaceTensionStrengthB = 0f
    var powderStrength = 0f
    var ejectionStrength = 0f
    var colorMixingStrength = 0f

    // public void assertNotSamePosition() {
    // for (int i = 0; i < count; i++) {
    // Vec2 vi = positionBuffer.data[i];
    // for (int j = i + 1; j < count; j++) {
    // Vec2 vj = positionBuffer.data[j];
    // assert(vi.x != vj.x || vi.y != vj.y);
    // }
    // }
    // }
    fun createParticle(def: ParticleDef): Int {
        if (count >= internalAllocatedCapacity) {
            val capacity = capacity
            if (internalAllocatedCapacity < capacity) {
                flagsBuffer.data = reallocateBuffer(
                    flagsBuffer,
                    internalAllocatedCapacity, capacity, false
                )
                positionBuffer.data = reallocateBuffer(
                    positionBuffer,
                    internalAllocatedCapacity, capacity, false
                )
                velocityBuffer.data = reallocateBuffer(
                    velocityBuffer,
                    internalAllocatedCapacity, capacity, false
                )
                accumulationBuffer = BufferUtils.reallocateBuffer(
                    accumulationBuffer, 0, internalAllocatedCapacity,
                    capacity, false
                )
                accumulation2Buffer = BufferUtils.reallocateBuffer(
                    Vec2::class.java,
                    accumulation2Buffer, 0, internalAllocatedCapacity,
                    capacity, true
                )
                depthBuffer = BufferUtils.reallocateBuffer(
                    depthBuffer, 0,
                    internalAllocatedCapacity, capacity, true
                )
                colorBuffer.data = reallocateBuffer(
                    colorBuffer,
                    internalAllocatedCapacity, capacity, true
                )
                groupBuffer = BufferUtils.reallocateBuffer(
                    ParticleGroup::class.java,
                    groupBuffer, 0, internalAllocatedCapacity, capacity,
                    false
                )
                userDataBuffer.data = reallocateBuffer(
                    userDataBuffer,
                    internalAllocatedCapacity, capacity, true
                )
                internalAllocatedCapacity = capacity
            }
        }
        if (count >= internalAllocatedCapacity) {
            return Settings.invalidParticleIndex
        }
        val index = count++
        flagsBuffer.data!![index] = def.flags
        positionBuffer.data!![index].set(def.position)
        // assertNotSamePosition();
        velocityBuffer.data!![index].set(def.velocity)
        groupBuffer!![index] = null
        if (depthBuffer != null) {
            depthBuffer!![index] = 0f
        }
        if (colorBuffer.data != null || def.color != null) {
            colorBuffer.data = requestParticleBuffer(
                colorBuffer.dataClass,
                colorBuffer.data
            )
            colorBuffer.data!![index]!!.set(def.color)
        }
        if (userDataBuffer.data != null || def.userData != null) {
            userDataBuffer.data = requestParticleBuffer(
                userDataBuffer.dataClass, userDataBuffer.data
            )
            userDataBuffer.data!![index] = def.userData
        }
        if (proxyCount >= proxyCapacity) {
            val oldCapacity = proxyCapacity
            val newCapacity = if (proxyCount != 0) 2 * proxyCount else Settings.minParticleBufferCapacity
            proxyBuffer = BufferUtils.reallocateBuffer(
                Proxy::class.java, proxyBuffer,
                oldCapacity, newCapacity
            )
            proxyCapacity = newCapacity
        }
        proxyBuffer!![proxyCount++]!!.index = index
        return index
    }

    private val capacity: Int
        private get() {
            var capacity = if (count != 0) 2 * count else Settings.minParticleBufferCapacity
            capacity = limitCapacity(capacity, maxCount)
            capacity = limitCapacity(capacity, flagsBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, positionBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, velocityBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, colorBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, userDataBuffer.userSuppliedCapacity)
            return capacity
        }

    fun destroyParticle(index: Int, callDestructionListener: Boolean) {
        var flags = ParticleType.zombieParticle
        if (callDestructionListener) {
            flags = flags or ParticleType.destructionListener
        }
        flagsBuffer.data!![index] = flagsBuffer.data!![index] or flags
    }

    private val temp = AABB()
    private val dpcallback = DestroyParticlesInShapeCallback()
    fun destroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener: Boolean): Int {
        dpcallback.init(this, shape, xf, callDestructionListener)
        shape.computeAABB(temp, xf, 0)
        world.queryAABB(dpcallback, temp)
        return dpcallback.destroyed
    }

    fun destroyParticlesInGroup(group: ParticleGroup, callDestructionListener: Boolean) {
        for (i in group.firstIndex until group.lastIndex) {
            destroyParticle(i, callDestructionListener)
        }
    }

    private val temp2 = AABB()
    private val tempVec = Vec2()
    private val tempTransform = Transform()
    private val tempTransform2 = Transform()
    private val createParticleGroupCallback = CreateParticleGroupCallback()
    private val tempParticleDef = ParticleDef()
    fun createParticleGroup(groupDef: ParticleGroupDef): ParticleGroup {
        val stride = particleStride
        val identity = tempTransform
        identity.setIdentity()
        val transform = tempTransform2
        transform.setIdentity()
        val firstIndex = count
        if (groupDef.shape != null) {
            val particleDef = tempParticleDef
            particleDef.flags = groupDef.flags
            particleDef.color = groupDef.color
            particleDef.userData = groupDef.userData
            val shape = groupDef.shape
            transform.set(groupDef.position, groupDef.angle)
            val aabb = temp
            val childCount = shape!!.childCount
            for (childIndex in 0 until childCount) {
                if (childIndex == 0) {
                    shape.computeAABB(aabb, identity, childIndex)
                } else {
                    val childAABB = temp2
                    shape.computeAABB(childAABB, identity, childIndex)
                    aabb.combine(childAABB)
                }
            }
            val upperBoundY = aabb.upperBound.y
            val upperBoundX = aabb.upperBound.x
            var y = MathUtils.floor(aabb.lowerBound.y / stride) * stride
            while (y < upperBoundY) {
                var x = MathUtils.floor(aabb.lowerBound.x / stride) * stride
                while (x < upperBoundX) {
                    val p = tempVec
                    p.x = x
                    p.y = y
                    if (shape.testPoint(identity, p)) {
                        Transform.mulToOut(transform, p, p)
                        particleDef.position.x = p.x
                        particleDef.position.y = p.y
                        p.subLocal(groupDef.position)
                        Vec2.crossToOutUnsafe(groupDef.angularVelocity, p, particleDef.velocity)
                        particleDef.velocity.addLocal(groupDef.linearVelocity)
                        createParticle(particleDef)
                    }
                    x += stride
                }
                y += stride
            }
        }
        val lastIndex = count
        val group = ParticleGroup()
        group.system = this
        group.firstIndex = firstIndex
        group.lastIndex = lastIndex
        group.groupFlags = groupDef.groupFlags
        group.strength = groupDef.strength
        group.userData = groupDef.userData
        group.transform.set(transform)
        group.destroyAutomatically = groupDef.destroyAutomatically
        group.prev = null
        group.next = groupList
        if (groupList != null) {
            groupList!!.prev = group
        }
        groupList = group
        ++groupCount
        for (i in firstIndex until lastIndex) {
            groupBuffer!![i] = group
        }
        updateContacts(true)
        if (groupDef.flags and pairFlags != 0) {
            for (k in 0 until contactCount) {
                val contact = contactBuffer!![k]
                var a = contact!!.indexA
                var b = contact.indexB
                if (a > b) {
                    val temp = a
                    a = b
                    b = temp
                }
                if (firstIndex <= a && b < lastIndex) {
                    if (pairCount >= pairCapacity) {
                        val oldCapacity = pairCapacity
                        val newCapacity = if (pairCount != 0) 2 * pairCount else Settings.minParticleBufferCapacity
                        pairBuffer = BufferUtils.reallocateBuffer(
                            Pair::class.java,
                            pairBuffer, oldCapacity, newCapacity
                        )
                        pairCapacity = newCapacity
                    }
                    val pair = pairBuffer!![pairCount]
                    pair!!.indexA = a
                    pair.indexB = b
                    pair.flags = contact.flags
                    pair.strength = groupDef.strength
                    pair.distance = MathUtils.distance(
                        positionBuffer.data!![a],
                        positionBuffer.data!![b]
                    )
                    pairCount++
                }
            }
        }
        if (groupDef.flags and triadFlags != 0) {
            val diagram = VoronoiDiagram(lastIndex - firstIndex)
            for (i in firstIndex until lastIndex) {
                diagram.addGenerator(positionBuffer.data!![i], i)
            }
            diagram.generate(stride / 2)
            createParticleGroupCallback.system = this
            createParticleGroupCallback.def = groupDef
            createParticleGroupCallback.firstIndex = firstIndex
            diagram.getNodes(createParticleGroupCallback)
        }
        if (groupDef.groupFlags and ParticleGroupType.solidParticleGroup != 0) {
            computeDepthForGroup(group)
        }
        return group
    }

    fun joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup) {
        assert(groupA !== groupB)
        RotateBuffer(groupB.firstIndex, groupB.lastIndex, count)
        assert(groupB.lastIndex == count)
        RotateBuffer(groupA.firstIndex, groupA.lastIndex, groupB.firstIndex)
        assert(groupA.lastIndex == groupB.firstIndex)
        var particleFlags = 0
        for (i in groupA.firstIndex until groupB.lastIndex) {
            particleFlags = particleFlags or flagsBuffer.data!![i]
        }
        updateContacts(true)
        if (particleFlags and pairFlags != 0) {
            for (k in 0 until contactCount) {
                val contact = contactBuffer!![k]
                var a = contact!!.indexA
                var b = contact.indexB
                if (a > b) {
                    val temp = a
                    a = b
                    b = temp
                }
                if (groupA.firstIndex <= a && a < groupA.lastIndex && groupB.firstIndex <= b && b < groupB.lastIndex) {
                    if (pairCount >= pairCapacity) {
                        val oldCapacity = pairCapacity
                        val newCapacity = if (pairCount != 0) 2 * pairCount else Settings.minParticleBufferCapacity
                        pairBuffer = BufferUtils.reallocateBuffer(
                            Pair::class.java,
                            pairBuffer, oldCapacity, newCapacity
                        )
                        pairCapacity = newCapacity
                    }
                    val pair = pairBuffer!![pairCount]
                    pair!!.indexA = a
                    pair.indexB = b
                    pair.flags = contact.flags
                    pair.strength = MathUtils.min(groupA.strength, groupB.strength)
                    pair.distance = MathUtils.distance(
                        positionBuffer.data!![a],
                        positionBuffer.data!![b]
                    )
                    pairCount++
                }
            }
        }
        if (particleFlags and triadFlags != 0) {
            val diagram = VoronoiDiagram(groupB.lastIndex - groupA.firstIndex)
            for (i in groupA.firstIndex until groupB.lastIndex) {
                if (flagsBuffer.data!![i] and ParticleType.zombieParticle == 0) {
                    diagram.addGenerator(positionBuffer.data!![i], i)
                }
            }
            diagram.generate(particleStride / 2)
            val callback = JoinParticleGroupsCallback()
            callback.system = this
            callback.groupA = groupA
            callback.groupB = groupB
            diagram.getNodes(callback)
        }
        for (i in groupB.firstIndex until groupB.lastIndex) {
            groupBuffer!![i] = groupA
        }
        val groupFlags = groupA.groupFlags or groupB.groupFlags
        groupA.groupFlags = groupFlags
        groupA.lastIndex = groupB.lastIndex
        groupB.firstIndex = groupB.lastIndex
        destroyParticleGroup(groupB)
        if (groupFlags and ParticleGroupType.solidParticleGroup != 0) {
            computeDepthForGroup(groupA)
        }
    }

    // Only called from solveZombie() or joinParticleGroups().
    fun destroyParticleGroup(group: ParticleGroup?) {
        assert(groupCount > 0)
        assert(group != null)
        if (world.particleDestructionListener != null) {
            world.particleDestructionListener!!.sayGoodbye(group)
        }
        for (i in group!!.firstIndex until group.lastIndex) {
            groupBuffer!![i] = null
        }
        if (group.prev != null) {
            group.prev!!.next = group.next
        }
        if (group.next != null) {
            group.next!!.prev = group.prev
        }
        if (group === groupList) {
            groupList = group.next
        }
        --groupCount
    }

    fun computeDepthForGroup(group: ParticleGroup) {
        for (i in group.firstIndex until group.lastIndex) {
            accumulationBuffer!![i] = 0f
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            if (a >= group.firstIndex && a < group.lastIndex && b >= group.firstIndex && b < group.lastIndex) {
                val w = contact.weight
                accumulationBuffer!![a] += w
                accumulationBuffer!![b] += w
            }
        }
        depthBuffer = requestParticleBuffer(depthBuffer)
        for (i in group.firstIndex until group.lastIndex) {
            val w = accumulationBuffer!![i]
            depthBuffer!![i] = if (w < 0.8f) 0f else Float.MAX_VALUE
        }
        val iterationCount = group.particleCount
        for (t in 0 until iterationCount) {
            var updated = false
            for (k in 0 until contactCount) {
                val contact = contactBuffer!![k]
                val a = contact!!.indexA
                val b = contact.indexB
                if (a >= group.firstIndex && a < group.lastIndex && b >= group.firstIndex && b < group.lastIndex) {
                    val r = 1 - contact.weight
                    val ap0 = depthBuffer!![a]
                    val bp0 = depthBuffer!![b]
                    val ap1 = bp0 + r
                    val bp1 = ap0 + r
                    if (ap0 > ap1) {
                        depthBuffer!![a] = ap1
                        updated = true
                    }
                    if (bp0 > bp1) {
                        depthBuffer!![b] = bp1
                        updated = true
                    }
                }
            }
            if (!updated) {
                break
            }
        }
        for (i in group.firstIndex until group.lastIndex) {
            val p = depthBuffer!![i]
            if (p < Float.MAX_VALUE) {
                depthBuffer!![i] *= particleDiameter
            } else {
                depthBuffer!![i] = 0f
            }
        }
    }

    fun addContact(a: Int, b: Int) {
        assert(a != b)
        val pa = positionBuffer.data!![a]
        val pb = positionBuffer.data!![b]
        val dx = pb.x - pa.x
        val dy = pb.y - pa.y
        val d2 = dx * dx + dy * dy
        // assert(d2 != 0);
        if (d2 < squaredDiameter) {
            if (contactCount >= contactCapacity) {
                val oldCapacity = contactCapacity
                val newCapacity = if (contactCount != 0) 2 * contactCount else Settings.minParticleBufferCapacity
                contactBuffer = BufferUtils.reallocateBuffer(
                    ParticleContact::class.java, contactBuffer, oldCapacity,
                    newCapacity
                )
                contactCapacity = newCapacity
            }
            val invD = if (d2 != 0f) MathUtils.sqrt(1 / d2) else Float.MAX_VALUE
            val contact = contactBuffer!![contactCount]
            contact!!.indexA = a
            contact.indexB = b
            contact.flags = flagsBuffer.data!![a] or flagsBuffer.data!![b]
            contact.weight = 1 - d2 * invD * inverseDiameter
            contact.normal.x = invD * dx
            contact.normal.y = invD * dy
            contactCount++
        }
    }

    fun updateContacts(exceptZombie: Boolean) {
        for (p in 0 until proxyCount) {
            val proxy = proxyBuffer!![p]
            val i = proxy!!.index
            val pos = positionBuffer.data!![i]
            proxy.tag = computeTag(inverseDiameter * pos.x, inverseDiameter * pos.y)
        }
        Arrays.sort(proxyBuffer, 0, proxyCount)
        contactCount = 0
        var c_index = 0
        for (i in 0 until proxyCount) {
            val a = proxyBuffer!![i]
            val rightTag = computeRelativeTag(a!!.tag, 1, 0)
            for (j in i + 1 until proxyCount) {
                val b = proxyBuffer!![j]
                if (rightTag < b!!.tag) {
                    break
                }
                addContact(a.index, b.index)
            }
            val bottomLeftTag = computeRelativeTag(a.tag, -1, 1)
            while (c_index < proxyCount) {
                val c = proxyBuffer!![c_index]
                if (bottomLeftTag <= c!!.tag) {
                    break
                }
                c_index++
            }
            val bottomRightTag = computeRelativeTag(a.tag, 1, 1)
            for (b_index in c_index until proxyCount) {
                val b = proxyBuffer!![b_index]
                if (bottomRightTag < b!!.tag) {
                    break
                }
                addContact(a.index, b.index)
            }
        }
        if (exceptZombie) {
            var j = contactCount
            var i = 0
            while (i < j) {
                if (contactBuffer!![i]!!.flags and ParticleType.zombieParticle != 0) {
                    --j
                    val temp = contactBuffer!![j]
                    contactBuffer!![j] = contactBuffer!![i]
                    contactBuffer!![i] = temp
                    --i
                }
                i++
            }
            contactCount = j
        }
    }

    private val ubcCallback = UpdateBodyContactsCallback()
    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L2608-L2708
     */
    fun updateBodyContacts() {
        val aabb = temp
        aabb.lowerBound.x = Float.MAX_VALUE
        aabb.lowerBound.y = Float.MAX_VALUE
        aabb.upperBound.x = -Float.MAX_VALUE
        aabb.upperBound.y = -Float.MAX_VALUE
        for (i in 0 until count) {
            val p = positionBuffer.data!![i]
            Vec2.minToOut(aabb.lowerBound, p, aabb.lowerBound)
            Vec2.maxToOut(aabb.upperBound, p, aabb.upperBound)
        }
        aabb.lowerBound.x -= particleDiameter
        aabb.lowerBound.y -= particleDiameter
        aabb.upperBound.x += particleDiameter
        aabb.upperBound.y += particleDiameter
        bodyContactCount = 0
        ubcCallback.system = this
        world.queryAABB(ubcCallback, aabb)
    }

    private val scCallback = SolveCollisionCallback()
    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L2752-L2852
     */
    fun solveCollision(step: TimeStep) {
        val aabb = temp
        val lowerBound = aabb.lowerBound
        val upperBound = aabb.upperBound
        lowerBound.x = Float.MAX_VALUE
        lowerBound.y = Float.MAX_VALUE
        upperBound.x = -Float.MAX_VALUE
        upperBound.y = -Float.MAX_VALUE
        for (i in 0 until count) {
            val v = velocityBuffer.data!![i]
            val p1 = positionBuffer.data!![i]
            val p1x = p1.x
            val p1y = p1.y
            val p2x = p1x + step.dt * v.x
            val p2y = p1y + step.dt * v.y
            val bx = Math.min(p1x, p2x)
            val by = Math.min(p1y, p2y)
            lowerBound.x = Math.min(lowerBound.x, bx)
            lowerBound.y = Math.min(lowerBound.y, by)
            val b1x = Math.max(p1x, p2x)
            val b1y = Math.max(p1y, p2y)
            upperBound.x = Math.max(upperBound.x, b1x)
            upperBound.y = Math.max(upperBound.y, b1y)
        }
        scCallback.step = step
        scCallback.system = this
        world.queryAABB(scCallback, aabb)
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L2973-L3095
     */
    fun solve(step: TimeStep) {
        ++timestamp
        if (count == 0) {
            return
        }
        allParticleFlags = 0
        for (i in 0 until count) {
            allParticleFlags = allParticleFlags or flagsBuffer.data!![i]
        }
        if (allParticleFlags and ParticleType.zombieParticle != 0) {
            solveZombie()
        }
        if (count == 0) {
            return
        }
        allGroupFlags = 0
        var group: ParticleGroup? = groupList
        while (group != null) {
            allGroupFlags = allGroupFlags or group.groupFlags
            group = group.next
        }
        val gravityX = step.dt * gravityScale * world.gravity.x
        val gravityY = step.dt * gravityScale * world.gravity.y
        val criticalVelocityYSquared = getCriticalVelocitySquared(step)
        for (i in 0 until count) {
            val v = velocityBuffer.data!![i]
            v.x += gravityX
            v.y += gravityY
            val v2 = v.x * v.x + v.y * v.y
            if (v2 > criticalVelocityYSquared) {
                val a = if (v2 == 0f) Float.MAX_VALUE else MathUtils.sqrt(criticalVelocityYSquared / v2)
                v.x *= a
                v.y *= a
            }
        }
        solveCollision(step)
        if (allGroupFlags and ParticleGroupType.rigidParticleGroup != 0) {
            solveRigid(step)
        }
        if (allParticleFlags and ParticleType.wallParticle != 0) {
            solveWall(step)
        }
        for (i in 0 until count) {
            val pos = positionBuffer.data!![i]
            val vel = velocityBuffer.data!![i]
            pos.x += step.dt * vel.x
            pos.y += step.dt * vel.y
        }
        updateBodyContacts()
        updateContacts(false)
        if (allParticleFlags and ParticleType.viscousParticle != 0) {
            solveViscous(step)
        }
        if (allParticleFlags and ParticleType.powderParticle != 0) {
            solvePowder(step)
        }
        if (allParticleFlags and ParticleType.tensileParticle != 0) {
            solveTensile(step)
        }
        if (allParticleFlags and ParticleType.elasticParticle != 0) {
            solveElastic(step)
        }
        if (allParticleFlags and ParticleType.springParticle != 0) {
            solveSpring(step)
        }
        if (allGroupFlags and ParticleGroupType.solidParticleGroup != 0) {
            solveSolid(step)
        }
        if (allParticleFlags and ParticleType.colorMixingParticle != 0) {
            solveColorMixing(step)
        }
        solvePressure(step)
        solveDamping(step)
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3195-L3258
     */
    fun solvePressure(step: TimeStep) {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        for (i in 0 until count) {
            accumulationBuffer!![i] = 0f
        }
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            val a = contact!!.index
            val w = contact.weight
            accumulationBuffer!![a] += w
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            val w = contact.weight
            accumulationBuffer!![a] += w
            accumulationBuffer!![b] += w
        }
        // ignores powder particles
        if (allParticleFlags and noPressureFlags != 0) {
            for (i in 0 until count) {
                if (flagsBuffer.data!![i] and noPressureFlags != 0) {
                    accumulationBuffer!![i] = 0f
                }
            }
        }
        // calculates pressure as a linear function of density
        val pressurePerWeight = pressureStrength * getCriticalPressure(step)
        for (i in 0 until count) {
            val w = accumulationBuffer!![i]
            val h = pressurePerWeight * MathUtils.max(
                0.0f,
                MathUtils.min(w, Settings.maxParticleWeight) - Settings.minParticleWeight
            )
            accumulationBuffer!![i] = h
        }
        // applies pressure between each particle in contact
        val velocityPerPressure = step.dt / (density * particleDiameter)
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            val a = contact!!.index
            val b = contact.body
            val w = contact.weight
            val m = contact.mass
            val n = contact.normal
            val p = positionBuffer.data!![a]
            val h = accumulationBuffer!![a] + pressurePerWeight * w
            val f = tempVec
            val coef = velocityPerPressure * w * m * h
            f.x = coef * n.x
            f.y = coef * n.y
            val velData = velocityBuffer.data!![a]
            val particleInvMass = particleInvMass
            velData.x -= particleInvMass * f.x
            velData.y -= particleInvMass * f.y
            b!!.applyLinearImpulse(f, p, true)
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            val w = contact.weight
            val n = contact.normal
            val h = accumulationBuffer!![a] + accumulationBuffer!![b]
            val fx = velocityPerPressure * w * h * n.x
            val fy = velocityPerPressure * w * h * n.y
            val velDataA = velocityBuffer.data!![a]
            val velDataB = velocityBuffer.data!![b]
            velDataA.x -= fx
            velDataA.y -= fy
            velDataB.x += fx
            velDataB.y += fy
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3260-L3304
     */
    fun solveDamping(step: TimeStep) {
        // reduces the normal velocity of each contact
        val damping = dampingStrength
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            val a = contact!!.index
            val b = contact.body
            val w = contact.weight
            val m = contact.mass
            val n = contact.normal
            val p = positionBuffer.data!![a]
            val tempX = p.x - b!!.sweep.c.x
            val tempY = p.y - b.sweep.c.y
            val velA = velocityBuffer.data!![a]
            // getLinearVelocityFromWorldPointToOut, with -= velA
            var vx = -b.angularVelocity * tempY + b.linearVelocity.x - velA.x
            var vy = b.angularVelocity * tempX + b.linearVelocity.y - velA.y
            // done
            val vn = vx * n.x + vy * n.y
            if (vn < 0) {
                val f = tempVec
                f.x = damping * w * m * vn * n.x
                f.y = damping * w * m * vn * n.y
                val invMass = particleInvMass
                velA.x += invMass * f.x
                velA.y += invMass * f.y
                f.x = -f.x
                f.y = -f.y
                b.applyLinearImpulse(f, p, true)
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            val w = contact.weight
            val n = contact.normal
            val velA = velocityBuffer.data!![a]
            val velB = velocityBuffer.data!![b]
            val vx = velB.x - velA.x
            val vy = velB.y - velA.y
            val vn = vx * n.x + vy * n.y
            if (vn < 0) {
                val fx = damping * w * vn * n.x
                val fy = damping * w * vn * n.y
                velA.x += fx
                velA.y += fy
                velB.x -= fx
                velB.y -= fy
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3506-L3515
     */
    fun solveWall(step: TimeStep) {
        for (i in 0 until count) {
            if (flagsBuffer.data!![i] and ParticleType.wallParticle != 0) {
                val r = velocityBuffer.data!![i]
                r.x = 0.0f
                r.y = 0.0f
            }
        }
    }

    private val tempVec2 = Vec2()
    private val tempRot = Rot()
    private val tempXf = Transform()
    private val tempXf2 = Transform()

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3517-L3541
     */
    fun solveRigid(step: TimeStep) {
        var group: ParticleGroup? = groupList
        while (group != null) {
            if (group.groupFlags and ParticleGroupType.rigidParticleGroup != 0) {
                group.updateStatistics()
                val temp = tempVec
                val cross = tempVec2
                val rotation = tempRot
                rotation.set(step.dt * group.angularVelocity)
                Rot.mulToOutUnsafe(rotation, group.center, cross)
                temp.set(group.linearVelocity).mulLocal(step.dt).addLocal(group.center).subLocal(cross)
                tempXf.p.set(temp)
                tempXf.q.set(rotation)
                Transform.mulToOut(tempXf, group.transform, group.transform)
                val velocityTransform = tempXf2
                velocityTransform.p.x = step.inverseDt * tempXf.p.x
                velocityTransform.p.y = step.inverseDt * tempXf.p.y
                velocityTransform.q.s = step.inverseDt * tempXf.q.s
                velocityTransform.q.c = step.inverseDt * (tempXf.q.c - 1)
                for (i in group.firstIndex until group.lastIndex) {
                    Transform.mulToOutUnsafe(
                        velocityTransform,
                        positionBuffer.data!![i], velocityBuffer.data!![i]
                    )
                }
            }
            group = group.next
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3543-L3583
     */
    fun solveElastic(step: TimeStep) {
        val elasticStrength = step.inverseDt * elasticStrength
        for (k in 0 until triadCount) {
            val triad = triadBuffer!![k]
            if (triad!!.flags and ParticleType.elasticParticle != 0) {
                val a = triad.indexA
                val b = triad.indexB
                val c = triad.indexC
                val oa = triad.pa
                val ob = triad.pb
                val oc = triad.pc
                val pa = positionBuffer.data!![a]
                val pb = positionBuffer.data!![b]
                val pc = positionBuffer.data!![c]
                val px = 1f / 3 * (pa.x + pb.x + pc.x)
                val py = 1f / 3 * (pa.y + pb.y + pc.y)
                var rs = Vec2.cross(oa, pa) + Vec2.cross(ob, pb) + Vec2.cross(oc, pc)
                var rc = Vec2.dot(oa, pa) + Vec2.dot(ob, pb) + Vec2.dot(oc, pc)
                val r2 = rs * rs + rc * rc
                val invR = if (r2 == 0f) Float.MAX_VALUE else MathUtils.sqrt(1f / r2)
                rs *= invR
                rc *= invR
                val strength = elasticStrength * triad.strength
                val roax = rc * oa.x - rs * oa.y
                val roay = rs * oa.x + rc * oa.y
                val robx = rc * ob.x - rs * ob.y
                val roby = rs * ob.x + rc * ob.y
                val rocx = rc * oc.x - rs * oc.y
                val rocy = rs * oc.x + rc * oc.y
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val vc = velocityBuffer.data!![c]
                va.x += strength * (roax - (pa.x - px))
                va.y += strength * (roay - (pa.y - py))
                vb.x += strength * (robx - (pb.x - px))
                vb.y += strength * (roby - (pb.y - py))
                vc.x += strength * (rocx - (pc.x - px))
                vc.y += strength * (rocy - (pc.y - py))
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3585-L3610
     */
    fun solveSpring(step: TimeStep) {
        val springStrength = step.inverseDt * springStrength
        for (k in 0 until pairCount) {
            val pair = pairBuffer!![k]
            if (pair!!.flags and ParticleType.springParticle != 0) {
                val a = pair.indexA
                val b = pair.indexB
                val pa = positionBuffer.data!![a]
                val pb = positionBuffer.data!![b]
                val dx = pb.x - pa.x
                val dy = pb.y - pa.y
                val r0 = pair.distance
                var r1 = MathUtils.sqrt(dx * dx + dy * dy)
                if (r1 == 0f) r1 = Float.MAX_VALUE
                val strength = springStrength * pair.strength
                val fx = strength * (r0 - r1) / r1 * dx
                val fy = strength * (r0 - r1) / r1 * dy
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3612-L3658
     */
    fun solveTensile(step: TimeStep) {
        accumulation2Buffer = requestParticleBuffer(
            Vec2::class.java,
            accumulation2Buffer
        )
        for (i in 0 until count) {
            accumulationBuffer!![i] = 0f
            accumulation2Buffer!![i].setZero()
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            if (contact!!.flags and ParticleType.tensileParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val n = contact.normal
                accumulationBuffer!![a] += w
                accumulationBuffer!![b] += w
                val a2A = accumulation2Buffer!![a]
                val a2B = accumulation2Buffer!![b]
                val inter = (1 - w) * w
                a2A.x -= inter * n.x
                a2A.y -= inter * n.y
                a2B.x += inter * n.x
                a2B.y += inter * n.y
            }
        }
        val strengthA = surfaceTensionStrengthA * criticalVelocity
        val strengthB = surfaceTensionStrengthB * criticalVelocity
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            if (contact!!.flags and ParticleType.tensileParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val n = contact.normal
                val a2A = accumulation2Buffer!![a]
                val a2B = accumulation2Buffer!![b]
                val h = accumulationBuffer!![a] + accumulationBuffer!![b]
                val sx = a2B.x - a2A.x
                val sy = a2B.y - a2A.y
                val fn = (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w
                val fx = fn * n.x
                val fy = fn * n.y
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3660-L3694
     */
    fun solveViscous(step: TimeStep) {
        val viscousStrength = viscousStrength
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            val a = contact!!.index
            if (flagsBuffer.data!![a] and ParticleType.viscousParticle != 0) {
                val b = contact.body
                val w = contact.weight
                val m = contact.mass
                val p = positionBuffer.data!![a]
                val va = velocityBuffer.data!![a]
                val tempX = p.x - b!!.sweep.c.x
                val tempY = p.y - b.sweep.c.y
                val vx = -b.angularVelocity * tempY + b.linearVelocity.x - va.x
                val vy = b.angularVelocity * tempX + b.linearVelocity.y - va.y
                val f = tempVec
                val pInvMass = particleInvMass
                f.x = viscousStrength * m * w * vx
                f.y = viscousStrength * m * w * vy
                va.x += pInvMass * f.x
                va.y += pInvMass * f.y
                f.x = -f.x
                f.y = -f.y
                b.applyLinearImpulse(f, p, true)
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            if (contact!!.flags and ParticleType.viscousParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val vx = vb.x - va.x
                val vy = vb.y - va.y
                val fx = viscousStrength * w * vx
                val fy = viscousStrength * w * vy
                va.x += fx
                va.y += fy
                vb.x -= fx
                vb.y -= fy
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3719-L3740
     */
    fun solvePowder(step: TimeStep) {
        val powderStrength = powderStrength * criticalVelocity
        val minWeight = 1.0f - Settings.particleStride
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            val a = contact!!.index
            if (flagsBuffer.data!![a] and ParticleType.powderParticle != 0) {
                val w = contact.weight
                if (w > minWeight) {
                    val b = contact.body
                    val m = contact.mass
                    val p = positionBuffer.data!![a]
                    val n = contact.normal
                    val f = tempVec
                    val va = velocityBuffer.data!![a]
                    val inter = powderStrength * m * (w - minWeight)
                    val pInvMass = particleInvMass
                    f.x = inter * n.x
                    f.y = inter * n.y
                    va.x -= pInvMass * f.x
                    va.y -= pInvMass * f.y
                    b!!.applyLinearImpulse(f, p, true)
                }
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            if (contact!!.flags and ParticleType.powderParticle != 0) {
                val w = contact.weight
                if (w > minWeight) {
                    val a = contact.indexA
                    val b = contact.indexB
                    val n = contact.normal
                    val va = velocityBuffer.data!![a]
                    val vb = velocityBuffer.data!![b]
                    val inter = powderStrength * (w - minWeight)
                    val fx = inter * n.x
                    val fy = inter * n.y
                    va.x -= fx
                    va.y -= fy
                    vb.x += fx
                    vb.y += fy
                }
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3742-L3762
     */
    fun solveSolid(step: TimeStep) {
        // applies extra repulsive force from solid particle groups
        depthBuffer = requestParticleBuffer(depthBuffer)
        val ejectionStrength = step.inverseDt * ejectionStrength
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            if (groupBuffer!![a] !== groupBuffer!![b]) {
                val w = contact.weight
                val n = contact.normal
                val h = depthBuffer!![a] + depthBuffer!![b]
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val inter = ejectionStrength * h * w
                val fx = inter * n.x
                val fy = inter * n.y
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3774-L3796
     */
    fun solveColorMixing(step: TimeStep) {
        // mixes color between contacting particles
        colorBuffer.data = requestParticleBuffer(
            ParticleColor::class.java,
            colorBuffer.data
        )
        val colorMixing256 = (256 * colorMixingStrength).toInt()
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            val a = contact!!.indexA
            val b = contact.indexB
            if (flagsBuffer.data!![a] and flagsBuffer.data!![b] and ParticleType.colorMixingParticle != 0) {
                val colorA = colorBuffer.data!![a]
                val colorB = colorBuffer.data!![b]
                val dr = colorMixing256 * (colorB!!.r.toInt() and 0xFF - (colorA!!.r.toInt() and 0xFF)) shr 8
                val dg = colorMixing256 * (colorB.g.toInt() and 0xFF - (colorA.g.toInt() and 0xFF)) shr 8
                val db = colorMixing256 * (colorB.b.toInt() and 0xFF - (colorA.b.toInt() and 0xFF)) shr 8
                val da = colorMixing256 * (colorB.a.toInt() and 0xFF - (colorA.a.toInt() and 0xFF)) shr 8
                colorA.r = (colorA.r + dr.toByte()).toByte()
                colorA.g = (colorA.g + dg.toByte()).toByte()
                colorA.b = (colorA.b + db.toByte()).toByte()
                colorA.a = (colorA.a + da.toByte()).toByte()
                colorB.r = (colorB.r - dr.toByte()).toByte()
                colorB.g = (colorB.g - dg.toByte()).toByte()
                colorB.b = (colorB.b - db.toByte()).toByte()
                colorB.a = (colorB.a - da.toByte()).toByte()
            }
        }
    }

    /**
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.cpp#L3798-L3892
     */
    fun solveZombie() {
        // removes particles with a zombie flag
        var newCount = 0
        val newIndices = IntArray(count)
        for (i in 0 until count) {
            val flags = flagsBuffer.data!![i]
            if (flags and ParticleType.zombieParticle != 0) {
                val destructionListener = world.particleDestructionListener
                if (flags and ParticleType.destructionListener != 0 && destructionListener != null) {
                    destructionListener.sayGoodbye(i)
                }
                newIndices[i] = Settings.invalidParticleIndex
            } else {
                newIndices[i] = newCount
                if (i != newCount) {
                    flagsBuffer.data!![newCount] = flagsBuffer.data!![i]
                    positionBuffer.data!![newCount].set(positionBuffer.data!![i])
                    velocityBuffer.data!![newCount].set(velocityBuffer.data!![i])
                    groupBuffer!![newCount] = groupBuffer!![i]
                    if (depthBuffer != null) {
                        depthBuffer!![newCount] = depthBuffer!![i]
                    }
                    if (colorBuffer.data != null) {
                        colorBuffer.data!![newCount]!!.set(colorBuffer.data!![i])
                    }
                    if (userDataBuffer.data != null) {
                        userDataBuffer.data!![newCount] = userDataBuffer.data!![i]
                    }
                }
                newCount++
            }
        }
        // update proxies
        for (k in 0 until proxyCount) {
            val proxy = proxyBuffer!![k]
            proxy!!.index = newIndices[proxy.index]
        }
        // Proxy lastProxy = std.remove_if(
        // proxyBuffer, proxyBuffer + proxyCount,
        // Test.IsProxyInvalid);
        // proxyCount = (int) (lastProxy - proxyBuffer);
        var j = proxyCount
        var i = 0
        while (i < j) {
            if (Test.IsProxyInvalid(proxyBuffer!![i])) {
                --j
                val temp = proxyBuffer!![j]
                proxyBuffer!![j] = proxyBuffer!![i]
                proxyBuffer!![i] = temp
                --i
            }
            i++
        }
        proxyCount = j
        // update contacts
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            contact!!.indexA = newIndices[contact.indexA]
            contact.indexB = newIndices[contact.indexB]
        }
        // ParticleContact lastContact = std.remove_if(
        // contactBuffer, contactBuffer + contactCount,
        // Test.IsContactInvalid);
        // contactCount = (int) (lastContact - contactBuffer);
        j = contactCount
        i = 0
        while (i < j) {
            if (Test.IsContactInvalid(contactBuffer!![i])) {
                --j
                val temp = contactBuffer!![j]
                contactBuffer!![j] = contactBuffer!![i]
                contactBuffer!![i] = temp
                --i
            }
            i++
        }
        contactCount = j
        // update particle-body contacts
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            contact!!.index = newIndices[contact.index]
        }
        // ParticleBodyContact lastBodyContact = std.remove_if(
        // bodyContactBuffer, bodyContactBuffer + bodyContactCount,
        // Test.IsBodyContactInvalid);
        // bodyContactCount = (int) (lastBodyContact - bodyContactBuffer);
        j = bodyContactCount
        i = 0
        while (i < j) {
            if (Test.IsBodyContactInvalid(bodyContactBuffer!![i])) {
                --j
                val temp = bodyContactBuffer!![j]
                bodyContactBuffer!![j] = bodyContactBuffer!![i]
                bodyContactBuffer!![i] = temp
                --i
            }
            i++
        }
        bodyContactCount = j
        // update pairs
        for (k in 0 until pairCount) {
            val pair = pairBuffer!![k]
            pair!!.indexA = newIndices[pair.indexA]
            pair.indexB = newIndices[pair.indexB]
        }
        // Pair lastPair = std.remove_if(pairBuffer, pairBuffer +
        // pairCount, Test.IsPairInvalid);
        // pairCount = (int) (lastPair - pairBuffer);
        j = pairCount
        i = 0
        while (i < j) {
            if (Test.IsPairInvalid(pairBuffer!![i])) {
                --j
                val temp = pairBuffer!![j]
                pairBuffer!![j] = pairBuffer!![i]
                pairBuffer!![i] = temp
                --i
            }
            i++
        }
        pairCount = j
        // update triads
        for (k in 0 until triadCount) {
            val triad = triadBuffer!![k]
            triad!!.indexA = newIndices[triad.indexA]
            triad.indexB = newIndices[triad.indexB]
            triad.indexC = newIndices[triad.indexC]
        }
        // Triad lastTriad =
        // std.remove_if(triadBuffer, triadBuffer + triadCount,
        // Test.isTriadInvalid);
        // triadCount = (int) (lastTriad - triadBuffer);
        j = triadCount
        i = 0
        while (i < j) {
            if (Test.IsTriadInvalid(triadBuffer!![i])) {
                --j
                val temp = triadBuffer!![j]
                triadBuffer!![j] = triadBuffer!![i]
                triadBuffer!![i] = temp
                --i
            }
            i++
        }
        triadCount = j
        // update groups
        var group: ParticleGroup? = groupList
        while (group != null) {
            var firstIndex = newCount
            var lastIndex = 0
            var modified = false
            for (i in group.firstIndex until group.lastIndex) {
                j = newIndices[i]
                if (j >= 0) {
                    firstIndex = MathUtils.min(firstIndex, j)
                    lastIndex = MathUtils.max(lastIndex, j + 1)
                } else {
                    modified = true
                }
            }
            if (firstIndex < lastIndex) {
                group.firstIndex = firstIndex
                group.lastIndex = lastIndex
                if (modified) {
                    if (group.groupFlags and ParticleGroupType.rigidParticleGroup != 0) {
                        group.toBeSplit = true
                    }
                }
            } else {
                group.firstIndex = 0
                group.lastIndex = 0
                if (group.destroyAutomatically) {
                    group.toBeDestroyed = true
                }
            }
            group = group.next
        }
        // update particle count
        count = newCount
        // world.stackAllocator.Free(newIndices);
        // destroy bodies with no particles
        group = groupList
        while (group != null) {
            val next = group.next
            if (group.toBeDestroyed) {
                destroyParticleGroup(group)
            } else if (group.toBeSplit) {
                // TODO: split the group
            }
            group = next
        }
    }

    private class NewIndices {
        var start = 0
        var mid = 0
        var end = 0
        fun getIndex(i: Int): Int {
            return if (i < start) {
                i
            } else if (i < mid) {
                i + end - mid
            } else if (i < end) {
                i + start - mid
            } else {
                i
            }
        }
    }

    private val newIndices = NewIndices()
    fun RotateBuffer(start: Int, mid: Int, end: Int) {
        // move the particles assigned to the given group toward the end of
        // the array
        if (start == mid || mid == end) {
            return
        }
        newIndices.start = start
        newIndices.mid = mid
        newIndices.end = end
        BufferUtils.rotate(flagsBuffer.data!!, start, mid, end)
        BufferUtils.rotate(positionBuffer.data!!, start, mid, end)
        BufferUtils.rotate(velocityBuffer.data!!, start, mid, end)
        BufferUtils.rotate(groupBuffer!!, start, mid, end)
        if (depthBuffer != null) {
            BufferUtils.rotate(depthBuffer!!, start, mid, end)
        }
        if (colorBuffer.data != null) {
            BufferUtils.rotate(colorBuffer.data!!, start, mid, end)
        }
        if (userDataBuffer.data != null) {
            BufferUtils.rotate(userDataBuffer.data!!, start, mid, end)
        }
        // update proxies
        for (k in 0 until proxyCount) {
            val proxy = proxyBuffer!![k]
            proxy!!.index = newIndices.getIndex(proxy.index)
        }
        // update contacts
        for (k in 0 until contactCount) {
            val contact = contactBuffer!![k]
            contact!!.indexA = newIndices.getIndex(contact.indexA)
            contact.indexB = newIndices.getIndex(contact.indexB)
        }
        // update particle-body contacts
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer!![k]
            contact!!.index = newIndices.getIndex(contact.index)
        }
        // update pairs
        for (k in 0 until pairCount) {
            val pair = pairBuffer!![k]
            pair!!.indexA = newIndices.getIndex(pair.indexA)
            pair.indexB = newIndices.getIndex(pair.indexB)
        }
        // update triads
        for (k in 0 until triadCount) {
            val triad = triadBuffer!![k]
            triad!!.indexA = newIndices.getIndex(triad.indexA)
            triad.indexB = newIndices.getIndex(triad.indexB)
            triad.indexC = newIndices.getIndex(triad.indexC)
        }
        // update groups
        var group: ParticleGroup? = groupList
        while (group != null) {
            group.firstIndex = newIndices.getIndex(group.firstIndex)
            group.lastIndex = newIndices.getIndex(group.lastIndex - 1) + 1
            group = group.next
        }
    }

    var particleRadius: Float
        get() = particleDiameter / 2
        set(radius) {
            particleDiameter = 2 * radius
            squaredDiameter = particleDiameter * particleDiameter
            inverseDiameter = 1 / particleDiameter
        }
    var particleDensity: Float
        get() = density
        set(density) {
            this.density = density
            inverseDensity = 1 / this.density
        }

    fun getCriticalVelocity(step: TimeStep): Float {
        return particleDiameter * step.inverseDt
    }

    fun getCriticalVelocitySquared(step: TimeStep): Float {
        val velocity = getCriticalVelocity(step)
        return velocity * velocity
    }

    fun getCriticalPressure(step: TimeStep): Float {
        return density * getCriticalVelocitySquared(step)
    }

    val particleStride: Float
        get() = Settings.particleStride * particleDiameter
    val particleMass: Float
        get() {
            val stride = particleStride
            return density * stride * stride
        }
    val particleInvMass: Float
        get() = 1.777777f * inverseDensity * inverseDiameter * inverseDiameter
    val particleFlagsBuffer: IntArray?
        get() = flagsBuffer.data
    val particlePositionBuffer: Array<Vec2>?
        get() = positionBuffer.data
    val particleVelocityBuffer: Array<Vec2>?
        get() = velocityBuffer.data
    val particleColorBuffer: Array<ParticleColor?>?
        get() {
            colorBuffer.data = requestParticleBuffer(
                ParticleColor::class.java,
                colorBuffer.data
            )
            return colorBuffer.data
        }
    val particleUserDataBuffer: Array<Any?>?
        get() {
            userDataBuffer.data = requestParticleBuffer(
                Any::class.java,
                userDataBuffer.data
            )
            return userDataBuffer.data
        }
    val particleGroupList: Array<ParticleGroup?>?
        get() = groupBuffer

    fun setParticleFlagsBuffer(buffer: IntArray?, capacity: Int) {
        setParticleBuffer(flagsBuffer, buffer, capacity)
    }

    fun setParticlePositionBuffer(buffer: Array<Vec2>?, capacity: Int) {
        setParticleBuffer(positionBuffer, buffer, capacity)
    }

    fun setParticleVelocityBuffer(buffer: Array<Vec2>?, capacity: Int) {
        setParticleBuffer(velocityBuffer, buffer, capacity)
    }

    fun setParticleColorBuffer(buffer: Array<ParticleColor>?, capacity: Int) {
        setParticleBuffer(colorBuffer, buffer, capacity)
    }

    fun setParticleUserDataBuffer(buffer: Array<Any>?, capacity: Int) {
        setParticleBuffer(userDataBuffer, buffer, capacity)
    }

    private fun requestParticleBuffer(buffer: FloatArray?): FloatArray {
        var buffer = buffer
        if (buffer == null) {
            buffer = FloatArray(internalAllocatedCapacity)
        }
        return buffer
    }

    class ParticleBuffer<T>(val dataClass: Class<T>) {
        var data: Array<T?>? = null
        var userSuppliedCapacity = 0
    }

    class ParticleBufferInt {
        var data: IntArray? = null
        var userSuppliedCapacity = 0
    }

    /** Used for detecting particle contacts  */
    class Proxy : Comparable<Proxy?> {
        var index = 0
        var tag: Long = 0
        override fun compareTo(other: Proxy): Int {
            return if (tag - other.tag < 0) -1 else if (other.tag == tag) 0 else 1
        }

        override fun equals(obj: Any?): Boolean {
            if (this === obj) return true
            if (obj == null) return false
            if (javaClass != obj.javaClass) return false
            val other = obj as Proxy
            return tag == other.tag
        }
    }

    /**
     * Connection between two particles.
     *
     * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L120-L134
     */
    class Pair {
        /**
         * Index of the respective particle making a pair.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L123-L124
         */
        var indexA = 0

        /**
         * Index of the respective particle making a pair.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L123-L124
         */
        var indexB = 0

        /**
         * The logical sum of the particle flags. See the b2ParticleFlag enum.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L126-L127
         */
        var flags = 0

        /**
         * The strength of cohesion among the particles.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L129-L130
         */
        var strength = 0f

        /**
         * The initial distance of the particles.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L132-L133
         */
        var distance = 0f
    }

    /**
     * Connection between three particles.
     */
    class Triad {
        /**
         * Index of the respective particle making triad.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L139-L140
         */
        var indexA = 0

        /**
         * Index of the respective particle making triad.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L139-L140
         */
        var indexB = 0

        /**
         * Index of the respective particle making triad.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L139-L140
         */
        var indexC = 0

        /**
         * The logical sum of the particle flags.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L142-L143
         */
        var flags = 0

        /**
         * The strength of cohesion among the particles.
         *
         * @repolink https://github.com/google/liquidfun/blob/7f20402173fd143a3988c921bc384459c6a858f2/liquidfun/Box2D/Box2D/Particle/b2ParticleSystem.h#L145-L146
         */
        var strength = 0f
        val pa = Vec2()
        val pb = Vec2()
        val pc = Vec2()
        var ka = 0f
        var kb = 0f
        var kc = 0f
        var s = 0f
    }

    /**
     * Callback used with VoronoiDiagram.
     */
    class CreateParticleGroupCallback : VoronoiDiagramCallback {
        override fun callback(a: Int, b: Int, c: Int) {
            val pa = system!!.positionBuffer.data!![a]
            val pb = system!!.positionBuffer.data!![b]
            val pc = system!!.positionBuffer.data!![c]
            val dabx = pa.x - pb.x
            val daby = pa.y - pb.y
            val dbcx = pb.x - pc.x
            val dbcy = pb.y - pc.y
            val dcax = pc.x - pa.x
            val dcay = pc.y - pa.y
            val maxDistanceSquared = Settings.maxTriadDistanceSquared * system!!.squaredDiameter
            if (dabx * dabx + daby * daby < maxDistanceSquared && dbcx * dbcx + dbcy * dbcy < maxDistanceSquared && dcax * dcax + dcay * dcay < maxDistanceSquared) {
                if (system!!.triadCount >= system!!.triadCapacity) {
                    val oldCapacity = system!!.triadCapacity
                    val newCapacity = if (system!!.triadCount != 0) 2 * system!!.triadCount else Settings.minParticleBufferCapacity
                    system!!.triadBuffer = BufferUtils.reallocateBuffer(
                        Triad::class.java, system!!.triadBuffer, oldCapacity,
                        newCapacity
                    )
                    system!!.triadCapacity = newCapacity
                }
                val triad = system!!.triadBuffer!![system!!.triadCount]
                triad!!.indexA = a
                triad.indexB = b
                triad.indexC = c
                triad.flags = (system!!.flagsBuffer.data!![a]
                        or system!!.flagsBuffer.data!![b]
                        or system!!.flagsBuffer.data!![c])
                triad.strength = def!!.strength
                val midPointx = 1f / 3 * (pa.x + pb.x + pc.x)
                val midPointy = 1f / 3 * (pa.y + pb.y + pc.y)
                triad.pa.x = pa.x - midPointx
                triad.pa.y = pa.y - midPointy
                triad.pb.x = pb.x - midPointx
                triad.pb.y = pb.y - midPointy
                triad.pc.x = pc.x - midPointx
                triad.pc.y = pc.y - midPointy
                triad.ka = -(dcax * dabx + dcay * daby)
                triad.kb = -(dabx * dbcx + daby * dbcy)
                triad.kc = -(dbcx * dcax + dbcy * dcay)
                triad.s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa)
                system!!.triadCount++
            }
        }

        var system: ParticleSystem? = null
        var def: ParticleGroupDef? = null // pointer
        var firstIndex = 0
    }

    // Callback used with VoronoiDiagram.
    class JoinParticleGroupsCallback : VoronoiDiagramCallback {
        override fun callback(a: Int, b: Int, c: Int) {
            // Create a triad if it will contain particles from both groups.
            val countA = ((if (a < groupB!!.firstIndex) 1 else 0)
                    + (if (b < groupB!!.firstIndex) 1 else 0)
                    + if (c < groupB!!.firstIndex) 1 else 0)
            if (countA > 0 && countA < 3) {
                val af = system!!.flagsBuffer.data!![a]
                val bf = system!!.flagsBuffer.data!![b]
                val cf = system!!.flagsBuffer.data!![c]
                if (af and bf and cf and triadFlags != 0) {
                    val pa = system!!.positionBuffer.data!![a]
                    val pb = system!!.positionBuffer.data!![b]
                    val pc = system!!.positionBuffer.data!![c]
                    val dabx = pa.x - pb.x
                    val daby = pa.y - pb.y
                    val dbcx = pb.x - pc.x
                    val dbcy = pb.y - pc.y
                    val dcax = pc.x - pa.x
                    val dcay = pc.y - pa.y
                    val maxDistanceSquared = Settings.maxTriadDistanceSquared * system!!.squaredDiameter
                    if (dabx * dabx + daby * daby < maxDistanceSquared && dbcx * dbcx + dbcy * dbcy < maxDistanceSquared && dcax * dcax + dcay * dcay < maxDistanceSquared) {
                        if (system!!.triadCount >= system!!.triadCapacity) {
                            val oldCapacity = system!!.triadCapacity
                            val newCapacity = if (system!!.triadCount != 0) 2 * system!!.triadCount else Settings.minParticleBufferCapacity
                            system!!.triadBuffer = BufferUtils.reallocateBuffer(
                                Triad::class.java, system!!.triadBuffer,
                                oldCapacity, newCapacity
                            )
                            system!!.triadCapacity = newCapacity
                        }
                        val triad = system!!.triadBuffer!![system!!.triadCount]
                        triad!!.indexA = a
                        triad.indexB = b
                        triad.indexC = c
                        triad.flags = af or bf or cf
                        triad.strength = MathUtils.min(groupA!!.strength, groupB!!.strength)
                        val midPointx = 1f / 3 * (pa.x + pb.x + pc.x)
                        val midPointy = 1f / 3 * (pa.y + pb.y + pc.y)
                        triad.pa.x = pa.x - midPointx
                        triad.pa.y = pa.y - midPointy
                        triad.pb.x = pb.x - midPointx
                        triad.pb.y = pb.y - midPointy
                        triad.pc.x = pc.x - midPointx
                        triad.pc.y = pc.y - midPointy
                        triad.ka = -(dcax * dabx + dcay * daby)
                        triad.kb = -(dabx * dbcx + daby * dbcy)
                        triad.kc = -(dbcx * dcax + dbcy * dcay)
                        triad.s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa)
                        system!!.triadCount++
                    }
                }
            }
        }

        var system: ParticleSystem? = null
        var groupA: ParticleGroup? = null
        var groupB: ParticleGroup? = null
    }

    class DestroyParticlesInShapeCallback : ParticleQueryCallback {
        var system: ParticleSystem? = null
        var shape: Shape? = null
        var xf: Transform? = null
        var callDestructionListener = false
        var destroyed = 0
        fun init(
            system: ParticleSystem?, shape: Shape?, xf: Transform?,
            callDestructionListener: Boolean
        ) {
            this.system = system
            this.shape = shape
            this.xf = xf
            destroyed = 0
            this.callDestructionListener = callDestructionListener
        }

        override fun reportParticle(index: Int): Boolean {
            assert(index >= 0 && index < system!!.count)
            if (shape!!.testPoint(xf!!, system!!.positionBuffer.data!![index])) {
                system!!.destroyParticle(index, callDestructionListener)
                destroyed++
            }
            return true
        }
    }

    class UpdateBodyContactsCallback : QueryCallback {
        var system: ParticleSystem? = null
        private val tempVec = Vec2()
        override fun reportFixture(fixture: Fixture): Boolean {
            if (fixture.isSensor) {
                return true
            }
            val shape = fixture.shape
            val b = fixture.body
            val bp = b.worldCenter
            val bm = b.mass
            val bI = b.inertia - bm * b.localCenter.lengthSquared()
            val invBm = if (bm > 0) 1 / bm else 0f
            val invBI = if (bI > 0) 1 / bI else 0f
            val childCount = shape.childCount
            for (childIndex in 0 until childCount) {
                val aabb = fixture.getAABB(childIndex)
                val aabblowerBoundx = aabb.lowerBound.x - system!!.particleDiameter
                val aabblowerBoundy = aabb.lowerBound.y - system!!.particleDiameter
                val aabbupperBoundx = aabb.upperBound.x + system!!.particleDiameter
                val aabbupperBoundy = aabb.upperBound.y + system!!.particleDiameter
                val firstProxy = lowerBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabblowerBoundx, system!!.inverseDiameter * aabblowerBoundy)
                )
                val lastProxy = upperBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabbupperBoundx, system!!.inverseDiameter * aabbupperBoundy)
                )
                var proxy = firstProxy
                while (proxy != lastProxy) {
                    val a = system!!.proxyBuffer!![proxy]!!.index
                    val ap = system!!.positionBuffer.data!![a]
                    if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y && ap.y <= aabbupperBoundy) {
                        var d: Float
                        val n = tempVec
                        d = fixture.computeDistance(ap, childIndex, n)
                        if (d < system!!.particleDiameter) {
                            val invAm = if (system!!.flagsBuffer.data!![a] and ParticleType.wallParticle != 0) 0f else system!!.particleInvMass
                            val rpx = ap.x - bp.x
                            val rpy = ap.y - bp.y
                            val rpn = rpx * n.y - rpy * n.x
                            if (system!!.bodyContactCount >= system!!.bodyContactCapacity) {
                                val oldCapacity = system!!.bodyContactCapacity
                                val newCapacity = if (system!!.bodyContactCount != 0) 2 * system!!.bodyContactCount else Settings.minParticleBufferCapacity
                                system!!.bodyContactBuffer = BufferUtils.reallocateBuffer(
                                    ParticleBodyContact::class.java,
                                    system!!.bodyContactBuffer,
                                    oldCapacity, newCapacity
                                )
                                system!!.bodyContactCapacity = newCapacity
                            }
                            val contact = system!!.bodyContactBuffer!![system!!.bodyContactCount]
                            contact!!.index = a
                            contact.body = b
                            contact.weight = 1 - d * system!!.inverseDiameter
                            contact.normal.x = -n.x
                            contact.normal.y = -n.y
                            contact.mass = 1 / (invAm + invBm + invBI * rpn * rpn)
                            system!!.bodyContactCount++
                        }
                    }
                    ++proxy
                }
            }
            return true
        }
    }

    class SolveCollisionCallback : QueryCallback {
        var system: ParticleSystem? = null
        var step: TimeStep? = null
        private val input = RayCastInput()
        private val output = RayCastOutput()
        private val tempVec = Vec2()
        private val tempVec2 = Vec2()
        override fun reportFixture(fixture: Fixture): Boolean {
            if (fixture.isSensor) {
                return true
            }
            val shape = fixture.shape
            val body = fixture.body
            val childCount = shape.childCount
            for (childIndex in 0 until childCount) {
                val aabb = fixture.getAABB(childIndex)
                val aabblowerBoundx = aabb.lowerBound.x - system!!.particleDiameter
                val aabblowerBoundy = aabb.lowerBound.y - system!!.particleDiameter
                val aabbupperBoundx = aabb.upperBound.x + system!!.particleDiameter
                val aabbupperBoundy = aabb.upperBound.y + system!!.particleDiameter
                val firstProxy = lowerBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabblowerBoundx, system!!.inverseDiameter * aabblowerBoundy)
                )
                val lastProxy = upperBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabbupperBoundx, system!!.inverseDiameter * aabbupperBoundy)
                )
                var proxy = firstProxy
                while (proxy != lastProxy) {
                    val a = system!!.proxyBuffer!![proxy]!!.index
                    val ap = system!!.positionBuffer.data!![a]
                    if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y && ap.y <= aabbupperBoundy) {
                        val av = system!!.velocityBuffer.data!![a]
                        val temp = tempVec
                        Transform.mulTransToOutUnsafe(body.xf0, ap, temp)
                        Transform.mulToOutUnsafe(body.xf, temp, input.p1)
                        input.p2.x = ap.x + step!!.dt * av.x
                        input.p2.y = ap.y + step!!.dt * av.y
                        input.maxFraction = 1f
                        if (fixture.raycast(output, input, childIndex)) {
                            val p = tempVec
                            p.x = ((1 - output.fraction) * input.p1.x
                                    + output.fraction * input.p2.x
                                    + Settings.linearSlop * output.normal.x)
                            p.y = ((1 - output.fraction) * input.p1.y
                                    + output.fraction * input.p2.y
                                    + Settings.linearSlop * output.normal.y)
                            val vx = step!!.inverseDt * (p.x - ap.x)
                            val vy = step!!.inverseDt * (p.y - ap.y)
                            av.x = vx
                            av.y = vy
                            val particleMass = system!!.particleMass
                            val ax = particleMass * (av.x - vx)
                            val ay = particleMass * (av.y - vy)
                            val b = output.normal
                            val fdn = ax * b.x + ay * b.y
                            val f = tempVec2
                            f.x = fdn * b.x
                            f.y = fdn * b.y
                            body.applyLinearImpulse(f, p, true)
                        }
                    }
                    ++proxy
                }
            }
            return true
        }
    }

    internal object Test {
        fun IsProxyInvalid(proxy: Proxy?): Boolean {
            return proxy!!.index < 0
        }

        fun IsContactInvalid(contact: ParticleContact?): Boolean {
            return contact!!.indexA < 0 || contact.indexB < 0
        }

        fun IsBodyContactInvalid(contact: ParticleBodyContact?): Boolean {
            return contact!!.index < 0
        }

        fun IsPairInvalid(pair: Pair?): Boolean {
            return pair!!.indexA < 0 || pair.indexB < 0
        }

        fun IsTriadInvalid(triad: Triad?): Boolean {
            return triad!!.indexA < 0 || triad.indexB < 0 || triad.indexC < 0
        }
    }

    companion object {
        /**
         * All particle types that require creating pairs
         */
        private const val pairFlags = ParticleType.springParticle

        /**
         * All particle types that require creating triads
         */
        private const val triadFlags = ParticleType.elasticParticle

        /**
         * All particle types that require computing depth
         */
        private const val noPressureFlags = ParticleType.powderParticle
        const val xTruncBits = 12
        const val yTruncBits = 12
        const val tagBits = 8 * 4 - 1 /* sizeof(int) */
        const val yOffset = (1 shl yTruncBits - 1).toLong()
        const val yShift = tagBits - yTruncBits
        const val xShift = tagBits - yTruncBits - xTruncBits
        const val xScale = (1 shl xShift).toLong()
        val xOffset = xScale * (1 shl xTruncBits - 1)
        const val xMask = (1 shl xTruncBits) - 1
        const val yMask = (1 shl yTruncBits) - 1
        fun computeTag(x: Float, y: Float): Long {
            return ((y + yOffset).toLong() shl yShift) + (xScale * x).toLong() + xOffset
        }

        fun computeRelativeTag(tag: Long, x: Int, y: Int): Long {
            return tag + (y.toLong() shl yShift) + (x.toLong() shl xShift)
        }

        fun limitCapacity(capacity: Int, maxCount: Int): Int {
            return if (maxCount != 0 && capacity > maxCount) maxCount else capacity
        }

        // reallocate a buffer
        fun <T> reallocateBuffer(
            buffer: ParticleBuffer<T>, oldCapacity: Int,
            newCapacity: Int, deferred: Boolean
        ): Array<T?>? {
            assert(newCapacity > oldCapacity)
            return BufferUtils.reallocateBuffer(
                buffer.dataClass, buffer.data,
                buffer.userSuppliedCapacity, oldCapacity, newCapacity,
                deferred
            )
        }

        fun reallocateBuffer(
            buffer: ParticleBufferInt, oldCapacity: Int,
            newCapacity: Int, deferred: Boolean
        ): IntArray? {
            assert(newCapacity > oldCapacity)
            return BufferUtils.reallocateBuffer(
                buffer.data,
                buffer.userSuppliedCapacity, oldCapacity, newCapacity,
                deferred
            )
        }

        private fun lowerBound(ray: Array<Proxy?>, length: Int, tag: Long): Int {
            var left = 0
            var length = length
            var step: Int
            var curr: Int
            while (length > 0) {
                step = length / 2
                curr = left + step
                if (ray[curr]!!.tag < tag) {
                    left = curr + 1
                    length -= step + 1
                } else {
                    length = step
                }
            }
            return left
        }

        private fun upperBound(ray: Array<Proxy?>, length: Int, tag: Long): Int {
            var left = 0
            var length = length
            var step: Int
            var curr: Int
            while (length > 0) {
                step = length / 2
                curr = left + step
                if (ray[curr]!!.tag <= tag) {
                    left = curr + 1
                    length -= step + 1
                } else {
                    length = step
                }
            }
            return left
        }
    }

    init {
        timestamp = 0
        allParticleFlags = 0
        allGroupFlags = 0
        density = 1f
        inverseDensity = 1f
        gravityScale = 1f
        particleDiameter = 1f
        inverseDiameter = 1f
        squaredDiameter = 1f
        count = 0
        internalAllocatedCapacity = 0
        maxCount = 0
        proxyCount = 0
        proxyCapacity = 0
        contactCount = 0
        contactCapacity = 0
        bodyContactCount = 0
        bodyContactCapacity = 0
        pairCount = 0
        pairCapacity = 0
        triadCount = 0
        triadCapacity = 0
        groupCount = 0
        pressureStrength = 0.05f
        dampingStrength = 1.0f
        elasticStrength = 0.25f
        springStrength = 0.25f
        viscousStrength = 0.25f
        surfaceTensionStrengthA = 0.1f
        surfaceTensionStrengthB = 0.2f
        powderStrength = 0.5f
        ejectionStrength = 0.5f
        colorMixingStrength = 0.5f
        flagsBuffer = ParticleBufferInt()
        positionBuffer = ParticleBuffer(Vec2::class.java)
        velocityBuffer = ParticleBuffer(Vec2::class.java)
        colorBuffer = ParticleBuffer(ParticleColor::class.java)
        userDataBuffer = ParticleBuffer(Any::class.java)
    }
}
