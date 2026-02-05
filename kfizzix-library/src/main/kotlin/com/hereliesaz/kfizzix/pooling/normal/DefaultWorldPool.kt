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
package com.hereliesaz.kfizzix.pooling.normal

import com.hereliesaz.kfizzix.collision.AABB
import com.hereliesaz.kfizzix.collision.Collision
import com.hereliesaz.kfizzix.collision.Distance
import com.hereliesaz.kfizzix.collision.TimeOfImpact
import com.hereliesaz.kfizzix.common.Mat22
import com.hereliesaz.kfizzix.common.Mat33
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.common.Vec3
import com.hereliesaz.kfizzix.dynamics.contacts.ChainAndCircleContact
import com.hereliesaz.kfizzix.dynamics.contacts.ChainAndPolygonContact
import com.hereliesaz.kfizzix.dynamics.contacts.CircleContact
import com.hereliesaz.kfizzix.dynamics.contacts.Contact
import com.hereliesaz.kfizzix.dynamics.contacts.EdgeAndCircleContact
import com.hereliesaz.kfizzix.dynamics.contacts.EdgeAndPolygonContact
import com.hereliesaz.kfizzix.dynamics.contacts.PolygonAndCircleContact
import com.hereliesaz.kfizzix.dynamics.contacts.PolygonContact
import com.hereliesaz.kfizzix.pooling.DynamicStack
import com.hereliesaz.kfizzix.pooling.WorldPool
import com.hereliesaz.kfizzix.pooling.arrays.FloatArrayPool
import com.hereliesaz.kfizzix.pooling.arrays.IntArrayPool

/**
 * Provides object pooling for all objects used in the engine. Objects retrieved
 * from here should only be used temporarily, and then pushed back (except *
 * arrays).
 *
 * @author Daniel Murphy
 */
class DefaultWorldPool(argSize: Int, argContainerSize: Int) : WorldPool {
    private val vecs: OrderedStack<Vec2>
    private val vec3s: OrderedStack<Vec3>
    private val mats: OrderedStack<Mat22>
    private val mat33s: OrderedStack<Mat33>
    private val aabbs: OrderedStack<AABB>
    private val rots: OrderedStack<Rot>
    private val afloats = HashMap<Int, FloatArray>()
    private val aints = HashMap<Int, IntArray>()
    private val avecs = HashMap<Int, Array<Vec2>>()
    private val world: WorldPool = this

    private val pcstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return PolygonContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            return Array(size) { newInstance() }
        }
    }

    private val ccstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return CircleContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            return Array(size) { newInstance() }
        }
    }

    private val cpstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return PolygonAndCircleContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            @Suppress("UNCHECKED_CAST")
            return arrayOfNulls<Contact>(size) as Array<Contact>
        }
    }

    private val ecstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return EdgeAndCircleContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            @Suppress("UNCHECKED_CAST")
            return arrayOfNulls<Contact>(size) as Array<Contact>
        }
    }

    private val epstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return EdgeAndPolygonContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            return Array(size) { newInstance() }
        }
    }

    private val chcstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return ChainAndCircleContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            return Array(size) { newInstance() }
        }
    }

    private val chpstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return ChainAndPolygonContact(world)
        }
        override fun newArray(size: Int): Array<Contact> {
            @Suppress("UNCHECKED_CAST")
            return arrayOfNulls<Contact>(size) as Array<Contact>
        }
    }

    private val _collision: Collision
    private val _timeOfImpact: TimeOfImpact
    private val _distance: Distance

    private val intArrayPool = IntArrayPool()
    private val floatArrayPool = FloatArrayPool()

    init {
        vecs = object : OrderedStack<Vec2>(argSize, argContainerSize) {
            override fun newInstance(): Vec2 {
                return Vec2()
            }
        }
        vec3s = object : OrderedStack<Vec3>(argSize, argContainerSize) {
            override fun newInstance(): Vec3 {
                return Vec3()
            }
        }
        mats = object : OrderedStack<Mat22>(argSize, argContainerSize) {
            override fun newInstance(): Mat22 {
                return Mat22()
            }
        }
        aabbs = object : OrderedStack<AABB>(argSize, argContainerSize) {
            override fun newInstance(): AABB {
                return AABB()
            }
        }
        rots = object : OrderedStack<Rot>(argSize, argContainerSize) {
            override fun newInstance(): Rot {
                return Rot()
            }
        }
        mat33s = object : OrderedStack<Mat33>(argSize, argContainerSize) {
            override fun newInstance(): Mat33 {
                return Mat33()
            }
        }
        _distance = Distance()
        _collision = Collision(this)
        _timeOfImpact = TimeOfImpact(this)
    }

    override fun getPolyContactStack(): DynamicStack<Contact> {
        return pcstack
    }

    override fun getCircleContactStack(): DynamicStack<Contact> {
        return ccstack
    }

    override fun getPolyCircleContactStack(): DynamicStack<Contact> {
        return cpstack
    }

    override fun getEdgeCircleContactStack(): DynamicStack<Contact> {
        return ecstack
    }

    override fun getEdgePolyContactStack(): DynamicStack<Contact> {
        return epstack
    }

    override fun getChainCircleContactStack(): DynamicStack<Contact> {
        return chcstack
    }

    override fun getChainPolyContactStack(): DynamicStack<Contact> {
        return chpstack
    }

    override fun popVec2(): Vec2 {
        return vecs.pop()
    }

    override fun popVec2(num: Int): Array<Vec2> {
        return vecs.pop(num)
    }

    override fun pushVec2(num: Int) {
        vecs.push(num)
    }

    override fun popVec3(): Vec3 {
        return vec3s.pop()
    }

    override fun popVec3(num: Int): Array<Vec3> {
        return vec3s.pop(num)
    }

    override fun pushVec3(num: Int) {
        vec3s.push(num)
    }

    override fun popMat22(): Mat22 {
        return mats.pop()
    }

    override fun popMat22(num: Int): Array<Mat22> {
        return mats.pop(num)
    }

    override fun pushMat22(num: Int) {
        mats.push(num)
    }

    override fun popMat33(): Mat33 {
        return mat33s.pop()
    }

    override fun pushMat33(num: Int) {
        mat33s.push(num)
    }

    override fun popAABB(): AABB {
        return aabbs.pop()
    }

    override fun popAABB(num: Int): Array<AABB> {
        return aabbs.pop(num)
    }

    override fun pushAABB(num: Int) {
        aabbs.push(num)
    }

    override fun popRot(): Rot {
        return rots.pop()
    }

    override fun pushRot(num: Int) {
        rots.push(num)
    }

    override fun getCollision(): Collision = _collision

    override fun getTimeOfImpact(): TimeOfImpact = _timeOfImpact

    override fun getDistance(): Distance = _distance

    override fun getFloatArray(argLength: Int): FloatArray {
        if (!afloats.containsKey(argLength)) {
            afloats[argLength] = floatArrayPool.get(argLength)
        }
        val array = afloats[argLength]!!
        assert(array.size == argLength) { "Array not built with correct length" }
        return array
    }

    override fun getIntArray(argLength: Int): IntArray {
        if (!aints.containsKey(argLength)) {
            aints[argLength] = intArrayPool.get(argLength)
        }
        val array = aints[argLength]!!
        assert(array.size == argLength) { "Array not built with correct length" }
        return array
    }

    override fun getVec2Array(argLength: Int): Array<Vec2> {
        if (!avecs.containsKey(argLength)) {
            val ray = Array(argLength) { Vec2() }
            avecs[argLength] = ray
        }
        assert(avecs[argLength]!!.size == argLength) { "Array not built with correct length" }
        return avecs[argLength]!!
    }
}
