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
package com.hereliesaz.kbox2d.serialization

import com.hereliesaz.kbox2d.collision.shapes.Shape
import com.hereliesaz.kbox2d.dynamics.Body
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.dynamics.World
import com.hereliesaz.kbox2d.dynamics.joints.Joint
import java.io.IOException
import java.io.InputStream

/**
 * Deserializer for kbox2d, used to deserialize any aspect of the physics world.
 *
 * @author Daniel Murphy
 */
interface JbDeserializer {
    /**
     * Sets the object listener, which allows the user to process each physics
     * object with a tag to do any sort of custom logic.
     *
     * @param listener the listener to use
     */
    fun setObjectListener(listener: ObjectListener)

    /**
     * Sets a listener for unsupported exceptions. If a listener is provided, the
     * deserializer will call the listener with the exception instead of throwing it.
     * This allows the deserialization process to continue even if some objects are not supported.
     *
     * @param listener the listener to use
     */
    fun setUnsupportedListener(listener: UnsupportedListener)

    /**
     * Deserializes a [World] from the given [InputStream].
     *
     * @param input the input stream to read from
     * @return the deserialized world
     * @throws IOException if an I/O error occurs
     * @throws UnsupportedObjectException if a read physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeWorld(input: InputStream): World

    /**
     * Deserializes a [Body] from the given [InputStream] and adds it to the given [World].
     *
     * @param world the world to add the body to
     * @param input the input stream to read from
     * @return the deserialized body
     * @throws IOException if an I/O error occurs
     * @throws UnsupportedObjectException if a read physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeBody(world: World, input: InputStream): Body

    /**
     * Deserializes a [Fixture] from the given [InputStream] and adds it to the given [Body].
     *
     * @param body the body to add the fixture to
     * @param input the input stream to read from
     * @return the deserialized fixture
     * @throws IOException if an I/O error occurs
     * @throws UnsupportedObjectException if a read physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeFixture(body: Body, input: InputStream): Fixture

    /**
     * Deserializes a [Shape] from the given [InputStream].
     *
     * @param input the input stream to read from
     * @return the deserialized shape
     * @throws IOException if an I/O error occurs
     * @throws UnsupportedObjectException if a read physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeShape(input: InputStream): Shape

    /**
     * Deserializes a [Joint] from the given [InputStream] and adds it to the given [World].
     *
     * @param world the world to add the joint to
     * @param input the input stream to read from
     * @param bodyMap a map from body indices to bodies
     * @param jointMap a map from joint indices to joints
     * @return the deserialized joint
     * @throws IOException if an I/O error occurs
     * @throws UnsupportedObjectException if a read physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeJoint(
        world: World, input: InputStream,
        bodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>
    ): Joint

    /**
     * Called for each physics object with a tag defined.
     *
     * @author Daniel Murphy
     */
    interface ObjectListener {
        /**
         * Called when a world is processed.
         * @param world the world
         * @param tag the tag associated with the world, or null if none
         */
        fun processWorld(world: World, tag: Long?)

        /**
         * Called when a body is processed.
         * @param body the body
         * @param tag the tag associated with the body, or null if none
         */
        fun processBody(body: Body, tag: Long?)

        /**
         * Called when a fixture is processed.
         * @param fixture the fixture
         * @param tag the tag associated with the fixture, or null if none
         */
        fun processFixture(fixture: Fixture, tag: Long?)

        /**
         * Called when a shape is processed.
         * @param shape the shape
         * @param tag the tag associated with the shape, or null if none
         */
        fun processShape(shape: Shape, tag: Long?)

        /**
         * Called when a joint is processed.
         * @param joint the joint
         * @param tag the tag associated with the joint, or null if none
         */
        fun processJoint(joint: Joint, tag: Long?)
    }
}
