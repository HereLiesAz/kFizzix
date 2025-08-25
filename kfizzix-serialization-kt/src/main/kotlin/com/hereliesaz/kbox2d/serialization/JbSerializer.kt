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

/**
 * Serializer for kbox2d, used to serialize any aspect of the physics world.
 *
 * @author Daniel Murphy
 */
interface JbSerializer {
    /**
     * Sets the object signer for the serializer. This allows the user to
     * specify a 'tag' for each main physics object, which is then referenced
     * later at deserialization for the user.
     *
     * @param signer the object signer to use
     */
    fun setObjectSigner(signer: ObjectSigner)

    /**
     * Sets a listener for unsupported exceptions. If a listener is provided, the
     * serializer will call the listener with the exception instead of throwing it.
     * This allows the serialization process to continue even if some objects are not supported.
     *
     * @param listener the listener to use
     */
    fun setUnsupportedListener(listener: UnsupportedListener)

    /**
     * Serializes the given [World].
     *
     * @param world the world to serialize
     * @return the result of the serialization
     * @throws UnsupportedObjectException if a physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(world: World): SerializationResult

    /**
     * Serializes a given [Body].
     *
     * @param body the body to serialize
     * @return the result of the serialization
     * @throws UnsupportedObjectException if a physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(body: Body): SerializationResult

    /**
     * Serializes a given [Fixture].
     *
     * @param fixture the fixture to serialize
     * @return the result of the serialization
     * @throws UnsupportedObjectException if a physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(fixture: Fixture): SerializationResult

    /**
     * Serializes a given [Shape].
     *
     * @param shape the shape to serialize
     * @return the result of the serialization
     * @throws UnsupportedObjectException if a physics object is unsupported and no listener is set.
     * @see [setUnsupportedListener]
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(shape: Shape): SerializationResult

    /**
     * Serializes a given [Joint].
     * Joints need to reference bodies and sometimes other joints, so this method requires maps
     * to look up the indices of these objects.
     *
     * @param joint the joint to serialize
     * @param bodyIndexMap a map from bodies to their indices
     * @param jointIndexMap a map from joints to their indices
     * @return the result of the serialization
     */
    fun serialize(joint: Joint, bodyIndexMap: Map<Body, Int>, jointIndexMap: Map<Joint, Int>): SerializationResult

    /**
     * Interface that allows the serializer to look up tags for each object,
     * which can be used later during deserializing by the developer.
     *
     * @author Daniel Murphy
     */
    interface ObjectSigner {
        /**
         * Gets the tag for the given [World].
         *
         * @param world the world
         * @return the tag for the world, or null if no tag is associated
         */
        fun getTag(world: World): Long?

        /**
         * Gets the tag for the given [Body].
         *
         * @param body the body
         * @return the tag for the body, or null if no tag is associated
         */
        fun getTag(body: Body): Long?

        /**
         * Gets the tag for the given [Shape].
         *
         * @param shape the shape
         * @return the tag for the shape, or null if no tag is associated
         */
        fun getTag(shape: Shape): Long?

        /**
         * Gets the tag for the given [Fixture].
         *
         * @param fixture the fixture
         * @return the tag for the fixture, or null if no tag is associated
         */
        fun getTag(fixture: Fixture): Long?

        /**
         * Gets the tag for the given [Joint].
         *
         * @param joint the joint
         * @return the tag for the joint, or null if no tag is associated
         */
        fun getTag(joint: Joint): Long?
    }
}
