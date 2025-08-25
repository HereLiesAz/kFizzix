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

import java.io.Serializable

/**
 * A transform contains translation and rotation. It is used to represent the
 * position and orientation of rigid frames.
 *
 * @param position the position of the transform
 * @param rotation the rotation of the transform
 * @constructor Creates a new transform with the given position and rotation.
 * @author Daniel Murphy
 */
data class Transform(
    /**
     * The translation caused by the transform
     */
    @JvmField val p: Vec2 = Vec2(),
    /**
     * A matrix representing a rotation
     */
    @JvmField val q: Rot = Rot()
) : Serializable {

    /**
     * Initialize using a position vector and a rotation matrix.
     *
     * @param _position the position of the transform
     * @param _R the rotation of the transform
     */
    constructor(position: Vec2, rotation: Rot) : this(position.clone(), rotation.clone())

    /**
     * Set this to equal another transform.
     *
     * @param xf the transform to copy from
     * @return this transform for chaining
     */
    fun set(xf: Transform): Transform {
        p.set(xf.p)
        q.set(xf.q)
        return this
    }

    /**
     * Set this based on the position and angle.
     *
     * @param p the position
     * @param angle the angle in radians
     */
    fun set(p: Vec2, angle: Float) {
        this.p.set(p)
        q.set(angle)
    }

    /** Set this to the identity transform. */
    fun setIdentity() {
        p.setZero()
        q.setIdentity()
    }

    override fun toString(): String {
        return "XForm:\nPosition: $p\nR: \n$q\n"
    }

    companion object {
        private const val serialVersionUID = 1L

        /**
         * Multiplies a transform by a vector.
         *
         * @param T the transform
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mul(T: Transform, v: Vec2): Vec2 {
            return Vec2((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y)
        }

        /**
         * Multiplies a transform by a vector and stores the result in the output vector.
         *
         * @param T the transform
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOut(T: Transform, v: Vec2, out: Vec2) {
            val tempY = (T.q.s * v.x + T.q.c * v.y) + T.p.y
            out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x
            out.y = tempY
        }

        /**
         * Multiplies a transform by a vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param T the transform
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOutUnsafe(T: Transform, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x
            out.y = (T.q.s * v.x + T.q.c * v.y) + T.p.y
        }

        /**
         * Multiplies the transpose of a transform by a vector.
         *
         * @param T the transform to transpose
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mulTrans(T: Transform, v: Vec2): Vec2 {
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            return Vec2((T.q.c * px + T.q.s * py), (-T.q.s * px + T.q.c * py))
        }

        /**
         * Multiplies the transpose of a transform by a vector and stores the result in the output vector.
         *
         * @param T the transform to transpose
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulTransToOut(T: Transform, v: Vec2, out: Vec2) {
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            val tempY = (-T.q.s * px + T.q.c * py)
            out.x = (T.q.c * px + T.q.s * py)
            out.y = tempY
        }

        /**
         * Multiplies the transpose of a transform by a vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param T the transform to transpose
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulTransToOutUnsafe(T: Transform, v: Vec2, out: Vec2) {
            assert(v !== out)
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            out.x = (T.q.c * px + T.q.s * py)
            out.y = (-T.q.s * px + T.q.c * py)
        }

        /**
         * Multiplies two transforms.
         *
         * @param A the first transform
         * @param B the second transform
         * @return a new transform containing the result
         */
        @JvmStatic
        fun mul(A: Transform, B: Transform): Transform {
            val C = Transform()
            Rot.mulUnsafe(A.q, B.q, C.q)
            Rot.mulToOutUnsafe(A.q, B.p, C.p)
            C.p.addLocal(A.p)
            return C
        }

        /**
         * Multiplies two transforms and stores the result in the output transform.
         *
         * @param A the first transform
         * @param B the second transform
         * @param out the transform to store the result in
         */
        @JvmStatic
        fun mulToOut(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            Rot.mul(A.q, B.q, out.q)
            Rot.mulToOut(A.q, B.p, out.p)
            out.p.addLocal(A.p)
        }

        /**
         * Multiplies two transforms and stores the result in the output transform.
         * This is an unsafe version that assumes the output transform is not the same as the input transforms.
         *
         * @param A the first transform
         * @param B the second transform
         * @param out the transform to store the result in
         */
        @JvmStatic
        fun mulToOutUnsafe(A: Transform, B: Transform, out: Transform) {
            assert(out !== B)
            assert(out !== A)
            Rot.mulUnsafe(A.q, B.q, out.q)
            Rot.mulToOutUnsafe(A.q, B.p, out.p)
            out.p.addLocal(A.p)
        }

        private val pool = Vec2()

        /**
         * Multiplies the transpose of a transform by another transform.
         *
         * @param A the first transform (will be transposed)
         * @param B the second transform
         * @return a new transform containing the result
         */
        @JvmStatic
        fun mulTrans(A: Transform, B: Transform): Transform {
            val C = Transform()
            Rot.mulTransUnsafe(A.q, B.q, C.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTransUnsafe(A.q, pool, C.p)
            return C
        }

        /**
         * Multiplies the transpose of a transform by another transform and stores the result in the output transform.
         *
         * @param A the first transform (will be transposed)
         * @param B the second transform
         * @param out the transform to store the result in
         */
        @JvmStatic
        fun mulTransToOut(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            Rot.mulTrans(A.q, B.q, out.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTrans(A.q, pool, out.p)
        }

        /**
         * Multiplies the transpose of a transform by another transform and stores the result in the output transform.
         * This is an unsafe version that assumes the output transform is not the same as the input transforms.
         *
         * @param A the first transform (will be transposed)
         * @param B the second transform
         * @param out the transform to store the result in
         */
        @JvmStatic
        fun mulTransToOutUnsafe(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            assert(out !== B)
            Rot.mulTransUnsafe(A.q, B.q, out.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTransUnsafe(A.q, pool, out.p)
        }
    }
}
