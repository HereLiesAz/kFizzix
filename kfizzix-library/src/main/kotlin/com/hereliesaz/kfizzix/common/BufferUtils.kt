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
package com.hereliesaz.kfizzix.common

import java.lang.reflect.Array

/**
 * @author Daniel Murphy
 */
object BufferUtils {
    /**
     * Reallocate a buffer.
     */
    fun <T> reallocateBuffer(
        klass: Class<T>, oldBuffer: Array<T>?,
        oldCapacity: Int, newCapacity: Int
    ): Array<T> {
        assert(newCapacity > oldCapacity)
        val newBuffer = Array.newInstance(klass, newCapacity) as Array<T>
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity)
        }
        for (i in oldCapacity until newCapacity) {
            try {
                newBuffer[i] = klass.getDeclaredConstructor().newInstance()
            } catch (e: Exception) {
                throw RuntimeException(e)
            }
        }
        return newBuffer
    }

    /**
     * Reallocate a buffer.
     */
    fun reallocateBuffer(
        oldBuffer: IntArray?, oldCapacity: Int,
        newCapacity: Int
    ): IntArray {
        assert(newCapacity > oldCapacity)
        val newBuffer = IntArray(newCapacity)
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity)
        }
        return newBuffer
    }

    /**
     * Reallocate a buffer.
     */
    fun reallocateBuffer(
        oldBuffer: FloatArray?, oldCapacity: Int,
        newCapacity: Int
    ): FloatArray {
        assert(newCapacity > oldCapacity)
        val newBuffer = FloatArray(newCapacity)
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity)
        }
        return newBuffer
    }

    /**
     * Reallocate a buffer. A 'deferred' buffer is reallocated only if it is not
     * NULL. If 'userSuppliedCapacity' is not zero, buffer is user supplied and
     * must be kept.
     */
    fun <T> reallocateBuffer(
        klass: Class<T>, buffer: Array<T>?,
        userSuppliedCapacity: Int, oldCapacity: Int, newCapacity: Int,
        deferred: Boolean
    ): Array<T>? {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(klass, buffer, oldCapacity, newCapacity)
        }
        return buffer
    }

    /**
     * Reallocate an int buffer. A 'deferred' buffer is reallocated only if it
     * is not NULL. If 'userSuppliedCapacity' is not zero, buffer is user
     * supplied and must be kept.
     */
    fun reallocateBuffer(
        buffer: IntArray?, userSuppliedCapacity: Int,
        oldCapacity: Int, newCapacity: Int, deferred: Boolean
    ): IntArray? {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(buffer, oldCapacity, newCapacity)
        }
        return buffer
    }

    /**
     * Reallocate a float buffer. A 'deferred' buffer is reallocated only if it
     * is not NULL. If 'userSuppliedCapacity' is not zero, buffer is user
     * supplied and must be kept.
     */
    fun reallocateBuffer(
        buffer: FloatArray?,
        userSuppliedCapacity: Int, oldCapacity: Int, newCapacity: Int,
        deferred: Boolean
    ): FloatArray? {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(buffer, oldCapacity, newCapacity)
        }
        return buffer
    }

    /**
     * Rotate an array, see std::rotate
     */
    fun <T> rotate(ray: Array<T>, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }

    /**
     * Rotate an array, see std::rotate
     */
    fun rotate(ray: IntArray, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }

    /**
     * Rotate an array, see std::rotate
     */
    fun rotate(ray: FloatArray, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }
}
