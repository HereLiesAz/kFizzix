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
package com.hereliesaz.kfizzix.collision

/**
 * Contact ids to facilitate warm starting.
 * @author Daniel Murphy
 */
class ContactID : Comparable<ContactID> {
    var indexA: Byte = 0
    var indexB: Byte = 0
    var typeA: Byte = 0
    var typeB: Byte = 0

    fun set(c: ContactID) {
        indexA = c.indexA
        indexB = c.indexB
        typeA = c.typeA
        typeB = c.typeB
    }

    fun zero() {
        indexA = 0
        indexB = 0
        typeA = 0
        typeB = 0
    }

    fun flip() {
        val tempIndex = indexA
        indexA = indexB
        indexB = tempIndex
        val tempType = typeA
        typeA = typeB
        typeB = tempType
    }

    fun copy(): ContactID {
        val c = ContactID()
        c.set(this)
        return c
    }

    fun isEqual(other: ContactID): Boolean {
        return indexA == other.indexA && indexB == other.indexB && typeA == other.typeA && typeB == other.typeB
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as ContactID
        return isEqual(other)
    }

    override fun hashCode(): Int {
        var result = indexA.toInt()
        result = 31 * result + indexB.toInt()
        result = 31 * result + typeA.toInt()
        result = 31 * result + typeB.toInt()
        return result
    }

    override fun toString(): String {
        return "ContactID(indexA=$indexA, indexB=$indexB, typeA=$typeA, typeB=$typeB)"
    }

    override fun compareTo(other: ContactID): Int {
        if (indexA != other.indexA) return indexA - other.indexA
        if (indexB != other.indexB) return indexB - other.indexB
        if (typeA != other.typeA) return typeA - other.typeA
        return typeB - other.typeB
    }
}
