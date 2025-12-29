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
package com.hereliesaz.kfizzix.collision

import com.hereliesaz.kfizzix.collision.Distance.DistanceProxy
import com.hereliesaz.kfizzix.common.Transform

/**
 * Input for Distance. You have to option to use the shape radii in the
 * computation.
 */
class DistanceInput(
    var proxyA: DistanceProxy = DistanceProxy(),
    var proxyB: DistanceProxy = DistanceProxy(),
    var transformA: Transform = Transform(),
    var transformB: Transform = Transform(),
    var useRadii: Boolean = false
) {
    override fun toString(): String {
        return "DistanceInput(proxyA=$proxyA, proxyB=$proxyB, transformA=$transformA, transformB=$transformB, useRadii=$useRadii)"
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as DistanceInput
        if (proxyA != other.proxyA) return false
        if (proxyB != other.proxyB) return false
        if (transformA != other.transformA) return false
        if (transformB != other.transformB) return false
        if (useRadii != other.useRadii) return false
        return true
    }

    override fun hashCode(): Int {
        var result = proxyA.hashCode()
        result = 31 * result + proxyB.hashCode()
        result = 31 * result + transformA.hashCode()
        result = 31 * result + transformB.hashCode()
        result = 31 * result + useRadii.hashCode()
        return result
    }
}
