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

/**
 * Similar to `javax.vecmath.Color3f` holder
 *
 * @author ewjordan
 */
class Color3f {
    var x = 0f
    var y = 0f
    var z = 0f

    constructor() {
        x = 0f
        y = 0f
        z = 0f
    }

    constructor(r: Float, g: Float, b: Float) {
        x = r
        y = g
        z = b
    }

    fun set(r: Float, g: Float, b: Float) {
        x = r
        y = g
        z = b
    }

    fun set(argColor: Color3f) {
        x = argColor.x
        y = argColor.y
        z = argColor.z
    }

    companion object {
        val WHITE = Color3f(1f, 1f, 1f)
        val BLACK = Color3f(0f, 0f, 0f)
        val BLUE = Color3f(0f, 0f, 1f)
        val GREEN = Color3f(0f, 1f, 0f)
        val RED = Color3f(1f, 0f, 0f)
    }
}
