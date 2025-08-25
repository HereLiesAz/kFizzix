package com.hereliesaz.kbox2d.testbed.javafx

import java.util.HashMap

class DoubleArray {
    private val map: MutableMap<Int, DoubleArray> = HashMap()
    operator fun get(argLength: Int): DoubleArray {
        assert(argLength > 0)
        if (!map.containsKey(argLength)) {
            map[argLength] = getInitializedArray(argLength)
        }
        assert(map[argLength]!!.size == argLength) { "Array not built of correct length" }
        return map[argLength]!!
    }

    protected fun getInitializedArray(argLength: Int): DoubleArray {
        return DoubleArray(argLength)
    }
}
