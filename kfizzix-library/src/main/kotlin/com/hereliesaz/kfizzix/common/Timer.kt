package com.hereliesaz.kfizzix.common

class Timer {
    private var resetNanos: Long = 0

    init {
        reset()
    }

    fun reset() {
        resetNanos = System.nanoTime()
    }

    val milliseconds: Float
        get() = (System.nanoTime() - resetNanos).toFloat() / 1000 / 1000
}
