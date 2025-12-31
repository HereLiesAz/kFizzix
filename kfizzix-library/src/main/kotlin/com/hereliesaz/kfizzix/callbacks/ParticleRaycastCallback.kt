package com.hereliesaz.kfizzix.callbacks

import com.hereliesaz.kfizzix.common.Vec2

interface ParticleRaycastCallback {
    /**
     * Called for each particle found in the query. You control how the ray cast
     * proceeds by returning a float:
     * return -1: ignore this particle and continue
     * return 0: terminate the ray cast
     * return fraction: clip the ray to this point
     * return 1: don't clip the ray and continue
     * @param index the particle index
     * @param point the point of initial intersection
     * @param normal the normal vector at the point of intersection
     * @param fraction the fraction along the ray at the point of intersection
     * @return -1 to filter, 0 to terminate, fraction to clip the ray for
     * closest hit, 1 to continue
     */
    fun reportParticle(index: Int, point: Vec2, normal: Vec2, fraction: Float): Float
}
