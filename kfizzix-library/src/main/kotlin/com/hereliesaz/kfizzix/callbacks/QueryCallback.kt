package com.hereliesaz.kfizzix.callbacks

import com.hereliesaz.kfizzix.dynamics.Fixture

/**
 * Callback class for AABB queries.
 * See World.queryAABB
 */
interface QueryCallback {
    /**
     * Called for each fixture found in the query AABB.
     * @return false to terminate the query.
     */
    fun reportFixture(fixture: Fixture): Boolean
}
