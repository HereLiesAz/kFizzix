package com.hereliesaz.kfizzix.dynamics.contacts

import com.hereliesaz.kfizzix.collision.Manifold.ManifoldType
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Vec2

class ContactPositionConstraint {
    var localPoints: Array<Vec2> = Array(Settings.maxManifoldPoints) { Vec2() }
    val localNormal: Vec2 = Vec2()
    val localPoint: Vec2 = Vec2()
    var indexA: Int = 0
    var indexB: Int = 0
    var invMassA: Float = 0f
    var invMassB: Float = 0f
    val localCenterA: Vec2 = Vec2()
    val localCenterB: Vec2 = Vec2()
    var invIA: Float = 0f
    var invIB: Float = 0f
    var type: ManifoldType? = null
    var radiusA: Float = 0f
    var radiusB: Float = 0f
    var pointCount: Int = 0
}
