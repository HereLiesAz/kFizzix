package com.hereliesaz.kfizzix.common

/**
 * Marks APIs that are not thread-safe or have other usage constraints
 * and should be used with caution.
 */
@RequiresOptIn(
    level = RequiresOptIn.Level.WARNING,
    message = "This API is not thread-safe or has other constraints. Please read the documentation carefully before use."
)
@Retention(AnnotationRetention.BINARY)
@Target(AnnotationTarget.CLASS, AnnotationTarget.FUNCTION)
annotation class DelicateFizzixApi