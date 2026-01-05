pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        google()
        maven {
            url = uri("https://plugins.gradle.org/m2/")
        }
    }
}

plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "0.8.0"
}

rootProject.name = "KFizzix"

include(
    ":kfizzix-library",
    ":kfizzix-serialization-kt",
    //":kfizzix-testbed-javafx-kt",
    //":jbox2d-testbed-jogl",
    //":kfizzix-magic-eight-ball",
    //":kfizzix-8-ball",
    //":kfizzix-dam-breaker"
)
