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

rootProject.name = "KFizzix"

include(
    ":kfizzix-library",
    ":kfizzix-serialization-kt",
    ":kfizzix-testbed-javafx",
    ":kfizzix-testbed-javafx-kt",
    ":jbox2d-testbed-jogl",
    ":kfizzix-magic-eight-ball"
)
