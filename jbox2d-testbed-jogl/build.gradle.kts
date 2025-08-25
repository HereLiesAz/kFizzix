import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm")
    application
}

application {
    mainClass.set("de.pirckheimer_gymnasium.jbox2d.testbed.framework.jogl.JoglTestbedMain")
}

dependencies {
    implementation(project(":kfizzix-library"))
    implementation("de.pirckheimer-gymnasium:jbox2d-serialization:3.1.0")
    implementation("de.pirckheimer-gymnasium:jbox2d-testbed:3.1.0")
    implementation("org.jogamp.gluegen:gluegen-rt-main:2.1.2")
    implementation("org.jogamp.jogl:jogl-all-main:2.1.2")
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "17"
}
