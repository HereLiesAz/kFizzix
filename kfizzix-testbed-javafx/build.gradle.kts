import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    application
    id("org.openjfx.javafxplugin")
}

javafx {
    version = "17.0.2"
    modules("javafx.controls")
}

dependencies {
    implementation("org.apache.commons:commons-math3:3.6.1")
    implementation(project(":kfizzix-library"))
    implementation(project(":kfizzix-serialization-kt"))
    implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8")
}

application {
    mainClass.set("com.hereliesaz.kfizzix.testbed.TestbedMain")
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "11"
}
