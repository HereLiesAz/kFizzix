import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm")
    application
}

repositories {
    mavenCentral()
}

dependencies {
    implementation(kotlin("stdlib"))
    implementation(project(":kfizzix-library"))
    implementation(project(":kfizzix-serialization-kt"))
    implementation("org.apache.commons:commons-math3:3.6.1")
    implementation("org.openjfx:javafx-controls:17.0.2")
    implementation("org.openjfx:javafx-fxml:17.0.2")
}

application {
    mainClass.set("com.hereliesaz.kfizzix.testbed.javafx.TestbedMain")
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "17"
}
