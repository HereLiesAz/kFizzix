import com.google.protobuf.gradle.*
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {

    kotlin("jvm")
    id("com.google.protobuf") version "0.9.4"
}

repositories {
    mavenCentral()
}

dependencies {
    implementation(kotlin("stdlib"))
    implementation("com.google.protobuf:protobuf-java:3.25.4")
    implementation("com.google.protobuf:protobuf-kotlin:3.25.4")
    implementation(project(":kfizzix-library"))
}

protobuf {
    protoc {
        artifact = "com.google.protobuf:protoc:3.25.4"
    }
}

kotlin {
    jvmToolchain(17)
}
