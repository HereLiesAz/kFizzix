
import org.gradle.jvm.toolchain.JavaLanguageVersion
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm") version "1.9.21" apply false
    application
    id("org.openjfx.javafxplugin") version "0.0.13" apply false
}

allprojects {
    group = "com.hereliesaz.kfizzix"
    version = "1.0.0-SNAPSHOT"


    repositories {
        mavenCentral()
    }
}

subprojects {
    apply(plugin = "org.jetbrains.kotlin.jvm")

    java {
        toolchain {
            languageVersion.set(JavaLanguageVersion.of(17))
        }
    }
}

