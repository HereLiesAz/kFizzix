
import org.gradle.jvm.toolchain.JavaLanguageVersion
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm") version "1.9.21" apply false
    id("org.jetbrains.kotlin.android") version "1.9.21" apply false
    id("com.android.application") version "8.2.1" apply false
    id("org.openjfx.javafxplugin") version "0.0.13" apply false
}

allprojects {
    group = "com.hereliesaz.kfizzix"
    version = "1.0.0-SNAPSHOT"


    repositories {
        mavenCentral()
        google()
    }
}

subprojects {
    if (project.name != "kfizzix-android-app") {
        apply(plugin = "java")
        apply(plugin = "org.jetbrains.kotlin.jvm")

        afterEvaluate {
            java {
                toolchain {
                    languageVersion.set(JavaLanguageVersion.of(17))
                }
            }
        }
    }
}

