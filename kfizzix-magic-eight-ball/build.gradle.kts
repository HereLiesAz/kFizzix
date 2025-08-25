import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    application
    id("org.openjfx.javafxplugin")
}

javafx {
    version = "17.0.2"
    modules("javafx.controls", "javafx.graphics")
}

dependencies {
    implementation(project(":kfizzix-library"))
    implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8")
}

application {
    mainClass.set("com.hereliesaz.kfizzix.magic8ball.Magic8BallMain")
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "17"
}
