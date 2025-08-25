package com.hereliesaz.kfizzix.magic8ball

import javafx.application.Application
import javafx.scene.Scene
import javafx.scene.control.Label
import javafx.scene.layout.StackPane
import javafx.stage.Stage

class Magic8BallMain : Application() {
    override fun start(primaryStage: Stage) {
        primaryStage.title = "KFizzix Magic 8-Ball"
        val root = StackPane()
        root.children.add(Label("Hello, Magic 8-Ball!"))
        primaryStage.scene = Scene(root, 800.0, 600.0)
        primaryStage.show()
    }
}

fun main(args: Array<String>) {
    Application.launch(Magic8BallMain::class.java, *args)
}
