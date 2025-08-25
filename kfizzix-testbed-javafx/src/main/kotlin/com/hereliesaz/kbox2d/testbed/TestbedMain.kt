/*
 * Copyright (c) 2013, Daniel Murphy All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kbox2d.testbed

import com.hereliesaz.kbox2d.testbed.framework.TestList
import com.hereliesaz.kbox2d.testbed.framework.AbstractTestbedController.MouseBehavior
import com.hereliesaz.kbox2d.testbed.framework.AbstractTestbedController.UpdateBehavior
import com.hereliesaz.kbox2d.testbed.framework.javafx.DebugDrawJavaFX
import com.hereliesaz.kbox2d.testbed.framework.javafx.TestPanelJavaFX
import com.hereliesaz.kbox2d.testbed.framework.javafx.TestbedControllerJavaFX
import com.hereliesaz.kbox2d.testbed.framework.javafx.TestbedSidePanel
import com.hereliesaz.kbox2d.testbed.framework.TestbedModel
import javafx.application.Application
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.control.Alert
import javafx.scene.control.ScrollPane
import javafx.scene.layout.BorderPane
import javafx.stage.Stage

/**
 * The entry point for the testbed application.
 * This class sets up the JavaFX application and initializes the testbed.
 *
 * @author Daniel Murphy
 */
class TestbedMain : Application() {
    /**
     * The main entry point for the JavaFX application.
     * This method is called after the application has been launched.
     *
     * @param primaryStage the primary stage for this application
     */
    override fun start(primaryStage: Stage) {
        val model = TestbedModel()
        val controller = TestbedControllerJavaFX(
            model, UpdateBehavior.UPDATE_CALLED, MouseBehavior.NORMAL
        ) { _, _ ->
            Alert(Alert.AlertType.ERROR).showAndWait()
        }
        val testbed = BorderPane()
        val panel = TestPanelJavaFX(model, controller, testbed)
        model.panel = panel
        model.debugDraw = DebugDrawJavaFX(panel, true)
        TestList.populateModel(model)
        testbed.center = panel
        testbed.right = ScrollPane(TestbedSidePanel(model, controller))
        val scene = Scene(
            testbed, (TestPanelJavaFX.INIT_WIDTH + 175).toDouble(),
            TestPanelJavaFX.INIT_HEIGHT.toDouble()
        )
        primaryStage.scene = scene
        primaryStage.title = "kbox2d Testbed"
        primaryStage.show()
        println(System.getProperty("java.home"))
        Platform.runLater {
            controller.playTest(0)
            controller.start()
        }
    }

    companion object {
        /**
         * The main entry point for the application.
         *
         * @param args the command line arguments
         */
        @JvmStatic
        fun main(args: Array<String>) {
            launch(TestbedMain::class.java, *args)
        }
    }
}
