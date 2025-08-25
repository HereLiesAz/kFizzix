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
package com.hereliesaz.kbox2d.testbed.javafx

import com.hereliesaz.kbox2d.testbed.framework.AbstractTestbedController
import com.hereliesaz.kbox2d.testbed.framework.TestbedModel
import com.hereliesaz.kbox2d.testbed.framework.TestbedPanel
import javafx.application.Platform
import javafx.beans.value.ChangeListener
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.input.MouseButton
import javafx.scene.layout.BorderPane
import javafx.scene.paint.Color
import javafx.scene.text.Font
import org.slf4j.LoggerFactory

/**
 * @author Daniel Murphy
 */
class TestPanelJavaFX : Canvas, TestbedPanel {
    private val controller: AbstractTestbedController

    constructor(model: TestbedModel,
                controller: AbstractTestbedController, parent: BorderPane) : this(model, controller) {
        // bind canvas size to parent
        widthProperty().bind(parent.widthProperty().subtract(175))
        heightProperty().bind(parent.heightProperty())
    }

    constructor(model: TestbedModel,
                controller: AbstractTestbedController) : super(INIT_WIDTH.toDouble(), INIT_HEIGHT.toDouble()) {
        this.controller = controller
        updateSize(INIT_WIDTH.toDouble(), INIT_HEIGHT.toDouble())
        JavaFXPanelHelper.addHelpAndPanelListeners(this, model, controller,
                SCREEN_DRAG_BUTTON)
        val sizeListener = ChangeListener<Number> { prop, oldValue, newValue -> updateSize(width, height) }
        widthProperty().addListener(sizeListener)
        heightProperty().addListener(sizeListener)
        graphicsContext2D.font = Font("Courier New", 12.0)
    }

    override fun maxWidth(height: Double): Double {
        return Double.MAX_VALUE
    }

    override fun maxHeight(width: Double): Double {
        return Double.MAX_VALUE
    }

    val dbGraphics: GraphicsContext
        get() = graphicsContext2D

    private fun updateSize(width: Double, height: Double) {
        controller.updateExtents(width.toFloat() / 2, height.toFloat() / 2)
    }

    fun render(): Boolean {
        val dbg = dbGraphics
        dbg.fill = Color.BLACK
        val bounds = boundsInLocal
        dbg.fillRect(bounds.minX, bounds.minX, bounds.width,
                bounds.height)
        return true
    }

    fun paintScreen() {}
    override fun grabFocus() {
        Platform.runLater { requestFocus() }
    }

    companion object {
        private val log = LoggerFactory.getLogger(TestPanelJavaFX::class.java)
        val SCREEN_DRAG_BUTTON = MouseButton.SECONDARY
                .ordinal
        const val INIT_WIDTH = 600
        const val INIT_HEIGHT = 600
    }
}
