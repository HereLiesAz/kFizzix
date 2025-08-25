package com.hereliesaz.kbox2d.testbed.javafx

import com.google.common.collect.Lists
import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.testbed.framework.AbstractTestbedController
import com.hereliesaz.kbox2d.testbed.framework.TestbedCamera.ZoomType
import com.hereliesaz.kbox2d.testbed.framework.TestbedModel
import com.hereliesaz.kbox2d.testbed.framework.TestbedTest
import javafx.scene.Node
import javafx.scene.input.KeyCode
import javafx.scene.input.MouseButton
import javafx.scene.input.MouseEvent

object JavaFXPanelHelper {
    var screenDragButtonDown = false
    var mouseJointButtonDown = false

    /**
     * Adds common help text and listeners for awt-based testbeds.
     */
    fun addHelpAndPanelListeners(panel: Node,
                                 model: TestbedModel,
                                 controller: AbstractTestbedController,
                                 screenDragButton: Int) {
        val oldDragMouse = Vec2()
        val mouse = Vec2()
        val help = Lists.newArrayList<String>()
        help.add("Click and drag the left mouse button to move objects.")
        help.add("Click and drag the right mouse button to move the view.")
        help.add("Shift-Click to aim a bullet, or press space.")
        help.add("Scroll to zoom in/out on the mouse position")
        help.add("Press '[' or ']' to change tests, and 'r' to restart.")
        model.setImplSpecificHelp(help)
        panel.setOnZoom { zoomEvent ->
            val currTest = model.currTest ?: return@setOnZoom
            val zoom = if (zoomEvent.zoomFactor > 1) ZoomType.ZOOM_IN else ZoomType.ZOOM_OUT
            currTest.camera.zoomToPoint(mouse, zoom)
        }
        panel.setOnScroll { scrollEvent ->
            val currTest = model.currTest
            if (currTest == null || scrollEvent.deltaY == 0.0) {
                return@setOnScroll
            }
            val zoom = if (scrollEvent.deltaY > 1) ZoomType.ZOOM_IN else ZoomType.ZOOM_OUT
            currTest.camera.zoomToPoint(mouse, zoom)
        }
        panel.setOnMouseReleased { mouseEvent ->
            if (toInt(mouseEvent.button) == screenDragButton) {
                screenDragButtonDown = false
            } else if (model.codedKeys[toInt(KeyCode.SHIFT)]
                    && !mouseJointButtonDown) {
                controller.queueMouseUp(toVec(mouseEvent),
                        TestbedTest.BOMB_SPAWN_BUTTON)
            } else {
                if (toInt(mouseEvent
                                .button) == TestbedTest.MOUSE_JOINT_BUTTON) {
                    mouseJointButtonDown = false
                }
                controller.queueMouseUp(
                        Vec2(mouseEvent.x.toFloat(),
                                mouseEvent.y.toFloat()),
                        toInt(mouseEvent.button))
            }
        }
        panel.setOnMousePressed { mouseEvent ->
            if (toInt(mouseEvent.button) == screenDragButton) {
                screenDragButtonDown = true
                oldDragMouse.set(toVec(mouseEvent))
                return@setOnMousePressed
            } else if (model.codedKeys[toInt(KeyCode.SHIFT)]) {
                controller.queueMouseDown(toVec(mouseEvent),
                        TestbedTest.BOMB_SPAWN_BUTTON)
            } else {
                if (toInt(mouseEvent
                                .button) == TestbedTest.MOUSE_JOINT_BUTTON) {
                    mouseJointButtonDown = true
                }
                controller.queueMouseDown(toVec(mouseEvent),
                        toInt(mouseEvent.button))
            }
        }
        panel.setOnMouseMoved { mouseEvent ->
            mouse.set(toVec(mouseEvent))
            controller.queueMouseMove(toVec(mouseEvent))
        }
        panel.setOnMouseDragged { mouseEvent ->
            mouse.set(toVec(mouseEvent))
            if (screenDragButtonDown) {
                val currTest = model.currTest ?: return@setOnMouseDragged
                val diff = oldDragMouse.sub(mouse)
                currTest.camera.moveWorld(diff)
                oldDragMouse.set(mouse)
            } else if (mouseJointButtonDown) {
                controller.queueMouseDrag(Vec2(mouse),
                        TestbedTest.MOUSE_JOINT_BUTTON)
            } else if (model.codedKeys[toInt(KeyCode.SHIFT)]) {
                controller.queueMouseDrag(toVec(mouseEvent),
                        TestbedTest.BOMB_SPAWN_BUTTON)
            } else {
                controller.queueMouseDrag(toVec(mouseEvent),
                        toInt(mouseEvent.button))
            }
        }
        panel.setOnKeyReleased { keyEvent ->
            val keyName = keyEvent.text
            val c = if (keyName.length == 1) keyName[0] else '\u0000'
            controller.queueKeyReleased(c, toInt(keyEvent.code))
        }
        panel.setOnKeyPressed { keyEvent ->
            val keyName = keyEvent.text
            val c = if (keyName.length == 1) keyName[0] else '\u0000'
            controller.queueKeyPressed(c, toInt(keyEvent.code))
            when (c) {
                '[' -> controller.lastTest()
                ']' -> controller.nextTest()
                'r' -> controller.reset()
                ' ' -> controller.queueLaunchBomb()
                'p' -> controller.queuePause()
            }
        }
    }

    private fun toInt(key: KeyCode): Int {
        return key.ordinal
    }

    private fun toInt(button: MouseButton): Int {
        return button.ordinal
    }

    private fun toVec(mouseEvent: MouseEvent): Vec2 {
        return Vec2(mouseEvent.x.toFloat(), mouseEvent.y.toFloat())
    }
}
