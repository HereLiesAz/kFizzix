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

import com.hereliesaz.kbox2d.testbed.framework.*
import com.hereliesaz.kbox2d.testbed.framework.TestbedModel.ListItem
import com.hereliesaz.kbox2d.testbed.framework.TestbedSetting.SettingType
import javafx.collections.ObservableList
import javafx.geometry.Pos
import javafx.scene.control.*
import javafx.scene.layout.BorderPane
import javafx.scene.layout.HBox
import javafx.scene.layout.Pane
import javafx.scene.layout.VBox
import javax.swing.ComboBoxModel
import javax.swing.DefaultComboBoxModel
import javax.swing.event.ListDataEvent
import javax.swing.event.ListDataListener

/**
 * The testbed side panel. Facilitates test and setting changes.
 *
 * @author Daniel Murphy
 */
class TestbedSidePanel(private val model: TestbedModel,
                       private val controller: AbstractTestbedController) : BorderPane() {
    lateinit var tests: ComboBox<ListItem>
    private val pauseButton = Button("Pause")
    private val stepButton = Button("Step")
    private val resetButton = Button("Reset")
    private val quitButton = Button("Quit")
    var saveButton = Button("Save")
    var loadButton = Button("Load")
    private fun updateTests(model: ComboBoxModel<ListItem>) {
        val list = tests.itemsProperty().get()
        list.clear()
        for (i in 0 until model.size) {
            list.add(model.getElementAt(i) as ListItem)
        }
    }

    fun initComponents() {
        // setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        val settings = model.settings
        val top = VBox()
        // top.setLayout(new GridLayout(0, 1));
// top.setBorder(BorderFactory.createCompoundBorder(new
// EtchedBorder(EtchedBorder.LOWERED),
// BorderFactory.createEmptyBorder(10, 10, 10, 10)));
        val testList = model.comboModel as DefaultComboBoxModel<*>
        testList.addListDataListener(object : ListDataListener {
            override fun intervalRemoved(e: ListDataEvent) {
                updateTests(e.source as ComboBoxModel<ListItem>)
            }

            override fun intervalAdded(e: ListDataEvent) {
                updateTests(e.source as ComboBoxModel<ListItem>)
            }

            override fun contentsChanged(e: ListDataEvent) {
                updateTests(e.source as ComboBoxModel<ListItem>)
            }
        })
        tests = ComboBox()
        updateTests(testList as ComboBoxModel<ListItem>)
        tests.onAction = javafx.event.EventHandler { testSelected() }
        tests.cellFactory = javafx.util.Callback { param: ListView<ListItem?>? ->
            object : ListCell<ListItem?>() {
                override fun updateItem(item: ListItem?, empty: Boolean) {
                    super.updateItem(item, empty)
                    if (item != null) {
                        text = if (item.isCategory) item.category else item.test.testName
                        isDisable = item.isCategory
                    }
                }
            }
        }
        top.children.add(Label("Choose a test:"))
        top.children.add(tests)
        addSettings(top, settings, SettingType.DRAWING)
        setTop(top)
        val middle = VBox()
        // middle.setLayout(new GridLayout(0, 1));
// middle.setBorder(BorderFactory.createCompoundBorder(new
// EtchedBorder(EtchedBorder.LOWERED),
// BorderFactory.createEmptyBorder(5, 10, 5, 10)));
        addSettings(middle, settings, SettingType.ENGINE)
        center = middle
        pauseButton.alignment = Pos.CENTER
        stepButton.alignment = Pos.CENTER
        resetButton.alignment = Pos.CENTER
        saveButton.alignment = Pos.CENTER
        loadButton.alignment = Pos.CENTER
        quitButton.alignment = Pos.CENTER
        val buttonGroups = HBox()
        val buttons1 = VBox()
        buttons1.children.add(resetButton)
        val buttons2 = VBox()
        buttons2.children.add(pauseButton)
        buttons2.children.add(stepButton)
        val buttons3 = VBox()
        buttons3.children.add(saveButton)
        buttons3.children.add(loadButton)
        buttons3.children.add(quitButton)
        buttonGroups.children.add(buttons1)
        buttonGroups.children.add(buttons2)
        buttonGroups.children.add(buttons3)
        bottom = buttonGroups
    }

    protected fun testSelected() {
        val testNum = tests.selectionModel.selectedIndex
        controller.playTest(testNum)
    }

    fun addListeners() {
        pauseButton.onAction = javafx.event.EventHandler { e ->
            if (model.settings.pause) {
                model.settings.pause = false
                pauseButton.text = "Pause"
            } else {
                model.settings.pause = true
                pauseButton.text = "Resume"
            }
            model.panel.grabFocus()
        }
        stepButton.onAction = javafx.event.EventHandler { e ->
            model.settings.singleStep = true
            if (!model.settings.pause) {
                model.settings.pause = true
                pauseButton.text = "Resume"
            }
            model.panel.grabFocus()
        }
        resetButton.onAction = javafx.event.EventHandler { e -> controller.reset() }
        quitButton.onAction = javafx.event.EventHandler { e -> System.exit(0) }
        saveButton.onAction = javafx.event.EventHandler { e -> controller.save() }
        loadButton.onAction = javafx.event.EventHandler { e -> controller.load() }
    }

    private fun addSettings(argPanel: Pane, argSettings: TestbedSettings,
                            argIgnore: SettingType) {
        for (setting in argSettings.settings) {
            if (setting.settingsType == argIgnore) {
                continue
            }
            when (setting.constraintType) {
                TestbedSetting.ConstraintType.RANGE -> {
                    val text = Label(setting.name + ": " + setting.value)
                    val slider = Slider(setting.min.toDouble(), setting.max.toDouble(),
                            setting.value.toDouble())
                    // slider.setMaximumSize(new Dimension(200, 20));
                    slider.valueProperty()
                            .addListener { prop, oldValue, newValue -> stateChanged(slider) }
                    putClientProperty(slider, "name", setting.name)
                    putClientProperty(slider, SETTING_TAG, setting)
                    putClientProperty(slider, LABEL_TAG, text)
                    argPanel.children.add(text)
                    argPanel.children.add(slider)
                }
                TestbedSetting.ConstraintType.BOOLEAN -> {
                    val checkbox = CheckBox(setting.name)
                    checkbox.isSelected = setting.enabled
                    checkbox.selectedProperty()
                            .addListener { prop, oldValue, newValue -> stateChanged(checkbox) }
                    putClientProperty(checkbox, SETTING_TAG, setting)
                    argPanel.children.add(checkbox)
                }
            }
        }
    }

    private fun <T> getClientProperty(control: Control, tag: String): T? {
        val map = control.userData as Map<String, Any>?
        return if (map != null) map[tag] as T? else null
    }

    private fun putClientProperty(control: Control, tag: String, o: Any) {
        var map = control.userData as MutableMap<String, Any>?
        if (map == null) {
            map = HashMap()
            control.userData = map
        }
        map[tag] = o
    }

    fun stateChanged(control: Control) {
        val setting = getClientProperty<TestbedSetting>(control, SETTING_TAG)!!
        when (setting.constraintType) {
            TestbedSetting.ConstraintType.BOOLEAN -> {
                val box = control as CheckBox
                setting.enabled = box.isSelected
            }
            TestbedSetting.ConstraintType.RANGE -> {
                val slider = control as Slider
                setting.value = slider.value.toInt()
                val label = getClientProperty<Label>(slider, LABEL_TAG)!!
                label.text = setting.name + ": " + setting.value
            }
        }
        model.panel.grabFocus()
    }

    companion object {
        private const val SETTING_TAG = "settings"
        private const val LABEL_TAG = "label"
    }

    init {
        model.addTestChangeListener { argTest, argIndex ->
            tests.selectionModel.select(argIndex)
            saveButton.isDisable = !argTest.isSaveLoadEnabled
            loadButton.isDisable = !argTest.isSaveLoadEnabled
        }
    }
}
