package com.hereliesaz.kfizzix.showcase

import android.os.Bundle
import android.view.View
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.FrameLayout
import android.widget.Spinner
import androidx.appcompat.app.AppCompatActivity
import com.hereliesaz.kfizzix.showcase.tests.*

class MainActivity : AppCompatActivity() {

    private lateinit var testbedView: TestbedView
    private lateinit var spinner: Spinner

    private val tests by lazy {
         listOf<Test>(
             HelloWorldTest(),
             ShapesTest(),
             JointsTest(),
             ParticlesTest(),
             CollisionTest(),
             DominoTest()
         )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Setup layout programmatically
        val container = FrameLayout(this)
        testbedView = TestbedView(this)
        container.addView(testbedView)

        spinner = Spinner(this)
        // Add spinner on top
        val params = FrameLayout.LayoutParams(
            FrameLayout.LayoutParams.WRAP_CONTENT,
            FrameLayout.LayoutParams.WRAP_CONTENT
        )
        container.addView(spinner, params)

        setContentView(container)

        setupSpinner()
    }

    private fun setupSpinner() {
        val adapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, tests.map { it.getTestName() })
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        spinner.adapter = adapter

        spinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
             override fun onItemSelected(parent: AdapterView<*>?, view: View?, position: Int, id: Long) {
                 if (tests.isNotEmpty()) {
                     testbedView.currentTest = tests[position]
                 }
             }

             override fun onNothingSelected(parent: AdapterView<*>?) {}
        }

        // Load first test
        if (tests.isNotEmpty()) {
            testbedView.currentTest = tests[0]
        }
    }
}
