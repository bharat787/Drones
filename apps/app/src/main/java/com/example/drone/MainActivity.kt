package com.example.drone

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.Toast
import androidx.databinding.DataBindingUtil
import com.example.drone.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
    }

    /** Called when the user taps the Send button */
    val hvr_text = "will add hover functionality"
    val sendTo_text = "will add sendTo functionality"
    val track_text = "inbound for future updates"
    val duration = Toast.LENGTH_SHORT

    //val hvr_btn : Button = findViewById(R.id.hover_button)

    fun hover_msg(view: View) {
        // Do something in response to button
        val toast = Toast.makeText(applicationContext, hvr_text, duration)
        toast.show()
        val intent = Intent(this, getHeight :: class.java)
        startActivity(intent)

    }

    fun sendTo_msg(view: View) {
        // Do something in response to button
        val toast = Toast.makeText(applicationContext, sendTo_text, duration)
        toast.show()
        val intent = Intent(this, MapsActivity :: class.java)
        startActivity(intent)
    }

    fun track_msg(view: View) {
        // Do something in response to button
        val toast = Toast.makeText(applicationContext, track_text, duration)
        toast.show()
    }



    /*  private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        //setContentView(R.layout.activity_main)

        binding = DataBindingUtil.setContentView(this, R.layout.activity_main)

        binding.hoverButton.setOnClickListener{
            // TODO add funtion
        }

        binding.sendToButton.setOnClickListener{
            // TODO add function
        }

        binding.trackButton.setOnClickListener{
            // TODO add function
        }

    }

        */
}