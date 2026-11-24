package com.example.gravediggerrc

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.View
import android.view.WindowManager
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import org.json.JSONObject

class MainActivity : AppCompatActivity() {

    companion object {
        // Nom Bluetooth de l'ESP32 (identique à BT_NAME dans le firmware)
        const val DEVICE_NAME = "CamionRC"
    }

    private lateinit var webView: WebView
    private lateinit var bridge: BluetoothBridge

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) {
                bridge.start()
            } else {
                bridge.setStatus("Permission Bluetooth refusée (Réglages Android > Applications)")
            }
        }

    @SuppressLint("SetJavaScriptEnabled", "ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        hideSystemBars()

        webView = findViewById(R.id.webView)
        webView.webViewClient = WebViewClient()
        webView.settings.javaScriptEnabled = true
        webView.settings.domStorageEnabled = true
        webView.overScrollMode = View.OVER_SCROLL_NEVER
        webView.isHapticFeedbackEnabled = false
        webView.setOnLongClickListener { true } // pas de menu "copier" lors d'un appui long

        bridge = BluetoothBridge(
            context = this,
            deviceName = DEVICE_NAME,
            onStatus = { s ->
                runOnUiThread {
                    webView.evaluateJavascript(
                        "window.onBtStatus && window.onBtStatus(${JSONObject.quote(s)})", null
                    )
                }
            },
            onRx = { line ->
                runOnUiThread {
                    webView.evaluateJavascript(
                        "window.onBtRx && window.onBtRx(${JSONObject.quote(line)})", null
                    )
                }
            }
        )
        webView.addJavascriptInterface(bridge, "AndroidBridge")
        webView.loadUrl("file:///android_asset/index.html")

        startBluetooth()
    }

    private fun startBluetooth() {
        val needsRuntimePermission = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) !=
            PackageManager.PERMISSION_GRANTED
        if (needsRuntimePermission) {
            permissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            bridge.start()
        }
    }

    private fun hideSystemBars() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        val controller = WindowInsetsControllerCompat(window, window.decorView)
        controller.hide(WindowInsetsCompat.Type.systemBars())
        controller.systemBarsBehavior =
            WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) hideSystemBars()
    }

    override fun onPause() {
        // Sécurité : demande l'arrêt si l'appli passe en arrière-plan
        // (l'ESP32 s'arrête de toute façon seul après 300 ms sans trame).
        bridge.send("J,0,0,0\n")
        super.onPause()
    }

    override fun onDestroy() {
        bridge.stop()
        webView.destroy()
        super.onDestroy()
    }
}
