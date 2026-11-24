package com.example.gravediggerrc

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.webkit.JavascriptInterface
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/**
 * Pont Bluetooth Classic (SPP / RFCOMM) entre la page web et l'ESP32.
 *
 * - Cherche parmi les appareils DÉJÀ ASSOCIÉS celui qui s'appelle [deviceName].
 * - Se connecte tout seul et se reconnecte automatiquement si la liaison tombe.
 * - La page web appelle AndroidBridge.send("J,vitesse,direction,boutons\n").
 *   Si le Bluetooth est occupé, seule la trame la PLUS RÉCENTE est envoyée
 *   (pas d'accumulation = latence minimale).
 */
@SuppressLint("MissingPermission")
class BluetoothBridge(
    context: Context,
    private val deviceName: String,
    private val onStatus: (String) -> Unit,
    private val onRx: (String) -> Unit
) {
    private val sppUuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    private val adapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private val writer = Executors.newSingleThreadExecutor()
    private val running = AtomicBoolean(false)
    private val sending = AtomicBoolean(false)
    private val latest = AtomicReference<String?>(null)

    @Volatile private var socket: BluetoothSocket? = null
    @Volatile private var out: OutputStream? = null
    @Volatile private var status: String = "Bluetooth : démarrage…"
    private var worker: Thread? = null

    fun setStatus(s: String) {
        status = s
        onStatus(s)
    }

    @JavascriptInterface
    fun getStatus(): String = status

    /** Démarre la boucle de connexion (une seule fois). */
    fun start() {
        if (!running.compareAndSet(false, true)) return
        val t = Thread { connectionLoop() }
        t.isDaemon = true
        t.start()
        worker = t
    }

    fun stop() {
        running.set(false)
        closeSocket()
        writer.shutdownNow()
        worker?.interrupt()
    }

    /** Appelé par la page web (thread de fond) : envoie une trame texte. */
    @JavascriptInterface
    fun send(message: String) {
        latest.set(message)
        if (sending.compareAndSet(false, true)) {
            try {
                writer.execute { drain() }
            } catch (e: Exception) {
                sending.set(false)
            }
        }
    }

    private fun drain() {
        try {
            while (true) {
                val m = latest.getAndSet(null) ?: break
                val o = out ?: continue // pas connecté : la trame est abandonnée
                try {
                    o.write(m.toByteArray(Charsets.US_ASCII))
                    o.flush()
                } catch (e: IOException) {
                    closeSocket()
                }
            }
        } finally {
            sending.set(false)
        }
    }

    private fun connectionLoop() {
        val ad = adapter
        if (ad == null) {
            setStatus("Pas de Bluetooth sur ce téléphone")
            return
        }
        while (running.get()) {
            try {
                if (!ad.isEnabled) {
                    setStatus("Active le Bluetooth du téléphone")
                    Thread.sleep(1500)
                    continue
                }
                val device = ad.bondedDevices.firstOrNull {
                    (it.name ?: "").equals(deviceName, ignoreCase = true)
                }
                if (device == null) {
                    setStatus("Associe « $deviceName » dans Réglages > Bluetooth")
                    Thread.sleep(2000)
                    continue
                }
                setStatus("Connexion à $deviceName…")
                ad.cancelDiscovery()
                val s = device.createRfcommSocketToServiceRecord(sppUuid)
                socket = s
                s.connect() // bloque jusqu'à la connexion ou l'échec
                out = s.outputStream
                setStatus("Connecté à $deviceName")
                readLoop(s) // bloque tant que la liaison est vivante
                setStatus("Liaison perdue — reconnexion…")
            } catch (e: InterruptedException) {
                break
            } catch (e: SecurityException) {
                setStatus("Permission Bluetooth manquante")
                try { Thread.sleep(2000) } catch (ie: InterruptedException) { break }
            } catch (e: IOException) {
                setStatus("Liaison indisponible — nouvel essai…")
            }
            closeSocket()
            try { Thread.sleep(1000) } catch (ie: InterruptedException) { break }
        }
        closeSocket()
    }

    /** Lit ce que l'ESP32 envoie (télémétrie "T,nom,valeur") et détecte la déconnexion. */
    private fun readLoop(s: BluetoothSocket) {
        val input = s.inputStream
        val buf = ByteArray(128)
        val line = StringBuilder()
        while (running.get()) {
            val n = input.read(buf) // IOException si la liaison tombe
            if (n < 0) return
            for (i in 0 until n) {
                val c = buf[i].toInt().toChar()
                if (c == '\n') {
                    if (line.isNotEmpty()) onRx(line.toString().trim())
                    line.setLength(0)
                } else if (line.length < 100) {
                    line.append(c)
                }
            }
        }
    }

    private fun closeSocket() {
        out = null
        try {
            socket?.close()
        } catch (e: IOException) {
            // rien à faire
        }
        socket = null
    }
}
