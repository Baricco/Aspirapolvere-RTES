package it.maurice.maurice

import android.content.Context
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel

// Canale "maurice/wifi_network" (docs/wifi.md, sezione 10.2): sulla rete del robot, che non ha internet, Android
// manderebbe le richieste sui dati mobili. bindToWifi lega le richieste dell'app alla rete Wi-Fi di adesso, unbind le
// rimette sulla rete scelta dal sistema.
class MainActivity : FlutterActivity() {
    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        val connectivity = getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, "maurice/wifi_network").setMethodCallHandler { call, result ->
            when (call.method) {
                "bindToWifi" -> {
                    val wifi = connectivity.allNetworks.firstOrNull { network ->
                        connectivity.getNetworkCapabilities(network)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
                    }
                    connectivity.bindProcessToNetwork(wifi)
                    result.success(wifi != null)
                }
                "unbind" -> {
                    connectivity.bindProcessToNetwork(null)
                    result.success(null)
                }
                else -> result.notImplemented()
            }
        }
    }
}
