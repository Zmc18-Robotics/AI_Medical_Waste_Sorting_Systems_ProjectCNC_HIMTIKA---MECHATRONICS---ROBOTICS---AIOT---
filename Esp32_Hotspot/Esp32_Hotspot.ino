#include <WiFi.h>

// Nama WiFi (SSID) dan Password disesuaikan dengan yang ada di ESP32-CAM
const char *ssid = "";
const char *password = "";

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 HOST / HOTSPOT ===");

  // Mengatur ESP32 agar berfungsi sebagai Access Point (AP) saja
  WiFi.mode(WIFI_AP);
  
  Serial.println("Mengkonfigurasi Access Point...");
  
  // Memulai Access Point dengan SSID dan Password
  bool success = WiFi.softAP(ssid, password);
  
  if(success) {
    Serial.println("Hotspot Berhasil Dibuat!");
    
    // Menampilkan IP Address dari Hotspot (Default biasanya 192.168.4.1)
    IPAddress IP = WiFi.softAPIP();
    Serial.print("IP Address Hotspot (Jalan Raya): ");
    Serial.println(IP);
    Serial.println("Siap menerima koneksi dari ESP32-CAM dan perangkat lain...");
  } else {
    Serial.println("Gagal membuat Hotspot!");
  }
}

void loop() {
  // Mengecek dan menampilkan jumlah perangkat yang terhubung
  static int last_stations = -1;
  int current_stations = WiFi.softAPgetStationNum();
  
  if (current_stations != last_stations) {
    Serial.print("Jumlah perangkat terhubung: ");
    Serial.println(current_stations);
    last_stations = current_stations;
  }
  
  delay(1000); // Delay agar loop tidak berjalan terlalu cepat
}
