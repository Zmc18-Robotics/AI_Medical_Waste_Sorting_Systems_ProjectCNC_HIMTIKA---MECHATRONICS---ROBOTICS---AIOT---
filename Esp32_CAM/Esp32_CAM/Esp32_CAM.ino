// ============================================
// ESP32-CAM : Streaming TCP + Kontrol Senter via WebSocket
// Board: AI-THINKER ESP32-CAM
// ============================================
#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsServer.h>   // Library: WebSockets by Markus Sattler
#include <ArduinoJson.h>        // Library: ArduinoJson by Benoit Blanchon

// ─── KONFIGURASI WIFI ───────────────────────────────────
#define WIFI_SSID   ""
#define WIFI_PASS   ""

// ─── PIN KAMERA (AI-THINKER) ────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── PIN SENTER ─────────────────────────────────────────
#define PIN_FLASH 4

// ─── OBJEK SERVER ───────────────────────────────────────
WiFiServer       tcpServer(80);
WebSocketsServer wsServer(81);

bool flashOn = false;  // default OFF — dikontrol dari Python (tekan F) atau web dashboard

// ─── DEKLARASI FUNGSI ───────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void setFlash(bool on);

// ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-CAM START ===");

  // Konfigurasi kamera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera gagal: 0x%x\n", err);
    return;
  }
  Serial.println("Kamera OK");

  // ── Senter default OFF setelah camera init ───────────────
  // GPIO 4 dipakai bersama SD card driver, jadi harus di-set
  // ulang setelah esp_camera_init() agar tidak di-override
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);   // ← OFF secara default
  flashOn = false;
  Serial.println("Senter OFF (default). Kontrol via Python key F / web dashboard.");

  // Koneksi WiFi
  // ── KITA MATIKAN IP STATIS AGAR DAPAT IP OTOMATIS (DHCP) DARI HOTSPOT ──
  // IPAddress staticIP(192, 168, 4, 3); 
  // IPAddress gateway(192, 168, 4, 1);
  // IPAddress subnet(255, 255, 255, 0);
  // WiFi.config(staticIP, gateway, subnet);

  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  Serial.print("IP ESP32-CAM: ");
  Serial.println(WiFi.localIP());

  // Mulai server TCP (streaming gambar)
  tcpServer.begin();

  // Mulai server WebSocket (kontrol senter)
  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
  Serial.println("Server siap (TCP:80, WS:81)");
}

// ────────────────────────────────────────────────────────
void loop() {
  // WebSocket selalu diproses dulu
  wsServer.loop();

  // --- Tangani client TCP (streaming gambar) ---
  WiFiClient client = tcpServer.accept();
  if (client) {
    Serial.println("[TCP] Client terhubung");
    client.setNoDelay(true);

    while (client.connected()) {
      // Proses WebSocket di dalam loop streaming juga
      wsServer.loop();

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        delay(10);
        continue;
      }

      uint32_t len = fb->len;
      uint8_t lenBuf[4] = {
        (uint8_t)((len >> 24) & 0xFF),
        (uint8_t)((len >> 16) & 0xFF),
        (uint8_t)((len >>  8) & 0xFF),
        (uint8_t)( len        & 0xFF)
      };

      bool ok = (client.write(lenBuf, 4) == 4) &&
                (client.write(fb->buf, len) == len);

      esp_camera_fb_return(fb);

      if (!ok) break;

      // Tambahkan sedikit delay (30ms) untuk memberi nafas pada WiFi 
      // dan mencegah ESP32-CAM overheat / nge-lag
      delay(30);
    }

    Serial.println("[TCP] Client putus");
    client.stop();
  }
}

// ────────────────────────────────────────────────────────
// Event WebSocket (menerima perintah senter dari Python)
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client #%d terhubung\n", num);
    // Kirim status flash saat ini ke client yang baru connect
    String status = "{\"type\":\"flash_status\",\"state\":" + String(flashOn ? 1 : 0) + "}";
    wsServer.sendTXT(num, status);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client #%d putus\n", num);
    return;
  }
  if (type != WStype_TEXT) return;

  // Parse JSON
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("[WS] JSON gagal: ");
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "flash") == 0) {
    // Jika ada field "state": set ke nilai tersebut
    // Jika tidak ada (hanya {"cmd":"flash"}): toggle
    if (doc.containsKey("state")) {
      int s = doc["state"] | 0;
      setFlash(s != 0);
    } else {
      setFlash(!flashOn);  // toggle
    }
  }
}

// ────────────────────────────────────────────────────────
// Set / broadcast status senter
void setFlash(bool on) {
  flashOn = on;
  digitalWrite(PIN_FLASH, flashOn ? HIGH : LOW);
  Serial.printf("[Senter] %s\n", flashOn ? "ON" : "OFF");
  // Broadcast ke semua WS client
  String status = "{\"type\":\"flash_status\",\"state\":" + String(flashOn ? 1 : 0) + "}";
  wsServer.broadcastTXT(status);
}
