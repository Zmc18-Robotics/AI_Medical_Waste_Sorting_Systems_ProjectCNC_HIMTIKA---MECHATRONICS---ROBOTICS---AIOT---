// ============================================
// ESP32 Biasa : Output LED via TCP
// ============================================
#include <WiFi.h>

const char* ssid = "Absolute Solver";
const char* password = "CynIsTheCutestBotEver333";
const int LED_PIN = 2;   // LED bawaan banyak board ESP32

WiFiServer server(8080);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED mati awal

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung");
  Serial.print("IP ESP32 Output: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server TCP siap di port 8080");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client terhubung (Python)");
    while (client.connected()) {
      if (client.available()) {
        String perintah = client.readStringUntil('\n');
        perintah.trim();  // buang whitespace
        Serial.print("Perintah diterima: ");
        Serial.println(perintah);

        if (perintah == "LED_ON") {
          digitalWrite(LED_PIN, HIGH);
          Serial.println("LED ON");
        } else if (perintah == "LED_OFF") {
          digitalWrite(LED_PIN, LOW);
          Serial.println("LED OFF");
        } else {
          Serial.println("Perintah tidak dikenal");
        }
      }
    }
    Serial.println("Client terputus");
    client.stop();
  }
}