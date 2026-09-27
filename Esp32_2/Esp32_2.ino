/*
 * ============================================================
 *   SMART FACTORY - SAFETY SENSOR NODE (ESP32 KE-2)
 *   Platform  : ESP32 DevKit C v4
 *   Sensor    : Flame Sensor (DOUT), Ultrasonik x4 (HC-SR04)
 *   Author    : Mc.Zminecrafter18
 *
 *   Fungsi:
 *   - Deteksi api via Flame Sensor (DOUT Active LOW, Instant Trigger)
 *   - Ultrasonik 1: Deteksi limbah anomali (tidak tersortir, 1-15 cm)
 *   - Ultrasonik 2: Konfirmasi limbah berhasil tersortir (3 pintu, 1-10 cm)
 *   - Kirim data JSON ke ESP32 ke-1 via UART Serial2 (GPIO 17 TX)
 *
 *   ★ WIRING WAJIB KE ESP32 KE-1:
 *     ESP32-2 TX (GPIO 17) ──── ESP32-1 RX (GPIO 13)
 *     ESP32-2 RX (GPIO 16) ──── ESP32-1 TX (GPIO 25)
 *     ESP32-2 GND          ──── ESP32-1 GND   (Common Ground!)
 *
 *   ★ WIRING SENSOR KE ESP32 KE-2:
 *     1. Flame Sensor : VCC -> 3.3V/5V, GND -> GND, DOUT -> GPIO 13
 *     2. US1 (Anomali): VCC -> 5V (VIN), GND -> GND, TRIG -> GPIO 25, ECHO -> GPIO 26
 *     3. US2 (Sortir) : VCC -> 5V (VIN), GND -> GND, TRIG bersama -> GPIO 14
 *                       ECHO Pintu A (Servo 1) -> GPIO 32
 *                       ECHO Pintu B (Servo 2) -> GPIO 33
 *                       ECHO Pintu C (Servo 3) -> GPIO 35
 * ============================================================
 */

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_FLAME_DOUT   13  // Digital In (Active LOW = api terdeteksi)
#define PIN_US1_TRIG     25  // Ultrasonik 1: TRIG (Anomali)
#define PIN_US1_ECHO     26  // Ultrasonik 1: ECHO
#define PIN_US2_TRIG     14  // Ultrasonik 2: TRIG bersama (Sortir 3 Pintu)
#define PIN_US2_ECHO_A   32  // Ultrasonik 2: ECHO Pintu A – Servo 1 / Infeksius
#define PIN_US2_ECHO_B   33  // Ultrasonik 2: ECHO Pintu B – Servo 2 / Non-Infeksius
#define PIN_US2_ECHO_C   35  // Ultrasonik 2: ECHO Pintu C – Servo 3 / B3
#define PIN_UART2_TX     17  // UART TX → RX GPIO13 ESP32 ke-1
#define PIN_UART2_RX     16  // UART RX ← TX GPIO25 ESP32 ke-1

// ── Konfigurasi & Ambang Batas ───────────────────────────────
#define FLAME_READ_INTERVAL     50   // ms – pembacaan responsif 20x per detik
#define FLAME_HOLD_TIME_MS    1500   // ms – tahan alert api minimal 1.5 detik agar tidak flicker
#define US_READ_INTERVAL       150   // ms – baca ultrasonik
#define UART_SEND_INTERVAL     200   // ms – kirim JSON via UART 5x per detik
#define DEBUG_INTERVAL         800   // ms – Serial Monitor debug
#define US1_THRESHOLD_CM        20   // cm – batas deteksi anomali (1-20 cm)
#define US2_THRESHOLD_CM        12   // cm – batas konfirmasi sortir (1-15 cm)
#define US_MIN_CM                1   // cm – batas bawah validitas
#define US_TIMEOUT_US        25000   // µs – timeout pulseIn standar HC-SR04 (~4.3m)

// ── State Variabel ────────────────────────────────────────────
bool     flameAlert       = false;
uint32_t flameLastTrigger = 0;
int      flameRawVal      = 1;

bool     us1Active        = false;
uint8_t  us2Door          = 0;       // Pintu US2 aktif: 0=none, 1=A, 2=B, 3=C

float    lastDist1        = 999.0f;
float    lastDistA        = 999.0f;
float    lastDistB        = 999.0f;
float    lastDistC        = 999.0f;

uint32_t flameLastRead    = 0;
uint32_t usLastRead       = 0;
uint32_t uartLastSend     = 0;
uint32_t lastDebug        = 0;

// ── Forward Declarations ──────────────────────────────────────
void handleFlameSensor(uint32_t now);
void handleUltrasonics(uint32_t now);
void handleIncomingUART(uint32_t now);
void sendUART(uint32_t now);
float measureCm(uint8_t trigPin, uint8_t echoPin);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // UART2 ke ESP32 ke-1 (Baud 9600 8N1, RX=16, TX=17)
  Serial2.begin(9600, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);

  // Flame sensor pin (Active LOW)
  pinMode(PIN_FLAME_DOUT, INPUT_PULLUP);

  // Ultrasonik 1 (Anomali)
  pinMode(PIN_US1_TRIG,   OUTPUT);
  pinMode(PIN_US1_ECHO,   INPUT);
  digitalWrite(PIN_US1_TRIG, LOW);

  // Ultrasonik 2 (Sortir 3 Pintu)
  pinMode(PIN_US2_TRIG,   OUTPUT);
  pinMode(PIN_US2_ECHO_A, INPUT);
  pinMode(PIN_US2_ECHO_B, INPUT);
  pinMode(PIN_US2_ECHO_C, INPUT);
  digitalWrite(PIN_US2_TRIG, LOW);

  Serial.println("\n========================================================");
  Serial.println("       ESP32 KE-2 : SAFETY SENSOR NODE v2.5            ");
  Serial.println("========================================================");
  Serial.printf("  Flame Sensor DOUT : GPIO %d (Active LOW)\n", PIN_FLAME_DOUT);
  Serial.printf("  US1 Anomali       : TRIG=%d, ECHO=%d (Max %d cm)\n", PIN_US1_TRIG, PIN_US1_ECHO, US1_THRESHOLD_CM);
  Serial.printf("  US2 Sortir        : TRIG=%d | ECHO A=%d, B=%d, C=%d (Max %d cm)\n",
                PIN_US2_TRIG, PIN_US2_ECHO_A, PIN_US2_ECHO_B, PIN_US2_ECHO_C, US2_THRESHOLD_CM);
  Serial.printf("  UART Serial2      : TX=GPIO %d -> ESP32-1 RX13 | RX=GPIO %d\n", PIN_UART2_TX, PIN_UART2_RX);
  Serial.println("========================================================\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  handleFlameSensor(now);
  handleUltrasonics(now);
  handleIncomingUART(now);
  sendUART(now);

  // Serial Monitor live telemetri setiap 800ms
  if (now - lastDebug >= DEBUG_INTERVAL) {
    lastDebug = now;
    Serial.println("--- [ESP32-2 TELEMETRI LIVE] ---");
    Serial.printf("  🔥 Flame DOUT (GPIO 13): Raw=%d | Alert=%s\n",
                  flameRawVal, flameAlert ? "!! BAHAYA (API) !!" : "AMAN");
    Serial.printf("  📏 US1 Anomali (GPIO 26): %.1f cm | Status: %s\n",
                  lastDist1, us1Active ? "TERDETEKSI ANOMALI" : "Normal");
    Serial.printf("  🚪 US2 Sortir (Pintu A) : %.1f cm | (Pintu B): %.1f cm | (Pintu C): %.1f cm\n",
                  lastDistA, lastDistB, lastDistC);
    Serial.printf("  🎯 Pintu Aktif          : %s\n",
                  (us2Door == 1) ? "Pintu 1 (Infeksius)" :
                  (us2Door == 2) ? "Pintu 2 (Non-Infeksius)" :
                  (us2Door == 3) ? "Pintu 3 (B3)" : "Tidak ada");
    Serial.printf("  📤 UART TX JSON         : {\"f\":%d,\"u1\":%d,\"u2\":%d,\"d1\":%.1f,\"dA\":%.1f,\"dB\":%.1f,\"dC\":%.1f,\"up\":%lu}\n",
                  flameAlert ? 1 : 0, us1Active ? 1 : 0, us2Door,
                  lastDist1, lastDistA, lastDistB, lastDistC, (unsigned long)(now / 1000));
    Serial.println("--------------------------------\n");
  }
}

// ============================================================
//  FLAME SENSOR (INSTANT TRIGGER + HOLD TIME)
//  Active LOW: Sensor menghasilkan LOW (0) saat ada api
// ============================================================
void handleFlameSensor(uint32_t now) {
  if (now - flameLastRead < FLAME_READ_INTERVAL) return;
  flameLastRead = now;

  flameRawVal = digitalRead(PIN_FLAME_DOUT);

  // Trigger instan jika pin membaca LOW (0)
  if (flameRawVal == LOW) {
    flameAlert = true;
    flameLastTrigger = now;
  } else {
    // Tahan status alert minimal FLAME_HOLD_TIME_MS agar tidak hilang sesaat
    if (flameAlert && (now - flameLastTrigger >= FLAME_HOLD_TIME_MS)) {
      flameAlert = false;
    }
  }
}

// ============================================================
//  PENGUKURAN JARAK ULTRASONIK (HC-SR04)
// ============================================================
float measureCm(uint8_t trigPin, uint8_t echoPin) {
  // Pulsa trigger 10us
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Baca durasi echo dengan timeout pendek (10ms ~1.7 meter)
  unsigned long dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (dur == 0 || dur > US_TIMEOUT_US) return 999.0f; // Timeout

  float cm = (float)dur / 58.0f;
  if (cm < (float)US_MIN_CM || cm > 200.0f) return 999.0f;
  return cm;
}

void handleUltrasonics(uint32_t now) {
  if (now - usLastRead < US_READ_INTERVAL) return;
  usLastRead = now;

  // 1. Baca US1 (Anomali)
  lastDist1 = measureCm(PIN_US1_TRIG, PIN_US1_ECHO);
  us1Active = (lastDist1 >= (float)US_MIN_CM && lastDist1 <= (float)US1_THRESHOLD_CM);

  // Jeda kecil sebelum membaca sensor berikutnya
  delay(15);

  // 2. Baca US2 Pintu A (Servo 1)
  lastDistA = measureCm(PIN_US2_TRIG, PIN_US2_ECHO_A);
  delay(20); // Jeda akustik pantulan gelombang suara

  // 3. Baca US2 Pintu B (Servo 2)
  lastDistB = measureCm(PIN_US2_TRIG, PIN_US2_ECHO_B);
  delay(20); // Jeda akustik

  // 4. Baca US2 Pintu C (Servo 3)
  lastDistC = measureCm(PIN_US2_TRIG, PIN_US2_ECHO_C);

  // Evaluasi pintu yang mendeteksi objek
  bool aOk = (lastDistA >= (float)US_MIN_CM && lastDistA <= (float)US2_THRESHOLD_CM);
  bool bOk = (lastDistB >= (float)US_MIN_CM && lastDistB <= (float)US2_THRESHOLD_CM);
  bool cOk = (lastDistC >= (float)US_MIN_CM && lastDistC <= (float)US2_THRESHOLD_CM);

  if      (aOk) us2Door = 1;
  else if (bOk) us2Door = 2;
  else if (cOk) us2Door = 3;
  else          us2Door = 0;
}

// ============================================================
//  TERIMA PERINTAH PROBE / PING DARI ESP32 KE-1 (UART RX)
// ============================================================
String rxCmdBuffer = "";
void handleIncomingUART(uint32_t now) {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      rxCmdBuffer.trim();
      if (rxCmdBuffer.length() > 0) {
        Serial.printf("[UART RX<-ESP32-1] Perintah: %s\n", rxCmdBuffer.c_str());
        // Balas instan PONG
        Serial2.printf("{\"pong\":1,\"f\":%d,\"u1\":%d,\"u2\":%d,\"d1\":%.1f,\"dA\":%.1f,\"dB\":%.1f,\"dC\":%.1f,\"up\":%lu}\n",
                       flameAlert ? 1 : 0,
                       us1Active  ? 1 : 0,
                       (int)us2Door,
                       lastDist1, lastDistA, lastDistB, lastDistC,
                       (unsigned long)(now / 1000));
      }
      rxCmdBuffer = "";
    } else if (c != '\r' && rxCmdBuffer.length() < 64) {
      rxCmdBuffer += c;
    }
  }
}

// ============================================================
//  KIRIM DATA TELEMETRI SECARA KONTINYU KE ESP32 KE-1
// ============================================================
void sendUART(uint32_t now) {
  if (now - uartLastSend < UART_SEND_INTERVAL) return;
  uartLastSend = now;

  Serial2.printf("{\"f\":%d,\"u1\":%d,\"u2\":%d,\"d1\":%.1f,\"dA\":%.1f,\"dB\":%.1f,\"dC\":%.1f,\"up\":%lu}\n",
                 flameAlert ? 1 : 0,
                 us1Active  ? 1 : 0,
                 (int)us2Door,
                 lastDist1, lastDistA, lastDistB, lastDistC,
                 (unsigned long)(now / 1000));
}
