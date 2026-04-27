/*
 * ============================================================
 *   SMART FACTORY - CONVEYOR AUTOMATION SYSTEM
 *   Platform  : ESP32 DevKit C v4
 *   Author    : Mc.Zminecrafter18
 * ============================================================
 *
 *  PIN MAP:
 *  ┌─────────────────────┬──────────┬─────────────────────────┐
 *  │ Komponen            │ Pin ESP32│ Catatan                 │
 *  ├─────────────────────┼──────────┼─────────────────────────┤
 *  │ LCD I2C SDA         │ 21       │                         │
 *  │ LCD I2C SCL         │ 22       │                         │
 *  │ Ultrasonic TRIG     │  4       │                         │
 *  │ Ultrasonic ECHO     │  5       │                         │
 *  │ Buzzer              │ 32       │                         │
 *  │ Relay (Motor)       │ 19       │                         │
 *  │ Servo 1 (Kat.2)     │ 33       │                         │
 *  │ Servo 2 (Kat.3)     │ 25       │                         │
 *  │ Button              │ 18       │                         │
 *  │ MQ-2 AOUT           │ 26       │ Analog                  │
 *  │ MQ-2 DOUT           │ 27       │ tidak dipakai           │
 *  │ Water Level AOUT    │ 14       │ Analog (ADC1_CH6)       │
 *  │ Flame Sensor AOUT   │ 34       │ Analog                  │
 *  │ Flame Sensor DOUT   │ 13       │ Digital                 │
 *  │ RGB LED - Red       │ 15       │ PWM                     │
 *  │ RGB LED - Green     │  2       │ PWM                     │
 *  │ RGB LED - Blue      │ 23       │ PWM                     │
 *  └─────────────────────┴──────────┴─────────────────────────┘
 *
 *  LOGIKA SISTEM:
 *  - Mesin menyala otomatis saat boot
 *  - Kategori 1 : jarak 16–19 cm → catat + buzzer beep
 *  - Kategori 2 : jarak 10–15 cm → tunggu 0.2 dtk → Servo1 90°
 *  - Kategori 3 : jarak  5– 9 cm → tunggu 0.6 dtk → Servo2 90°
 *  - MQ-2 warm-up 20 dtk non-blocking, kalibrasi otomatis
 *  - Ultrasonik: pulseIn() original (terbukti bekerja)
 *    → MQ-2, water level, flame, dan ultrasonik dijalankan bergantian
 *  - Gas/asap terdeteksi      : Buzzer alarm + Relay OFF
 *  - Air berlebih terdeteksi  : Buzzer alarm + Relay OFF
 *  - Api terdeteksi           : Buzzer alarm + Relay OFF
 *  - RGB LED indikator kategori  : Kat1=Hijau, Kat2=Kuning, Kat3=Biru
 *  - RGB LED indikator bahaya    : Api=Merah-Orange kelip, Gas=Ungu-Pink kelip
 *                                  Air=Biru muda-Putih kelip
 *
 *  CATATAN KALIBRASI WATER LEVEL:
 *  - Upload kode, buka Serial Monitor 115200 baud
 *  - Lihat baris "[WATER] ADC=xxx" saat sensor KERING → catat nilai (misal 150)
 *  - Celupkan sensor ke air → catat nilai (misal 1200)
 *  - Set WATER_THRESHOLD_ON  = nilai_basah - 200  (misal 1000)
 *  - Set WATER_THRESHOLD_OFF = nilai_basah - 400  (misal 800)
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_TRIG        4
#define PIN_ECHO        5
#define PIN_BUZZER     32
#define PIN_RELAY      19
#define PIN_SERVO1     33
#define PIN_SERVO2     25
#define PIN_BUTTON     18
#define PIN_GAS_AOUT   26
#define PIN_GAS_DOUT   27
#define PIN_WATER_AOUT 14
#define PIN_FLAME_AOUT 34
#define PIN_FLAME_DOUT 13

// ── RGB LED (common cathode) ──────────────────────────────────
#define PIN_RGB_R      15
#define PIN_RGB_G       2
#define PIN_RGB_B      23
#define PWM_FREQ   5000
#define PWM_RES       8   // 8-bit: 0–255

// ── Kategori Jarak (cm) ──────────────────────────────────────
#define DIST_CAT1_MIN  15
#define DIST_CAT1_MAX  19.5f
#define DIST_CAT2_MIN  9
#define DIST_CAT2_MAX  14
#define DIST_CAT3_MIN   2
#define DIST_CAT3_MAX   8

// ── Servo ────────────────────────────────────────────────────
#define SERVO_PUSH_ANGLE    90
#define SERVO_IDLE_ANGLE     0
#define CAT2_PUSH_DELAY    200    // 0.2 detik
#define CAT3_PUSH_DELAY    600    // 0.6 detik
#define SERVO_HOLD_TIME   1000
#define SERVO_RETURN_MS    800

// ── MQ-2 ─────────────────────────────────────────────────────
#define MQ2_THRESHOLD_DELTA  150
#define MQ2_THRESHOLD_HYST    50
#define MQ2_WARMUP_MS       20000
#define MQ2_READ_INTERVAL    500

// ── Water Level ──────────────────────────────────────────────
// ESP32 ADC 12-bit: range 0–4095
// Sensor water level modul biasa: kering ~0–300, basah ~800–2500+
//
// !! CARA KALIBRASI !!
// 1. Upload kode ini, buka Serial Monitor 115200 baud
// 2. Lihat baris "[WATER] ADC=xxx" saat sensor KERING → catat nilai
// 3. Celupkan sensor ke air → lihat nilai baru → catat
// 4. Set WATER_THRESHOLD_ON  = (nilai_basah × 0.7)  → sesuaikan
//    Set WATER_THRESHOLD_OFF = (nilai_basah × 0.5)  → sesuaikan
//
// Nilai default di bawah sudah diturunkan dari 2000 → 800
// agar cocok dengan sebagian besar modul water level pasaran.
#define WATER_THRESHOLD_ON    800   // ADC > nilai ini → air berlebih
#define WATER_THRESHOLD_OFF   600   // ADC < nilai ini → air aman (histeresis)
#define WATER_READ_INTERVAL   500   // Baca tiap 500ms

// ── Flame Sensor ─────────────────────────────────────────────
// D0 = penentu utama (LOW = api), dikalibrasi via trimpot di modul
// A0 = monitoring saja via Serial, tidak dipakai logika
#define FLAME_READ_INTERVAL    500  // Baca tiap 500ms

// ── Timing ───────────────────────────────────────────────────
#define DEBOUNCE_MS      50
#define ITEM_COOLDOWN  2500
#define US_READ_INTERVAL   200

// ── Objek ────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2;

// ── State Global ─────────────────────────────────────────────
int      totalBarang     = 0;
bool     machineON       = true;
bool     gasAlert        = false;
bool     waterAlert      = false;
bool     flameAlert      = false;
bool     servoProcessing = false;

// MQ-2
int      mq2Baseline    = 0;
int      mq2ThreshOn    = 9999;
int      mq2ThreshOff   = 9999;
bool     mq2WarmupDone  = false;
uint32_t mq2WarmupStart = 0;
uint32_t mq2LastRead    = 0;

// Water Level
uint32_t waterLastRead  = 0;

// Flame Sensor
uint32_t flameLastRead  = 0;

// RGB LED
struct RGBColor { uint8_t r, g, b; };
RGBColor rgbCurrent     = {0, 0, 0};
RGBColor rgbBlink1      = {0, 0, 0};
RGBColor rgbBlink2      = {0, 0, 0};
bool     rgbBlinking    = false;
bool     rgbBlinkState  = false;
uint32_t rgbBlinkTimer  = 0;
#define  RGB_BLINK_MS   300   // kecepatan kelap-kelip alert

// Warna kategori barang
const RGBColor COL_OFF       = {  0,   0,   0};
const RGBColor COL_CAT1      = {  0, 255,   0};   // Hijau
const RGBColor COL_CAT2      = {255, 180,   0};   // Kuning
const RGBColor COL_CAT3      = {  0,   0, 255};   // Biru
// Warna alert (kelap-kelip antara dua warna)
const RGBColor COL_FLAME_A   = {255,   0,   0};   // Merah
const RGBColor COL_FLAME_B   = {255,  60,   0};   // Orange
const RGBColor COL_GAS_A     = {180,   0, 255};   // Ungu
const RGBColor COL_GAS_B     = {255,   0, 180};   // Pink
const RGBColor COL_WATER_A   = {  0, 180, 255};   // Biru muda
const RGBColor COL_WATER_B   = {255, 255, 255};   // Putih

// Ultrasonik
uint32_t usLastRead     = 0;

// Button debounce
bool     lastBtnRaw   = HIGH;
bool     btnConfirmed = HIGH;
uint32_t lastDebounce = 0;

// Cooldown deteksi barang
uint32_t lastDetectTime = 0;

// LCD gas/water-clear non-blocking
bool     lcdClearPending = false;
uint32_t lcdClearTimer   = 0;
#define  LCD_CLEAR_DELAY  1500

// Servo state machine
enum ServoState { SERVO_IDLE, SERVO_WAITING, SERVO_PUSHING, SERVO_RETURNING };
struct ServoCtrl {
  ServoState state  = SERVO_IDLE;
  uint32_t   timer  = 0;
  uint32_t   waitMs = 0;
};
ServoCtrl sv1, sv2;

// Buzzer non-blocking
enum BuzzerMode { BZ_OFF, BZ_BEEP, BZ_ALARM };
BuzzerMode buzzerMode     = BZ_OFF;
uint32_t   buzzerTimer    = 0;
uint32_t   buzzerDuration = 0;

// ── Forward Declarations ─────────────────────────────────────
float readDistance();
void  detectItem(uint32_t now);
void  handleServo(ServoCtrl &sv, Servo &motor, uint32_t now, const char* label);
void  handleGasSensor(uint32_t now);
void  handleWaterSensor(uint32_t now);
void  handleFlameSensor(uint32_t now);
void  handleButton(uint32_t now);
void  handleBuzzer(uint32_t now);
void  handleRGB(uint32_t now);
void  writeRGB(RGBColor c);
void  setRGB(RGBColor c);
void  setRGBBlink(RGBColor c1, RGBColor c2);
void  setRGBOff();
void  startBeep(uint32_t ms);
void  stopBuzzer();
void  setMachine(bool on);
void  updateLCD();
void  updateLCDKategori(int kat);

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SMART FACTORY CONVEYOR BOOTING... ===");

  pinMode(PIN_TRIG,       OUTPUT);
  pinMode(PIN_ECHO,       INPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_BUTTON,     INPUT_PULLUP);
  pinMode(PIN_GAS_AOUT,   INPUT);
  pinMode(PIN_GAS_DOUT,   INPUT);
  // !! PENTING: PIN_WATER_AOUT (14) harus INPUT, TIDAK INPUT_PULLUP
  // Pullup internal akan memaksa nilai ADC ke atas dan membuatnya
  // selalu terlihat "basah" walaupun sensor kering.
  pinMode(PIN_WATER_AOUT, INPUT);
  pinMode(PIN_FLAME_AOUT, INPUT);
  pinMode(PIN_FLAME_DOUT, INPUT_PULLDOWN);  // pulldown agar tidak floating

  // RGB LED — PWM setup (ESP32 Arduino Core v3.x)
  ledcAttach(PIN_RGB_R, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_G, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_B, PWM_FREQ, PWM_RES);
  setRGBOff();

  analogReadResolution(12);   // 12-bit: 0–4095

  digitalWrite(PIN_TRIG,   LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY,  HIGH);

  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo1.write(SERVO_IDLE_ANGLE);
  servo2.write(SERVO_IDLE_ANGLE);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("SMART CONVEYOR");
  lcd.setCursor(0, 1); lcd.print("MQ2 warm-up...");

  mq2WarmupStart = millis();

  Serial.println("[MQ-2]  Warm-up dimulai (non-blocking, 20 dtk).");
  Serial.println("[WATER] Sensor siap di pin 14 (GPIO14 / ADC1_CH6).");
  Serial.println("[WATER] Threshold ON=" + String(WATER_THRESHOLD_ON)
               + " | OFF=" + String(WATER_THRESHOLD_OFF));
  Serial.println("[WATER] Pantau nilai ADC di Serial untuk kalibrasi!");
  Serial.println("[FLAME] Sensor siap di pin A0=34 / D0=13.");
  Serial.println("=== SISTEM SIAP — Mesin ON ===");
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  handleButton(now);
  handleBuzzer(now);
  handleRGB(now);
  handleServo(sv1, servo1, now, "SERVO1");
  handleServo(sv2, servo2, now, "SERVO2");

  // Gas, water level, flame, dan ultrasonik TIDAK dipanggil bersamaan.
  // Gas & water & flame: tiap 500ms | Ultrasonik: tiap 200ms
  handleGasSensor(now);
  handleWaterSensor(now);
  handleFlameSensor(now);

  // LCD clear non-blocking setelah alert hilang
  if (lcdClearPending && (now - lcdClearTimer >= LCD_CLEAR_DELAY)) {
    lcdClearPending = false;
    updateLCD();
  }

  // Ultrasonik hanya jalan saat mesin ON, tidak ada alert, servo idle
  if (machineON && !gasAlert && !waterAlert && !flameAlert && !servoProcessing) {
    if (now - usLastRead >= US_READ_INTERVAL) {
      usLastRead = now;
      detectItem(now);
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  BACA JARAK — pulseIn() original, terbukti bekerja
// ─────────────────────────────────────────────────────────────
float readDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}

// ─────────────────────────────────────────────────────────────
//  DETEKSI BARANG
// ─────────────────────────────────────────────────────────────
void detectItem(uint32_t now) {
  float dist = readDistance();

  if (dist < 0) {
    Serial.println("[SENSOR] Timeout (tidak ada objek)");
    return;
  }

  Serial.printf("[SENSOR] Jarak: %.1f cm\n", dist);

  int kat = 0;
  if      (dist >= DIST_CAT1_MIN && dist <= DIST_CAT1_MAX) kat = 1;
  else if (dist >= DIST_CAT2_MIN && dist <= DIST_CAT2_MAX) kat = 2;
  else if (dist >= DIST_CAT3_MIN && dist <= DIST_CAT3_MAX) kat = 3;

  if (kat == 0) return;

  totalBarang++;
  lastDetectTime  = now;
  servoProcessing = true;

  Serial.printf("  ✔ Kategori %d | Total: %d\n", kat, totalBarang);
  startBeep(300);
  updateLCDKategori(kat);

  // RGB indikator kategori
  if      (kat == 1) setRGB(COL_CAT1);
  else if (kat == 2) setRGB(COL_CAT2);
  else if (kat == 3) setRGB(COL_CAT3);

  if (kat == 2) {
    sv1.state  = SERVO_WAITING;
    sv1.waitMs = CAT2_PUSH_DELAY;
    sv1.timer  = now;
    Serial.println("  → Servo1 dijadwalkan (0.2 dtk)");
  } else if (kat == 3) {
    sv2.state  = SERVO_WAITING;
    sv2.waitMs = CAT3_PUSH_DELAY;
    sv2.timer  = now;
    Serial.println("  → Servo2 dijadwalkan (0.6 dtk)");
  } else {
    servoProcessing = false;
  }
}

// ─────────────────────────────────────────────────────────────
//  STATE MACHINE SERVO (generik)
// ─────────────────────────────────────────────────────────────
void handleServo(ServoCtrl &sv, Servo &motor, uint32_t now, const char* label) {
  switch (sv.state) {

    case SERVO_WAITING:
      if (now - sv.timer >= sv.waitMs) {
        motor.write(SERVO_PUSH_ANGLE);
        Serial.printf("[%s] Mendorong barang...\n", label);
        sv.timer = now;
        sv.state = SERVO_PUSHING;
      }
      break;

    case SERVO_PUSHING:
      if (now - sv.timer >= SERVO_HOLD_TIME) {
        motor.write(SERVO_IDLE_ANGLE);
        Serial.printf("[%s] Kembali ke posisi awal.\n", label);
        sv.timer = now;
        sv.state = SERVO_RETURNING;
      }
      break;

    case SERVO_RETURNING:
      if (now - sv.timer >= SERVO_RETURN_MS) {
        sv.state        = SERVO_IDLE;
        servoProcessing = false;
        setRGBOff();
        updateLCD();
        Serial.printf("[%s] Selesai.\n", label);
      }
      break;

    default:
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  GAS SENSOR MQ-2
// ─────────────────────────────────────────────────────────────
void handleGasSensor(uint32_t now) {

  // ── Fase warm-up non-blocking ────────────────────────────
  if (!mq2WarmupDone) {
    uint32_t elapsed = now - mq2WarmupStart;

    static uint32_t lastCountdown = 0;
    if (now - lastCountdown >= 1000) {
      lastCountdown = now;
      int sisa = (int)((MQ2_WARMUP_MS - elapsed) / 1000) + 1;
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("MQ2 warm-up:");
      lcd.setCursor(0, 1); lcd.print(sisa); lcd.print(" dtk lagi...");
      Serial.printf("[MQ-2] Warming up... %d dtk lagi\n", sisa);
    }

    if (elapsed < MQ2_WARMUP_MS) return;

    // ── Kalibrasi baseline setelah warm-up ───────────────
    mq2WarmupDone = true;
    long sum = 0;
    for (int i = 0; i < 20; i++) {
      sum += analogRead(PIN_GAS_AOUT);
    }
    mq2Baseline  = (int)(sum / 20);
    mq2ThreshOn  = mq2Baseline + MQ2_THRESHOLD_DELTA;
    mq2ThreshOff = mq2Baseline + MQ2_THRESHOLD_DELTA - MQ2_THRESHOLD_HYST;

    Serial.printf("[MQ-2] Kalibrasi selesai.\n");
    Serial.printf("       Baseline=%d | ThreshON=%d | ThreshOFF=%d\n",
                  mq2Baseline, mq2ThreshOn, mq2ThreshOff);
    Serial.println("       Jika false alarm → naikkan MQ2_THRESHOLD_DELTA.");

    updateLCD();
    return;
  }

  if (now - mq2LastRead < MQ2_READ_INTERVAL) return;
  mq2LastRead = now;

  int gasVal = analogRead(PIN_GAS_AOUT);

  Serial.printf("[MQ-2] ADC=%d | Baseline=%d | ThreshON=%d | Alert=%s\n",
                gasVal, mq2Baseline, mq2ThreshOn, gasAlert ? "YA" : "tidak");

  if (!gasAlert && gasVal > mq2ThreshOn) {
    gasAlert = true;
    setMachine(false);
    buzzerMode = BZ_ALARM;
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_GAS_A, COL_GAS_B);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! BAHAYA ASAP!!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(gasVal);
    Serial.printf("[MQ-2] ASAP terdeteksi! ADC=%d (thresh=%d)\n",
                  gasVal, mq2ThreshOn);
  }
  else if (gasAlert && gasVal < mq2ThreshOff) {
    gasAlert = false;
    if (!waterAlert && !flameAlert) {
      stopBuzzer();
      setRGBOff();
      setMachine(true);
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Udara bersih.");
      lcd.setCursor(0, 1); lcd.print("Mesin nyala...");
      lcdClearPending = true;
      lcdClearTimer   = now;
    }
    Serial.printf("[MQ-2] Aman. ADC=%d\n", gasVal);
  }
}

// ─────────────────────────────────────────────────────────────
//  WATER LEVEL SENSOR
// ─────────────────────────────────────────────────────────────
void handleWaterSensor(uint32_t now) {

  // Tunggu MQ-2 selesai warm-up dulu agar tidak ganggu LCD
  if (!mq2WarmupDone) return;

  if (now - waterLastRead < WATER_READ_INTERVAL) return;
  waterLastRead = now;

  int waterVal = analogRead(PIN_WATER_AOUT);

  // ── Log SELALU (bukan hanya saat alert) ──────────────────
  // Gunakan ini untuk kalibrasi: cek nilai saat kering vs saat ada air
  Serial.printf("[WATER] ADC=%d | ThreshON=%d | ThreshOFF=%d | Alert=%s\n",
                waterVal,
                WATER_THRESHOLD_ON,
                WATER_THRESHOLD_OFF,
                waterAlert ? "YA" : "tidak");

  if (!waterAlert && waterVal > WATER_THRESHOLD_ON) {
    waterAlert = true;
    setMachine(false);
    buzzerMode = BZ_ALARM;
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_WATER_A, COL_WATER_B);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! BANJIR/AIR !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(waterVal);
    Serial.printf("[WATER] *** Air berlebih! ADC=%d (thresh=%d) ***\n",
                  waterVal, WATER_THRESHOLD_ON);
  }
  else if (waterAlert && waterVal < WATER_THRESHOLD_OFF) {
    waterAlert = false;
    if (!gasAlert && !flameAlert) {
      stopBuzzer();
      setRGBOff();
      setMachine(true);
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Air aman.");
      lcd.setCursor(0, 1); lcd.print("Mesin nyala...");
      lcdClearPending = true;
      lcdClearTimer   = now;
    }
    Serial.printf("[WATER] Level air aman. ADC=%d\n", waterVal);
  }
}

// ─────────────────────────────────────────────────────────────
//  FLAME SENSOR
// ─────────────────────────────────────────────────────────────
void handleFlameSensor(uint32_t now) {

  // Tunggu MQ-2 selesai warm-up dulu agar tidak ganggu LCD
  if (!mq2WarmupDone) return;

  if (now - flameLastRead < FLAME_READ_INTERVAL) return;
  flameLastRead = now;

  // D0 sebagai penentu — HIGH = ada api, LOW = aman
  // INPUT_PULLDOWN di setup() memastikan pin tidak floating
  bool d0Fire   = (digitalRead(PIN_FLAME_DOUT) == HIGH);
  int  flameVal = analogRead(PIN_FLAME_AOUT);

  Serial.printf("[FLAME] D0=%s | A0_ADC=%d | Alert=%s\n",
                d0Fire ? "HIGH(api)" : "LOW(aman)",
                flameVal,
                flameAlert ? "YA" : "tidak");

  if (!flameAlert && d0Fire) {
    flameAlert = true;
    setMachine(false);
    buzzerMode = BZ_ALARM;
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_FLAME_A, COL_FLAME_B);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! BAHAYA API !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(flameVal);
    Serial.printf("[FLAME] Api terdeteksi! D0=HIGH, ADC=%d\n", flameVal);
  }
  else if (flameAlert && !d0Fire) {
    flameAlert = false;
    if (!gasAlert && !waterAlert) {
      stopBuzzer();
      setRGBOff();
      setMachine(true);
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Api padam.");
      lcd.setCursor(0, 1); lcd.print("Mesin nyala...");
      lcdClearPending = true;
      lcdClearTimer   = now;
    }
    Serial.printf("[FLAME] Aman. D0=LOW, ADC=%d\n", flameVal);
  }
}

// ─────────────────────────────────────────────────────────────
//  BUTTON — Toggle Mesin
// ─────────────────────────────────────────────────────────────
void handleButton(uint32_t now) {
  bool raw = digitalRead(PIN_BUTTON);

  if (raw != lastBtnRaw) lastDebounce = now;
  lastBtnRaw = raw;

  if ((now - lastDebounce) <= DEBOUNCE_MS) return;

  if (raw != btnConfirmed) {
    btnConfirmed = raw;
    if (btnConfirmed == LOW) {
      if (gasAlert || waterAlert || flameAlert) {
        Serial.println("[BTN] Ditolak: ada alert aktif (gas/air/api)!");
        return;
      }
      setMachine(!machineON);
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  SET MESIN ON / OFF
// ─────────────────────────────────────────────────────────────
void setMachine(bool on) {
  machineON = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  Serial.printf("[MESIN] %s\n", on ? "ON" : "OFF");

  if (!on) {
    servo1.write(SERVO_IDLE_ANGLE);
    servo2.write(SERVO_IDLE_ANGLE);
    sv1 = {SERVO_IDLE, 0, 0};
    sv2 = {SERVO_IDLE, 0, 0};
    servoProcessing = false;
    stopBuzzer();
  }
  updateLCD();
}

// ─────────────────────────────────────────────────────────────
//  BUZZER
// ─────────────────────────────────────────────────────────────
void startBeep(uint32_t ms) {
  if (buzzerMode == BZ_ALARM) return;
  buzzerMode     = BZ_BEEP;
  buzzerDuration = ms;
  buzzerTimer    = millis();
  digitalWrite(PIN_BUZZER, HIGH);
}

void stopBuzzer() {
  buzzerMode = BZ_OFF;
  digitalWrite(PIN_BUZZER, LOW);
}

void handleBuzzer(uint32_t now) {
  if (buzzerMode == BZ_BEEP) {
    if (now - buzzerTimer >= buzzerDuration) stopBuzzer();
  }
}

// ─────────────────────────────────────────────────────────────
//  RGB LED
// ─────────────────────────────────────────────────────────────
void writeRGB(RGBColor c) {
  ledcWrite(PIN_RGB_R, c.r);
  ledcWrite(PIN_RGB_G, c.g);
  ledcWrite(PIN_RGB_B, c.b);
}

void setRGB(RGBColor c) {
  rgbBlinking = false;
  rgbCurrent  = c;
  writeRGB(c);
}

void setRGBOff() {
  setRGB(COL_OFF);
}

void setRGBBlink(RGBColor c1, RGBColor c2) {
  rgbBlink1     = c1;
  rgbBlink2     = c2;
  rgbBlinking   = true;
  rgbBlinkState = true;
  rgbBlinkTimer = millis();
  writeRGB(c1);
}

void handleRGB(uint32_t now) {
  if (!rgbBlinking) return;
  if (now - rgbBlinkTimer < RGB_BLINK_MS) return;
  rgbBlinkTimer = now;
  rgbBlinkState = !rgbBlinkState;
  writeRGB(rgbBlinkState ? rgbBlink1 : rgbBlink2);
}

// ─────────────────────────────────────────────────────────────
//  LCD
// ─────────────────────────────────────────────────────────────
void updateLCD() {
  lcd.clear();
  if (machineON) {
    lcd.setCursor(0, 0); lcd.print("Total: "); lcd.print(totalBarang);
    lcd.setCursor(0, 1); lcd.print("Menunggu barang");
  } else {
    lcd.setCursor(0, 0); lcd.print("Mesin: MATI");
    lcd.setCursor(0, 1); lcd.print("Total: "); lcd.print(totalBarang);
  }
}

void updateLCDKategori(int kat) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Total: "); lcd.print(totalBarang);
  lcd.setCursor(0, 1); lcd.print("Kategori "); lcd.print(kat);
}
