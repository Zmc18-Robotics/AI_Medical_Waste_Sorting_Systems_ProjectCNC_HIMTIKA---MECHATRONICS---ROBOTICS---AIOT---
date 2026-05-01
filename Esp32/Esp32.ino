/*
 * ============================================================
 *   SMART FACTORY - SERVO + MOTOR DC CONTROL (Web IoT)
 *   Platform  : ESP32 DevKit C v4
 *   Author    : Mc.Zminecrafter18
 *   Modified  : Motor toggle ON/OFF, preset speed, pin info
 *
 *   ★ CARA PAKAI:
 *   1. Isi WIFI_SSID dan WIFI_PASS di bawah
 *   2. Upload sketch (1 file saja: .ino)
 *   3. Buka Serial Monitor → catat IP yang tampil
 *   4. Ketik IP di browser → dashboard langsung muncul!
 *
 *  Library yang dibutuhkan:
 *  - ESP32Servo
 *  - WebSockets (by Markus Sattler)
 *  - ArduinoJson
 *  - LiquidCrystal_I2C
 *
 *  PIN MAP:
 *  ┌─────────────────────┬──────────┬─────────────────────────┐
 *  │ Komponen            │ Pin ESP32│ Catatan                 │
 *  ├─────────────────────┼──────────┼─────────────────────────┤
 *  │ LCD I2C SDA         │ 21       │                         │
 *  │ LCD I2C SCL         │ 22       │                         │
 *  │ Buzzer              │ 32       │                         │
 *  │ Push Button         │  5       │ INPUT_PULLUP            │
 *  │ Servo 1             │ 33       │ Manual via Web          │
 *  │ Servo 2             │ 25       │ Manual via Web          │
 *  │ Servo 3             │ 18       │ Manual via Web          │
 *  │ MQ-2 AOUT           │ 26       │ Analog                  │
 *  │ MQ-2 DOUT           │ 27       │ tidak dipakai           │
 *  │ Water Level AOUT    │ 14       │ Analog (ADC1_CH6)       │
 *  │ Flame Sensor AOUT   │ 34       │ Analog                  │
 *  │ Flame Sensor DOUT   │ 13       │ Digital                 │
 *  │ RGB LED - Red       │ 15       │ PWM                     │
 *  │ RGB LED - Green     │  2       │ PWM                     │
 *  │ RGB LED - Blue      │ 23       │ PWM                     │
 *  │ L298N IN1           │  4       │ Arah motor              │
 *  │ L298N IN2           │ 17       │ Arah motor              │
 *  │ L298N ENA           │ 16       │ PWM kecepatan           │
 *  └─────────────────────┴──────────┴─────────────────────────┘
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ── WiFi ─────────────────────────────────────────────────────
#define WIFI_SSID   "Absolute Solver"
#define WIFI_PASS   "CynIsTheCutestBotEver333"

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_BUZZER      32
#define PIN_BUTTON       5
#define PIN_SERVO1      33
#define PIN_SERVO2      19
#define PIN_SERVO3      18
#define PIN_GAS_AOUT    26
#define PIN_GAS_DOUT    27
#define PIN_WATER_AOUT  14
#define PIN_FLAME_AOUT  34
#define PIN_FLAME_DOUT  13

// ── RGB LED ───────────────────────────────────────────────────
#define PIN_RGB_R       15
#define PIN_RGB_G        2
#define PIN_RGB_B       23
#define PWM_FREQ      5000
#define PWM_RES          8

// ── Motor DC L298N ────────────────────────────────────────────
#define PIN_MOTOR_IN1    4
#define PIN_MOTOR_IN2   17
#define PIN_MOTOR_ENA   16
#define PWM_MOTOR_FREQ  1000
#define PWM_MOTOR_RES      8

// ── MQ-2 ─────────────────────────────────────────────────────
#define MQ2_THRESHOLD_DELTA  150
#define MQ2_THRESHOLD_HYST    50
#define MQ2_WARMUP_MS       20000
#define MQ2_READ_INTERVAL     500

// ── Water Level ──────────────────────────────────────────────
#define WATER_THRESHOLD_ON    800
#define WATER_THRESHOLD_OFF   600
#define WATER_READ_INTERVAL   500

// ── Flame ─────────────────────────────────────────────────────
#define FLAME_READ_INTERVAL   500

// ── Servo ─────────────────────────────────────────────────────
#define SERVO_MIN_ANGLE    0
#define SERVO_MAX_ANGLE  180

// ── LCD ───────────────────────────────────────────────────────
#define LCD_CLEAR_DELAY  1500

// ── WS Broadcast ─────────────────────────────────────────────
#define WS_BROADCAST_MS  500

// ── Button Debounce ──────────────────────────────────────────
#define BTN_DEBOUNCE_MS   50

// ── RGB Blink ────────────────────────────────────────────────
#define RGB_BLINK_MS     300

// ─────────────────────────────────────────────────────────────
//  DASHBOARD HTML
// ─────────────────────────────────────────────────────────────
const char DASHBOARD_HTML[] PROGMEM =
"<!DOCTYPE html>\n"
"<html lang=\"id\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
"<title>Smart Factory — Control</title>\n"
"<style>\n"
"  @import url('https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&family=IBM+Plex+Sans:wght@300;400;600&display=swap');\n"
"  :root {\n"
"    --bg:#0d0f14;--surface:#161a23;--surface2:#1e2330;\n"
"    --border:#2a3045;--accent:#00e5ff;--accent2:#ff6b35;\n"
"    --text:#e8ecf0;--muted:#6b7a8d;--danger:#ff3b3b;\n"
"    --warn:#ffb020;--ok:#22d55a;\n"
"    --font-mono:'Space Mono',monospace;\n"
"    --font-sans:'IBM Plex Sans',sans-serif;\n"
"    --radius:8px;\n"
"  }\n"
"  *{box-sizing:border-box;margin:0;padding:0;}\n"
"  body{background:var(--bg);color:var(--text);font-family:var(--font-sans);min-height:100vh;}\n"
"  header{background:var(--surface);border-bottom:1px solid var(--border);padding:14px 24px;\n"
"    display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:100;}\n"
"  .logo{font-family:var(--font-mono);font-size:13px;letter-spacing:0.12em;color:var(--accent);text-transform:uppercase;}\n"
"  .logo span{color:var(--muted);}\n"
"  .conn-badge{font-family:var(--font-mono);font-size:11px;padding:4px 12px;border-radius:20px;\n"
"    border:1px solid;letter-spacing:0.08em;text-transform:uppercase;transition:all 0.3s;}\n"
"  .conn-badge.disconnected{border-color:var(--danger);color:var(--danger);}\n"
"  .conn-badge.connected{border-color:var(--ok);color:var(--ok);}\n"
"  .conn-badge.connecting{border-color:var(--warn);color:var(--warn);}\n"
"  main{max-width:900px;margin:0 auto;padding:24px 20px;}\n"
"  .alert-banner{display:none;align-items:center;gap:10px;\n"
"    background:rgba(255,59,59,0.12);border:1px solid var(--danger);border-radius:var(--radius);\n"
"    padding:12px 16px;margin-bottom:16px;font-size:13px;color:var(--danger);\n"
"    animation:pulse-border 1s infinite;}\n"
"  .alert-banner.active{display:flex;}\n"
"  @keyframes pulse-border{0%,100%{border-color:var(--danger)}50%{border-color:transparent}}\n"
"  .alert-dot{width:8px;height:8px;border-radius:50%;background:var(--danger);animation:blink 0.6s infinite;flex-shrink:0;}\n"
"  @keyframes blink{0%,100%{opacity:1}50%{opacity:0.2}}\n"
"  .grid-2{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:16px;}\n"
"  .grid-3{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-bottom:16px;}\n"
"  @media(max-width:640px){.grid-2{grid-template-columns:1fr}.grid-3{grid-template-columns:1fr}}\n"
"  .card{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:16px;}\n"
"  .card-title{font-family:var(--font-mono);font-size:10px;letter-spacing:0.15em;color:var(--muted);\n"
"    text-transform:uppercase;margin-bottom:14px;}\n"
"  .servo-card{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);\n"
"    padding:18px;transition:border-color 0.2s;}\n"
"  .servo-card.active{border-color:var(--accent);}\n"
"  .servo-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;}\n"
"  .servo-name{font-family:var(--font-mono);font-size:11px;letter-spacing:0.12em;color:var(--muted);text-transform:uppercase;}\n"
"  .servo-angle{font-family:var(--font-mono);font-size:28px;font-weight:700;color:var(--accent);letter-spacing:-0.02em;}\n"
"  .servo-angle span{font-size:14px;color:var(--muted);}\n"
"  .servo-vis{display:flex;justify-content:center;margin:10px 0 14px;}\n"
"  input[type=range]{-webkit-appearance:none;width:100%;height:4px;background:var(--border);border-radius:2px;outline:none;cursor:pointer;}\n"
"  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;border-radius:50%;\n"
"    background:var(--accent);border:2px solid var(--bg);transition:transform 0.1s;}\n"
"  input[type=range]::-webkit-slider-thumb:active{transform:scale(1.2);}\n"
"  .servo-presets{display:flex;gap:6px;margin-top:12px;}\n"
"  .preset-btn{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);\n"
"    color:var(--muted);font-family:var(--font-mono);font-size:11px;padding:7px 4px;cursor:pointer;text-align:center;transition:all 0.15s;}\n"
"  .preset-btn:hover{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.05);}\n"
"  .preset-btn.active-preset{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.1);}\n"
"  .sensor-value{font-family:var(--font-mono);font-size:22px;font-weight:700;margin-bottom:4px;}\n"
"  .sensor-label{font-size:12px;color:var(--muted);margin-bottom:10px;}\n"
"  .sensor-bar-wrap{background:var(--bg);border-radius:3px;height:4px;overflow:hidden;margin-top:8px;}\n"
"  .sensor-bar{height:100%;border-radius:3px;transition:width 0.4s;background:var(--ok);}\n"
"  .sensor-bar.warn{background:var(--warn);}\n"
"  .sensor-bar.alert{background:var(--danger);}\n"
"  .status-pill{display:inline-flex;align-items:center;gap:5px;font-family:var(--font-mono);font-size:10px;\n"
"    letter-spacing:0.08em;text-transform:uppercase;padding:4px 10px;border-radius:20px;border:1px solid;}\n"
"  .status-pill.ok{border-color:var(--ok);color:var(--ok);}\n"
"  .status-pill.danger{border-color:var(--danger);color:var(--danger);background:rgba(255,59,59,0.08);}\n"
"  .status-pill.offline{border-color:var(--muted);color:var(--muted);}\n"
"  .warmup-bar{background:var(--bg);border-radius:3px;height:6px;overflow:hidden;margin-top:10px;}\n"
"  .warmup-fill{height:100%;background:var(--warn);border-radius:3px;transition:width 1s linear;}\n"
"  .section-label{font-family:var(--font-mono);font-size:10px;letter-spacing:0.18em;color:var(--muted);\n"
"    text-transform:uppercase;margin:20px 0 10px;display:flex;align-items:center;gap:8px;}\n"
"  .section-label::after{content:'';flex:1;height:1px;background:var(--border);}\n"
"  /* ── MOTOR CARD ── */\n"
"  .motor-card{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:18px;}\n"
"  .motor-top{display:flex;align-items:center;justify-content:space-between;margin-bottom:4px;}\n"
"  .motor-status-text{font-family:var(--font-mono);font-size:24px;font-weight:700;}\n"
"  .motor-pin-info{font-family:var(--font-mono);font-size:11px;color:var(--muted);}\n"
"  .motor-speed-row{display:flex;align-items:center;gap:12px;margin:14px 0 6px;}\n"
"  .motor-speed-label{font-family:var(--font-mono);font-size:11px;color:var(--muted);white-space:nowrap;}\n"
"  .motor-speed-val{font-family:var(--font-mono);font-size:13px;color:var(--accent);min-width:36px;text-align:right;}\n"
"  .motor-presets{display:flex;gap:6px;margin-bottom:14px;}\n"
"  .motor-preset-btn{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:var(--radius);\n"
"    color:var(--muted);font-family:var(--font-mono);font-size:11px;padding:6px 4px;cursor:pointer;text-align:center;transition:all 0.15s;}\n"
"  .motor-preset-btn:hover{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.05);}\n"
"  .motor-preset-btn.active-preset{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.1);}\n"
"  .motor-btns{display:flex;gap:10px;}\n"
"  .motor-btn{flex:1;padding:14px 8px;border:1px solid var(--border);border-radius:var(--radius);\n"
"    background:var(--bg);color:var(--muted);font-family:var(--font-mono);font-size:13px;font-weight:700;\n"
"    cursor:pointer;text-align:center;transition:all 0.15s;letter-spacing:0.08em;text-transform:uppercase;}\n"
"  .motor-btn:hover{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.05);}\n"
"  .motor-btn.active-fwd{border-color:var(--ok);color:var(--ok);background:rgba(34,213,90,0.08);}\n"
"  .motor-btn.active-bwd{border-color:var(--accent2);color:var(--accent2);background:rgba(255,107,53,0.08);}\n"
"  .motor-btn.stop-btn{border-color:var(--danger);color:var(--danger);}\n"
"  .motor-btn.stop-btn:hover{background:rgba(255,59,59,0.08);}\n"
"  /* ── BUTTON CARD ── */\n"
"  .btn-pill{display:inline-flex;align-items:center;gap:6px;font-family:var(--font-mono);font-size:11px;\n"
"    letter-spacing:0.1em;text-transform:uppercase;padding:5px 12px;border-radius:20px;border:1px solid;}\n"
"  .btn-pill.pressed{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.08);}\n"
"  .btn-pill.released{border-color:var(--muted);color:var(--muted);}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<header>\n"
"  <div class=\"logo\">Smart Factory <span>// Motor + Servo</span></div>\n"
"  <div class=\"conn-badge disconnected\" id=\"connBadge\"></div>\n"
"</header>\n"
"<main>\n"
"  <div class=\"alert-banner\" id=\"alertBanner\">\n"
"    <div class=\"alert-dot\"></div>\n"
"    <strong id=\"alertText\">Alert aktif!</strong>\n"
"  </div>\n"
"\n"
"  <div class=\"section-label\">Status sistem</div>\n"
"  <div class=\"grid-2\">\n"
"    <div class=\"card\">\n"
"      <div class=\"card-title\">MQ-2 Gas/Asap</div>\n"
"      <div class=\"sensor-value\" id=\"gasVal\">—</div>\n"
"      <div class=\"sensor-label\">ADC raw (0–4095)</div>\n"
"      <div id=\"gasPill\" class=\"status-pill offline\">Offline</div>\n"
"      <div class=\"sensor-bar-wrap\"><div class=\"sensor-bar\" id=\"gasBar\" style=\"width:0%\"></div></div>\n"
"      <div id=\"warmupSection\" style=\"margin-top:16px;display:none;\">\n"
"        <div style=\"display:flex;justify-content:space-between;align-items:center;\">\n"
"          <span style=\"font-size:12px;color:var(--muted);\">MQ-2 warm-up</span>\n"
"          <span style=\"font-family:var(--font-mono);font-size:12px;color:var(--warn);\" id=\"warmupSec\">20s</span>\n"
"        </div>\n"
"        <div class=\"warmup-bar\"><div class=\"warmup-fill\" id=\"warmupFill\" style=\"width:0%\"></div></div>\n"
"      </div>\n"
"      <div id=\"warmupDone\" style=\"margin-top:12px;font-size:12px;color:var(--ok);font-family:var(--font-mono);display:none;\">&#10003; MQ-2 siap</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <div class=\"card-title\">Water Level</div>\n"
"      <div class=\"sensor-value\" id=\"waterVal\">—</div>\n"
"      <div class=\"sensor-label\">ADC raw · threshold 800</div>\n"
"      <div id=\"waterPill\" class=\"status-pill offline\">Offline</div>\n"
"      <div class=\"sensor-bar-wrap\"><div class=\"sensor-bar\" id=\"waterBar\" style=\"width:0%\"></div></div>\n"
"    </div>\n"
"  </div>\n"
"  <div class=\"grid-2\">\n"
"    <div class=\"card\">\n"
"      <div class=\"card-title\">Flame Sensor</div>\n"
"      <div class=\"sensor-value\" id=\"flameADC\">—</div>\n"
"      <div class=\"sensor-label\">ADC (A0)</div>\n"
"      <div id=\"flamePill\" class=\"status-pill offline\">Offline</div>\n"
"      <div style=\"font-size:11px;color:var(--muted);margin-top:8px;font-family:var(--font-mono);\">D0: <span id=\"flameD0\">—</span></div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <div class=\"card-title\">Push Button (GPIO 5)</div>\n"
"      <div style=\"margin-top:8px;\">\n"
"        <div id=\"btnPill\" class=\"btn-pill released\">Released</div>\n"
"      </div>\n"
"      <div style=\"font-size:12px;color:var(--muted);margin-top:12px;\">Tekan tombol untuk toggle motor maju/berhenti</div>\n"
"    </div>\n"
"  </div>\n"
"\n"
"  <div class=\"section-label\">Motor DC (L298N)</div>\n"
"  <div class=\"motor-card\" style=\"margin-bottom:16px;\">\n"
"    <div class=\"card-title\">Motor DC — IN1:GPIO4 · IN2:GPIO17 · ENA:GPIO16</div>\n"
"    <div class=\"motor-top\">\n"
"      <div class=\"motor-status-text\" id=\"motorStatusText\" style=\"color:var(--muted);\">STOP</div>\n"
"      <div class=\"motor-pin-info\" id=\"motorPinInfo\">IN1:L &nbsp; IN2:L &nbsp; ENA:0</div>\n"
"    </div>\n"
"    <div class=\"motor-speed-row\">\n"
"      <span class=\"motor-speed-label\">Kecepatan:</span>\n"
"      <input type=\"range\" min=\"0\" max=\"255\" value=\"200\" step=\"5\" id=\"motorSpeed\"\n"
"             oninput=\"speedInput(this.value)\" onchange=\"speedSend(this.value)\" style=\"flex:1;\">\n"
"      <span class=\"motor-speed-val\" id=\"motorSpeedVal\">200</span>\n"
"    </div>\n"
"    <div style=\"display:flex;justify-content:flex-end;margin-top:4px;margin-bottom:10px;\">\n"
"      <span style=\"font-family:var(--font-mono);font-size:11px;color:var(--muted);\" id=\"motorSpeedPct\">78%</span>\n"
"    </div>\n"
"    <div class=\"motor-presets\">\n"
"      <div class=\"motor-preset-btn\" onclick=\"motorPreset(51)\">Pelan<br>20%</div>\n"
"      <div class=\"motor-preset-btn\" onclick=\"motorPreset(102)\">Lambat<br>40%</div>\n"
"      <div class=\"motor-preset-btn\" onclick=\"motorPreset(153)\">Sedang<br>60%</div>\n"
"      <div class=\"motor-preset-btn active-preset\" id=\"mpreset200\" onclick=\"motorPreset(200)\">Cepat<br>78%</div>\n"
"      <div class=\"motor-preset-btn\" onclick=\"motorPreset(255)\">Maks<br>100%</div>\n"
"    </div>\n"
"    <div class=\"motor-btns\">\n"
"      <div class=\"motor-btn\" id=\"btnFwd\" onclick=\"motorToggle('fwd')\">&#9650; Maju</div>\n"
"      <div class=\"motor-btn stop-btn\" onclick=\"motorCmd('stop')\">&#9632; Stop</div>\n"
"      <div class=\"motor-btn\" id=\"btnBwd\" onclick=\"motorToggle('bwd')\">&#9660; Mundur</div>\n"
"    </div>\n"
"  </div>\n"
"\n"
"  <div class=\"section-label\">Servo control</div>\n"
"  <div class=\"grid-3\">\n"
"    <div class=\"servo-card\" id=\"sc1\">\n"
"      <div class=\"servo-header\"><div class=\"servo-name\">Servo 1 — pin 33</div></div>\n"
"      <div class=\"servo-angle\" id=\"sa1\">0 <span>deg</span></div>\n"
"      <div class=\"servo-vis\">\n"
"        <svg width=\"80\" height=\"50\" viewBox=\"0 0 80 50\">\n"
"          <path d=\"M10,45 A35,35 0 0,1 70,45\" fill=\"none\" stroke=\"#2a3045\" stroke-width=\"4\" stroke-linecap=\"round\"/>\n"
"          <line id=\"needle1\" x1=\"40\" y1=\"45\" x2=\"40\" y2=\"12\" stroke=\"#00e5ff\" stroke-width=\"2.5\" stroke-linecap=\"round\"/>\n"
"          <circle cx=\"40\" cy=\"45\" r=\"4\" fill=\"#00e5ff\"/>\n"
"        </svg>\n"
"      </div>\n"
"      <input type=\"range\" min=\"0\" max=\"180\" value=\"0\" step=\"1\" id=\"sl1\"\n"
"             oninput=\"servoInput(1,this.value)\" onchange=\"servoSend(1,this.value)\">\n"
"      <div class=\"servo-presets\">\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(1,0)\">0°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(1,45)\">45°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(1,90)\">90°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(1,135)\">135°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(1,180)\">180°</div>\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"servo-card\" id=\"sc2\">\n"
"      <div class=\"servo-header\"><div class=\"servo-name\">Servo 2 — pin 25</div></div>\n"
"      <div class=\"servo-angle\" id=\"sa2\">0 <span>deg</span></div>\n"
"      <div class=\"servo-vis\">\n"
"        <svg width=\"80\" height=\"50\" viewBox=\"0 0 80 50\">\n"
"          <path d=\"M10,45 A35,35 0 0,1 70,45\" fill=\"none\" stroke=\"#2a3045\" stroke-width=\"4\" stroke-linecap=\"round\"/>\n"
"          <line id=\"needle2\" x1=\"40\" y1=\"45\" x2=\"40\" y2=\"12\" stroke=\"#00e5ff\" stroke-width=\"2.5\" stroke-linecap=\"round\"/>\n"
"          <circle cx=\"40\" cy=\"45\" r=\"4\" fill=\"#00e5ff\"/>\n"
"        </svg>\n"
"      </div>\n"
"      <input type=\"range\" min=\"0\" max=\"180\" value=\"0\" step=\"1\" id=\"sl2\"\n"
"             oninput=\"servoInput(2,this.value)\" onchange=\"servoSend(2,this.value)\">\n"
"      <div class=\"servo-presets\">\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(2,0)\">0°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(2,45)\">45°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(2,90)\">90°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(2,135)\">135°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(2,180)\">180°</div>\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"servo-card\" id=\"sc3\">\n"
"      <div class=\"servo-header\"><div class=\"servo-name\">Servo 3 — pin 18</div></div>\n"
"      <div class=\"servo-angle\" id=\"sa3\">0 <span>deg</span></div>\n"
"      <div class=\"servo-vis\">\n"
"        <svg width=\"80\" height=\"50\" viewBox=\"0 0 80 50\">\n"
"          <path d=\"M10,45 A35,35 0 0,1 70,45\" fill=\"none\" stroke=\"#2a3045\" stroke-width=\"4\" stroke-linecap=\"round\"/>\n"
"          <line id=\"needle3\" x1=\"40\" y1=\"45\" x2=\"40\" y2=\"12\" stroke=\"#00e5ff\" stroke-width=\"2.5\" stroke-linecap=\"round\"/>\n"
"          <circle cx=\"40\" cy=\"45\" r=\"4\" fill=\"#00e5ff\"/>\n"
"        </svg>\n"
"      </div>\n"
"      <input type=\"range\" min=\"0\" max=\"180\" value=\"0\" step=\"1\" id=\"sl3\"\n"
"             oninput=\"servoInput(3,this.value)\" onchange=\"servoSend(3,this.value)\">\n"
"      <div class=\"servo-presets\">\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(3,0)\">0°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(3,45)\">45°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(3,90)\">90°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(3,135)\">135°</div>\n"
"        <div class=\"preset-btn\" onclick=\"servoPreset(3,180)\">180°</div>\n"
"      </div>\n"
"    </div>\n"
"  </div>\n"
"  <div style=\"height:32px;\"></div>\n"
"</main>\n"
"\n"
"<script>\n"
"let ws=null,connected=false;\n"
"let mq2Ready=false,warmupStartTime=null,warmupInterval=null;\n"
"let motorState='stop',motorSpeed=200;\n"
"const MOTOR_PRESETS=[51,102,153,200,255];\n"
"\n"
"function setConnBadge(state){\n"
"  const b=document.getElementById('connBadge');\n"
"  b.className='conn-badge '+state;\n"
"  b.textContent={connected:'Connected',disconnected:'Disconnected',connecting:'Connecting...'}[state]||state;\n"
"}\n"
"\n"
"function connectWS(){\n"
"  setConnBadge('connecting');\n"
"  ws=new WebSocket('ws://'+window.location.hostname+':81');\n"
"  ws.onopen=()=>{connected=true;setConnBadge('connected');};\n"
"  ws.onmessage=(evt)=>{try{const d=JSON.parse(evt.data);if(d.type==='status')updateUI(d);}catch(e){}};\n"
"  ws.onclose=ws.onerror=()=>{connected=false;ws=null;setConnBadge('disconnected');stopWarmup();setTimeout(connectWS,3000);};\n"
"}\n"
"\n"
"function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}\n"
"\n"
"function updateUI(d){\n"
"  const alerts=[];\n"
"  if(d.gasAlert)   alerts.push('GAS/ASAP');\n"
"  if(d.waterAlert) alerts.push('AIR BERLEBIH');\n"
"  if(d.flameAlert) alerts.push('API');\n"
"  const banner=document.getElementById('alertBanner');\n"
"  if(alerts.length){\n"
"    banner.classList.add('active');\n"
"    document.getElementById('alertText').textContent='\\u26A0 BAHAYA: '+alerts.join(' + ')+'!';\n"
"  } else {banner.classList.remove('active');}\n"
"\n"
"  if(d.mq2Ready){\n"
"    document.getElementById('gasVal').textContent=d.gasADC;\n"
"    const gPct=Math.min(100,(d.gasADC/4095)*100);\n"
"    const gBar=document.getElementById('gasBar');\n"
"    gBar.style.width=gPct+'%';\n"
"    gBar.className='sensor-bar'+(d.gasAlert?' alert':gPct>40?' warn':'');\n"
"    const gp=document.getElementById('gasPill');\n"
"    gp.className='status-pill '+(d.gasAlert?'danger':'ok');\n"
"    gp.textContent=d.gasAlert?'ASAP TERDETEKSI':'Normal';\n"
"  }\n"
"\n"
"  document.getElementById('waterVal').textContent=d.waterADC;\n"
"  const wPct=Math.min(100,(d.waterADC/4095)*100);\n"
"  const wBar=document.getElementById('waterBar');\n"
"  wBar.style.width=wPct+'%';\n"
"  wBar.className='sensor-bar'+(d.waterAlert?' alert':wPct>30?' warn':'');\n"
"  const wp=document.getElementById('waterPill');\n"
"  wp.className='status-pill '+(d.waterAlert?'danger':'ok');\n"
"  wp.textContent=d.waterAlert?'AIR BERLEBIH':'Aman';\n"
"\n"
"  document.getElementById('flameADC').textContent=d.flameADC;\n"
"  document.getElementById('flameD0').textContent=d.flameD0?'HIGH (Api!)':'LOW (Aman)';\n"
"  const fp=document.getElementById('flamePill');\n"
"  fp.className='status-pill '+(d.flameAlert?'danger':'ok');\n"
"  fp.textContent=d.flameAlert?'API TERDETEKSI':'Aman';\n"
"\n"
"  const bp=document.getElementById('btnPill');\n"
"  bp.className='btn-pill '+(d.btnPressed?'pressed':'released');\n"
"  bp.textContent=d.btnPressed?'Pressed':'Released';\n"
"\n"
"  motorState=d.motorState||'stop';\n"
"  motorSpeed=d.motorSpeed||200;\n"
"  document.getElementById('motorSpeed').value=motorSpeed;\n"
"  document.getElementById('motorSpeedVal').textContent=motorSpeed;\n"
"  document.getElementById('motorSpeedPct').textContent=Math.round(motorSpeed/255*100)+'%';\n"
"  updateMotorUI();\n"
"  updateMotorPresetHighlight(motorSpeed);\n"
"\n"
"  if(!d.mq2Ready){\n"
"    document.getElementById('warmupSection').style.display='block';\n"
"    document.getElementById('warmupDone').style.display='none';\n"
"    if(!warmupStartTime) startWarmupAnim();\n"
"  } else {\n"
"    document.getElementById('warmupSection').style.display='none';\n"
"    document.getElementById('warmupDone').style.display='block';\n"
"    stopWarmup(); mq2Ready=true;\n"
"  }\n"
"\n"
"  updateServoUI(1,d.servo1); updateServoUI(2,d.servo2); updateServoUI(3,d.servo3);\n"
"}\n"
"\n"
"function updateMotorUI(){\n"
"  const st=document.getElementById('motorStatusText');\n"
"  const bf=document.getElementById('btnFwd');\n"
"  const bb=document.getElementById('btnBwd');\n"
"  const pi=document.getElementById('motorPinInfo');\n"
"  bf.className='motor-btn';\n"
"  bb.className='motor-btn';\n"
"  if(motorState==='fwd'){\n"
"    st.textContent='MAJU'; st.style.color='var(--ok)';\n"
"    bf.classList.add('active-fwd');\n"
"    pi.textContent='IN1:H \\u00a0 IN2:L \\u00a0 ENA:'+motorSpeed;\n"
"  } else if(motorState==='bwd'){\n"
"    st.textContent='MUNDUR'; st.style.color='var(--accent2)';\n"
"    bb.classList.add('active-bwd');\n"
"    pi.textContent='IN1:L \\u00a0 IN2:H \\u00a0 ENA:'+motorSpeed;\n"
"  } else {\n"
"    st.textContent='STOP'; st.style.color='var(--muted)';\n"
"    pi.textContent='IN1:L \\u00a0 IN2:L \\u00a0 ENA:0';\n"
"  }\n"
"}\n"
"\n"
"/* Toggle: tekan maju saat sudah maju → stop. Begitu juga mundur. */\n"
"function motorToggle(dir){\n"
"  if(motorState===dir){motorCmd('stop');}\n"
"  else {motorCmd(dir);}\n"
"}\n"
"\n"
"function motorCmd(dir){\n"
"  send({cmd:'motor',dir:dir,speed:motorSpeed});\n"
"}\n"
"\n"
"function motorPreset(v){\n"
"  motorSpeed=v;\n"
"  document.getElementById('motorSpeed').value=v;\n"
"  document.getElementById('motorSpeedVal').textContent=v;\n"
"  document.getElementById('motorSpeedPct').textContent=Math.round(v/255*100)+'%';\n"
"  updateMotorPresetHighlight(v);\n"
"  /* Langsung kirim kecepatan baru, arah ikut state sekarang */\n"
"  send({cmd:'motor',dir:motorState,speed:v});\n"
"}\n"
"\n"
"function updateMotorPresetHighlight(v){\n"
"  const btns=document.querySelectorAll('.motor-preset-btn');\n"
"  MOTOR_PRESETS.forEach((p,i)=>{\n"
"    btns[i].className='motor-preset-btn'+(p===v?' active-preset':'');\n"
"  });\n"
"}\n"
"\n"
"function speedInput(v){\n"
"  v=parseInt(v); motorSpeed=v;\n"
"  document.getElementById('motorSpeedVal').textContent=v;\n"
"  document.getElementById('motorSpeedPct').textContent=Math.round(v/255*100)+'%';\n"
"  updateMotorPresetHighlight(v);\n"
"}\n"
"\n"
"function speedSend(v){\n"
"  v=parseInt(v); motorSpeed=v;\n"
"  /* Kirim ke ESP32, arah pakai motorState saat ini */\n"
"  send({cmd:'motor',dir:motorState,speed:v});\n"
"}\n"
"\n"
"function updateServoUI(id,angle){\n"
"  document.getElementById('sa'+id).innerHTML=angle+' <span>deg</span>';\n"
"  document.getElementById('sl'+id).value=angle;\n"
"  rotateNeedle(id,angle);\n"
"}\n"
"\n"
"function rotateNeedle(id,angle){\n"
"  const n=document.getElementById('needle'+id); if(!n)return;\n"
"  const rad=((180-angle)/180)*Math.PI,cx=40,cy=45,len=28;\n"
"  const ex=cx+len*Math.cos(Math.PI-rad),ey=cy-len*Math.sin(Math.PI-rad);\n"
"  n.setAttribute('x2',ex.toFixed(1)); n.setAttribute('y2',ey.toFixed(1));\n"
"}\n"
"\n"
"function servoInput(id,val){\n"
"  val=parseInt(val);\n"
"  document.getElementById('sa'+id).innerHTML=val+' <span>deg</span>';\n"
"  rotateNeedle(id,val);\n"
"  document.getElementById('sc'+id).classList.add('active');\n"
"}\n"
"\n"
"let servoTimer={};\n"
"function servoSend(id,val){\n"
"  clearTimeout(servoTimer[id]);\n"
"  servoTimer[id]=setTimeout(()=>{send({cmd:'servo',id:id,angle:parseInt(val)});document.getElementById('sc'+id).classList.remove('active');},50);\n"
"}\n"
"\n"
"function servoPreset(id,angle){\n"
"  document.getElementById('sl'+id).value=angle;\n"
"  servoInput(id,angle); servoSend(id,angle);\n"
"}\n"
"\n"
"function startWarmupAnim(){\n"
"  warmupStartTime=Date.now();\n"
"  warmupInterval=setInterval(()=>{\n"
"    const elapsed=(Date.now()-warmupStartTime)/1000,total=20;\n"
"    const pct=Math.min(100,(elapsed/total)*100),remaining=Math.max(0,Math.ceil(total-elapsed));\n"
"    document.getElementById('warmupFill').style.width=pct+'%';\n"
"    document.getElementById('warmupSec').textContent=remaining+'s';\n"
"  },200);\n"
"}\n"
"\n"
"function stopWarmup(){if(warmupInterval){clearInterval(warmupInterval);warmupInterval=null;}warmupStartTime=null;}\n"
"\n"
"[1,2,3].forEach(id=>rotateNeedle(id,0));\n"
"connectWS();\n"
"</script>\n"
"</body>\n"
"</html>\n";

// ─────────────────────────────────────────────────────────────
//  OBJEK
// ─────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2, servo3;
WebServer        server(80);
WebSocketsServer webSocket(81);

// ─────────────────────────────────────────────────────────────
//  STATE GLOBAL
// ─────────────────────────────────────────────────────────────
bool   gasAlert   = false;
bool   waterAlert = false;
bool   flameAlert = false;

int    servoAngle1 = 0;
int    servoAngle2 = 0;
int    servoAngle3 = 0;

// Motor
String   motorState = "stop";   // "fwd" | "bwd" | "stop"
int      motorSpeed = 200;      // 0–255

// Push Button
bool     btnPressed   = false;
bool     btnLastState = HIGH;
uint32_t btnDebounce  = 0;

// MQ-2
int      mq2Baseline   = 0;
int      mq2ThreshOn   = 9999;
int      mq2ThreshOff  = 9999;
bool     mq2WarmupDone = false;
uint32_t mq2WarmupStart = 0;
uint32_t mq2LastRead    = 0;

// Water & Flame
uint32_t waterLastRead = 0;
uint32_t flameLastRead = 0;

// RGB Blink
struct RGBColor { uint8_t r, g, b; };
bool     rgbBlinking   = false;
bool     rgbBlinkState = false;
uint32_t rgbBlinkTimer = 0;
RGBColor rgbBlink1 = {0,0,0};
RGBColor rgbBlink2 = {0,0,0};

const RGBColor COL_OFF     = {  0,   0,   0};
const RGBColor COL_FLAME_A = {255,   0,   0};
const RGBColor COL_FLAME_B = {255,  60,   0};
const RGBColor COL_GAS_A   = {180,   0, 255};
const RGBColor COL_GAS_B   = {255,   0, 180};
const RGBColor COL_WATER_A = {  0, 180, 255};
const RGBColor COL_WATER_B = {255, 255, 255};

// LCD
bool     lcdClearPending = false;
uint32_t lcdClearTimer   = 0;

// Sensor data
int  lastGasVal   = 0;
int  lastWaterVal = 0;
int  lastFlameADC = 0;
bool lastFlameD0  = false;

uint32_t lastWsBroadcast = 0;

// ─────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────
void handleGasSensor(uint32_t now);
void handleWaterSensor(uint32_t now);
void handleFlameSensor(uint32_t now);
void handleButton(uint32_t now);
void handleRGB(uint32_t now);
void writeRGB(RGBColor c);
void setRGBBlink(RGBColor c1, RGBColor c2);
void setRGBOff();
void setRGBSolid(RGBColor c);
void setMotor(String dir, int spd);
void stopMotor();
void updateLCD();
void stopBuzzer();
void broadcastStatus();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
String buildStatusJson();

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SMART FACTORY BOOTING ===");

  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_BUTTON,     INPUT_PULLUP);
  pinMode(PIN_GAS_AOUT,   INPUT);
  pinMode(PIN_GAS_DOUT,   INPUT);
  pinMode(PIN_WATER_AOUT, INPUT);
  pinMode(PIN_FLAME_AOUT, INPUT);
  pinMode(PIN_FLAME_DOUT, INPUT_PULLDOWN);

  // Motor DC
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  ledcAttach(PIN_MOTOR_ENA, PWM_MOTOR_FREQ, PWM_MOTOR_RES);
  stopMotor();

  // RGB
  ledcAttach(PIN_RGB_R, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_G, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_B, PWM_FREQ, PWM_RES);
  setRGBOff();

  analogReadResolution(12);
  digitalWrite(PIN_BUZZER, LOW);

  // ========== SERVO SETUP ==========
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);

  servo1.attach(PIN_SERVO1, 500, 2400);
  delay(20);
  servo2.attach(PIN_SERVO2, 500, 2400);
  delay(20);
  servo3.attach(PIN_SERVO3, 500, 2400);
  delay(20);

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  delay(200);

  Serial.print("[Servo] servo2 attached? ");
  Serial.println(servo2.attached() ? "YES" : "NO");
  // ====================================

  Wire.begin(21, 22);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1); lcd.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print("."); tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Buka browser: http://%s\n", WiFi.localIP().toString().c_str());
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("IP:");
    lcd.setCursor(0, 1); lcd.print(WiFi.localIP().toString());
    delay(2000);
  } else {
    Serial.println("\n[WiFi] GAGAL! Cek SSID/password.");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi GAGAL!");
    lcd.setCursor(0, 1); lcd.print("Cek kredensial");
  }

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", DASHBOARD_HTML);
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  mq2WarmupStart = millis();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("MQ2 warm-up...");
  lcd.setCursor(0, 1); lcd.print(WiFi.localIP().toString());

  Serial.println("[SYSTEM] Siap.");
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();
  webSocket.loop();
  server.handleClient();

  handleRGB(now);
  handleGasSensor(now);
  handleWaterSensor(now);
  handleFlameSensor(now);
  handleButton(now);

  if (lcdClearPending && (now - lcdClearTimer >= LCD_CLEAR_DELAY)) {
    lcdClearPending = false;
    updateLCD();
  }

  if (now - lastWsBroadcast >= WS_BROADCAST_MS) {
    lastWsBroadcast = now;
    broadcastStatus();
  }
}

// ─────────────────────────────────────────────────────────────
//  PUSH BUTTON
// ─────────────────────────────────────────────────────────────
void handleButton(uint32_t now) {
  bool reading = digitalRead(PIN_BUTTON);

  if (reading != btnLastState) {
    btnDebounce = now;
  }

  if ((now - btnDebounce) >= BTN_DEBOUNCE_MS) {
    bool currentlyPressed = (reading == LOW);

    if (currentlyPressed && !btnPressed) {
      btnPressed = true;
      // Toggle: maju/stop
      if (motorState == "stop" || motorState == "bwd") {
        setMotor("fwd", motorSpeed);
      } else {
        stopMotor();
      }
      broadcastStatus();
      Serial.printf("[BTN] Ditekan → Motor: %s\n", motorState.c_str());
    } else if (!currentlyPressed && btnPressed) {
      btnPressed = false;
      Serial.println("[BTN] Dilepas");
      broadcastStatus();
    }
  }

  btnLastState = reading;
}

// ─────────────────────────────────────────────────────────────
//  MOTOR DC
// ─────────────────────────────────────────────────────────────
void setMotor(String dir, int spd) {
  spd = constrain(spd, 0, 255);
  motorSpeed = spd;
  motorState = dir;

  if (dir == "fwd") {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    ledcWrite(PIN_MOTOR_ENA, spd);
    Serial.printf("[MOTOR] Maju, speed=%d\n", spd);
  } else if (dir == "bwd") {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    ledcWrite(PIN_MOTOR_ENA, spd);
    Serial.printf("[MOTOR] Mundur, speed=%d\n", spd);
  } else {
    stopMotor();
    return;
  }
  updateLCD();
}

void stopMotor() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  ledcWrite(PIN_MOTOR_ENA, 0);
  motorState = "stop";
  Serial.println("[MOTOR] Stop");
  updateLCD();
}

// ─────────────────────────────────────────────────────────────
//  WEBSOCKET EVENT
// ─────────────────────────────────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client #%d terhubung.\n", num);
    String initJson = buildStatusJson();
    webSocket.sendTXT(num, initJson);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client #%d terputus.\n", num);
    return;
  }
  if (type != WStype_TEXT) return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  // ── Servo command ──
  if (strcmp(cmd, "servo") == 0) {
    int id    = doc["id"] | 0;
    int angle = constrain((int)(doc["angle"] | 0), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    if (id == 1) {
      if (!servo1.attached()) servo1.attach(PIN_SERVO1, 500, 2400);
      servoAngle1 = angle;
      servo1.write(angle);
      Serial.printf("[WS] Servo1 → %d°\n", angle);
    }
    else if (id == 2) {
      if (!servo2.attached()) servo2.attach(PIN_SERVO2, 500, 2400);
      servoAngle2 = angle;
      servo2.write(angle);
      delay(10);
      Serial.printf("[WS] Servo2 → %d° (attached=%d)\n", angle, servo2.attached());
    }
    else if (id == 3) {
      if (!servo3.attached()) servo3.attach(PIN_SERVO3, 500, 2400);
      servoAngle3 = angle;
      servo3.write(angle);
      Serial.printf("[WS] Servo3 → %d°\n", angle);
    }

    updateLCD();
    broadcastStatus();
    return;
  }

  // ── Motor command ──
  if (strcmp(cmd, "motor") == 0) {
    const char* dir = doc["dir"] | "stop";
    int spd = constrain((int)(doc["speed"] | 200), 0, 255);

    if ((gasAlert || waterAlert || flameAlert) && strcmp(dir, "stop") != 0) {
      Serial.println("[MOTOR] Ditolak — ada alert bahaya aktif!");
      broadcastStatus();
      return;
    }

    setMotor(String(dir), spd);
    broadcastStatus();
    return;
  }
}

// ─────────────────────────────────────────────────────────────
//  STATUS JSON
// ─────────────────────────────────────────────────────────────
String buildStatusJson() {
  StaticJsonDocument<512> doc;
  doc["type"]       = "status";
  doc["gasAlert"]   = gasAlert;
  doc["waterAlert"] = waterAlert;
  doc["flameAlert"] = flameAlert;
  doc["gasADC"]     = lastGasVal;
  doc["waterADC"]   = lastWaterVal;
  doc["flameADC"]   = lastFlameADC;
  doc["flameD0"]    = lastFlameD0;
  doc["servo1"]     = servoAngle1;
  doc["servo2"]     = servoAngle2;
  doc["servo3"]     = servoAngle3;
  doc["mq2Ready"]   = mq2WarmupDone;
  doc["motorState"] = motorState;
  doc["motorSpeed"] = motorSpeed;
  doc["btnPressed"] = btnPressed;
  String out;
  serializeJson(doc, out);
  return out;
}

void broadcastStatus() {
  String json = buildStatusJson();
  webSocket.broadcastTXT(json);
}

// ─────────────────────────────────────────────────────────────
//  GAS SENSOR MQ-2
// ─────────────────────────────────────────────────────────────
void handleGasSensor(uint32_t now) {
  if (!mq2WarmupDone) {
    uint32_t elapsed = now - mq2WarmupStart;
    static uint32_t lastCountdown = 0;
    if (now - lastCountdown >= 1000) {
      lastCountdown = now;
      int sisa = (int)((MQ2_WARMUP_MS - elapsed) / 1000) + 1;
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("MQ2 warm-up:");
      lcd.setCursor(0, 1); lcd.print(sisa); lcd.print("s | ");
      lcd.print(WiFi.localIP().toString().substring(
        WiFi.localIP().toString().lastIndexOf('.') + 1));
    }
    if (elapsed < MQ2_WARMUP_MS) return;

    mq2WarmupDone = true;
    long sum = 0;
    for (int i = 0; i < 20; i++) sum += analogRead(PIN_GAS_AOUT);
    mq2Baseline  = (int)(sum / 20);
    mq2ThreshOn  = mq2Baseline + MQ2_THRESHOLD_DELTA;
    mq2ThreshOff = mq2Baseline + MQ2_THRESHOLD_DELTA - MQ2_THRESHOLD_HYST;
    Serial.printf("[MQ-2] Baseline=%d ThreshON=%d ThreshOFF=%d\n",
                  mq2Baseline, mq2ThreshOn, mq2ThreshOff);
    updateLCD();
    return;
  }

  if (now - mq2LastRead < MQ2_READ_INTERVAL) return;
  mq2LastRead = now;
  lastGasVal = analogRead(PIN_GAS_AOUT);

  if (!gasAlert && lastGasVal > mq2ThreshOn) {
    gasAlert = true;
    stopMotor();
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_GAS_A, COL_GAS_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BAHAYA ASAP!!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastGasVal);
  } else if (gasAlert && lastGasVal < mq2ThreshOff) {
    gasAlert = false;
    if (!waterAlert && !flameAlert) {
      stopBuzzer(); setRGBOff();
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Udara bersih.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  WATER LEVEL SENSOR
// ─────────────────────────────────────────────────────────────
void handleWaterSensor(uint32_t now) {
  if (!mq2WarmupDone) return;
  if (now - waterLastRead < WATER_READ_INTERVAL) return;
  waterLastRead = now;
  lastWaterVal = analogRead(PIN_WATER_AOUT);

  if (!waterAlert && lastWaterVal > WATER_THRESHOLD_ON) {
    waterAlert = true;
    stopMotor();
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_WATER_A, COL_WATER_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BANJIR/AIR !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastWaterVal);
  } else if (waterAlert && lastWaterVal < WATER_THRESHOLD_OFF) {
    waterAlert = false;
    if (!gasAlert && !flameAlert) {
      stopBuzzer(); setRGBOff();
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Air aman.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  FLAME SENSOR
// ─────────────────────────────────────────────────────────────
void handleFlameSensor(uint32_t now) {
  if (!mq2WarmupDone) return;
  if (now - flameLastRead < FLAME_READ_INTERVAL) return;
  flameLastRead = now;
  lastFlameD0  = (digitalRead(PIN_FLAME_DOUT) == HIGH);
  lastFlameADC = analogRead(PIN_FLAME_AOUT);

  if (!flameAlert && lastFlameD0) {
    flameAlert = true;
    stopMotor();
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_FLAME_A, COL_FLAME_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BAHAYA API !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastFlameADC);
  } else if (flameAlert && !lastFlameD0) {
    flameAlert = false;
    if (!gasAlert && !waterAlert) {
      stopBuzzer(); setRGBOff();
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Api padam.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  BUZZER
// ─────────────────────────────────────────────────────────────
void stopBuzzer() {
  digitalWrite(PIN_BUZZER, LOW);
}

// ─────────────────────────────────────────────────────────────
//  RGB LED
// ─────────────────────────────────────────────────────────────
void writeRGB(RGBColor c) {
  ledcWrite(PIN_RGB_R, c.r);
  ledcWrite(PIN_RGB_G, c.g);
  ledcWrite(PIN_RGB_B, c.b);
}
void setRGBSolid(RGBColor c) { rgbBlinking = false; writeRGB(c); }
void setRGBOff()              { setRGBSolid(COL_OFF); }
void setRGBBlink(RGBColor c1, RGBColor c2) {
  rgbBlink1 = c1; rgbBlink2 = c2;
  rgbBlinking = true; rgbBlinkState = true;
  rgbBlinkTimer = millis(); writeRGB(c1);
}
void handleRGB(uint32_t now) {
  if (!rgbBlinking || now - rgbBlinkTimer < RGB_BLINK_MS) return;
  rgbBlinkTimer = now;
  rgbBlinkState = !rgbBlinkState;
  writeRGB(rgbBlinkState ? rgbBlink1 : rgbBlink2);
}

// ─────────────────────────────────────────────────────────────
//  LCD
// ─────────────────────────────────────────────────────────────
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("M:");
  if      (motorState == "fwd")  lcd.print("MAJU");
  else if (motorState == "bwd")  lcd.print("MNDUR");
  else                           lcd.print("STOP");
  lcd.print(" S:");
  lcd.print(motorSpeed);
  lcd.setCursor(0, 1);
  lcd.print("S1:"); lcd.print(servoAngle1);
  lcd.print(" S2:"); lcd.print(servoAngle2);
}
