/*
 * ============================================================
 *   SMART FACTORY - SERVO MANUAL CONTROL (Web IoT)
 *   Platform  : ESP32 DevKit C v4
 *   Author    : Mc.Zminecrafter18
 *
 *   ★ CARA PAKAI:
 *   1. Isi WIFI_SSID dan WIFI_PASS di bawah
 *   2. Upload sketch (1 file saja: .ino)
 *   3. Buka Serial Monitor → catat IP yang tampil
 *   4. Ketik IP di browser → dashboard langsung muncul!
 *      Tidak perlu file HTML terpisah.
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
 *  │ Relay (Motor)       │ 19       │                         │
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

const char DASHBOARD_HTML[] PROGMEM =
  "<!DOCTYPE html>\n"
  "<html lang=\"id\">\n"
  "<head>\n"
  "<meta charset=\"UTF-8\">\n"
  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
  "<title>Smart Factory — Servo Control</title>\n"
  "<style>\n"
  "  @import url('https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&family=IBM+Plex+Sans:wght@300;400;600&display=swap');\n"
  "\n"
  "  :root {\n"
  "    --bg: #0d0f14;\n"
  "    --surface: #161a23;\n"
  "    --surface2: #1e2330;\n"
  "    --border: #2a3045;\n"
  "    --accent: #00e5ff;\n"
  "    --accent2: #ff6b35;\n"
  "    --text: #e8ecf0;\n"
  "    --muted: #6b7a8d;\n"
  "    --danger: #ff3b3b;\n"
  "    --warn: #ffb020;\n"
  "    --ok: #22d55a;\n"
  "    --font-mono: 'Space Mono', monospace;\n"
  "    --font-sans: 'IBM Plex Sans', sans-serif;\n"
  "    --radius: 8px;\n"
  "  }\n"
  "\n"
  "  * { box-sizing: border-box; margin: 0; padding: 0; }\n"
  "\n"
  "  body {\n"
  "    background: var(--bg);\n"
  "    color: var(--text);\n"
  "    font-family: var(--font-sans);\n"
  "    min-height: 100vh;\n"
  "    padding: 0;\n"
  "  }\n"
  "\n"
  "  /* ── HEADER ───────────────────────────── */\n"
  "  header {\n"
  "    background: var(--surface);\n"
  "    border-bottom: 1px solid var(--border);\n"
  "    padding: 14px 24px;\n"
  "    display: flex;\n"
  "    align-items: center;\n"
  "    justify-content: space-between;\n"
  "    position: sticky;\n"
  "    top: 0;\n"
  "    z-index: 100;\n"
  "  }\n"
  "\n"
  "  .logo {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 13px;\n"
  "    letter-spacing: 0.12em;\n"
  "    color: var(--accent);\n"
  "    text-transform: uppercase;\n"
  "  }\n"
  "\n"
  "  .logo span { color: var(--muted); }\n"
  "\n"
  "  .conn-badge {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 11px;\n"
  "    padding: 4px 12px;\n"
  "    border-radius: 20px;\n"
  "    border: 1px solid;\n"
  "    letter-spacing: 0.08em;\n"
  "    text-transform: uppercase;\n"
  "    transition: all 0.3s;\n"
  "  }\n"
  "  .conn-badge.disconnected { border-color: var(--danger); color: var(--danger); }\n"
  "  .conn-badge.connected    { border-color: var(--ok);     color: var(--ok); }\n"
  "  .conn-badge.connecting   { border-color: var(--warn);   color: var(--warn); }\n"
  "\n"
  "  /* ── LAYOUT ───────────────────────────── */\n"
  "  main { max-width: 900px; margin: 0 auto; padding: 24px 20px; }\n"
  "\n"
  "  /* ── IP CONNECT BAR ───────────────────── */\n"
  "  .connect-bar {\n"
  "    background: var(--surface);\n"
  "    border: 1px solid var(--border);\n"
  "    border-radius: var(--radius);\n"
  "    padding: 16px 20px;\n"
  "    display: flex;\n"
  "    align-items: center;\n"
  "    gap: 12px;\n"
  "    margin-bottom: 20px;\n"
  "  }\n"
  "\n"
  "  .connect-bar label {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 11px;\n"
  "    color: var(--muted);\n"
  "    letter-spacing: 0.1em;\n"
  "    text-transform: uppercase;\n"
  "    white-space: nowrap;\n"
  "  }\n"
  "\n"
  "  .connect-bar input {\n"
  "    flex: 1;\n"
  "    background: var(--bg);\n"
  "    border: 1px solid var(--border);\n"
  "    border-radius: var(--radius);\n"
  "    color: var(--text);\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 13px;\n"
  "    padding: 8px 12px;\n"
  "    outline: none;\n"
  "    transition: border-color 0.2s;\n"
  "  }\n"
  "  .connect-bar input:focus { border-color: var(--accent); }\n"
  "\n"
  "  .btn-connect {\n"
  "    background: var(--accent);\n"
  "    color: var(--bg);\n"
  "    border: none;\n"
  "    border-radius: var(--radius);\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 12px;\n"
  "    font-weight: 700;\n"
  "    padding: 9px 20px;\n"
  "    cursor: pointer;\n"
  "    letter-spacing: 0.08em;\n"
  "    text-transform: uppercase;\n"
  "    transition: opacity 0.2s;\n"
  "    white-space: nowrap;\n"
  "  }\n"
  "  .btn-connect:hover { opacity: 0.85; }\n"
  "  .btn-connect:disabled { opacity: 0.4; cursor: not-allowed; }\n"
  "\n"
  "  /* ── ALERT BANNER ─────────────────────── */\n"
  "  .alert-banner {\n"
  "    display: none;\n"
  "    align-items: center;\n"
  "    gap: 10px;\n"
  "    background: rgba(255, 59, 59, 0.12);\n"
  "    border: 1px solid var(--danger);\n"
  "    border-radius: var(--radius);\n"
  "    padding: 12px 16px;\n"
  "    margin-bottom: 16px;\n"
  "    font-size: 13px;\n"
  "    color: var(--danger);\n"
  "    animation: pulse-border 1s infinite;\n"
  "  }\n"
  "  .alert-banner.active { display: flex; }\n"
  "\n"
  "  @keyframes pulse-border {\n"
  "    0%, 100% { border-color: var(--danger); }\n"
  "    50%       { border-color: transparent; }\n"
  "  }\n"
  "\n"
  "  .alert-dot {\n"
  "    width: 8px; height: 8px;\n"
  "    border-radius: 50%;\n"
  "    background: var(--danger);\n"
  "    animation: blink 0.6s infinite;\n"
  "    flex-shrink: 0;\n"
  "  }\n"
  "  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.2} }\n"
  "\n"
  "  /* ── GRID ─────────────────────────────── */\n"
  "  .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin-bottom: 16px; }\n"
  "  .grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; margin-bottom: 16px; }\n"
  "\n"
  "  @media (max-width: 640px) {\n"
  "    .grid-2 { grid-template-columns: 1fr; }\n"
  "    .grid-3 { grid-template-columns: 1fr; }\n"
  "  }\n"
  "\n"
  "  /* ── CARDS ────────────────────────────── */\n"
  "  .card {\n"
  "    background: var(--surface);\n"
  "    border: 1px solid var(--border);\n"
  "    border-radius: var(--radius);\n"
  "    padding: 16px;\n"
  "  }\n"
  "\n"
  "  .card-title {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 10px;\n"
  "    letter-spacing: 0.15em;\n"
  "    color: var(--muted);\n"
  "    text-transform: uppercase;\n"
  "    margin-bottom: 14px;\n"
  "  }\n"
  "\n"
  "  /* ── SERVO CARD ───────────────────────── */\n"
  "  .servo-card {\n"
  "    background: var(--surface);\n"
  "    border: 1px solid var(--border);\n"
  "    border-radius: var(--radius);\n"
  "    padding: 18px;\n"
  "    position: relative;\n"
  "    overflow: hidden;\n"
  "    transition: border-color 0.2s;\n"
  "  }\n"
  "  .servo-card.active { border-color: var(--accent); }\n"
  "\n"
  "  .servo-header {\n"
  "    display: flex;\n"
  "    align-items: center;\n"
  "    justify-content: space-between;\n"
  "    margin-bottom: 14px;\n"
  "  }\n"
  "\n"
  "  .servo-name {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 11px;\n"
  "    letter-spacing: 0.12em;\n"
  "    color: var(--muted);\n"
  "    text-transform: uppercase;\n"
  "  }\n"
  "\n"
  "  .servo-angle {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 28px;\n"
  "    font-weight: 700;\n"
  "    color: var(--accent);\n"
  "    letter-spacing: -0.02em;\n"
  "  }\n"
  "\n"
  "  .servo-angle span { font-size: 14px; color: var(--muted); }\n"
  "\n"
  "  /* Visual arc */\n"
  "  .servo-vis {\n"
  "    display: flex;\n"
  "    justify-content: center;\n"
  "    margin: 10px 0 14px;\n"
  "  }\n"
  "\n"
  "  .servo-vis svg { overflow: visible; }\n"
  "\n"
  "  /* Slider */\n"
  "  input[type=range] {\n"
  "    -webkit-appearance: none;\n"
  "    width: 100%;\n"
  "    height: 4px;\n"
  "    background: var(--border);\n"
  "    border-radius: 2px;\n"
  "    outline: none;\n"
  "    cursor: pointer;\n"
  "  }\n"
  "  input[type=range]::-webkit-slider-thumb {\n"
  "    -webkit-appearance: none;\n"
  "    width: 18px; height: 18px;\n"
  "    border-radius: 50%;\n"
  "    background: var(--accent);\n"
  "    border: 2px solid var(--bg);\n"
  "    transition: transform 0.1s;\n"
  "  }\n"
  "  input[type=range]::-webkit-slider-thumb:active { transform: scale(1.2); }\n"
  "\n"
  "  .servo-presets {\n"
  "    display: flex;\n"
  "    gap: 6px;\n"
  "    margin-top: 12px;\n"
  "  }\n"
  "\n"
  "  .preset-btn {\n"
  "    flex: 1;\n"
  "    background: var(--bg);\n"
  "    border: 1px solid var(--border);\n"
  "    border-radius: var(--radius);\n"
  "    color: var(--muted);\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 11px;\n"
  "    padding: 7px 4px;\n"
  "    cursor: pointer;\n"
  "    text-align: center;\n"
  "    transition: all 0.15s;\n"
  "  }\n"
  "  .preset-btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,229,255,0.05); }\n"
  "  .preset-btn:active { transform: scale(0.97); }\n"
  "\n"
  "  /* ── SENSOR CARDS ─────────────────────── */\n"
  "  .sensor-value {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 22px;\n"
  "    font-weight: 700;\n"
  "    margin-bottom: 4px;\n"
  "  }\n"
  "\n"
  "  .sensor-label {\n"
  "    font-size: 12px;\n"
  "    color: var(--muted);\n"
  "    margin-bottom: 10px;\n"
  "  }\n"
  "\n"
  "  .sensor-bar-wrap {\n"
  "    background: var(--bg);\n"
  "    border-radius: 3px;\n"
  "    height: 4px;\n"
  "    overflow: hidden;\n"
  "    margin-top: 8px;\n"
  "  }\n"
  "\n"
  "  .sensor-bar {\n"
  "    height: 100%;\n"
  "    border-radius: 3px;\n"
  "    transition: width 0.4s;\n"
  "    background: var(--ok);\n"
  "  }\n"
  "  .sensor-bar.warn  { background: var(--warn); }\n"
  "  .sensor-bar.alert { background: var(--danger); }\n"
  "\n"
  "  .status-pill {\n"
  "    display: inline-flex;\n"
  "    align-items: center;\n"
  "    gap: 5px;\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 10px;\n"
  "    letter-spacing: 0.08em;\n"
  "    text-transform: uppercase;\n"
  "    padding: 4px 10px;\n"
  "    border-radius: 20px;\n"
  "    border: 1px solid;\n"
  "  }\n"
  "  .status-pill.ok      { border-color: var(--ok);     color: var(--ok); }\n"
  "  .status-pill.danger  { border-color: var(--danger);  color: var(--danger);\n"
  "                         background: rgba(255,59,59,0.08); }\n"
  "  .status-pill.offline { border-color: var(--muted);   color: var(--muted); }\n"
  "\n"
  "  /* ── MACHINE TOGGLE ───────────────────── */\n"
  "  .machine-row {\n"
  "    display: flex;\n"
  "    align-items: center;\n"
  "    justify-content: space-between;\n"
  "    gap: 12px;\n"
  "  }\n"
  "\n"
  "  .machine-info { flex: 1; }\n"
  "  .machine-label { font-family: var(--font-mono); font-size: 11px; letter-spacing: 0.12em; color: var(--muted); text-transform: uppercase; margin-bottom: 4px; }\n"
  "  .machine-status { font-size: 20px; font-weight: 600; }\n"
  "  .machine-status.on  { color: var(--ok); }\n"
  "  .machine-status.off { color: var(--danger); }\n"
  "\n"
  "  .toggle-btn {\n"
  "    width: 64px; height: 32px;\n"
  "    border-radius: 16px;\n"
  "    border: 1px solid var(--border);\n"
  "    background: var(--bg);\n"
  "    cursor: pointer;\n"
  "    position: relative;\n"
  "    transition: background 0.25s, border-color 0.25s;\n"
  "  }\n"
  "  .toggle-btn.on { background: rgba(34,213,90,0.15); border-color: var(--ok); }\n"
  "  .toggle-btn::after {\n"
  "    content: '';\n"
  "    position: absolute;\n"
  "    top: 5px; left: 5px;\n"
  "    width: 20px; height: 20px;\n"
  "    border-radius: 50%;\n"
  "    background: var(--muted);\n"
  "    transition: transform 0.25s, background 0.25s;\n"
  "  }\n"
  "  .toggle-btn.on::after { transform: translateX(32px); background: var(--ok); }\n"
  "\n"
  "  /* ── MQ2 WARMUP ───────────────────────── */\n"
  "  .warmup-bar {\n"
  "    background: var(--bg);\n"
  "    border-radius: 3px;\n"
  "    height: 6px;\n"
  "    overflow: hidden;\n"
  "    margin-top: 10px;\n"
  "  }\n"
  "  .warmup-fill {\n"
  "    height: 100%;\n"
  "    background: var(--warn);\n"
  "    border-radius: 3px;\n"
  "    transition: width 1s linear;\n"
  "  }\n"
  "\n"
  "  /* ── SECTION LABEL ────────────────────── */\n"
  "  .section-label {\n"
  "    font-family: var(--font-mono);\n"
  "    font-size: 10px;\n"
  "    letter-spacing: 0.18em;\n"
  "    color: var(--muted);\n"
  "    text-transform: uppercase;\n"
  "    margin: 20px 0 10px;\n"
  "    display: flex;\n"
  "    align-items: center;\n"
  "    gap: 8px;\n"
  "  }\n"
  "  .section-label::after {\n"
  "    content: '';\n"
  "    flex: 1;\n"
  "    height: 1px;\n"
  "    background: var(--border);\n"
  "  }\n"
  "\n"
  "  /* ── FLAME STATUS ─────────────────────── */\n"
  "  .flame-icon {\n"
  "    font-size: 24px;\n"
  "    line-height: 1;\n"
  "  }\n"
  "</style>\n"
  "</head>\n"
  "<body>\n"
  "\n"
  "<header>\n"
  "  <div class=\"logo\">Smart Factory <span>// Servo Control</span></div>\n"
  "  <div class=\"conn-badge disconnected\" id=\"connBadge\" style=\"font-size:11px;padding:4px 12px;\"></div>\n"
  "</header>\n"
  "\n"
  "<main>\n"
  "\n"
  "  <!-- Alert Banner -->\n"
  "  <div class=\"alert-banner\" id=\"alertBanner\">\n"
  "    <div class=\"alert-dot\"></div>\n"
  "    <strong id=\"alertText\">Alert aktif — mesin dimatikan otomatis!</strong>\n"
  "  </div>\n"
  "\n"
  "  <!-- Section: Mesin + Sensor -->\n"
  "  <div class=\"section-label\">Status sistem</div>\n"
  "\n"
  "  <div class=\"grid-2\">\n"
  "\n"
  "    <!-- Machine Card -->\n"
  "    <div class=\"card\">\n"
  "      <div class=\"card-title\">Relay / Mesin</div>\n"
  "      <div class=\"machine-row\">\n"
  "        <div class=\"machine-info\">\n"
  "          <div class=\"machine-label\">Status</div>\n"
  "          <div class=\"machine-status on\" id=\"machineStatus\">ON</div>\n"
  "        </div>\n"
  "        <button class=\"toggle-btn on\" id=\"machineToggle\" onclick=\"toggleMachine()\"></button>\n"
  "      </div>\n"
  "\n"
  "      <!-- MQ2 warmup -->\n"
  "      <div id=\"warmupSection\" style=\"margin-top:16px; display:none;\">\n"
  "        <div style=\"display:flex;justify-content:space-between;align-items:center;\">\n"
  "          <span style=\"font-size:12px;color:var(--muted);\">MQ-2 warm-up</span>\n"
  "          <span style=\"font-family:var(--font-mono);font-size:12px;color:var(--warn);\" id=\"warmupSec\">20s</span>\n"
  "        </div>\n"
  "        <div class=\"warmup-bar\">\n"
  "          <div class=\"warmup-fill\" id=\"warmupFill\" style=\"width:0%\"></div>\n"
  "        </div>\n"
  "      </div>\n"
  "      <div id=\"warmupDone\" style=\"margin-top:12px;font-size:12px;color:var(--ok);font-family:var(--font-mono);display:none;\">&#10003; MQ-2 siap</div>\n"
  "    </div>\n"
  "\n"
  "    <!-- Gas Sensor -->\n"
  "    <div class=\"card\">\n"
  "      <div class=\"card-title\">MQ-2 Gas/Asap</div>\n"
  "      <div class=\"sensor-value\" id=\"gasVal\">—</div>\n"
  "      <div class=\"sensor-label\">ADC raw (0–4095)</div>\n"
  "      <div id=\"gasPill\" class=\"status-pill offline\">Offline</div>\n"
  "      <div class=\"sensor-bar-wrap\">\n"
  "        <div class=\"sensor-bar\" id=\"gasBar\" style=\"width:0%\"></div>\n"
  "      </div>\n"
  "    </div>\n"
  "\n"
  "  </div>\n"
  "\n"
  "  <div class=\"grid-2\">\n"
  "\n"
  "    <!-- Water Level -->\n"
  "    <div class=\"card\">\n"
  "      <div class=\"card-title\">Water Level</div>\n"
  "      <div class=\"sensor-value\" id=\"waterVal\">—</div>\n"
  "      <div class=\"sensor-label\">ADC raw · threshold 800</div>\n"
  "      <div id=\"waterPill\" class=\"status-pill offline\">Offline</div>\n"
  "      <div class=\"sensor-bar-wrap\">\n"
  "        <div class=\"sensor-bar\" id=\"waterBar\" style=\"width:0%\"></div>\n"
  "      </div>\n"
  "    </div>\n"
  "\n"
  "    <!-- Flame Sensor -->\n"
  "    <div class=\"card\">\n"
  "      <div class=\"card-title\">Flame Sensor</div>\n"
  "      <div style=\"display:flex;align-items:center;gap:12px;margin-bottom:8px;\">\n"
  "        <div>\n"
  "          <div class=\"sensor-value\" id=\"flameADC\">—</div>\n"
  "          <div class=\"sensor-label\">ADC (A0)</div>\n"
  "        </div>\n"
  "      </div>\n"
  "      <div id=\"flamePill\" class=\"status-pill offline\">Offline</div>\n"
  "      <div style=\"font-size:11px;color:var(--muted);margin-top:8px;font-family:var(--font-mono);\">\n"
  "        D0: <span id=\"flameD0\">—</span>\n"
  "      </div>\n"
  "    </div>\n"
  "\n"
  "  </div>\n"
  "\n"
  "  <!-- Section: Servo -->\n"
  "  <div class=\"section-label\">Servo control</div>\n"
  "\n"
  "  <div class=\"grid-3\" id=\"servoGrid\">\n"
  "\n"
  "    <!-- Servo 1 -->\n"
  "    <div class=\"servo-card\" id=\"sc1\">\n"
  "      <div class=\"servo-header\">\n"
  "        <div class=\"servo-name\">Servo 1 — pin 33</div>\n"
  "      </div>\n"
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
  "\n"
  "    <!-- Servo 2 -->\n"
  "    <div class=\"servo-card\" id=\"sc2\">\n"
  "      <div class=\"servo-header\">\n"
  "        <div class=\"servo-name\">Servo 2 — pin 25</div>\n"
  "      </div>\n"
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
  "\n"
  "    <!-- Servo 3 -->\n"
  "    <div class=\"servo-card\" id=\"sc3\">\n"
  "      <div class=\"servo-header\">\n"
  "        <div class=\"servo-name\">Servo 3 — pin 18</div>\n"
  "      </div>\n"
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
  "\n"
  "  </div>\n"
  "\n"
  "  <div style=\"height:32px;\"></div>\n"
  "</main>\n"
  "\n"
  "<script>\n"
  "let ws = null;\n"
  "let connected = false;\n"
  "let mq2Ready  = false;\n"
  "let warmupStartTime = null;\n"
  "let warmupInterval  = null;\n"
  "\n"
  "function setConnBadge(state) {\n"
  "  const b = document.getElementById('connBadge');\n"
  "  if (!b) return;\n"
  "  b.className = 'conn-badge ' + state;\n"
  "  b.textContent = {connected:'Connected', disconnected:'Disconnected', connecting:'Connecting...'}[state] || state;\n"
  "}\n"
  "\n"
  "function connectWS() {\n"
  "  const host = window.location.hostname;\n"
  "  setConnBadge('connecting');\n"
  "\n"
  "  ws = new WebSocket('ws://' + host + ':81');\n"
  "\n"
  "  ws.onopen = () => {\n"
  "    connected = true;\n"
  "    setConnBadge('connected');\n"
  "  };\n"
  "\n"
  "  ws.onmessage = (evt) => {\n"
  "    try {\n"
  "      const d = JSON.parse(evt.data);\n"
  "      if (d.type === 'status') updateUI(d);\n"
  "    } catch(e) {}\n"
  "  };\n"
  "\n"
  "  ws.onclose = ws.onerror = () => {\n"
  "    connected = false;\n"
  "    ws = null;\n"
  "    setConnBadge('disconnected');\n"
  "    stopWarmup();\n"
  "    // Auto reconnect setelah 3 detik\n"
  "    setTimeout(connectWS, 3000);\n"
  "  };\n"
  "}\n"
  "\n"
  "function send(obj) {\n"
  "  if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));\n"
  "}\n"
  "\n"
  "// ── Update UI from ESP32 status ─────────────────────────────\n"
  "function updateUI(d) {\n"
  "  const ms = document.getElementById('machineStatus');\n"
  "  const mt = document.getElementById('machineToggle');\n"
  "  ms.textContent = d.machine ? 'ON' : 'OFF';\n"
  "  ms.className   = 'machine-status ' + (d.machine ? 'on' : 'off');\n"
  "  mt.className   = 'toggle-btn ' + (d.machine ? 'on' : '');\n"
  "\n"
  "  const alerts = [];\n"
  "  if (d.gasAlert)   alerts.push('GAS/ASAP');\n"
  "  if (d.waterAlert) alerts.push('AIR BERLEBIH');\n"
  "  if (d.flameAlert) alerts.push('API');\n"
  "  const banner = document.getElementById('alertBanner');\n"
  "  if (alerts.length) {\n"
  "    banner.classList.add('active');\n"
  "    document.getElementById('alertText').textContent =\n"
  "      '⚠ BAHAYA: ' + alerts.join(' + ') + ' — Mesin OFF otomatis!';\n"
  "  } else {\n"
  "    banner.classList.remove('active');\n"
  "  }\n"
  "\n"
  "  if (d.mq2Ready) {\n"
  "    document.getElementById('gasVal').textContent = d.gasADC;\n"
  "    const gPct = Math.min(100, (d.gasADC / 4095) * 100);\n"
  "    const gBar = document.getElementById('gasBar');\n"
  "    gBar.style.width = gPct + '%';\n"
  "    gBar.className = 'sensor-bar' + (d.gasAlert ? ' alert' : gPct > 40 ? ' warn' : '');\n"
  "    const gp = document.getElementById('gasPill');\n"
  "    gp.className = 'status-pill ' + (d.gasAlert ? 'danger' : 'ok');\n"
  "    gp.textContent = d.gasAlert ? 'ASAP TERDETEKSI' : 'Normal';\n"
  "  }\n"
  "\n"
  "  document.getElementById('waterVal').textContent = d.waterADC;\n"
  "  const wPct = Math.min(100, (d.waterADC / 4095) * 100);\n"
  "  const wBar = document.getElementById('waterBar');\n"
  "  wBar.style.width = wPct + '%';\n"
  "  wBar.className = 'sensor-bar' + (d.waterAlert ? ' alert' : wPct > 30 ? ' warn' : '');\n"
  "  const wp = document.getElementById('waterPill');\n"
  "  wp.className = 'status-pill ' + (d.waterAlert ? 'danger' : 'ok');\n"
  "  wp.textContent = d.waterAlert ? 'AIR BERLEBIH' : 'Aman';\n"
  "\n"
  "  document.getElementById('flameADC').textContent = d.flameADC;\n"
  "  document.getElementById('flameD0').textContent  = d.flameD0 ? 'HIGH (Api!)' : 'LOW (Aman)';\n"
  "  const fp = document.getElementById('flamePill');\n"
  "  fp.className = 'status-pill ' + (d.flameAlert ? 'danger' : 'ok');\n"
  "  fp.textContent = d.flameAlert ? 'API TERDETEKSI' : 'Aman';\n"
  "\n"
  "  if (!d.mq2Ready) {\n"
  "    document.getElementById('warmupSection').style.display = 'block';\n"
  "    document.getElementById('warmupDone').style.display    = 'none';\n"
  "    if (!warmupStartTime) startWarmupAnim();\n"
  "  } else {\n"
  "    document.getElementById('warmupSection').style.display = 'none';\n"
  "    document.getElementById('warmupDone').style.display    = 'block';\n"
  "    stopWarmup();\n"
  "    mq2Ready = true;\n"
  "  }\n"
  "\n"
  "  updateServoUI(1, d.servo1);\n"
  "  updateServoUI(2, d.servo2);\n"
  "  updateServoUI(3, d.servo3);\n"
  "}\n"
  "\n"
  "function updateServoUI(id, angle) {\n"
  "  document.getElementById('sa' + id).innerHTML = angle + ' <span>deg</span>';\n"
  "  document.getElementById('sl' + id).value = angle;\n"
  "  rotateNeedle(id, angle);\n"
  "}\n"
  "\n"
  "function rotateNeedle(id, angle) {\n"
  "  const needle = document.getElementById('needle' + id);\n"
  "  if (!needle) return;\n"
  "  const rad = ((180 - angle) / 180) * Math.PI;\n"
  "  const cx = 40, cy = 45, len = 28;\n"
  "  const ex = cx + len * Math.cos(Math.PI - rad);\n"
  "  const ey = cy - len * Math.sin(Math.PI - rad);\n"
  "  needle.setAttribute('x2', ex.toFixed(1));\n"
  "  needle.setAttribute('y2', ey.toFixed(1));\n"
  "}\n"
  "\n"
  "function servoInput(id, val) {\n"
  "  val = parseInt(val);\n"
  "  document.getElementById('sa' + id).innerHTML = val + ' <span>deg</span>';\n"
  "  rotateNeedle(id, val);\n"
  "  document.getElementById('sc' + id).classList.add('active');\n"
  "}\n"
  "\n"
  "let servoTimer = {};\n"
  "function servoSend(id, val) {\n"
  "  clearTimeout(servoTimer[id]);\n"
  "  servoTimer[id] = setTimeout(() => {\n"
  "    send({ cmd: 'servo', id: id, angle: parseInt(val) });\n"
  "    document.getElementById('sc' + id).classList.remove('active');\n"
  "  }, 50);\n"
  "}\n"
  "\n"
  "function servoPreset(id, angle) {\n"
  "  document.getElementById('sl' + id).value = angle;\n"
  "  servoInput(id, angle);\n"
  "  servoSend(id, angle);\n"
  "}\n"
  "\n"
  "function toggleMachine() {\n"
  "  const isOn = document.getElementById('machineToggle').classList.contains('on');\n"
  "  send({ cmd: 'machine', on: !isOn });\n"
  "}\n"
  "\n"
  "function startWarmupAnim() {\n"
  "  warmupStartTime = Date.now();\n"
  "  warmupInterval = setInterval(() => {\n"
  "    const elapsed   = (Date.now() - warmupStartTime) / 1000;\n"
  "    const total     = 20;\n"
  "    const pct       = Math.min(100, (elapsed / total) * 100);\n"
  "    const remaining = Math.max(0, Math.ceil(total - elapsed));\n"
  "    document.getElementById('warmupFill').style.width = pct + '%';\n"
  "    document.getElementById('warmupSec').textContent  = remaining + 's';\n"
  "  }, 200);\n"
  "}\n"
  "\n"
  "function stopWarmup() {\n"
  "  if (warmupInterval) { clearInterval(warmupInterval); warmupInterval = null; }\n"
  "  warmupStartTime = null;\n"
  "}\n"
  "\n"
  "// Init\n"
  "[1,2,3].forEach(id => rotateNeedle(id, 0));\n"
  "connectWS();\n"
  "</script>\n"
  "</body>\n"
  "</html>\n"
  "\n";

// ── WiFi Credentials ─────────────────────────────────────────
#define WIFI_SSID   "Absolute Solver"
#define WIFI_PASS   "CynIsTheCutestBotEver333"

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_BUZZER     32
#define PIN_RELAY      19
#define PIN_SERVO1     33
#define PIN_SERVO2     25
#define PIN_SERVO3     18
#define PIN_GAS_AOUT   26
#define PIN_GAS_DOUT   27
#define PIN_WATER_AOUT 14
#define PIN_FLAME_AOUT 34
#define PIN_FLAME_DOUT 13

// ── RGB LED ───────────────────────────────────────────────────
#define PIN_RGB_R      15
#define PIN_RGB_G       2
#define PIN_RGB_B      23
#define PWM_FREQ     5000
#define PWM_RES         8

// ── MQ-2 ─────────────────────────────────────────────────────
#define MQ2_THRESHOLD_DELTA  150
#define MQ2_THRESHOLD_HYST    50
#define MQ2_WARMUP_MS       20000
#define MQ2_READ_INTERVAL    500

// ── Water Level ──────────────────────────────────────────────
#define WATER_THRESHOLD_ON    800
#define WATER_THRESHOLD_OFF   600
#define WATER_READ_INTERVAL   500

// ── Flame Sensor ─────────────────────────────────────────────
#define FLAME_READ_INTERVAL   500

// ── Servo Limits ─────────────────────────────────────────────
#define SERVO_MIN_ANGLE    0
#define SERVO_MAX_ANGLE  180

// ── LCD Clear delay ──────────────────────────────────────────
#define LCD_CLEAR_DELAY  1500

// ── Objek ────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2, servo3;
WebServer         server(80);
WebSocketsServer  webSocket(81);

// ── State Global ─────────────────────────────────────────────
bool     machineON    = true;
bool     gasAlert     = false;
bool     waterAlert   = false;
bool     flameAlert   = false;

int      servoAngle1  = 0;
int      servoAngle2  = 0;
int      servoAngle3  = 0;

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
#define  RGB_BLINK_MS   300

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
#define WS_BROADCAST_MS 500

// ── Forward Declarations ─────────────────────────────────────
void handleGasSensor(uint32_t now);
void handleWaterSensor(uint32_t now);
void handleFlameSensor(uint32_t now);
void handleRGB(uint32_t now);
void writeRGB(RGBColor c);
void setRGBBlink(RGBColor c1, RGBColor c2);
void setRGBOff();
void setRGBSolid(RGBColor c);
void setMachine(bool on);
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
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_GAS_AOUT,   INPUT);
  pinMode(PIN_GAS_DOUT,   INPUT);
  pinMode(PIN_WATER_AOUT, INPUT);
  pinMode(PIN_FLAME_AOUT, INPUT);
  pinMode(PIN_FLAME_DOUT, INPUT_PULLDOWN);

  ledcAttach(PIN_RGB_R, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_G, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_RGB_B, PWM_FREQ, PWM_RES);
  setRGBOff();

  analogReadResolution(12);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY, HIGH);

  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo1.write(0); servo2.write(0); servo3.write(0);

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
    Serial.printf("[WiFi] ★ Buka browser: http://%s\n", WiFi.localIP().toString().c_str());
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

  // ── HTTP: serve dashboard HTML langsung ────────────────────
  // Browser buka http://<IP> → langsung tampil UI dashboard
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", DASHBOARD_HTML);
  });

  // Redirect semua path lain ke root
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });

  server.begin();

  // WebSocket di port 81
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

  if (strcmp(cmd, "servo") == 0) {
    int id    = doc["id"]    | 0;
    int angle = constrain((int)(doc["angle"] | 0), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    if      (id == 1) { servoAngle1 = angle; servo1.write(angle); }
    else if (id == 2) { servoAngle2 = angle; servo2.write(angle); }
    else if (id == 3) { servoAngle3 = angle; servo3.write(angle); }
    Serial.printf("[WS] Servo%d → %d°\n", id, angle);
    updateLCD(); broadcastStatus();
    return;
  }

  if (strcmp(cmd, "machine") == 0) {
    if (!gasAlert && !waterAlert && !flameAlert) {
      setMachine(doc["on"] | true);
      broadcastStatus();
    }
    return;
  }
}

// ─────────────────────────────────────────────────────────────
//  STATUS JSON
// ─────────────────────────────────────────────────────────────
String buildStatusJson() {
  StaticJsonDocument<512> doc;
  doc["type"]       = "status";
  doc["machine"]    = machineON;
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
  String out; serializeJson(doc, out);
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
        WiFi.localIP().toString().lastIndexOf('.')+1));
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
    updateLCD(); return;
  }

  if (now - mq2LastRead < MQ2_READ_INTERVAL) return;
  mq2LastRead = now;
  lastGasVal = analogRead(PIN_GAS_AOUT);

  if (!gasAlert && lastGasVal > mq2ThreshOn) {
    gasAlert = true; setMachine(false);
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_GAS_A, COL_GAS_B);
    lcd.clear(); lcd.setCursor(0,0); lcd.print("!! BAHAYA ASAP!!");
    lcd.setCursor(0,1); lcd.print("ADC:"); lcd.print(lastGasVal);
  } else if (gasAlert && lastGasVal < mq2ThreshOff) {
    gasAlert = false;
    if (!waterAlert && !flameAlert) {
      stopBuzzer(); setRGBOff(); setMachine(true);
      lcd.clear(); lcd.setCursor(0,0); lcd.print("Udara bersih.");
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
    waterAlert = true; setMachine(false);
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_WATER_A, COL_WATER_B);
    lcd.clear(); lcd.setCursor(0,0); lcd.print("!! BANJIR/AIR !!");
    lcd.setCursor(0,1); lcd.print("ADC:"); lcd.print(lastWaterVal);
  } else if (waterAlert && lastWaterVal < WATER_THRESHOLD_OFF) {
    waterAlert = false;
    if (!gasAlert && !flameAlert) {
      stopBuzzer(); setRGBOff(); setMachine(true);
      lcd.clear(); lcd.setCursor(0,0); lcd.print("Air aman.");
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
    flameAlert = true; setMachine(false);
    digitalWrite(PIN_BUZZER, HIGH);
    setRGBBlink(COL_FLAME_A, COL_FLAME_B);
    lcd.clear(); lcd.setCursor(0,0); lcd.print("!! BAHAYA API !!");
    lcd.setCursor(0,1); lcd.print("ADC:"); lcd.print(lastFlameADC);
  } else if (flameAlert && !lastFlameD0) {
    flameAlert = false;
    if (!gasAlert && !waterAlert) {
      stopBuzzer(); setRGBOff(); setMachine(true);
      lcd.clear(); lcd.setCursor(0,0); lcd.print("Api padam.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  MESIN & BUZZER
// ─────────────────────────────────────────────────────────────
void setMachine(bool on) {
  machineON = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  if (!on) stopBuzzer();
  updateLCD();
  Serial.printf("[MESIN] %s\n", on ? "ON" : "OFF");
}

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
  if (machineON) {
    lcd.setCursor(0, 0);
    lcd.print("S1:"); lcd.print(servoAngle1);
    lcd.print(" S2:"); lcd.print(servoAngle2);
    lcd.setCursor(0, 1);
    lcd.print("S3:"); lcd.print(servoAngle3);
    lcd.print(" MESIN:ON");
  } else {
    lcd.setCursor(0, 0); lcd.print("MESIN: MATI");
    lcd.setCursor(0, 1);
    lcd.print("S1:"); lcd.print(servoAngle1);
    lcd.print(" S2:"); lcd.print(servoAngle2);
  }
}
