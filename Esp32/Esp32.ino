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
 *  │ Servo 2             │ 19       │ Manual via Web          │
 *  │ Servo 3             │ 18       │ Manual via Web          │
 *  │ MQ-2 AOUT           │ 35       │ Analog (ADC1)           │
 *  │ MQ-2 DOUT           │ 27       │ tidak dipakai           │
 *  │ Water Level AOUT    │ 36       │ Analog (ADC1, pin VP)   │
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
#define WIFI_PASS   "CynIsMyRobo18z"

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_BUZZER      32
#define PIN_BUTTON       5
#define PIN_SERVO1      33
#define PIN_SERVO2      19
#define PIN_SERVO3      18
#define PIN_GAS_AOUT    35 // Pindah ke 35 (ADC1) karena ADC2 mati saat WiFi nyala
#define PIN_GAS_DOUT    27
#define PIN_WATER_AOUT  36 // Pindah ke 36 (ADC1) karena ADC2 mati saat WiFi nyala
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
#define PWM_MOTOR_FREQ   100   // 100Hz: jauh lebih dingin untuk L298N (vs 1000Hz)
#define PWM_MOTOR_RES      8
#define MOTOR_SPEED_KICK  204   // 80% kickstart (turun dari 100% agar L298N tidak overheat)
#define MOTOR_SPEED_RUN   200   // 78% speed normal
#define MOTOR_KICK_MS     600   // 600ms kickstart (sedikit lebih lama agar konveyor pasti jalan)

// ── MQ-2 ─────────────────────────────────────────────────────
#define MQ2_THRESHOLD_ON    2200   // Batas mutlak untuk mendeteksi asap tebal
#define MQ2_THRESHOLD_OFF   1800   // Batas mutlak untuk menganggap udara kembali bersih
#define MQ2_WARMUP_MS       20000
#define MQ2_READ_INTERVAL     500

// ── Water Level ──────────────────────────────────────────────
#define WATER_THRESHOLD_ON    150
#define WATER_THRESHOLD_OFF   100
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
const char DASHBOARD_HTML[] PROGMEM = R"====(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AIoT Control Panel</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');
  :root {
    --bg: #0d1117;
    --surface: #161b22;
    --surface2: #1c2128;
    --border: #30363d;
    --accent: #00d9ff;
    --accent2: #ff7043;
    --text: #e6edf3;
    --muted: #7d8590;
    --danger: #f85149;
    --warn: #e3b341;
    --ok: #3fb950;
    --purple: #bc8cff;
    --radius: 10px;
    --font: 'Inter', system-ui, sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: var(--font); min-height: 100vh; font-size: 14px; }

  /* ═══════════════════════════════════════
     IP CONFIG (untuk akses dari komputer)
  ═══════════════════════════════════════ */
  #ipConfigPage {
    min-height: 100vh;
    display: flex; align-items: center; justify-content: center;
    background: radial-gradient(ellipse at 50% 0%, rgba(0,217,255,0.06) 0%, transparent 60%);
  }
  .ip-box {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 16px; padding: 40px 36px; width: 100%; max-width: 400px;
    box-shadow: 0 24px 64px rgba(0,0,0,0.4);
  }
  .ip-logo { text-align: center; margin-bottom: 24px; }
  .ip-logo .brand { font-size: 22px; font-weight: 700; letter-spacing: -0.02em; }
  .ip-logo .brand span { color: var(--accent); }
  .ip-logo .sub { font-size: 12px; color: var(--muted); margin-top: 4px; }
  .ip-hint {
    font-size: 12px; color: var(--muted); background: var(--surface2);
    border: 1px solid var(--border); border-radius: 8px; padding: 10px 14px;
    margin-bottom: 20px; line-height: 1.6;
  }
  .ip-hint code { color: var(--accent); font-family: monospace; }

  /* ═══════════════════════════════════════
     LOGIN PAGE
  ═══════════════════════════════════════ */
  #loginPage {
    min-height: 100vh;
    display: flex; align-items: center; justify-content: center;
    background: radial-gradient(ellipse at 50% 0%, rgba(0,217,255,0.06) 0%, transparent 60%);
  }
  .login-box {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 16px;
    padding: 40px 36px;
    width: 100%; max-width: 400px;
    box-shadow: 0 24px 64px rgba(0,0,0,0.4);
  }
  .login-logo {
    text-align: center; margin-bottom: 28px;
  }
  .login-logo .brand {
    font-size: 20px; font-weight: 700; letter-spacing: -0.02em; color: var(--text);
  }
  .login-logo .brand span { color: var(--accent); }
  .login-logo .sub { font-size: 12px; color: var(--muted); margin-top: 4px; }
  .login-tabs {
    display: flex; border: 1px solid var(--border); border-radius: 8px;
    overflow: hidden; margin-bottom: 28px;
  }
  .login-tab {
    flex: 1; padding: 9px 0; text-align: center; font-size: 13px; font-weight: 500;
    cursor: pointer; transition: all 0.2s; color: var(--muted); background: transparent; border: none;
  }
  .login-tab.active { background: var(--surface2); color: var(--text); }
  .login-section { display: none; }
  .login-section.active { display: block; }

  /* User card */
  .user-card {
    background: var(--surface2); border: 1px solid var(--border);
    border-radius: 10px; padding: 20px; text-align: center; cursor: pointer;
    transition: all 0.2s;
  }
  .user-card:hover { border-color: var(--accent); background: rgba(0,217,255,0.04); }
  .user-card .icon { font-size: 32px; margin-bottom: 10px; }
  .user-card .title { font-size: 15px; font-weight: 600; margin-bottom: 4px; }
  .user-card .desc { font-size: 12px; color: var(--muted); }

  /* Admin form */
  .form-group { margin-bottom: 16px; }
  .form-label { font-size: 12px; font-weight: 500; color: var(--muted); margin-bottom: 6px; display: block; letter-spacing: 0.04em; text-transform: uppercase; }
  .form-input {
    width: 100%; background: var(--bg); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px 14px; color: var(--text); font-size: 14px;
    font-family: var(--font); outline: none; transition: border-color 0.15s;
  }
  .form-input:focus { border-color: var(--accent); }
  .form-input::placeholder { color: var(--muted); }
  .login-btn {
    width: 100%; background: var(--accent); color: #000; border: none;
    border-radius: 8px; padding: 11px; font-size: 14px; font-weight: 600;
    cursor: pointer; transition: opacity 0.15s; font-family: var(--font); margin-top: 4px;
  }
  .login-btn:hover { opacity: 0.85; }
  .login-err {
    display: none; background: rgba(248,81,73,0.08); border: 1px solid rgba(248,81,73,0.3);
    border-radius: 8px; padding: 10px 14px; font-size: 13px; color: var(--danger);
    margin-top: 14px; text-align: center;
  }
  .login-err.show { display: block; }

  /* ═══════════════════════════════════════
     MAIN APP (Hidden until login)
  ═══════════════════════════════════════ */
  #appPage { display: none; }

  /* HEADER */
  header {
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    padding: 0 24px; height: 56px;
    display: flex; align-items: center; justify-content: space-between;
    position: sticky; top: 0; z-index: 100;
  }
  .header-left { display: flex; align-items: center; gap: 14px; }
  .logo { font-size: 15px; font-weight: 700; color: var(--text); letter-spacing: -0.01em; }
  .logo span { color: var(--accent); }
  .logo-sub { font-size: 11px; color: var(--muted); font-weight: 400; }
  .role-badge {
    font-size: 11px; font-weight: 600; padding: 3px 10px;
    border-radius: 20px; border: 1px solid; letter-spacing: 0.05em; text-transform: uppercase;
  }
  .role-badge.admin { background: rgba(188,140,255,0.1); color: var(--purple); border-color: rgba(188,140,255,0.3); }
  .role-badge.user  { background: rgba(0,217,255,0.08); color: var(--accent); border-color: rgba(0,217,255,0.25); }
  .header-right { display: flex; align-items: center; gap: 12px; }
  .conn-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--danger); flex-shrink: 0; }
  .conn-dot.online { background: var(--ok); animation: pulse-dot 2s infinite; }
  @keyframes pulse-dot { 0%,100%{opacity:1} 50%{opacity:0.4} }
  .conn-label { font-size: 13px; color: var(--danger); }
  .conn-label.online { color: var(--ok); }
  .logout-btn {
    font-size: 12px; color: var(--muted); background: transparent;
    border: 1px solid var(--border); border-radius: 6px; padding: 5px 12px;
    cursor: pointer; font-family: var(--font); transition: all 0.15s;
  }
  .logout-btn:hover { color: var(--text); border-color: var(--muted); }

  /* MAIN LAYOUT */
  main { max-width: 1100px; margin: 0 auto; padding: 24px 20px 48px; }

  /* ALERT */
  .alert-banner {
    display: none; align-items: center; gap: 10px;
    background: rgba(248,81,73,0.08); border: 1px solid rgba(248,81,73,0.4);
    border-radius: var(--radius); padding: 12px 16px; margin-bottom: 20px;
    font-size: 13px; color: var(--danger);
    animation: pulse-border 1.2s infinite;
  }
  .alert-banner.active { display: flex; }
  @keyframes pulse-border { 0%,100%{border-color:rgba(248,81,73,0.4)} 50%{border-color:rgba(248,81,73,0.1)} }
  .alert-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--danger); animation: blink 0.7s infinite; flex-shrink: 0; }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.1} }

  /* SECTION LABEL */
  .section-label {
    font-size: 11px; font-weight: 500; letter-spacing: 0.1em;
    color: var(--muted); text-transform: uppercase;
    margin: 24px 0 12px; display: flex; align-items: center; gap: 10px;
  }
  .section-label::after { content: ''; flex: 1; height: 1px; background: var(--border); }

  /* GRID */
  .grid-4 { display: grid; grid-template-columns: repeat(4,1fr); gap: 12px; margin-bottom: 12px; }
  .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }
  .grid-3 { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; margin-bottom: 12px; }
  @media(max-width:900px){ .grid-4{grid-template-columns:1fr 1fr} }
  @media(max-width:600px){ .grid-4,.grid-2,.grid-3{grid-template-columns:1fr} }

  /* CARD */
  .card { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px; }
  .card-label { font-size: 11px; font-weight: 500; letter-spacing: 0.08em; color: var(--muted); text-transform: uppercase; margin-bottom: 12px; }
  .card-value { font-size: 28px; font-weight: 600; letter-spacing: -0.02em; margin-bottom: 2px; }
  .card-sub { font-size: 12px; color: var(--muted); margin-bottom: 10px; }

  /* STATUS PILL */
  .pill { display: inline-flex; align-items: center; gap: 5px; font-size: 11px; font-weight: 500; letter-spacing: 0.04em; text-transform: uppercase; padding: 3px 10px; border-radius: 20px; border: 1px solid; }
  .pill.ok { border-color: rgba(63,185,80,0.4); color: var(--ok); background: rgba(63,185,80,0.08); }
  .pill.danger { border-color: rgba(248,81,73,0.4); color: var(--danger); background: rgba(248,81,73,0.08); }
  .pill.offline { border-color: var(--border); color: var(--muted); }
  .pill-dot { width: 6px; height: 6px; border-radius: 50%; background: currentColor; }

  /* PROGRESS BAR */
  .bar-wrap { background: var(--bg); border-radius: 3px; height: 4px; overflow: hidden; margin-top: 8px; }
  .bar { height: 100%; border-radius: 3px; transition: width 0.5s, background 0.3s; background: var(--ok); }
  .bar.warn { background: var(--warn); }
  .bar.danger { background: var(--danger); }

  /* WARMUP */
  .warmup-wrap { margin-top: 12px; }
  .warmup-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; font-size: 12px; }
  .warmup-bar { background: var(--bg); border-radius: 3px; height: 4px; overflow: hidden; }
  .warmup-fill { height: 100%; background: var(--warn); border-radius: 3px; transition: width 1s linear; }

  /* YOLO CARD */
  .yolo-card { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px; margin-bottom: 12px; display: flex; align-items: center; gap: 20px; }
  .yolo-icon { width: 48px; height: 48px; border-radius: 10px; background: rgba(0,217,255,0.08); border: 1px solid rgba(0,217,255,0.2); display: flex; align-items: center; justify-content: center; flex-shrink: 0; font-size: 24px; }
  .yolo-name { font-size: 22px; font-weight: 600; color: var(--accent); margin-bottom: 2px; }
  .yolo-cat { font-size: 13px; color: var(--muted); }
  .yolo-servo { margin-top: 8px; font-size: 13px; font-weight: 500; color: var(--ok); }

  /* LIVE VIEW (User) */
  .live-card {
    background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius);
    padding: 20px; text-align: center; margin-bottom: 12px;
  }
  .live-badge {
    display: inline-flex; align-items: center; gap: 6px;
    font-size: 11px; font-weight: 600; letter-spacing: 0.08em; text-transform: uppercase;
    padding: 4px 12px; border-radius: 20px;
    background: rgba(248,81,73,0.1); color: var(--danger); border: 1px solid rgba(248,81,73,0.3);
    margin-bottom: 14px;
    animation: pulse-border 1.5s infinite;
  }
  .live-badge .blink-dot { width: 6px; height: 6px; border-radius: 50%; background: var(--danger); animation: blink 0.8s infinite; }
  .live-status-name { font-size: 32px; font-weight: 700; color: var(--accent); margin-bottom: 4px; letter-spacing: -0.02em; }
  .live-status-cat { font-size: 14px; color: var(--muted); margin-bottom: 12px; }
  .live-servo-info { font-size: 13px; font-weight: 500; color: var(--ok); padding: 8px 16px; background: rgba(63,185,80,0.06); border: 1px solid rgba(63,185,80,0.2); border-radius: 8px; display: inline-block; }
  .live-sensor-row { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; margin-top: 18px; }
  .live-sensor-chip {
    display: inline-flex; align-items: center; gap: 6px; padding: 6px 14px;
    background: var(--surface2); border: 1px solid var(--border); border-radius: 8px;
    font-size: 12px; color: var(--muted);
  }
  .live-sensor-chip.alert { border-color: rgba(248,81,73,0.4); color: var(--danger); background: rgba(248,81,73,0.06); }

  /* MOTOR CARD */
  .motor-card { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 18px; margin-bottom: 12px; }
  .motor-head { display: flex; align-items: flex-start; justify-content: space-between; margin-bottom: 14px; }
  .motor-state { font-size: 26px; font-weight: 700; letter-spacing: -0.02em; color: var(--muted); }
  .motor-pins { font-size: 11px; color: var(--muted); font-family: monospace; margin-top: 4px; }
  .speed-row { display: flex; align-items: center; gap: 12px; margin-bottom: 6px; }
  .speed-label { font-size: 12px; color: var(--muted); white-space: nowrap; }
  .speed-val { font-size: 13px; font-weight: 600; color: var(--accent); min-width: 38px; text-align: right; font-family: monospace; }

  input[type=range] { -webkit-appearance: none; width: 100%; height: 3px; background: var(--border); border-radius: 2px; outline: none; cursor: pointer; }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%; background: var(--accent); border: 2px solid var(--bg); transition: transform 0.1s; }
  input[type=range]::-webkit-slider-thumb:active { transform: scale(1.25); }

  .preset-row { display: flex; gap: 6px; margin: 10px 0; }
  .preset-btn { flex: 1; background: var(--bg); border: 1px solid var(--border); border-radius: 6px; color: var(--muted); font-size: 11px; padding: 6px 4px; cursor: pointer; text-align: center; transition: all 0.15s; font-family: monospace; line-height: 1.4; }
  .preset-btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,217,255,0.05); }
  .preset-btn.active { border-color: var(--accent); color: var(--accent); background: rgba(0,217,255,0.1); }

  .motor-btns { display: flex; gap: 10px; }
  .motor-btn { flex: 1; padding: 13px 8px; border: 1px solid var(--border); border-radius: var(--radius); background: var(--bg); color: var(--muted); font-size: 13px; font-weight: 600; cursor: pointer; text-align: center; transition: all 0.15s; letter-spacing: 0.04em; text-transform: uppercase; }
  .motor-btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,217,255,0.05); }
  .motor-btn.active-fwd { border-color: var(--ok); color: var(--ok); background: rgba(63,185,80,0.08); }
  .motor-btn.active-bwd { border-color: var(--accent2); color: var(--accent2); background: rgba(255,112,67,0.08); }
  .motor-btn.stop-btn { border-color: var(--danger); color: var(--danger); }
  .motor-btn.stop-btn:hover { background: rgba(248,81,73,0.08); }

  /* START BUTTON */
  .start-btn {
    width: 100%; padding: 14px; border-radius: var(--radius);
    background: linear-gradient(135deg, #00d9ff 0%, #00b8d9 100%);
    color: #000; font-size: 15px; font-weight: 700; border: none;
    cursor: pointer; transition: all 0.2s; letter-spacing: 0.04em;
    text-transform: uppercase; margin-top: 14px;
    box-shadow: 0 4px 20px rgba(0,217,255,0.3);
  }
  .start-btn:hover { opacity: 0.88; transform: translateY(-1px); box-shadow: 0 6px 24px rgba(0,217,255,0.45); }
  .start-btn:disabled { opacity: 0.4; cursor: not-allowed; transform: none; box-shadow: none; }
  .start-banner {
    display: flex; align-items: center; gap: 10px;
    background: rgba(0,217,255,0.06); border: 1px solid rgba(0,217,255,0.25);
    border-radius: 8px; padding: 10px 14px; margin-top: 10px; font-size: 12px; color: var(--muted);
  }
  .start-banner.running {
    background: rgba(63,185,80,0.06); border-color: rgba(63,185,80,0.3); color: var(--ok);
  }

  /* SERVO CARDS */
  .servo-card { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); padding: 16px; transition: border-color 0.2s; }
  .servo-card.active { border-color: rgba(0,217,255,0.5); }
  .servo-name { font-size: 11px; font-weight: 500; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 10px; }
  .servo-angle { font-size: 26px; font-weight: 700; color: var(--accent); letter-spacing: -0.02em; margin-bottom: 8px; }
  .servo-angle span { font-size: 13px; color: var(--muted); font-weight: 400; }
  .servo-vis { display: flex; justify-content: center; margin: 6px 0 12px; }

  /* BUTTON CARD */
  .btn-state { display: inline-flex; align-items: center; gap: 6px; font-size: 12px; font-weight: 500; letter-spacing: 0.04em; text-transform: uppercase; padding: 4px 12px; border-radius: 20px; border: 1px solid; }
  .btn-state.pressed { border-color: rgba(0,217,255,0.4); color: var(--accent); background: rgba(0,217,255,0.08); }
  .btn-state.released { border-color: var(--border); color: var(--muted); }

  /* ADMIN ONLY sections */
  .admin-only { display: none; }

  /* ═══════════════════════════════════════
     TOAST NOTIFICATION
  ═══════════════════════════════════════ */
  .toast {
    position: fixed; bottom: 24px; right: 24px;
    background: var(--surface); border: 1px solid var(--border); border-radius: 10px;
    padding: 12px 18px; font-size: 13px; color: var(--text);
    box-shadow: 0 8px 32px rgba(0,0,0,0.3);
    transform: translateY(80px); opacity: 0;
    transition: all 0.3s cubic-bezier(0.34,1.56,0.64,1);
    z-index: 9999; pointer-events: none;
  }
  .toast.show { transform: translateY(0); opacity: 1; }

  /* SORT NOTIFICATION (TOP) */
  .sort-popup {
    position: fixed; top: 0; left: 50%; transform: translateX(-50%) translateY(-100%);
    background: var(--ok); color: #000; font-weight: 600; font-size: 14px;
    padding: 12px 24px; border-radius: 0 0 12px 12px; z-index: 10000;
    box-shadow: 0 4px 12px rgba(63,185,80,0.3);
    transition: transform 0.4s cubic-bezier(0.34,1.56,0.64,1);
  }
  .sort-popup.show { transform: translateX(-50%) translateY(0); }

  /* SIDEBAR (STATISTIK LIMBAH) */
  .sidebar-overlay {
    position: fixed; top: 0; left: 0; right: 0; bottom: 0;
    background: rgba(0,0,0,0.6); z-index: 999; display: none; opacity: 0;
    transition: opacity 0.3s;
  }
  .sidebar-overlay.show { display: block; opacity: 1; }
  .sidebar {
    position: fixed; top: 0; right: -500px; bottom: 0; width: 100%; max-width: 500px;
    background: var(--surface); border-left: 1px solid var(--border);
    z-index: 1000; transition: right 0.3s cubic-bezier(0.34,1.56,0.64,1);
    display: flex; flex-direction: column; box-shadow: -10px 0 30px rgba(0,0,0,0.5);
  }
  .sidebar.show { right: 0; }
  .sidebar-header { padding: 20px; border-bottom: 1px solid var(--border); display: flex; justify-content: space-between; align-items: center; }
  .sidebar-title { font-size: 18px; font-weight: 700; color: var(--text); }
  .close-btn { background: transparent; border: none; color: var(--muted); font-size: 24px; cursor: pointer; }
  .sidebar-content { padding: 20px; overflow-y: auto; flex: 1; }
  .stats-layout { display: flex; gap: 20px; align-items: stretch; }
  .stats-chart { flex: 1; min-width: 200px; display: flex; align-items: center; justify-content: center; flex-direction: column; }
  .stats-list { flex: 1; display: flex; flex-direction: column; gap: 14px; }
  .category-group { background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; overflow: hidden; }
  .category-head { background: rgba(0,217,255,0.05); border-bottom: 1px solid var(--border); padding: 8px 12px; font-weight: 600; color: var(--accent); font-size: 13px; text-transform: uppercase; letter-spacing: 0.05em; }
  .category-item { display: flex; justify-content: space-between; padding: 8px 12px; border-bottom: 1px solid var(--border); font-size: 13px; }
  .category-item:last-child { border-bottom: none; }
  .category-total { display: flex; justify-content: space-between; padding: 8px 12px; background: rgba(0,0,0,0.2); font-weight: 700; color: #fff; font-size: 13px; }
  
  .svg-pie { width: 100%; max-width: 220px; height: auto; transform: rotate(-90deg); border-radius: 50%; }
  .legend-list { display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; margin-top: 14px; }
  .legend-item { display: flex; align-items: center; gap: 6px; font-size: 11px; color: var(--muted); }
  .legend-color { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  
  /* SENSOR CHART */
  .sensor-chart { margin-top: 24px; padding-top: 24px; border-top: 1px solid var(--border); }
  .sensor-title { font-size: 14px; font-weight: 600; color: var(--text); margin-bottom: 14px; text-transform: uppercase; letter-spacing: 0.05em; display:flex; justify-content:space-between; align-items:center; }
  .sensor-layout { display: flex; gap: 20px; align-items: center; }
  .svg-line { width: 100%; max-width: 250px; height: auto; background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; padding: 10px; box-sizing: border-box; overflow:visible; }
  .sensor-list { flex: 1; display: flex; flex-direction: column; gap: 8px; }
  .sensor-item { display: flex; justify-content: space-between; padding: 10px 14px; background: rgba(0,0,0,0.2); border: 1px solid var(--border); border-radius: 8px; font-size: 13px; align-items:center; }
  .sensor-dot { width: 10px; height: 10px; border-radius: 50%; display:inline-block; margin-right: 8px; flex-shrink: 0; }

  @media(max-width: 500px){ .stats-layout, .sensor-layout { flex-direction: column; } }

  .hamburger-btn {
    background: transparent; border: none; color: var(--text); font-size: 20px;
    cursor: pointer; padding: 5px; margin-left: 10px; transition: color 0.2s;
  }
  .hamburger-btn:hover { color: var(--accent); }
</style>
</head>
<body>

<!-- NOTIFICATION POPUPS -->
<div id="sortPopup" class="sort-popup">Berhasil menyortir limbah "..."</div>

<!-- SIDEBAR MODAL -->
<div id="sidebarOverlay" class="sidebar-overlay" onclick="closeSidebar()"></div>
<div id="sidebar" class="sidebar">
  <div class="sidebar-header">
    <div class="sidebar-title">📊 Total Limbah Tersortir</div>
    <button class="close-btn" onclick="closeSidebar()">×</button>
  </div>
  <div class="sidebar-content">
    <div class="stats-layout">
      <div class="stats-chart">
        <div id="svgContainer" style="width:100%; display:flex; justify-content:center;"></div>
        <div id="chartLegend" class="legend-list"></div>
      </div>
      <div class="stats-list" id="wasteList">
        <!-- JS akan render list disini -->
      </div>
    </div>
    <button onclick="resetStats()" style="width:100%; margin-top:20px; background:rgba(248,81,73,0.1); border:1px solid rgba(248,81,73,0.3); color:var(--danger); padding:10px; border-radius:8px; cursor:pointer; font-weight:600; transition:background 0.2s;" onmouseover="this.style.background='rgba(248,81,73,0.2)'" onmouseout="this.style.background='rgba(248,81,73,0.1)'">Reset Data Limbah</button>
    
    <!-- SENSOR ANALYTICS -->
    <div class="sensor-chart">
      <div class="sensor-title">
        📉 Analisis Sensor Bahaya
        <button onclick="resetSensorStats()" style="background:rgba(227,179,65,0.1); border:1px solid rgba(227,179,65,0.3); color:var(--warn); padding:4px 8px; border-radius:6px; cursor:pointer; font-size:11px; font-weight:600; transition:opacity 0.2s;" onmouseover="this.style.opacity='0.8'" onmouseout="this.style.opacity='1'">Reset</button>
      </div>
      <div class="sensor-layout">
        <div id="sensorSvgContainer" style="flex:1; display:flex; justify-content:center;"></div>
        <div id="sensorList" class="sensor-list"></div>
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════════════════════════════ IP CONFIG PAGE ═══════════════════════════════════════ -->
<div id="ipConfigPage" style="display:none;">
  <div class="ip-box">
    <div class="ip-logo">
      <div class="brand">AIoT <span>Control Panel</span></div>
      <div class="sub">Konfigurasi Koneksi ESP32</div>
    </div>
    <div class="ip-hint">
      Halaman ini dibuka dari komputer secara lokal.<br>
      Masukkan <strong>IP Address</strong> dari ESP32 Smart Bin kamu.<br>
      <code>Contoh: 10.132.39.50</code>
    </div>
    <div class="form-group">
      <label class="form-label">IP Address ESP32</label>
      <input class="form-input" type="text" id="esp32IpInput" placeholder="10.132.39.xx" autocomplete="off"
             onkeydown="if(event.key==='Enter')saveIp()">
    </div>
    <button class="login-btn" onclick="saveIp()">Hubungkan →</button>
    <div class="login-err" id="ipErr">Masukkan IP Address yang valid.</div>
  </div>
</div>

<!-- ═══════════════════════════════════════ LOGIN PAGE ═══════════════════════════════════════ -->
<div id="loginPage">
  <div class="login-box">
    <div class="login-logo">
      <div class="brand">AIoT <span>Control Panel</span></div>
      <div class="sub">Sistem Deteksi Limbah Medis</div>
    </div>

    <div class="login-tabs">
      <button class="login-tab active" id="tabUser" onclick="switchTab('user')">👤 Pengguna</button>
      <button class="login-tab" id="tabAdmin" onclick="switchTab('admin')">🛡 Admin</button>
    </div>

    <!-- User tab -->
    <div class="login-section active" id="sectionUser">
      <div class="user-card" onclick="loginAsUser()">
        <div class="icon">👁</div>
        <div class="title">Masuk sebagai Pengguna</div>
        <div class="desc">Akses live view status sistem & deteksi limbah.<br>Tidak ada kontrol perangkat.</div>
      </div>
    </div>

    <!-- Admin tab -->
    <div class="login-section" id="sectionAdmin">
      <div class="form-group">
        <label class="form-label">Username</label>
        <input class="form-input" type="text" id="adminUser" placeholder="Masukkan username" autocomplete="off">
      </div>
      <div class="form-group">
        <label class="form-label">Password</label>
        <input class="form-input" type="password" id="adminPass" placeholder="Masukkan password" onkeydown="if(event.key==='Enter')doAdminLogin()">
      </div>
      <button class="login-btn" onclick="doAdminLogin()">Masuk sebagai Admin</button>
      <div class="login-err" id="loginErr">Username atau password salah.</div>
    </div>
  </div>
</div>

<!-- ═══════════════════════════════════════ APP PAGE ═══════════════════════════════════════ -->
<div id="appPage">

<header>
  <div class="header-left">
    <div>
      <div class="logo">AIoT <span>Control Panel</span></div>
      <div class="logo-sub">Sistem Deteksi Limbah Medis</div>
    </div>
    <span class="role-badge" id="roleBadge">User</span>
  </div>
  <div class="header-right">
    <div style="display:flex;align-items:center;gap:7px;">
      <div class="conn-dot" id="connDot"></div>
      <span class="conn-label" id="connLabel" style="margin-right:8px;">Offline</span>
      <button style="background:var(--surface2);border:1px solid var(--border);color:var(--text);padding:4px 10px;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600;" onclick="connectWS()">Reconnect</button>
    </div>
    <button class="logout-btn" onclick="doLogout()">← Keluar</button>
    <button class="hamburger-btn" onclick="openSidebar()">☰</button>
  </div>
</header>

<main>

  <!-- ALERT -->
  <div class="alert-banner" id="alertBanner">
    <div class="alert-dot"></div>
    <strong id="alertText">Alert aktif!</strong>
  </div>

  <!-- ═══ USER: LIVE VIEW ═══ -->
  <div id="userView">
    <div class="section-label">Live Deteksi Kamera</div>
    <div class="live-card">
      <div class="live-badge"><span class="blink-dot"></span> Live</div>
      <div class="live-status-name" id="liveWasteName">Tidak Ada</div>
      <div class="live-status-cat" id="liveWasteCat">Kategori: —</div>
      <div id="liveServoInfo" class="live-servo-info" style="display:none;"></div>
    </div>
    <div class="section-label">Status Sensor</div>
    <div class="live-sensor-row">
      <div class="live-sensor-chip" id="chipGas">🔴 Gas: —</div>
      <div class="live-sensor-chip" id="chipWater">💧 Air: —</div>
      <div class="live-sensor-chip" id="chipFlame">🔥 Api: —</div>
      <div class="live-sensor-chip" id="chipMotor">⚙ Motor: —</div>
    </div>
  </div>

  <!-- ═══ ADMIN: FULL PANEL ═══ -->
  <div class="admin-only" id="adminView">

    <!-- MODE OPERASI & KEAMANAN -->
    <div class="section-label">Mode Operasi & Keamanan</div>
    <div class="card" style="margin-bottom: 20px;">
      <div style="display: flex; gap: 16px; align-items: flex-start; flex-wrap: wrap;">
        <div style="flex: 1; min-width: 250px;">
          <div class="card-label">Mode Konveyor</div>
          <select id="modeSelect" onchange="sendMode()" class="form-input" style="width: 100%; max-width: 300px; cursor: pointer;">
            <option value="0">Keep Going (Motor selalu menyala)</option>
            <option value="1">Less Energy (Hemat energi, nyala saat ada objek)</option>
          </select>
          <button id="startBtn" class="start-btn" onclick="sendStart()">▶ START SISTEM</button>
          <div id="startBanner" class="start-banner">
            <span>⏸</span>
            <span>Pilih mode lalu tekan <strong>START SISTEM</strong> untuk mulai.</span>
          </div>
        </div>
        <div id="safetyLockoutUI" style="display: none; flex: 1; min-width: 250px; background: rgba(248,81,73,0.1); border: 1px solid rgba(248,81,73,0.4); padding: 12px; border-radius: var(--radius); animation: pulse-border 1.5s infinite;">
          <div style="color: var(--danger); font-weight: 600; font-size: 13px; margin-bottom: 6px;">⚠ SISTEM TERKUNCI (SAFETY LOCKOUT)</div>
          <div style="color: var(--muted); font-size: 12px; margin-bottom: 10px;">Motor ditahan karena sensor sempat mendeteksi bahaya. Pastikan fisik aman sebelum melanjutkan.</div>
          <button onclick="sendResume()" style="width: 100%; padding: 10px; border-radius: 6px; background: var(--ok); color: #000; font-weight: 600; border: none; cursor: pointer; transition: opacity 0.2s;" onmouseover="this.style.opacity=0.8" onmouseout="this.style.opacity=1">⟳ Nyalakan Kembali</button>
        </div>
      </div>
    </div>

    <!-- SENTER KAMERA -->
    <div class="section-label">Senter Kamera (ESP32-CAM)</div>
    <div class="card" style="margin-bottom: 20px;">
      <div style="display: flex; gap: 20px; align-items: center; flex-wrap: wrap;">
        <div style="flex: 1; min-width: 200px;">
          <div class="card-label">Status Senter</div>
          <div id="flashStatusText" style="font-size: 26px; font-weight: 700; color: var(--muted); margin: 8px 0;">OFF</div>
          <div style="font-size: 12px; color: var(--muted);">Kontrol via tombol di bawah atau tekan <code style="color:var(--accent);">F</code> di Python</div>
        </div>
        <div style="display: flex; flex-direction: column; gap: 8px; min-width: 180px;">
          <button id="flashOnBtn" onclick="sendFlash(1)" style="padding: 11px 20px; border-radius: var(--radius); background: rgba(255,220,0,0.15); border: 1px solid rgba(255,220,0,0.4); color: #ffd700; font-size: 13px; font-weight: 600; cursor: pointer; transition: all 0.2s;" onmouseover="this.style.opacity='0.8'" onmouseout="this.style.opacity='1'">💡 Nyalakan Senter</button>
          <button id="flashOffBtn" onclick="sendFlash(0)" style="padding: 11px 20px; border-radius: var(--radius); background: var(--bg); border: 1px solid var(--border); color: var(--muted); font-size: 13px; font-weight: 600; cursor: pointer; transition: all 0.2s;" onmouseover="this.style.opacity='0.8'" onmouseout="this.style.opacity='1'">🔦 Matikan Senter</button>
        </div>
        <div style="flex: 1; min-width: 200px;">
          <div class="card-label">IP ESP32-CAM</div>
          <div style="display: flex; gap: 8px;">
            <input id="camIpInput" class="form-input" type="text" value="192.168.4.4" placeholder="192.168.4.4" style="max-width: 140px;" />
            <button onclick="connectCamWs()" style="padding: 0 12px; border-radius: var(--radius); background: var(--border); border: none; color: #fff; cursor: pointer; font-size: 12px;">Connect</button>
          </div>
          <div style="font-size: 11px; color: var(--muted); margin-top: 6px;" id="camWsStatusText">WebSocket: <span style="color:var(--danger)">Disconnected</span></div>
        </div>
      </div>
    </div>

    <!-- SENSORS -->
    <div class="section-label">Status Sensor</div>
    <div class="grid-4">
      <div class="card">
        <div class="card-label">MQ-2 Gas / Asap</div>
        <div class="card-value" id="gasVal" style="color:var(--purple);">—</div>
        <div class="card-sub">ADC raw (0–4095)</div>
        <div id="gasPill" class="pill offline"><span class="pill-dot"></span>Offline</div>
        <div class="bar-wrap"><div class="bar" id="gasBar" style="width:0%"></div></div>
        <div id="warmupSection" style="display:none;" class="warmup-wrap">
          <div class="warmup-row">
            <span style="color:var(--muted);">MQ-2 warm-up</span>
            <span style="color:var(--warn);font-family:monospace;font-size:12px;" id="warmupSec">20s</span>
          </div>
          <div class="warmup-bar"><div class="warmup-fill" id="warmupFill" style="width:0%"></div></div>
        </div>
        <div id="warmupDone" style="display:none;margin-top:10px;font-size:12px;color:var(--ok);font-family:monospace;">✓ MQ-2 siap</div>
      </div>

      <div class="card">
        <div class="card-label">Water Level</div>
        <div class="card-value" id="waterVal" style="color:#58a6ff;">—</div>
        <div class="card-sub">ADC raw · threshold 800</div>
        <div id="waterPill" class="pill offline"><span class="pill-dot"></span>Offline</div>
        <div class="bar-wrap"><div class="bar" id="waterBar" style="width:0%"></div></div>
      </div>

      <div class="card">
        <div class="card-label">Flame Sensor</div>
        <div class="card-value" id="flameADC" style="color:var(--accent2);">—</div>
        <div class="card-sub">ADC (A0)</div>
        <div id="flamePill" class="pill offline"><span class="pill-dot"></span>Offline</div>
        <div style="font-size:11px;color:var(--muted);margin-top:8px;font-family:monospace;">D0: <span id="flameD0">—</span></div>
      </div>

      <div class="card">
        <div class="card-label">Push Button GPIO 5</div>
        <div style="margin: 12px 0 8px;">
          <div id="btnPill" class="btn-state released"><span class="pill-dot"></span>Released</div>
        </div>
        <div style="font-size:12px;color:var(--muted);line-height:1.5;">Toggle motor maju / berhenti</div>
      </div>
    </div>

    <!-- YOLO -->
    <div class="section-label">Deteksi YOLOv8 Kamera</div>
    <div class="yolo-card">
      <div class="yolo-icon">🔍</div>
      <div>
        <div class="yolo-name" id="wasteName">Tidak Ada</div>
        <div class="yolo-cat" id="wasteCat">Kategori: —</div>
        <div id="wasteServoInfo" class="yolo-servo" style="display:none;"></div>
      </div>
    </div>

    <!-- MOTOR -->
    <div class="section-label">Motor DC (L298N)</div>
    <div class="motor-card">
      <div class="card-label" style="margin-bottom:10px;">IN1: GPIO4 &nbsp;·&nbsp; IN2: GPIO17 &nbsp;·&nbsp; ENA: GPIO16</div>
      <div class="motor-head">
        <div>
          <div class="motor-state" id="motorStatusText">STOP</div>
          <div class="motor-pins" id="motorPinInfo">IN1:L &nbsp; IN2:L &nbsp; ENA:0</div>
        </div>
        <div style="text-align:right;">
          <div id="motorSpeedPct" style="font-size:22px;font-weight:700;color:var(--accent);font-family:monospace;">78%</div>
          <div style="font-size:11px;color:var(--muted);">kecepatan</div>
        </div>
      </div>
      <div class="speed-row">
        <span class="speed-label">Kecepatan</span>
        <input type="range" min="0" max="255" value="200" step="5" id="motorSpeed"
               oninput="speedInput(this.value)" onchange="speedSend(this.value)" style="flex:1;">
        <span class="speed-val" id="motorSpeedVal">200</span>
      </div>
      <div class="preset-row">
        <div class="preset-btn" onclick="motorPreset(51)">Pelan<br>20%</div>
        <div class="preset-btn" onclick="motorPreset(102)">Lambat<br>40%</div>
        <div class="preset-btn" onclick="motorPreset(153)">Sedang<br>60%</div>
        <div class="preset-btn active" id="mpreset200" onclick="motorPreset(200)">Cepat<br>78%</div>
        <div class="preset-btn" onclick="motorPreset(255)">Maks<br>100%</div>
      </div>
      <div class="motor-btns">
        <div class="motor-btn" id="btnFwd" onclick="motorToggle('fwd')">▲ Maju</div>
        <div class="motor-btn stop-btn" onclick="motorCmd('stop')">■ Stop</div>
        <div class="motor-btn" id="btnBwd" onclick="motorToggle('bwd')">▼ Mundur</div>
      </div>
    </div>

    <!-- SERVO -->
    <div class="section-label">Servo Control</div>
    <div class="grid-3">
      <div class="servo-card" id="sc1">
        <div class="servo-name">Servo 1 — Infeksius (Pin 33)</div>
        <div class="servo-angle" id="sa1">0 <span>deg</span></div>
        <div class="servo-vis">
          <svg width="80" height="50" viewBox="0 0 80 50">
            <path d="M10,45 A35,35 0 0,1 70,45" fill="none" stroke="#30363d" stroke-width="4" stroke-linecap="round"/>
            <line id="needle1" x1="40" y1="45" x2="40" y2="12" stroke="#00d9ff" stroke-width="2.5" stroke-linecap="round"/>
            <circle cx="40" cy="45" r="4" fill="#00d9ff"/>
          </svg>
        </div>
        <input type="range" min="0" max="180" value="0" step="1" id="sl1"
               oninput="servoInput(1,this.value)" onchange="servoSend(1,this.value)">
        <div class="preset-row" style="margin-top:10px;">
          <div class="preset-btn" onclick="servoPreset(1,0)">0°</div>
          <div class="preset-btn" onclick="servoPreset(1,45)">45°</div>
          <div class="preset-btn" onclick="servoPreset(1,90)">90°</div>
          <div class="preset-btn" onclick="servoPreset(1,135)">135°</div>
          <div class="preset-btn" onclick="servoPreset(1,180)">180°</div>
        </div>
      </div>

      <div class="servo-card" id="sc2">
        <div class="servo-name">Servo 2 — Non-Infeksius (Pin 19)</div>
        <div class="servo-angle" id="sa2">0 <span>deg</span></div>
        <div class="servo-vis">
          <svg width="80" height="50" viewBox="0 0 80 50">
            <path d="M10,45 A35,35 0 0,1 70,45" fill="none" stroke="#30363d" stroke-width="4" stroke-linecap="round"/>
            <line id="needle2" x1="40" y1="45" x2="40" y2="12" stroke="#00d9ff" stroke-width="2.5" stroke-linecap="round"/>
            <circle cx="40" cy="45" r="4" fill="#00d9ff"/>
          </svg>
        </div>
        <input type="range" min="0" max="180" value="0" step="1" id="sl2"
               oninput="servoInput(2,this.value)" onchange="servoSend(2,this.value)">
        <div class="preset-row" style="margin-top:10px;">
          <div class="preset-btn" onclick="servoPreset(2,0)">0°</div>
          <div class="preset-btn" onclick="servoPreset(2,45)">45°</div>
          <div class="preset-btn" onclick="servoPreset(2,90)">90°</div>
          <div class="preset-btn" onclick="servoPreset(2,135)">135°</div>
          <div class="preset-btn" onclick="servoPreset(2,180)">180°</div>
        </div>
      </div>

      <div class="servo-card" id="sc3">
        <div class="servo-name">Servo 3 — B3 (Pin 18)</div>
        <div class="servo-angle" id="sa3">0 <span>deg</span></div>
        <div class="servo-vis">
          <svg width="80" height="50" viewBox="0 0 80 50">
            <path d="M10,45 A35,35 0 0,1 70,45" fill="none" stroke="#30363d" stroke-width="4" stroke-linecap="round"/>
            <line id="needle3" x1="40" y1="45" x2="40" y2="12" stroke="#00d9ff" stroke-width="2.5" stroke-linecap="round"/>
            <circle cx="40" cy="45" r="4" fill="#00d9ff"/>
          </svg>
        </div>
        <input type="range" min="0" max="180" value="0" step="1" id="sl3"
               oninput="servoInput(3,this.value)" onchange="servoSend(3,this.value)">
        <div class="preset-row" style="margin-top:10px;">
          <div class="preset-btn" onclick="servoPreset(3,0)">0°</div>
          <div class="preset-btn" onclick="servoPreset(3,45)">45°</div>
          <div class="preset-btn" onclick="servoPreset(3,90)">90°</div>
          <div class="preset-btn" onclick="servoPreset(3,135)">135°</div>
          <div class="preset-btn" onclick="servoPreset(3,180)">180°</div>
        </div>
      </div>
    </div>

    <!-- HARDWARE DIAGNOSTICS -->
    <div class="section-label">Hardware Diagnostics (Test Mode)</div>
    <div class="grid-3" style="margin-bottom: 20px;">
      <div class="motor-btn" style="border-color:var(--purple);color:var(--purple);" onclick="testHardware('yolo')">🎯 Simulasi Deteksi YOLO</div>
      <div class="motor-btn" style="border-color:var(--warn);color:var(--warn);" onclick="testHardware('buzzer')">🔊 Test Buzzer (1s)</div>
      <div class="motor-btn" style="border-color:var(--accent);color:var(--accent);" onclick="testHardware('motor')">⚙ Test Motor (Auto)</div>
      <div class="motor-btn" style="border-color:var(--muted);color:var(--muted);" onclick="testHardware('gas')">💨 Test Gas</div>
      <div class="motor-btn" style="border-color:var(--ok);color:var(--ok);" onclick="testHardware('water')">💧 Test Air</div>
      <div class="motor-btn" style="border-color:var(--danger);color:var(--danger);" onclick="testHardware('flame')">🔥 Test Api</div>
    </div>

  </div><!-- /adminView -->

</main>
</div><!-- /appPage -->

<!-- TOAST -->
<div class="toast" id="toast"></div>

<script>
// ═══════════════════════════════════════
//  IP CONFIG
// ═══════════════════════════════════════
// Dinamis mengikuti IP ESP32 di browser
let esp32Host = window.location.hostname;

function saveIp() {
  const ip = document.getElementById('esp32IpInput').value.trim();
  const ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
  if (!ipRegex.test(ip)) {
    document.getElementById('ipErr').className = 'login-err show';
    return;
  }
  esp32Host = ip;
  document.getElementById('ipConfigPage').style.display = 'none';
  document.getElementById('loginPage').style.display = 'flex';
}

// Saat halaman dimuat: cek apakah dibuka sebagai file lokal
window.addEventListener('load', () => {
  // Selalu tampilkan halaman login terlepas dari bagaimana file dibuka (localhost atau klik ganda langsung)
  document.getElementById('loginPage').style.display = 'flex';
  document.getElementById('ipConfigPage').style.display = 'none';
});

// ═══════════════════════════════════════
//  AUTH
// ═══════════════════════════════════════
const ADMIN_USER = 'Admin';
const ADMIN_PASS = 'Admin123';
let currentRole = null; // 'user' | 'admin'

function switchTab(tab) {
  document.getElementById('tabUser').className = 'login-tab' + (tab === 'user' ? ' active' : '');
  document.getElementById('tabAdmin').className = 'login-tab' + (tab === 'admin' ? ' active' : '');
  document.getElementById('sectionUser').className = 'login-section' + (tab === 'user' ? ' active' : '');
  document.getElementById('sectionAdmin').className = 'login-section' + (tab === 'admin' ? ' active' : '');
}

function loginAsUser() {
  currentRole = 'user';
  enterApp();
}

function doAdminLogin() {
  const u = document.getElementById('adminUser').value.trim();
  const p = document.getElementById('adminPass').value;
  if (u === ADMIN_USER && p === ADMIN_PASS) {
    currentRole = 'admin';
    enterApp();
  } else {
    document.getElementById('loginErr').className = 'login-err show';
    document.getElementById('adminPass').value = '';
  }
}

function enterApp() {
  document.getElementById('loginPage').style.display = 'none';
  document.getElementById('appPage').style.display = 'block';

  const badge = document.getElementById('roleBadge');
  if (currentRole === 'admin') {
    badge.textContent = 'Admin';
    badge.className = 'role-badge admin';
    document.getElementById('adminView').style.display = 'block';
    document.getElementById('userView').style.display = 'none';
  } else {
    badge.textContent = 'Pengguna';
    badge.className = 'role-badge user';
    document.getElementById('adminView').style.display = 'none';
    document.getElementById('userView').style.display = 'block';
  }

  connectWS();
  showToast('Selamat datang, ' + (currentRole === 'admin' ? 'Admin' : 'Pengguna') + '!');
}

function doLogout() {
  currentRole = null;
  if (ws) { try { ws.close(); } catch(e){} ws = null; }
  document.getElementById('appPage').style.display = 'none';
  document.getElementById('loginPage').style.display = 'flex';
  document.getElementById('adminUser').value = '';
  document.getElementById('adminPass').value = '';
  document.getElementById('loginErr').className = 'login-err';
}

// ═══════════════════════════════════════
//  TOAST
// ═══════════════════════════════════════
let toastTimer;
function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast show';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.className = 'toast'; }, 2800);
}

// ═══════════════════════════════════════
//  WEBSOCKET
// ═══════════════════════════════════════
let ws = null, connected = false;
let mq2Ready = false, warmupStartTime = null, warmupInterval = null;
let motorState = 'stop', motorSpeed = 200;
const MOTOR_PRESETS = [51, 102, 153, 200, 255];

function setConn(state) {
  const dot = document.getElementById('connDot');
  const lbl = document.getElementById('connLabel');
  if (state === 'connected') {
    dot.className = 'conn-dot online';
    lbl.className = 'conn-label online';
    lbl.textContent = 'Online (' + esp32Host + ')';
  } else if (state === 'connecting') {
    dot.style.background = 'var(--warn)'; dot.className = 'conn-dot';
    lbl.style.color = 'var(--warn)'; lbl.className = 'conn-label';
    lbl.textContent = 'Menghubungkan...';
  } else {
    dot.style.background = ''; dot.className = 'conn-dot';
    lbl.style.color = ''; lbl.className = 'conn-label';
    lbl.textContent = 'Offline (' + esp32Host + ')';
  }
}

function connectWS() {
  setConn('connecting');
  ws = new WebSocket('ws://' + esp32Host + ':81');
  ws.onopen = () => { connected = true; setConn('connected'); };
  ws.onmessage = (evt) => { try { const d = JSON.parse(evt.data); if (d.type === 'status') updateUI(d); } catch(e){} };
  ws.onclose = ws.onerror = () => { connected = false; ws = null; setConn('disconnected'); stopWarmup(); setTimeout(connectWS, 3000); };
}

function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

// ═══════════════════════════════════════
//  WASTE STATISTICS LOGIC
// ═══════════════════════════════════════
let wasteStats = JSON.parse(localStorage.getItem('wasteStats')) || {};
let sensorStats = JSON.parse(localStorage.getItem('sensorStats')) || { gas: 0, water: 0, flame: 0 };
let prevAlerts = { gas: false, water: false, flame: false };
let prevServoActive = 0;
let sortPopupTimer;

function showSortPopup(msg) {
  const p = document.getElementById('sortPopup');
  p.textContent = msg;
  p.classList.add('show');
  clearTimeout(sortPopupTimer);
  sortPopupTimer = setTimeout(() => { p.classList.remove('show'); }, 3000);
}

function openSidebar() {
  document.getElementById('sidebarOverlay').classList.add('show');
  document.getElementById('sidebar').classList.add('show');
  renderStats();
}
function closeSidebar() {
  document.getElementById('sidebarOverlay').classList.remove('show');
  document.getElementById('sidebar').classList.remove('show');
}

function resetStats() {
  if (confirm("Hapus seluruh data riwayat limbah tersortir?")) {
    wasteStats = {};
    localStorage.removeItem('wasteStats');
    renderStats();
  }
}

function resetSensorStats() {
  if (confirm("Hapus seluruh data riwayat analisis sensor?")) {
    sensorStats = { gas: 0, water: 0, flame: 0 };
    localStorage.removeItem('sensorStats');
    renderStats();
  }
}

function renderStats() {
  const listEl = document.getElementById('wasteList');
  const svgCont = document.getElementById('svgContainer');
  const legendCont = document.getElementById('chartLegend');
  listEl.innerHTML = ''; svgCont.innerHTML = ''; legendCont.innerHTML = '';
  
  // Periksa apakah format lama (tanpa kategori) atau kosong
  const keys = Object.keys(wasteStats);
  if (keys.length > 0 && typeof wasteStats[keys[0]] === 'number') {
    wasteStats = {}; // Reset jika format lama
    localStorage.removeItem('wasteStats');
  }
  
  if (Object.keys(wasteStats).length === 0) {
    listEl.innerHTML = '<div style="color:var(--muted); text-align:center; padding: 20px;">Belum ada data limbah.</div>';
    return;
  }

  const bgColors = ['#00d9ff', '#3fb950', '#ff7043', '#bc8cff', '#ffd700', '#f85149'];
  let colorIdx = 0;
  
  let categoryTotals = [];
  let grandTotal = 0;

  for (const cat in wasteStats) {
    let catTotal = 0;
    let itemsHtml = '';
    
    for (const name in wasteStats[cat]) {
      const count = wasteStats[cat][name];
      catTotal += count;
      itemsHtml += `<div class="category-item"><span>${name}</span><span>${count}</span></div>`;
    }
    
    grandTotal += catTotal;
    const catColor = bgColors[colorIdx % bgColors.length];
    categoryTotals.push({ label: cat, val: catTotal, color: catColor });
    
    const catDiv = document.createElement('div');
    catDiv.className = 'category-group';
    catDiv.innerHTML = `
      <div class="category-head" style="border-left: 4px solid ${catColor}">${cat}</div>
      ${itemsHtml}
      <div class="category-total"><span>Total</span><span>${catTotal}</span></div>
    `;
    listEl.appendChild(catDiv);
    colorIdx++;
  }

  // Render SVG Pie Chart
  if (grandTotal > 0) {
    let svgHtml = `<svg viewBox="0 0 32 32" class="svg-pie">`;
    let legendHtml = '';
    let cumulativePercent = 0;
    
    categoryTotals.forEach(item => {
      const percent = item.val / grandTotal;
      const dashArray = `${percent * 100} 100`;
      const dashOffset = -cumulativePercent * 100;
      
      svgHtml += `<circle r="16" cx="16" cy="16" fill="transparent" stroke="${item.color}" stroke-width="32" stroke-dasharray="${dashArray}" stroke-dashoffset="${dashOffset}"></circle>`;
      
      legendHtml += `<div class="legend-item"><div class="legend-color" style="background:${item.color}"></div>${item.label}</div>`;
      
      cumulativePercent += percent;
    });
    
    svgHtml += `</svg>`;
    svgCont.innerHTML = svgHtml;
    legendCont.innerHTML = legendHtml;
  }

  // ── RENDER SENSOR STATS (LINE CHART) ──
  const sList = document.getElementById('sensorList');
  const sSvg = document.getElementById('sensorSvgContainer');
  sList.innerHTML = ''; sSvg.innerHTML = '';

  const sData = [
    { key: 'gas', label: 'Gas/Asap', val: sensorStats.gas || 0, color: '#bc8cff' },
    { key: 'water', label: 'Air Berlebih', val: sensorStats.water || 0, color: '#00d9ff' },
    { key: 'flame', label: 'Api', val: sensorStats.flame || 0, color: '#f85149' }
  ];

  let maxVal = Math.max(1, ...sData.map(d => d.val));
  if (maxVal < 5) maxVal = 5; // Minimum scale for nice looking chart

  sData.forEach(item => {
    sList.innerHTML += `<div class="sensor-item"><div style="display:flex;align-items:center;"><span class="sensor-dot" style="background:${item.color}"></span>${item.label}</div><strong style="color:${item.color}; font-size:16px;">${item.val}</strong></div>`;
  });

  let svgW = 200, svgH = 100;
  let gap = svgW / 2; // 3 points = 2 intervals
  
  let pts = sData.map((d, i) => {
    let x = i * gap;
    let y = svgH - ((d.val / maxVal) * (svgH - 25)) - 12; // 12px bottom padding, leaving room at top
    return `${x},${y}`;
  });

  let polyHtml = `<polyline points="${pts.join(' ')}" fill="none" stroke="rgba(255,255,255,0.15)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" />`;
  let circlesHtml = sData.map((d, i) => {
    let x = i * gap;
    let y = svgH - ((d.val / maxVal) * (svgH - 25)) - 12;
    return `<circle cx="${x}" cy="${y}" r="4" fill="${d.color}" stroke="var(--surface2)" stroke-width="1.5" />
            <text x="${x}" y="${y - 10}" fill="${d.color}" font-size="11" text-anchor="middle" font-weight="bold">${d.val}</text>`;
  }).join('');

  sSvg.innerHTML = `<svg viewBox="-15 0 ${svgW + 30} ${svgH}" class="svg-line">
    ${polyHtml}
    ${circlesHtml}
  </svg>`;
}

// ═══════════════════════════════════════
//  UPDATE UI
// ═══════════════════════════════════════
function updateUI(d) {
  // ── ALERTS ──
  const alerts = [];
  if (d.gasAlert)   alerts.push('GAS/ASAP');
  if (d.waterAlert) alerts.push('AIR BERLEBIH');
  if (d.flameAlert) alerts.push('API');
  
  // Deteksi sensor untuk statistik — pakai cooldown 5 detik agar tidak double-count
  const now_ms = Date.now();
  const SENSOR_COOLDOWN = 5000; // ms
  if (!window._lastSensorCount) window._lastSensorCount = { gas: 0, water: 0, flame: 0 };
  
  if (d.gasAlert && !prevAlerts.gas && (now_ms - (window._lastSensorCount.gas||0)) > SENSOR_COOLDOWN) {
    sensorStats.gas = (sensorStats.gas||0)+1;
    window._lastSensorCount.gas = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  if (d.waterAlert && !prevAlerts.water && (now_ms - (window._lastSensorCount.water||0)) > SENSOR_COOLDOWN) {
    sensorStats.water = (sensorStats.water||0)+1;
    window._lastSensorCount.water = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  if (d.flameAlert && !prevAlerts.flame && (now_ms - (window._lastSensorCount.flame||0)) > SENSOR_COOLDOWN) {
    sensorStats.flame = (sensorStats.flame||0)+1;
    window._lastSensorCount.flame = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  prevAlerts.gas   = !!d.gasAlert;
  prevAlerts.water = !!d.waterAlert;
  prevAlerts.flame = !!d.flameAlert;

  const banner = document.getElementById('alertBanner');
  if (alerts.length) {
    banner.classList.add('active');
    document.getElementById('alertText').textContent = '⚠ BAHAYA: ' + alerts.join(' + ') + '!';
  } else { banner.classList.remove('active'); }

  // ── YOLO DETECTION (shared data) ──
  const wasteName  = d.wasteName || 'Tidak Ada';
  const wasteCat   = d.wasteCat  || '—';
  const servoNames = { 1: '▸ Servo 1 — Infeksius (Kantong Kuning)', 2: '▸ Servo 2 — Non-Infeksius (Kantong Hitam)', 3: '▸ Servo 3 — B3 (Kantong Merah)' };
  const servoText  = (d.servoActive && d.servoActive > 0 && wasteName !== 'Tidak Ada')
                     ? (servoNames[d.servoActive] || '▸ Servo ' + d.servoActive) : null;

  // Cek jika servo baru saja terbuka (menandakan limbah berhasil disortir)
  if (d.servoActive > 0 && prevServoActive === 0 && wasteName !== 'Tidak Ada') {
    // Cek format lama, jika iya reset
    const keys = Object.keys(wasteStats);
    if (keys.length > 0 && typeof wasteStats[keys[0]] === 'number') {
      wasteStats = {};
    }
    
    const cat = wasteCat !== '—' ? wasteCat : 'Lainnya';
    if (!wasteStats[cat]) wasteStats[cat] = {};
    if (!wasteStats[cat][wasteName]) wasteStats[cat][wasteName] = 0;
    
    wasteStats[cat][wasteName]++;
    localStorage.setItem('wasteStats', JSON.stringify(wasteStats));
    showSortPopup('Berhasil menyortir limbah "' + wasteName + '"');
    
    // Update chart jika sidebar sedang terbuka
    if (document.getElementById('sidebar').classList.contains('show')) {
      renderStats();
    }
  }
  prevServoActive = d.servoActive || 0;

  // User live view
  if (document.getElementById('userView').style.display !== 'none') {
    document.getElementById('liveWasteName').textContent = wasteName;
    document.getElementById('liveWasteCat').textContent = 'Kategori: ' + wasteCat;
    const lsi = document.getElementById('liveServoInfo');
    if (servoText) { lsi.textContent = servoText; lsi.style.display = 'inline-block'; }
    else { lsi.style.display = 'none'; }

    const chipGas   = document.getElementById('chipGas');
    const chipWater = document.getElementById('chipWater');
    const chipFlame = document.getElementById('chipFlame');
    const chipMotor = document.getElementById('chipMotor');
    chipGas.className   = 'live-sensor-chip' + (d.gasAlert ? ' alert' : '');
    chipGas.textContent = '🟣 Gas: ' + (d.gasADC !== undefined ? d.gasADC : '—');
    chipWater.className = 'live-sensor-chip' + (d.waterAlert ? ' alert' : '');
    chipWater.textContent = '💧 Air: ' + (d.waterADC !== undefined ? d.waterADC : '—');
    chipFlame.className = 'live-sensor-chip' + (d.flameAlert ? ' alert' : '');
    chipFlame.textContent = '🔥 Api: ' + (d.flameAlert ? 'TERDETEKSI' : 'Aman');
    chipMotor.textContent = '⚙ Motor: ' + (d.motorState || 'stop').toUpperCase();
  }

  // Admin full panel
  if (document.getElementById('adminView').style.display !== 'none') {
    // Mode & Safety
    if (d.mode !== undefined) document.getElementById('modeSelect').value = d.mode;
    if (d.safetyLockout) {
      document.getElementById('safetyLockoutUI').style.display = 'block';
    } else {
      document.getElementById('safetyLockoutUI').style.display = 'none';
    }
    // START Button state
    const startBtn    = document.getElementById('startBtn');
    const startBanner = document.getElementById('startBanner');
    if (d.systemStarted) {
      startBtn.textContent = '⏹ STOP / GANTI MODE';
      startBtn.style.background = 'linear-gradient(135deg, #f85149 0%, #d43a32 100%)';
      startBtn.style.boxShadow  = '0 4px 20px rgba(248,81,73,0.3)';
      startBanner.className = 'start-banner running';
      startBanner.innerHTML = '<span>▶</span><span>Sistem <strong>berjalan</strong>. Motor aktif sesuai mode.</span>';
    } else {
      startBtn.textContent = '▶ START SISTEM';
      startBtn.style.background = 'linear-gradient(135deg, #00d9ff 0%, #00b8d9 100%)';
      startBtn.style.boxShadow  = '0 4px 20px rgba(0,217,255,0.3)';
      startBanner.className = 'start-banner';
      startBanner.innerHTML = '<span>⏸</span><span>Pilih mode lalu tekan <strong>START SISTEM</strong> untuk mulai.</span>';
    }
    startBtn.disabled = !!d.safetyLockout;

    // Gas
    if (d.mq2Ready) {
      document.getElementById('gasVal').textContent = d.gasADC;
      const gPct = Math.min(100, (d.gasADC / 4095) * 100);
      const gBar = document.getElementById('gasBar');
      gBar.style.width = gPct + '%';
      gBar.className = 'bar' + (d.gasAlert ? ' danger' : gPct > 40 ? ' warn' : '');
      const gp = document.getElementById('gasPill');
      gp.className = 'pill ' + (d.gasAlert ? 'danger' : 'ok');
      gp.innerHTML = '<span class="pill-dot"></span>' + (d.gasAlert ? 'Asap Terdeteksi' : 'Normal');
    }

    // Water
    document.getElementById('waterVal').textContent = d.waterADC;
    const wPct = Math.min(100, (d.waterADC / 4095) * 100);
    const wBar = document.getElementById('waterBar');
    wBar.style.width = wPct + '%';
    wBar.className = 'bar' + (d.waterAlert ? ' danger' : wPct > 30 ? ' warn' : '');
    const wp = document.getElementById('waterPill');
    wp.className = 'pill ' + (d.waterAlert ? 'danger' : 'ok');
    wp.innerHTML = '<span class="pill-dot"></span>' + (d.waterAlert ? 'Air Berlebih' : 'Aman');

    // Flame
    document.getElementById('flameADC').textContent = d.flameADC;
    document.getElementById('flameD0').textContent = d.flameD0 ? 'HIGH (Api!)' : 'LOW (Aman)';
    const fp = document.getElementById('flamePill');
    fp.className = 'pill ' + (d.flameAlert ? 'danger' : 'ok');
    fp.innerHTML = '<span class="pill-dot"></span>' + (d.flameAlert ? 'Api Terdeteksi' : 'Aman');

    // Button
    const bp = document.getElementById('btnPill');
    bp.className = 'btn-state ' + (d.btnPressed ? 'pressed' : 'released');
    bp.innerHTML = '<span class="pill-dot"></span>' + (d.btnPressed ? 'Pressed' : 'Released');

    // Motor
    motorState = d.motorState || 'stop';
    motorSpeed = d.motorSpeed || 200;
    document.getElementById('motorSpeed').value = motorSpeed;
    document.getElementById('motorSpeedVal').textContent = motorSpeed;
    document.getElementById('motorSpeedPct').textContent = Math.round(motorSpeed / 255 * 100) + '%';
    updateMotorUI();
    updateMotorPresetHighlight(motorSpeed);

    // Warmup
    if (!d.mq2Ready) {
      document.getElementById('warmupSection').style.display = 'block';
      document.getElementById('warmupDone').style.display = 'none';
      if (!warmupStartTime) startWarmupAnim();
    } else {
      document.getElementById('warmupSection').style.display = 'none';
      document.getElementById('warmupDone').style.display = 'block';
      stopWarmup(); mq2Ready = true;
    }

    // YOLO
    document.getElementById('wasteName').textContent = wasteName;
    document.getElementById('wasteCat').textContent = 'Kategori: ' + wasteCat;
    const si = document.getElementById('wasteServoInfo');
    if (servoText) { si.textContent = servoText; si.style.display = 'block'; }
    else { si.style.display = 'none'; }

    // Servo
    updateServoUI(1, d.servo1);
    updateServoUI(2, d.servo2);
    updateServoUI(3, d.servo3);
  }
}

// ═══════════════════════════════════════
//  MOTOR
// ═══════════════════════════════════════
function updateMotorUI() {
  const st = document.getElementById('motorStatusText');
  const bf = document.getElementById('btnFwd');
  const bb = document.getElementById('btnBwd');
  const pi = document.getElementById('motorPinInfo');
  bf.className = 'motor-btn'; bb.className = 'motor-btn';
  if (motorState === 'fwd') {
    st.textContent = 'MAJU'; st.style.color = 'var(--ok)';
    bf.classList.add('active-fwd');
    pi.textContent = 'IN1:H   IN2:L   ENA:' + motorSpeed;
  } else if (motorState === 'bwd') {
    st.textContent = 'MUNDUR'; st.style.color = 'var(--accent2)';
    bb.classList.add('active-bwd');
    pi.textContent = 'IN1:L   IN2:H   ENA:' + motorSpeed;
  } else {
    st.textContent = 'STOP'; st.style.color = 'var(--muted)';
    pi.textContent = 'IN1:L   IN2:L   ENA:0';
  }
}

function motorToggle(dir) { if (motorState === dir) motorCmd('stop'); else motorCmd(dir); }
function motorCmd(dir) { send({ cmd: 'motor', dir: dir, speed: motorSpeed }); }

function motorPreset(v) {
  motorSpeed = v;
  document.getElementById('motorSpeed').value = v;
  document.getElementById('motorSpeedVal').textContent = v;
  document.getElementById('motorSpeedPct').textContent = Math.round(v / 255 * 100) + '%';
  updateMotorPresetHighlight(v);
  send({ cmd: 'motor', dir: motorState, speed: v });
}

function updateMotorPresetHighlight(v) {
  const btns = document.querySelectorAll('.motor-card .preset-btn');
  MOTOR_PRESETS.forEach((p, i) => { btns[i].className = 'preset-btn' + (p === v ? ' active' : ''); });
}

function speedInput(v) {
  v = parseInt(v); motorSpeed = v;
  document.getElementById('motorSpeedVal').textContent = v;
  document.getElementById('motorSpeedPct').textContent = Math.round(v / 255 * 100) + '%';
  updateMotorPresetHighlight(v);
}
function speedSend(v) { v = parseInt(v); motorSpeed = v; send({ cmd: 'motor', dir: motorState, speed: v }); }

// ═══════════════════════════════════════
//  FLASH (SENTER KAMERA) VIA WEBSOCKET
// ═══════════════════════════════════════
let camFlashState = false;
let camWs = null;

function getCamIp() {
  const inp = document.getElementById('camIpInput');
  return inp ? inp.value.trim() || '192.168.4.4' : '192.168.4.4';
}

function updateCamWsStatus(connected) {
  const st = document.getElementById('camWsStatusText');
  if (st) {
    if (connected) {
      st.innerHTML = 'WebSocket: <span style="color:var(--ok)">Connected (Port 81)</span>';
    } else {
      st.innerHTML = 'WebSocket: <span style="color:var(--danger)">Disconnected</span>';
    }
  }
}

function connectCamWs() {
  if (camWs && (camWs.readyState === WebSocket.CONNECTING || camWs.readyState === WebSocket.OPEN)) {
    return;
  }
  const camIp = getCamIp();
  console.log("Connecting to CAM WS: ws://" + camIp + ":81");
  camWs = new WebSocket('ws://' + camIp + ':81');
  camWs.onopen = () => {
    console.log("CAM WS Connected");
    updateCamWsStatus(true);
  };
  camWs.onmessage = (e) => {
    try {
      const d = JSON.parse(e.data);
      if (d.type === 'flash_status') {
        camFlashState = (d.state === 1);
        updateFlashUI();
      }
    } catch(err){}
  };
  camWs.onclose = () => {
    console.log("CAM WS Disconnected");
    camWs = null;
    updateCamWsStatus(false);
  };
  camWs.onerror = () => {
    console.log("CAM WS Error");
  };
}

function sendFlash(state) {
  if (!camWs || camWs.readyState !== WebSocket.OPEN) {
    showToast('⚠ Menghubungkan ke Kamera... coba lagi.');
    connectCamWs();
    return;
  }
  camWs.send(JSON.stringify({ cmd: "flash", state: state }));
}

function updateFlashUI() {
  const txt = document.getElementById('flashStatusText');
  const onBtn  = document.getElementById('flashOnBtn');
  const offBtn = document.getElementById('flashOffBtn');
  if (!txt) return;
  if (camFlashState) {
    txt.textContent = 'ON 💡';
    txt.style.color = '#ffd700';
    onBtn.style.background  = 'rgba(255,220,0,0.3)';
    onBtn.style.borderColor = '#ffd700';
    offBtn.style.background  = 'var(--bg)';
    offBtn.style.borderColor = 'var(--border)';
  } else {
    txt.textContent = 'OFF';
    txt.style.color = 'var(--muted)';
    onBtn.style.background  = 'rgba(255,220,0,0.08)';
    onBtn.style.borderColor = 'rgba(255,220,0,0.3)';
    offBtn.style.background  = 'rgba(0,217,255,0.08)';
    offBtn.style.borderColor = 'rgba(0,217,255,0.4)';
  }
}

// ═══════════════════════════════════════
//  MODE & SAFETY
// ═══════════════════════════════════════
function sendMode() {
  const m = parseInt(document.getElementById('modeSelect').value);
  send({ cmd: 'set_mode', mode: m });
  const modeNames = { 0: 'Keep Going', 1: 'Less Energy' };
  showToast('Mode: ' + (modeNames[m] || m) + '. Tekan START untuk mulai.');
}

function sendStart() {
  const startBtn = document.getElementById('startBtn');
  if (startBtn && startBtn.textContent.includes('STOP')) {
    // Jika sudah running, tombol jadi STOP — ganti mode untuk reset
    const m = parseInt(document.getElementById('modeSelect').value);
    send({ cmd: 'set_mode', mode: m });
    showToast('Sistem dihentikan. Tekan START untuk memulai ulang.');
  } else {
    send({ cmd: 'start_system' });
    showToast('Sistem di-START!');
  }
}

function sendResume() {
  send({ cmd: 'resume' });
  showToast('Safety Lockout direset. Tekan START untuk melanjutkan.');
}

// ═══════════════════════════════════════
//  SERVO
// ═══════════════════════════════════════
function updateServoUI(id, angle) {
  if (angle === undefined) return;
  document.getElementById('sa' + id).innerHTML = angle + ' <span>deg</span>';
  document.getElementById('sl' + id).value = angle;
  rotateNeedle(id, angle);
}

function rotateNeedle(id, angle) {
  const n = document.getElementById('needle' + id); if (!n) return;
  const rad = ((180 - angle) / 180) * Math.PI, cx = 40, cy = 45, len = 28;
  const ex = cx + len * Math.cos(Math.PI - rad), ey = cy - len * Math.sin(Math.PI - rad);
  n.setAttribute('x2', ex.toFixed(1)); n.setAttribute('y2', ey.toFixed(1));
}

function servoInput(id, val) {
  val = parseInt(val);
  document.getElementById('sa' + id).innerHTML = val + ' <span>deg</span>';
  rotateNeedle(id, val);
  document.getElementById('sc' + id).classList.add('active');
}

let servoTimer = {};
function servoSend(id, val) {
  clearTimeout(servoTimer[id]);
  servoTimer[id] = setTimeout(() => {
    send({ cmd: 'servo', id: id, angle: parseInt(val) });
    document.getElementById('sc' + id).classList.remove('active');
  }, 50);
}
function servoPreset(id, angle) {
  document.getElementById('sl' + id).value = angle;
  servoInput(id, angle); servoSend(id, angle);
}

// ═══════════════════════════════════════
//  TESTING
// ═══════════════════════════════════════
function testHardware(type) {
  if (type === 'yolo') {
    const items = [
      {id:1, w:'Plester (Test)', c:'Limbah Infeksius'},
      {id:2, w:'Kain Kasa (Test)', c:'Limbah Non-Infeksius'},
      {id:3, w:'Obat B3 (Test)', c:'Limbah B3'}
    ];
    const sel = items[Math.floor(Math.random() * items.length)];
    fetch(`http://${esp32Host}/api?cmd=servo&id=${sel.id}&angle=90&waste=${encodeURIComponent(sel.w)}&cat=${encodeURIComponent(sel.c)}`);
    showToast('Simulasi deteksi YOLO dikirim!');
    setTimeout(() => {
       fetch(`http://${esp32Host}/api?cmd=servo&id=${sel.id}&angle=0&waste=Tidak%20Ada&cat=-`);
    }, 3000);
  } else {
    send({ cmd: 'test', target: type });
    showToast('Menjalankan Test: ' + type.toUpperCase());
  }
}

// ═══════════════════════════════════════
//  MQ-2 WARMUP ANIMATION
// ═══════════════════════════════════════
function startWarmupAnim() {
  warmupStartTime = Date.now();
  warmupInterval = setInterval(() => {
    const elapsed = (Date.now() - warmupStartTime) / 1000, total = 20;
    const pct = Math.min(100, (elapsed / total) * 100), remaining = Math.max(0, Math.ceil(total - elapsed));
    document.getElementById('warmupFill').style.width = pct + '%';
    document.getElementById('warmupSec').textContent = remaining + 's';
  }, 200);
}
function stopWarmup() { if (warmupInterval) { clearInterval(warmupInterval); warmupInterval = null; } warmupStartTime = null; }

[1, 2, 3].forEach(id => rotateNeedle(id, 0));

window.onload = function() {
  updateStatusUI();
  updateTime();
  setInterval(updateTime, 1000);
  initWebSocket();
  connectCamWs(); // otomatis coba konek ke WS kamera saat buka dashboard
};
</script>
</body>
</html>

)====";

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
bool   safetyLockout = false;
bool   systemStarted = false; // false = belum di-START dari web, true = sistem berjalan

int    conveyorMode = 0; // 0 = Keep Going, 1 = Less Energy
uint32_t lessEnergyStopAt = 0;

int    servoAngle1 = 0;
int    servoAngle2 = 0;
int    servoAngle3 = 0;

// Waste Detection
String lastWasteName = "Tidak Ada";
String lastWasteCategory = "-";
int    lastActiveServo = 0;  // 0 = tidak ada, 1/2/3 = servo aktif

// Motor
String   motorState = "stop";   // "fwd" | "bwd" | "stop"
int      motorSpeed = 200;      // 0–255  (78% default — kecepatan konveyor)
uint32_t motorKickUntil     = 0;
uint32_t motorAutoReKickAt  = 0; // Waktu auto re-kick berikutnya (lawan Back-EMF)
uint32_t motorReKickUntil   = 0; // Berakhirnya pulse re-kick

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

int  lastGasVal   = 0;
bool lastGasD0    = false;
uint32_t gasDoutLowSince = 0;  // Kapan DOUT mulai LOW (debounce 2 detik)
int  lastWaterVal = 0;
int  lastFlameADC = 0;
bool lastFlameD0  = false;

uint32_t lastWsBroadcast = 0;
uint32_t lastDebugPrint  = 0;

// Testing Flags
uint32_t testGasUntil    = 0;
uint32_t testWaterUntil  = 0;
uint32_t testFlameUntil  = 0;
uint32_t testBuzzerUntil = 0;
uint32_t testMotorUntil  = 0;
int      testMotorStep   = 0;

// ── Servo Delay (sesuai jarak titik jatuh di konveyor) ───────
#define SERVO_DELAY_1  1700   // ms - Infeksius
#define SERVO_DELAY_2  1950   // ms - Non-Infeksius
#define SERVO_DELAY_3  2850   // ms - B3

struct ServoJob {
  bool     pending;
  uint32_t openAt;
  int      angle;
  String   waste;
  String   cat;
};
ServoJob servoJobs[4];  // index 1–3 dipakai, 0 diabaikan

// Auto-resume konveyor dihapus sesuai permintaan

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
void startConveyor();
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
  pinMode(PIN_GAS_AOUT,   INPUT_PULLDOWN); // Pulldown cegah floating pin dari sensor air
  pinMode(PIN_GAS_DOUT,   INPUT);           // MQ-2 punya resistor pull-up onboard sendiri
  pinMode(PIN_WATER_AOUT, INPUT);
  pinMode(PIN_FLAME_AOUT, INPUT);
  pinMode(PIN_FLAME_DOUT, INPUT_PULLDOWN);

  // Motor DC (Paksa ke Channel 4 agar tidak tabrakan dengan Servo)
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  ledcAttachChannel(PIN_MOTOR_ENA, PWM_MOTOR_FREQ, PWM_MOTOR_RES, 4);
  stopMotor();

  // RGB (Paksa ke Channel 5, 6, 7)
  ledcAttachChannel(PIN_RGB_R, PWM_FREQ, PWM_RES, 5);
  ledcAttachChannel(PIN_RGB_G, PWM_FREQ, PWM_RES, 6);
  ledcAttachChannel(PIN_RGB_B, PWM_FREQ, PWM_RES, 7);
  setRGBOff();

  analogReadResolution(12);
  
  // Buzzer (Paksa ke Channel 8 agar tidak membajak Timer Servo)
  ledcAttachChannel(PIN_BUZZER, 2000, 8, 8); // 2000Hz, 8-bit, Channel 8
  ledcWrite(PIN_BUZZER, 0);

  // ========== SERVO SETUP ==========
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  // Timer 3 sengaja TIDAK dialokasikan untuk Servo, 
  // agar bisa dipakai oleh ledcAttachChannel (Motor & RGB)

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

  Serial.print("[Servo] servo1 attached? ");
  Serial.println(servo1.attached() ? "YES" : "NO");
  Serial.print("[Servo] servo2 attached? ");
  Serial.println(servo2.attached() ? "YES" : "NO");
  Serial.print("[Servo] servo3 attached? ");
  Serial.println(servo3.attached() ? "YES" : "NO");
  // ====================================

  Wire.begin(21, 22);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1); lcd.print(WIFI_SSID);

  // ── KITA MATIKAN IP STATIS AGAR DAPAT IP OTOMATIS (DHCP) DARI HOTSPOT ──
  // IPAddress staticIP(192, 168, 4, 4); 
  // IPAddress gateway(192, 168, 4, 1);
  // IPAddress subnet(255, 255, 255, 0);
  // WiFi.config(staticIP, gateway, subnet);

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

  // --- JALUR HTTP KHUSUS UNTUK PYTHON YOLO (TEKNIK DRONE) ---
  server.on("/api", HTTP_GET, []() {
    if (server.hasArg("cmd")) {
      String cmd = server.arg("cmd");
      
      if (cmd == "servo") {
        int id    = server.arg("id").toInt();
        int angle = server.arg("angle").toInt();
        String w  = server.hasArg("waste") ? server.arg("waste") : lastWasteName;
        String c  = server.hasArg("cat")   ? server.arg("cat")   : lastWasteCategory;

        if (id >= 1 && id <= 3) {
          if (angle > 0 && (safetyLockout || !systemStarted)) {
            Serial.printf("[HTTP] Servo %d -> %d deg DIABAIKAN (Lockout/Belum Start)\n", id, angle);
          } else {
            // Hitung delay konveyor hanya saat MEMBUKA servo (angle > 0)
            uint32_t delayMs = 0;
            if (angle > 0) {
              if      (id == 1) delayMs = SERVO_DELAY_1;  // Infeksius     = 2.0 s
              else if (id == 2) delayMs = SERVO_DELAY_2;  // Non-Infeksius = 2.3 s
              else if (id == 3) delayMs = SERVO_DELAY_3;  // B3            = 2.6 s
            }

            servoJobs[id].pending = true;
            servoJobs[id].openAt  = millis() + delayMs;
            servoJobs[id].angle   = angle;
            servoJobs[id].waste   = w;
            servoJobs[id].cat     = c;

            Serial.printf("[HTTP] Servo %d -> %d deg dijadwalkan dalam %dms (Objek: %s)\n",
                          id, angle, delayMs, w.c_str());

            // Mode Less Energy: motor hanya nyala jika sistem sudah di-START dari web
            if (angle > 0 && conveyorMode == 1 && systemStarted && !safetyLockout && !gasAlert && !waterAlert && !flameAlert) {
              startConveyor();
              lessEnergyStopAt = millis() + 5000;
              Serial.println("[HTTP] Less Energy: Motor ON untuk 5 detik (ada objek)");
            }
          }
        }
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "machine") {
        String onStr = server.arg("on");
        if (onStr.equalsIgnoreCase("true")) {
          startConveyor();
          Serial.println("[HTTP] Mesin ON (kick-start 80%->60%)");
        } else {
          stopMotor();
          Serial.println("[HTTP] Mesin OFF");
        }
        server.send(200, "text/plain", "OK");
      }
      else {
        server.send(400, "text/plain", "Unknown command");
      }
      
      broadcastStatus(); // Update Dashboard Web UI
    } else {
      server.send(400, "text/plain", "Missing cmd argument");
    }
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

  // --- HARDWARE TEST HANDLER ---
  if (testBuzzerUntil > 0) {
    if (now >= testBuzzerUntil) { ledcWrite(PIN_BUZZER, 0); testBuzzerUntil = 0; }
  }
  if (testMotorUntil > 0) {
    if (testMotorStep == 1) {
      if (now >= testMotorUntil) { stopMotor(); testMotorStep = 2; testMotorUntil = now + 1000; }
    } else if (testMotorStep == 2) {
      if (now >= testMotorUntil) { setMotor("bwd", MOTOR_SPEED_KICK); testMotorStep = 3; testMotorUntil = now + 2000; }
    } else if (testMotorStep == 3) {
      if (now >= testMotorUntil) { stopMotor(); testMotorStep = 0; testMotorUntil = 0; }
    }
  }

  // --- MOTOR KICK-START RAMP-DOWN (80% -> 78%) ---
  if (motorKickUntil > 0 && now >= motorKickUntil && motorState == "fwd") {
    motorKickUntil = 0;
    motorSpeed = MOTOR_SPEED_RUN;
    ledcWrite(PIN_MOTOR_ENA, motorSpeed);
    motorAutoReKickAt = now + 8000; // Jadwalkan re-kick pertama 8 detik lagi
    Serial.println("[MOTOR] Ramp-down ke 78%");
    broadcastStatus();
  }

  // --- MOTOR AUTO RE-KICK (lawan Back-EMF setiap 8 detik) ---
  if (motorState == "fwd" && motorAutoReKickAt > 0 && now >= motorAutoReKickAt
      && motorKickUntil == 0 && motorReKickUntil == 0) {
    // Mulai pulse 100% selama 150ms
    ledcWrite(PIN_MOTOR_ENA, 255);
    motorReKickUntil  = now + 150;
    motorAutoReKickAt = 0;
    Serial.println("[MOTOR] Auto re-kick 100% (150ms)");
  }
  if (motorReKickUntil > 0 && now >= motorReKickUntil) {
    // Selesai pulse, kembali ke kecepatan normal
    motorReKickUntil  = 0;
    motorAutoReKickAt = now + 8000; // Jadwalkan re-kick berikutnya
    ledcWrite(PIN_MOTOR_ENA, motorSpeed);
    Serial.println("[MOTOR] Re-kick selesai, kembali ke 78%");
  }

  // --- SERVO SCHEDULED JOBS (delay konveyor) ---
  for (int i = 1; i <= 3; i++) {
    if (servoJobs[i].pending && now >= servoJobs[i].openAt) {
      servoJobs[i].pending = false;
      int a = servoJobs[i].angle;
      if      (i == 1) { servo1.write(a); servoAngle1 = a; }
      else if (i == 2) { servo2.write(a); servoAngle2 = a; }
      else if (i == 3) { servo3.write(a); servoAngle3 = a; }
      if (a > 0) {
        lastWasteName     = servoJobs[i].waste;
        lastWasteCategory = servoJobs[i].cat;
        lastActiveServo   = i;
        Serial.printf("[SERVO] Servo %d dibuka (%s)\n", i, lastWasteName.c_str());
      } else {
        if (i == lastActiveServo) { lastActiveServo = 0; }
        lastWasteName     = "Tidak Ada";
        lastWasteCategory = "-";
        Serial.printf("[SERVO] Servo %d ditutup\n", i);
      }
      broadcastStatus();
    }
  }

  // --- AUTO-RESUME KONVEYOR DIHAPUS ---
  // Sistem harus di-resume secara manual melalui UI Dashboard

  if (lcdClearPending && (now - lcdClearTimer >= LCD_CLEAR_DELAY)) {
    lcdClearPending = false;
    updateLCD();
  }

  if (conveyorMode == 1 && lessEnergyStopAt > 0 && now >= lessEnergyStopAt) {
    if (motorState != "stop") {
      stopMotor();
      Serial.println("[MOTOR] Mode Less Energy: Waktu 5 detik habis, motor stop.");
    }
    lessEnergyStopAt = 0;
  }

  if (now - lastWsBroadcast >= WS_BROADCAST_MS) {
    lastWsBroadcast = now;
    broadcastStatus();
  }
  
  if (now - lastDebugPrint >= 1000) {
    lastDebugPrint = now;
    Serial.printf("=== [DEBUG SENSOR] ===\n");
    Serial.printf("  GAS   (AOUT:35) : %d  | D0: %d\n", lastGasVal, lastGasD0);
    Serial.printf("  AIR   (AOUT:36) : %d\n", lastWaterVal);
    Serial.printf("  API   (AOUT:34) : %d  | D0: %d\n", lastFlameADC, lastFlameD0);
    Serial.printf("======================\n");
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
  motorKickUntil = 0;     // Hapus sisa timer kick (jika ada)
  motorReKickUntil = 0;   // Hapus sisa timer re-kick (jika ada)
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
      delay(10);
      Serial.printf("[WS] Servo3 → %d° (attached=%d)\n", angle, servo3.attached());
    }

    updateLCD();
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "set_mode") == 0) {
    conveyorMode = doc["mode"] | 0;
    // Setiap ganti mode, reset systemStarted — wajib tekan START lagi
    systemStarted = false;
    stopMotor();
    lessEnergyStopAt = 0;
    Serial.printf("[WS] Mode diubah menjadi: %d. Sistem di-reset, tunggu START.\n", conveyorMode);
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "start_system") == 0) {
    if (!safetyLockout && !gasAlert && !waterAlert && !flameAlert) {
      systemStarted = true;
      Serial.println("[WS] Sistem di-START dari web.");
      if (conveyorMode == 0) {
        // Keep Going: langsung jalankan konveyor
        startConveyor();
        Serial.println("[MOTOR] Keep Going: Konveyor mulai berjalan.");
      } else {
        // Less Energy: motor stand-by, akan nyala saat ada objek
        Serial.println("[MOTOR] Less Energy: Stand-by. Motor akan nyala saat ada objek.");
      }
    } else {
      Serial.println("[WS] START ditolak: ada alert bahaya aktif atau safety lockout!");
    }
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "resume") == 0) {
    // Paksa reset semua alert & lockout
    safetyLockout = false;
    gasAlert      = false;
    waterAlert    = false;
    flameAlert    = false;
    systemStarted = true; // Langsung mulai sistem kembali (tidak perlu tekan START lagi)
    
    Serial.println("[WS] Safety Lockout direset. Sistem langsung dilanjutkan.");
    
    if (conveyorMode == 0) {
      startConveyor();
      Serial.println("[MOTOR] Keep Going: Konveyor mulai berjalan (Resume).");
    } else {
      Serial.println("[MOTOR] Less Energy: Stand-by (Resume).");
    }
    
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

  // ── Test command ──
  if (strcmp(cmd, "test") == 0) {
    const char* target = doc["target"] | "";
    if (strcmp(target, "buzzer") == 0) {
      // Test buzzer: hanya beep, konveyor tidak berhenti
      testBuzzerUntil = millis() + 1000;
      ledcWrite(PIN_BUZZER, 128);
    } else if (strcmp(target, "gas") == 0) {
      // Test gas: hentikan konveyor + beep
      stopMotor();
      ledcWrite(PIN_BUZZER, 128); // 50% duty cycle
      testGasUntil      = millis() + 4000;
    } else if (strcmp(target, "water") == 0) {
      stopMotor();
      ledcWrite(PIN_BUZZER, 128);
      testWaterUntil    = millis() + 4000;
    } else if (strcmp(target, "flame") == 0) {
      stopMotor();
      ledcWrite(PIN_BUZZER, 128);
      testFlameUntil    = millis() + 4000;
    } else if (strcmp(target, "motor") == 0) {
      // Test motor: urutan maju 2s → stop 1s → mundur 2s → stop
      setMotor("fwd", motorSpeed);
      testMotorStep  = 1;
      testMotorUntil = millis() + 2000;
    }
    broadcastStatus();
    return;
  }
}

// ─────────────────────────────────────────────────────────────
//  STATUS JSON
// ─────────────────────────────────────────────────────────────
String buildStatusJson() {
  StaticJsonDocument<768> doc;
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
  doc["wasteName"]   = lastWasteName;
  doc["wasteCat"]    = lastWasteCategory;
  doc["servoActive"] = lastActiveServo;
  doc["mq2Ready"]   = mq2WarmupDone;
  doc["motorState"] = motorState;
  doc["motorSpeed"] = motorSpeed;
  doc["btnPressed"] = btnPressed;
  doc["mode"]          = conveyorMode;
  doc["safetyLockout"] = safetyLockout;
  doc["systemStarted"] = systemStarted;
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
    Serial.println("[MQ-2] Warm-up selesai.");
    updateLCD();
    return;
  }

  if (now - mq2LastRead < MQ2_READ_INTERVAL) return;
  mq2LastRead = now;

  // Abaikan sensor gas sepenuhnya jika Air atau Api sedang mendeteksi bahaya.
  // Ini mencegah 100% false-positive akibat ADC cross-talk atau voltage sag ekstrem!
  if (waterAlert || flameAlert) return;

  if (testGasUntil > 0 && now < testGasUntil) {
    lastGasVal = 4095;
    lastGasD0  = true;
    gasDoutLowSince = 0; // skip debounce saat test
  } else {
    analogRead(PIN_GAS_AOUT); // Dummy read (mengatasi ADC cross-talk)
    delay(2);                 // Beri waktu kapasitor internal ADC untuk stabil
    lastGasVal = analogRead(PIN_GAS_AOUT);
    bool rawD0 = (digitalRead(PIN_GAS_DOUT) == LOW);

    if (rawD0 && !lastGasD0) {
      // DOUT baru saja turun ke LOW — catat waktunya
      gasDoutLowSince = now;
    } else if (!rawD0) {
      // DOUT sudah HIGH lagi — reset timer
      gasDoutLowSince = 0;
    }
    // lastGasD0 hanya true jika sudah LOW terus-menerus >= 2000ms
    lastGasD0 = rawD0 && (gasDoutLowSince > 0) && (now - gasDoutLowSince >= 2000);
    Serial.printf("[GAS] rawD0=%d gasDoutLowSince=%lu durasi=%lums D0_valid=%d\n",
      rawD0, gasDoutLowSince, gasDoutLowSince ? (now - gasDoutLowSince) : 0, lastGasD0);
  }

  bool isDanger = lastGasD0 || (lastGasVal >= MQ2_THRESHOLD_ON);
  bool isSafe   = (!lastGasD0) && (lastGasVal <= MQ2_THRESHOLD_OFF);

  if (!gasAlert && isDanger) {
    gasAlert = true;
    safetyLockout = true;
    stopMotor();
    ledcWrite(PIN_BUZZER, 128); // 50% duty cycle
    setRGBBlink(COL_GAS_A, COL_GAS_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BAHAYA ASAP!!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastGasVal);
  } else if (gasAlert && isSafe) {
    gasAlert = false;
    if (!waterAlert && !flameAlert) {
      stopBuzzer(); setRGBOff();
      // startConveyor(); dihapus krn manual resume
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

  if (testWaterUntil > 0 && now < testWaterUntil) {
    lastWaterVal = 4095; // Simulasi air penuh
  } else {
    analogRead(PIN_WATER_AOUT); // Dummy read
    delay(2);
    lastWaterVal = analogRead(PIN_WATER_AOUT);
  }

  if (!waterAlert && lastWaterVal > WATER_THRESHOLD_ON) {
    waterAlert = true;
    safetyLockout = true;
    stopMotor();
    ledcWrite(PIN_BUZZER, 128);
    setRGBBlink(COL_WATER_A, COL_WATER_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BANJIR/AIR !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastWaterVal);
  } else if (waterAlert && lastWaterVal < WATER_THRESHOLD_OFF) {
    waterAlert = false;
    if (!gasAlert && !flameAlert) {
      stopBuzzer(); setRGBOff();
      // startConveyor(); dihapus krn manual resume
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

  if (testFlameUntil > 0 && now < testFlameUntil) {
    lastFlameD0 = true;
    lastFlameADC = 4095;
  } else {
    lastFlameD0  = (digitalRead(PIN_FLAME_DOUT) == HIGH);
    analogRead(PIN_FLAME_AOUT); // Dummy read
    delay(2);
    lastFlameADC = analogRead(PIN_FLAME_AOUT);
  }

  if (!flameAlert && (lastFlameD0 || lastFlameADC <= 3000)) {
    flameAlert = true;
    safetyLockout = true;
    stopMotor();
    ledcWrite(PIN_BUZZER, 128);
    setRGBBlink(COL_FLAME_A, COL_FLAME_B);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("!! BAHAYA API !!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastFlameADC);
  } else if (flameAlert && (!lastFlameD0 && lastFlameADC > 3000)) {
    flameAlert = false;
    if (!gasAlert && !waterAlert) {
      stopBuzzer(); setRGBOff();
      // startConveyor(); dihapus krn manual resume
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Api padam.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  BUZZER
// ─────────────────────────────────────────────────────────────
void stopBuzzer() {
  ledcWrite(PIN_BUZZER, 0);
}

// ─────────────────────────────────────────────────────────────
//  START CONVEYOR (Kick-start 80% → 60%)
// ─────────────────────────────────────────────────────────────
void startConveyor() {
  // Jalankan di 80% dulu agar konveyor bisa bergerak (tidak macet)
  setMotor("fwd", MOTOR_SPEED_KICK);
  motorKickUntil = millis() + MOTOR_KICK_MS;  // Setelah 400ms turun ke 60%
  Serial.println("[MOTOR] Kick-start 80% -> 60%");
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
