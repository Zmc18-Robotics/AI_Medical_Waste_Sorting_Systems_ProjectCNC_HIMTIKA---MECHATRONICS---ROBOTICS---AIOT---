/*
 * ============================================================
 *   SMART FACTORY - SERVO + MOTOR DC CONTROL (Web IoT)
 *   Platform  : ESP32 DevKit C v4
 *   Driver    : BTS7960 / IBT-2 43A High-Power H-Bridge
 *   Author    : Mc.Zminecrafter18
 *   Features  : BTS7960 H-Bridge PWM, MQ-2 Gas Calibrated, 
 *               Water Safety Lockout, 3-Servo Sorter, Web IoT
 *
 *   ★ CARA PAKAI:
 *   1. Isi WIFI_SSID dan WIFI_PASS di bawah
 *   2. Upload sketch ke ESP32
 *   3. Buka Serial Monitor (115200) → catat IP yang tampil
 *   4. Ketik IP di browser → dashboard langsung muncul!
 *
 *  Library yang dibutuhkan:
 *  - ESP32Servo
 *  - WebSockets (by Markus Sattler)
 *  - ArduinoJson
 *  - LiquidCrystal_I2C
 *
 *  PIN MAP BTS7960 VERSION (ESP32 KE-1):
 *  ┌─────────────────────────┬──────────┬─────────────────────────────────────┐
 *  │ Komponen                │ Pin ESP32│ Catatan                             │
 *  ├─────────────────────────┼──────────┼─────────────────────────────────────┤
 *  │ LCD I2C SDA             │ 21       │ I2C Data                            │
 *  │ LCD I2C SCL             │ 22       │ I2C Clock                           │
 *  │ Buzzer                  │ 32       │ PWM Ch9 (2000 Hz)                   │
 *  │ Push Button             │  5       │ INPUT_PULLUP                        │
 *  │ Servo 1 (Infeksius)     │ 33       │ Manual / Auto via Web (Timer 0)     │
 *  │ Servo 2 (Non-Infeksius) │ 19       │ Manual / Auto via Web (Timer 1)     │
 *  │ Servo 3 (B3)            │ 18       │ Manual / Auto via Web (Timer 2)     │
 *  │ MQ-2 AOUT               │ 35       │ Analog (ADC1_CH7, aman saat WiFi)   │
 *  │ MQ-2 DOUT               │ 27       │ Digital In (Active LOW)             │
 *  │ Water Level AOUT        │ 36       │ Analog (ADC1_CH0, pin VP)           │
 *  │ RGB LED - Red           │ 15       │ PWM Ch6                             │
 *  │ RGB LED - Green         │  2       │ PWM Ch7                             │
 *  │ RGB LED - Blue          │ 23       │ PWM Ch8                             │
 *  ├─────────────────────────┼──────────┼─────────────────────────────────────┤
 *  │ BTS7960 RPWM            │  4       │ PWM Ch4 (Maju / Forward PWM)        │
 *  │ BTS7960 LPWM            │ 17       │ PWM Ch5 (Mundur / Reverse PWM)      │
 *  │ BTS7960 R_EN + L_EN     │ 16       │ Digital Out (HIGH=Aktif, LOW=Standby)│
 *  │ BTS7960 VCC             │ 5V / VIN │ Logika Kontrol BTS7960              │
 *  │ BTS7960 GND             │ GND      │ Common Ground ESP32 + Power Supply  │
 *  │ BTS7960 B+ / B-         │ Ext Pwr  │ Sumber Daya Motor DC (12V / 24V)    │
 *  │ BTS7960 M+ / M-         │ Motor DC │ Terminal Motor DC                   │
 *  ├─────────────────────────┼──────────┼─────────────────────────────────────┤
 *  │ UART2 RX (← ESP32-2 TX) │ 13       │ Serial2 RX — terima data sensor     │
 *  │ UART2 TX (tidak dipakai)│ 25       │ Serial2 TX — reserved               │
 *  └─────────────────────────┴──────────┴─────────────────────────────────────┘
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
#define WIFI_SSID   "Absolute Solver2"
#define WIFI_PASS   "Roboticzzmc18"

// ── Pin Definitions ──────────────────────────────────────────
#define PIN_BUZZER      32
#define PIN_BUTTON       5
#define PIN_SERVO1      33
#define PIN_SERVO2      19
#define PIN_SERVO3      18
#define PIN_GAS_AOUT    35 // ADC1_CH7 - Aman saat WiFi aktif
#define PIN_GAS_DOUT    27 // Digital In (Active LOW)
#define PIN_WATER_AOUT  36 // ADC1_CH0 (pin VP)

// ── RGB LED ───────────────────────────────────────────────────
#define PIN_RGB_R       15
#define PIN_RGB_G        2
#define PIN_RGB_B       23
#define PWM_FREQ      5000
#define PWM_RES          8

// ── Motor DC BTS7960 (IBT-2 43A) ────────────────────────────
#define PIN_MOTOR_RPWM    4   // RPWM (Maju / Forward PWM)
#define PIN_MOTOR_LPWM   17   // LPWM (Mundur / Reverse PWM)
#define PIN_MOTOR_EN     16   // R_EN + L_EN dihubungkan bersama (HIGH=Aktif, LOW=Standby)
#define PWM_MOTOR_FREQ 5000   // 5kHz: frekuensi optimal & senyap untuk MOSFET BTS7960
#define PWM_MOTOR_RES     8   // 8-bit resolution (0 - 255)
#define MOTOR_SPEED_KICK 200  // ~78% kickstart (BTS7960 bertenaga & efisien)
#define MOTOR_SPEED_RUN  180  // ~70% kecepatan normal
#define MOTOR_KICK_MS    500  // 500ms durasi kickstart

// ── MQ-2 Gas & Smoke Sensor Settings ─────────────────────────
#define MQ2_WARMUP_MS         20000 // 20 detik pemanasan
#define MQ2_READ_INTERVAL       250 // Baca 4x per detik
#define MQ2_DELTA_ON           1200 // Kenaikan ADC di atas baseline untuk alarm asap pekat (kebal gas korek)
#define MQ2_DELTA_OFF           600 // Penurunan ADC untuk kembali normal
#define MQ2_ABS_MAX_ON         2400 // Batas atas mutlak asap kebakaran tebal
#define MQ2_ABS_MIN_OFF        1600 // Batas bawah mutlak

// ── Water Level ──────────────────────────────────────────────
#define WATER_THRESHOLD_ON    150
#define WATER_THRESHOLD_OFF   100
#define WATER_READ_INTERVAL   500

// ── UART2 (Terima data dari ESP32 ke-2) ──────────────────────
#define PIN_UART2_RX         13   // RX2 — dari TX GPIO17 ESP32 ke-2
#define PIN_UART2_TX         25   // TX2 — tidak digunakan (satu arah)
#define UART2_BAUD         9600
#define UART2_SORT_TIMEOUT 5000   // ms — max waktu tunggu konfirmasi US2 setelah servo tutup

// ── Servo ─────────────────────────────────────────────────────
#define SERVO_MIN_ANGLE    0
#define SERVO_MAX_ANGLE  180
#define SERVO_MIN_US     544  // Standar mikro-servo (544µs mencegah motor menabrak batas mekanik & shaking)
#define SERVO_MAX_US    2400  // Maksimum pulse width mikro-servo

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
﻿<!DOCTYPE html>
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

  /* SORT POPUP */
  .sort-popup {
    position: fixed; top: -80px; left: 50%; transform: translateX(-50%);
    background: linear-gradient(135deg, #1a3d2b, #1c2128);
    border: 1px solid rgba(63,185,80,0.4); border-radius: 10px;
    padding: 12px 24px; font-size: 14px; font-weight: 600;
    color: var(--ok); box-shadow: 0 8px 24px rgba(63,185,80,0.2);
    z-index: 9998; transition: top 0.5s cubic-bezier(0.34,1.56,0.64,1); white-space: nowrap;
  }
  .sort-popup.show { top: 20px; }

  /* SIDEBAR */
  .sidebar-overlay { display: none; position: fixed; inset: 0; background: rgba(0,0,0,0.5); z-index: 999; }
  .sidebar-overlay.show { display: block; }
  .sidebar {
    position: fixed; top: 0; right: -520px; bottom: 0; width: 100%; max-width: 520px;
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
  .stats-chart { flex: 1; min-width: 180px; display: flex; align-items: center; justify-content: center; flex-direction: column; }
  .stats-list { flex: 1; display: flex; flex-direction: column; gap: 14px; }
  .category-group { background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; overflow: hidden; }
  .category-head { background: rgba(0,217,255,0.05); border-bottom: 1px solid var(--border); padding: 8px 12px; font-weight: 600; color: var(--accent); font-size: 13px; text-transform: uppercase; letter-spacing: 0.05em; }
  .category-item { display: flex; justify-content: space-between; padding: 8px 12px; border-bottom: 1px solid var(--border); font-size: 13px; }
  .category-item:last-child { border-bottom: none; }
  .category-total { display: flex; justify-content: space-between; padding: 8px 12px; background: rgba(0,0,0,0.2); font-weight: 700; color: #fff; font-size: 13px; }
  .svg-pie { width: 100%; max-width: 200px; height: auto; transform: rotate(-90deg); border-radius: 50%; }
  .legend-list { display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; margin-top: 14px; }
  .legend-item { display: flex; align-items: center; gap: 6px; font-size: 11px; color: var(--muted); }
  .legend-color { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .sensor-chart { margin-top: 24px; padding-top: 24px; border-top: 1px solid var(--border); }
  .sensor-title { font-size: 14px; font-weight: 600; color: var(--text); margin-bottom: 14px; text-transform: uppercase; letter-spacing: 0.05em; display:flex; justify-content:space-between; align-items:center; }
  .sensor-layout { display: flex; gap: 20px; align-items: center; }
  .svg-line { width: 100%; max-width: 250px; height: auto; background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; padding: 10px; box-sizing: border-box; overflow:visible; }
  .sensor-list { flex: 1; display: flex; flex-direction: column; gap: 8px; }
  .sensor-item { display: flex; justify-content: space-between; padding: 10px 14px; background: rgba(0,0,0,0.2); border: 1px solid var(--border); border-radius: 8px; font-size: 13px; align-items:center; }
  .sensor-dot { width: 10px; height: 10px; border-radius: 50%; display:inline-block; margin-right: 8px; flex-shrink: 0; }
  @media(max-width: 500px){ .stats-layout, .sensor-layout { flex-direction: column; } }

  /* HAMBURGER */
  .hamburger-btn {
    background: transparent; border: none; color: var(--text); font-size: 20px;
    cursor: pointer; padding: 5px; margin-left: 10px; transition: color 0.2s;
  }
  .hamburger-btn:hover { color: var(--accent); }

  /* ═══════════════════════════════════════
     HARDWARE DIAGNOSTIC & SCANNER MATRIX
  ═══════════════════════════════════════ */
  .hw-scan-header {
    display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px; margin-bottom: 14px;
  }
  .scan-btn {
    background: linear-gradient(135deg, #00d9ff 0%, #0077ff 100%);
    color: #fff; border: none; padding: 11px 22px; border-radius: 10px;
    font-size: 13px; font-weight: 700; cursor: pointer; display: inline-flex;
    align-items: center; gap: 10px; box-shadow: 0 4px 20px rgba(0,217,255,0.35);
    transition: all 0.25s ease; letter-spacing: 0.02em;
  }
  .scan-btn:hover {
    transform: translateY(-2px); box-shadow: 0 6px 26px rgba(0,217,255,0.5); filter: brightness(1.1);
  }
  .scan-btn:active { transform: translateY(0); }
  .scan-btn.scanning {
    background: linear-gradient(135deg, #bc8cff 0%, #7928ca 100%);
    box-shadow: 0 4px 20px rgba(188,140,255,0.4);
    animation: radar-pulse 1.6s infinite;
    pointer-events: none;
  }
  @keyframes radar-pulse {
    0% { box-shadow: 0 0 0 0 rgba(0,217,255,0.7); }
    70% { box-shadow: 0 0 0 16px rgba(0,217,255,0); }
    100% { box-shadow: 0 0 0 0 rgba(0,217,255,0); }
  }
  .scan-radar-icon { display: inline-block; font-size: 16px; transition: transform 0.5s; }
  .scan-btn.scanning .scan-radar-icon { animation: spin 1s linear infinite; }
  @keyframes spin { 100% { transform: rotate(360deg); } }

  .scan-progress-box {
    display: none; background: rgba(0,217,255,0.06); border: 1px solid rgba(0,217,255,0.25);
    border-radius: 10px; padding: 12px 16px; margin-bottom: 16px; animation: fadeIn 0.3s ease;
  }
  .scan-progress-bar {
    width: 100%; height: 6px; background: var(--surface2); border-radius: 3px; overflow: hidden; margin: 8px 0 6px;
  }
  .scan-progress-fill {
    height: 100%; width: 0%; background: linear-gradient(90deg, #00d9ff, #bc8cff);
    transition: width 0.3s ease; border-radius: 3px;
  }
  .scan-status-text { font-size: 12px; color: var(--accent); font-family: monospace; }

  .hw-summary-banner {
    display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 10px;
    background: var(--surface2); border: 1px solid var(--border); border-radius: 10px; padding: 10px 16px; margin-bottom: 16px;
  }
  .hw-summary-item { font-size: 12px; color: var(--muted); display: flex; align-items: center; gap: 6px; }
  .hw-summary-item strong { color: var(--text); font-family: monospace; font-size: 13px; }

  .hw-grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 16px; margin-bottom: 24px;
  }
  .hw-card {
    background: var(--surface); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; position: relative; overflow: hidden; transition: border-color 0.2s, transform 0.2s;
  }
  .hw-card:hover { border-color: rgba(0,217,255,0.3); }
  .hw-card-title {
    font-size: 13px; font-weight: 700; color: var(--text); display: flex; align-items: center;
    justify-content: space-between; margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid var(--border);
  }
  .hw-card-title .icon-title { display: flex; align-items: center; gap: 8px; }
  .hw-device-list { display: flex; flex-direction: column; gap: 10px; }
  .hw-item {
    background: var(--surface2); border: 1px solid var(--border); border-radius: 8px;
    padding: 10px 12px; display: flex; justify-content: space-between; align-items: center; gap: 10px;
    transition: all 0.2s;
  }
  .hw-item:hover { background: rgba(255,255,255,0.03); }
  .hw-item-left { display: flex; flex-direction: column; gap: 2px; }
  .hw-item-name { font-size: 13px; font-weight: 600; color: var(--text); display: flex; align-items: center; gap: 6px; }
  .hw-item-meta { font-size: 11px; color: var(--muted); font-family: monospace; }
  .hw-item-right { display: flex; flex-direction: column; align-items: flex-end; gap: 4px; }
  .hw-badge {
    display: inline-flex; align-items: center; gap: 5px; font-size: 11px; font-weight: 700;
    padding: 3px 8px; border-radius: 20px; text-transform: uppercase; letter-spacing: 0.04em;
  }
  .hw-badge.ok { background: rgba(63,185,80,0.15); color: var(--ok); border: 1px solid rgba(63,185,80,0.35); }
  .hw-badge.warn { background: rgba(227,179,65,0.15); color: var(--warn); border: 1px solid rgba(227,179,65,0.35); }
  .hw-badge.fail { background: rgba(248,81,73,0.15); color: var(--danger); border: 1px solid rgba(248,81,73,0.35); }
  .hw-badge.pulse { animation: pulse-border 1.5s infinite; }
  .hw-val { font-size: 12px; font-family: monospace; color: var(--accent); font-weight: 600; }

  /* UART LINK METERS */
  .uart-link-box {
    background: linear-gradient(135deg, rgba(0,217,255,0.04) 0%, rgba(188,140,255,0.04) 100%);
    border: 1px solid rgba(0,217,255,0.2); border-radius: 10px; padding: 12px; margin-top: 10px;
  }
  .uart-flow {
    display: flex; align-items: center; justify-content: space-between; gap: 8px; margin-bottom: 8px;
  }
  .uart-node {
    background: var(--bg); border: 1px solid var(--border); border-radius: 6px; padding: 6px 10px;
    font-size: 11px; font-weight: 700; text-align: center; min-width: 90px;
  }
  .uart-line {
    flex: 1; height: 2px; background: linear-gradient(90deg, #00d9ff, #bc8cff); position: relative;
    overflow: hidden;
  }
  .uart-line::after {
    content: ''; position: absolute; top: -2px; left: 0; width: 20px; height: 6px;
    background: #fff; border-radius: 3px; box-shadow: 0 0 8px #00d9ff;
    animation: uart-flow-anim 1.5s infinite linear;
  }
  @keyframes uart-flow-anim { 0% { left: -20px; } 100% { left: 100%; } }
  .uart-line.offline::after { display: none; }
  .uart-line.offline { background: var(--border); }
  .troubleshoot-box {
    font-size: 11px; color: var(--muted); background: rgba(0,0,0,0.25); border-radius: 6px;
    padding: 8px 10px; margin-top: 8px; line-height: 1.5; border-left: 3px solid var(--accent);
  }

  /* ═══════════════════════════════════════
     ULTRASONIC LIVE RADAR & OBSTACLE MONITOR
  ═══════════════════════════════════════ */
  .radar-btn {
    display: inline-flex; align-items: center; gap: 8px; background: linear-gradient(135deg, rgba(0,217,255,0.15), rgba(188,140,255,0.2));
    border: 1px solid rgba(0,217,255,0.5); color: #fff; padding: 8px 14px; border-radius: 8px;
    font-size: 12px; font-weight: 700; cursor: pointer; transition: all 0.25s; box-shadow: 0 0 15px rgba(0,217,255,0.2);
  }
  .radar-btn:hover {
    background: linear-gradient(135deg, rgba(0,217,255,0.3), rgba(188,140,255,0.35));
    border-color: var(--accent); box-shadow: 0 0 25px rgba(0,217,255,0.4); transform: translateY(-1px);
  }
  .radar-modal-overlay {
    position: fixed; inset: 0; background: rgba(0,0,0,0.85); backdrop-filter: blur(8px);
    z-index: 2000; display: none; align-items: center; justify-content: center; padding: 20px;
  }
  .radar-modal-overlay.show { display: flex; animation: fadeIn 0.3s ease; }
  .radar-modal-card {
    background: var(--surface); border: 1px solid rgba(0,217,255,0.4); border-radius: 16px;
    width: 100%; max-width: 860px; max-height: 92vh; overflow-y: auto;
    box-shadow: 0 0 50px rgba(0,217,255,0.25); padding: 24px; position: relative;
  }
  .radar-modal-header {
    display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px;
    border-bottom: 1px solid var(--border); padding-bottom: 12px;
  }
  .radar-modal-title {
    font-size: 16px; font-weight: 700; color: var(--text); display: flex; align-items: center; gap: 8px;
  }
  .radar-modal-title span { color: var(--accent); }
  .radar-close-btn {
    background: var(--surface2); border: 1px solid var(--border); color: var(--text);
    width: 32px; height: 32px; border-radius: 50%; cursor: pointer; font-size: 16px;
    display: flex; align-items: center; justify-content: center; transition: all 0.2s;
  }
  .radar-close-btn:hover { background: var(--danger); border-color: var(--danger); color: #fff; }

  .radar-screen-container {
    display: flex; flex-direction: column; align-items: center; justify-content: center;
    background: radial-gradient(circle at center, rgba(0,217,255,0.06) 0%, rgba(13,17,23,0.98) 75%);
    border: 1px solid rgba(0,217,255,0.3); border-radius: 14px; padding: 20px; position: relative;
    overflow: hidden; margin-bottom: 20px;
  }
  .radar-canvas-wrap {
    position: relative; width: 320px; height: 320px; border-radius: 50%;
    border: 2px solid rgba(0,217,255,0.5); box-shadow: 0 0 30px rgba(0,217,255,0.15), inset 0 0 30px rgba(0,217,255,0.08);
    background: #090d12;
  }
  .radar-grid-svg {
    position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none;
  }
  .radar-sweep-beam {
    position: absolute; top: 0; left: 0; width: 100%; height: 100%; border-radius: 50%;
    pointer-events: none;
    background: conic-gradient(from 0deg, rgba(0,217,255,0.45) 0deg, rgba(0,217,255,0.05) 45deg, transparent 65deg);
    animation: radar-sweep-anim 3s linear infinite;
  }
  @keyframes radar-sweep-anim { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }

  /* Red Obstacle Blips (Titik Merah) */
  .obstacle-blip {
    position: absolute; width: 16px; height: 16px; border-radius: 50%;
    background: #ff3344; box-shadow: 0 0 14px #ff3344, 0 0 24px #ff3344;
    transform: translate(-50%, -50%); transition: all 0.25s ease; z-index: 10;
    display: flex; align-items: center; justify-content: center;
  }
  .obstacle-blip::after {
    content: ''; position: absolute; inset: -6px; border-radius: 50%;
    border: 2px solid #ff3344; animation: ping 1.2s cubic-bezier(0,0,0.2,1) infinite;
  }
  .obstacle-clear-blip {
    position: absolute; width: 12px; height: 12px; border-radius: 50%;
    background: var(--ok); box-shadow: 0 0 10px var(--ok);
    transform: translate(-50%, -50%); transition: all 0.25s ease; z-index: 10; opacity: 0.75;
  }
  .blip-label {
    position: absolute; top: -18px; white-space: nowrap; font-size: 10px; font-weight: 700;
    color: #fff; background: rgba(0,0,0,0.75); padding: 1px 5px; border-radius: 4px;
    border: 1px solid rgba(255,255,255,0.2); pointer-events: none;
  }

  /* 4 Radar Sensor Stations */
  .radar-stations-grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; width: 100%;
  }
  .station-card {
    background: var(--surface2); border: 1px solid var(--border); border-radius: 10px; padding: 14px;
    transition: all 0.2s; position: relative; overflow: hidden;
  }
  .station-card.obstacle-active {
    border-color: var(--danger); background: linear-gradient(180deg, rgba(248,81,73,0.12) 0%, var(--surface2) 100%);
    box-shadow: 0 0 15px rgba(248,81,73,0.25);
  }
  .station-header {
    display: flex; align-items: center; justify-content: space-between; margin-bottom: 6px;
  }
  .station-name { font-size: 12px; font-weight: 700; color: var(--text); display: flex; align-items: center; gap: 5px; }
  .station-dist {
    font-size: 22px; font-weight: 800; font-family: monospace; color: var(--accent); margin: 4px 0;
  }
  .station-card.obstacle-active .station-dist { color: var(--danger); }
  .station-bar-track {
    width: 100%; height: 6px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden; margin-top: 6px;
  }
  .station-bar-fill {
    height: 100%; width: 0%; border-radius: 3px; transition: width 0.25s ease, background 0.25s ease;
  }
  .station-status-pill {
    display: inline-flex; align-items: center; gap: 4px; font-size: 10px; font-weight: 700;
    padding: 2px 6px; border-radius: 12px; text-transform: uppercase; margin-top: 4px;
  }
  .station-status-pill.clear { background: rgba(63,185,80,0.15); color: var(--ok); }
  .station-status-pill.danger { background: rgba(248,81,73,0.2); color: var(--danger); }

  /* Hardware Test Mode & Buttons */
  .test-hw-btn {
    display: flex;
    align-items: center;
    gap: 12px;
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px 14px;
    cursor: pointer;
    text-align: left;
    transition: all 0.2s ease;
    position: relative;
    overflow: hidden;
  }
  .test-hw-btn:hover {
    border-color: var(--btn-color, var(--accent));
    background: rgba(255, 255, 255, 0.04);
    transform: translateY(-2px);
    box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
  }
  .test-hw-btn:active {
    transform: translateY(0);
  }
  .test-hw-icon {
    font-size: 22px;
    line-height: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    width: 38px;
    height: 38px;
    border-radius: 8px;
    background: rgba(255, 255, 255, 0.05);
    border: 1px solid rgba(255, 255, 255, 0.1);
    flex-shrink: 0;
  }
  .test-hw-content {
    flex: 1;
    min-width: 0;
  }
  .test-hw-title {
    font-size: 13px;
    font-weight: 700;
    color: var(--text);
    margin-bottom: 2px;
  }
  .test-hw-desc {
    font-size: 10px;
    color: var(--muted);
    line-height: 1.3;
  }
  .yolo-test-item {
    width: 100%;
    background: var(--surface2);
    border: 1px solid var(--border);
    border-left-width: 4px;
    border-radius: 8px;
    padding: 12px 14px;
    cursor: pointer;
    text-align: left;
    transition: all 0.2s;
  }
  .yolo-test-item:hover {
    background: rgba(255, 255, 255, 0.06);
    transform: translateX(3px);
  }
  .yolo-badge {
    font-size: 10px;
    font-weight: 700;
    padding: 2px 8px;
    border-radius: 12px;
  }
</style>
</head>
<body>

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

    <!-- Target ESP32 IP indicator -->
    <div style="margin-top:20px;padding-top:14px;border-top:1px solid var(--border);text-align:center;font-size:12px;color:var(--muted);">
      Target ESP32: <span id="loginTargetIp" style="color:var(--accent);font-weight:600;font-family:monospace;cursor:pointer;" onclick="promptChangeIp()" title="Klik untuk mengubah IP ESP32">192.168.4.2</span>
      <span style="cursor:pointer;color:var(--accent);margin-left:4px;font-size:11px;" onclick="promptChangeIp()">[Ubah IP]</span>
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
      <span class="conn-label" id="connLabel" style="margin-right:2px;">Offline</span>
      <button style="background:var(--surface2);border:1px solid var(--border);color:var(--accent);padding:3px 8px;border-radius:6px;cursor:pointer;font-size:11px;font-weight:600;" onclick="promptChangeIp()" title="Klik untuk mengubah IP ESP32">⚙ IP</button>
      <button style="background:var(--surface2);border:1px solid var(--border);color:var(--text);padding:4px 10px;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600;" onclick="connectWS()">Reconnect</button>
    </div>
    <button class="logout-btn" onclick="doLogout()">← Keluar</button>
    <button class="hamburger-btn" onclick="openSidebar()" title="Total Limbah">☰</button>
  </div>
</header>

<!-- SORT POPUP -->
<div id="sortPopup" class="sort-popup">Berhasil menyortir limbah "..."</div>

<!-- SIDEBAR OVERLAY -->
<div id="sidebarOverlay" class="sidebar-overlay" onclick="closeSidebar()"></div>
<div id="sidebar" class="sidebar">
  <div class="sidebar-header">
    <div class="sidebar-title">📊 Total Limbah Tersortir</div>
    <button class="close-btn" onclick="closeSidebar()">×</button>
  </div>
  <div class="sidebar-content">
    <div class="stats-layout">
      <div class="stats-chart">
        <div id="svgContainer" style="width:100%;display:flex;justify-content:center;"></div>
        <div id="chartLegend" class="legend-list"></div>
      </div>
      <div class="stats-list" id="wasteList"></div>
    </div>
    <button onclick="resetStats()" style="width:100%;margin-top:20px;background:rgba(248,81,73,0.1);border:1px solid rgba(248,81,73,0.3);color:var(--danger);padding:10px;border-radius:8px;cursor:pointer;font-weight:600;transition:background 0.2s;" onmouseover="this.style.background='rgba(248,81,73,0.2)'" onmouseout="this.style.background='rgba(248,81,73,0.1)'">Reset Data Limbah</button>
    <!-- SENSOR ANALYTICS -->
    <div class="sensor-chart">
      <div class="sensor-title">
        📉 Analisis Sensor Bahaya
        <button onclick="resetSensorStats()" style="background:rgba(227,179,65,0.1);border:1px solid rgba(227,179,65,0.3);color:var(--warn);padding:4px 8px;border-radius:6px;cursor:pointer;font-size:11px;font-weight:600;">Reset</button>
      </div>
      <div class="sensor-layout">
        <div id="sensorSvgContainer" style="flex:1;display:flex;justify-content:center;"></div>
        <div id="sensorList" class="sensor-list"></div>
      </div>
    </div>
  </div>
</div>

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
      <div class="live-sensor-chip" id="chipFlame">🔥 Api: —</div>
      <div class="live-sensor-chip" id="chipWater">💧 Air: —</div>
      <div class="live-sensor-chip" id="chipMotor">⚙ Motor: —</div>
    </div>
  </div>

  <!-- ═══ ADMIN: FULL PANEL ═══ -->
  <div class="admin-only" id="adminView">

    <!-- 🛠 HARDWARE DIAGNOSTIC & SCANNER MATRIX -->
    <div class="section-label" style="display:flex; justify-content:space-between; align-items:center;">
      <span>🛠 Diagnostik &amp; Pemindai Hardware</span>
      <span style="font-size:11px; color:var(--muted); font-weight:400; text-transform:none;">Deteksi Sensor, Aktuator &amp; Link UART ESP32 ke-2</span>
    </div>
    
    <div class="card" style="margin-bottom: 20px; border-color: rgba(0,217,255,0.25); background: linear-gradient(180deg, rgba(0,217,255,0.03) 0%, var(--surface) 100%);">
      <div class="hw-scan-header">
        <div>
          <div style="font-size: 15px; font-weight: 700; color: var(--text); margin-bottom: 4px;">Pemeriksaan Keberadaan Perangkat Keras</div>
          <div style="font-size: 12px; color: var(--muted);">Tekan tombol pindai untuk mengecek kelayakan sensor, driver motor, servo, LCD I2C, dan jalur TX/RX ESP32 ke-2.</div>
        </div>
        <div style="display:flex; gap:10px; flex-wrap:wrap; align-items:center;">
          <button class="radar-btn" onclick="openRadarModal()" title="Buka visualisasi radar dan deteksi halangan objek ultrasonik">
            <span>📡</span>
            <span>RADAR MONITOR ULTRASONIK LIVE</span>
          </button>
          <button id="hwScanBtn" class="scan-btn" onclick="triggerHardwareScan()">
            <span class="scan-radar-icon" id="scanRadarIcon">🔍</span>
            <span id="scanBtnLabel">PINDAI KEBERADAAN HARDWARE</span>
          </button>
        </div>
      </div>

      <!-- Scan Progress Animated Box -->
      <div id="scanProgressBox" class="scan-progress-box">
        <div style="display:flex; justify-content:space-between; align-items:center;">
          <span style="font-weight:600; font-size:12px; color:var(--text);">Status Pemindaian:</span>
          <span id="scanStepText" class="scan-status-text">Memeriksa bus &amp; sinyal UART...</span>
        </div>
        <div class="scan-progress-bar">
          <div id="scanProgressFill" class="scan-progress-fill"></div>
        </div>
      </div>

      <!-- Hardware Summary Banner -->
      <div class="hw-summary-banner">
        <div class="hw-summary-item">
          <span>📊 Total Terdeteksi:</span>
          <strong id="hwSummaryCount" style="color:var(--ok);">-- / 12 Perangkat</strong>
        </div>
        <div class="hw-summary-item">
          <span>📡 ESP32 ke-2:</span>
          <strong id="hwSummaryEsp2" style="color:var(--warn);">Memeriksa...</strong>
        </div>
        <div class="hw-summary-item">
          <span>⏱ Waktu Pindai:</span>
          <strong id="hwSummaryTime">Belum dipindai</strong>
        </div>
      </div>

      <!-- Hardware Grid -->
      <div class="hw-grid">
        
        <!-- CARD 1: ESP32-2 & UART LINK -->
        <div class="hw-card">
          <div class="hw-card-title">
            <div class="icon-title"><span>📡</span><span>ESP32 ke-2 (UART TX/RX)</span></div>
            <span id="badgeEsp2Main" class="hw-badge warn"><span class="pill-dot"></span>Checking</span>
          </div>

          <!-- UART Flow Diagram -->
          <div class="uart-link-box">
            <div class="uart-flow">
              <div class="uart-node" style="border-color:var(--accent);">ESP32-1<br><span style="color:var(--muted);font-size:10px;">Master</span></div>
              <div id="uartFlowLine" class="uart-line"></div>
              <div class="uart-node" style="border-color:var(--purple);">ESP32-2<br><span style="color:var(--muted);font-size:10px;">Sensor Node</span></div>
            </div>
            <div style="display:flex; justify-content:space-between; font-size:10px; color:var(--muted); font-family:monospace;">
              <span>RX: GPIO 13 ← TX 17</span>
              <span>TX: GPIO 25 → RX 16</span>
            </div>
          </div>

          <div class="hw-device-list" style="margin-top:12px;">
            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🔄</span><span>Jalur RX (Data Masuk)</span></div>
                <div class="hw-item-meta">GPIO 13 ← ESP32-2 TX17 (9600 Baud)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeUartRx" class="hw-badge ok">AKTIF</span>
                <span id="valUartRx" class="hw-val">0 pkt (0s lalu)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🔁</span><span>Jalur TX (Ping / Probe)</span></div>
                <div class="hw-item-meta">GPIO 25 → ESP32-2 RX16</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeUartTx" class="hw-badge ok">SIAP</span>
                <span id="valUartTx" class="hw-val">Bidirectional OK</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🔥</span><span>Flame Sensor (Api)</span></div>
                <div class="hw-item-meta">Node ESP32-2 Pin DOUT GPIO 13</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwFlame" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwFlame" class="hw-val">Aman (Normal)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>📏</span><span>US1 Anomali</span></div>
                <div class="hw-item-meta">TRIG 25 · ECHO 26 (ESP32-2)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwUs1" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwUs1" class="hw-val">-- cm</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>📐</span><span>US2 Sortir (3 Pintu)</span></div>
                <div class="hw-item-meta">Pintu A(32), B(33), C(35)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwUs2" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwUs2" class="hw-val">A:-- B:-- C:-- cm</span>
              </div>
            </div>
          </div>

          <div class="troubleshoot-box">
            <strong>★ Wiring UART:</strong> Pastikan kabel <strong>TX17 (ESP32-2) ➔ RX13 (ESP32-1)</strong>, <strong>TX25 (ESP32-1) ➔ RX16 (ESP32-2)</strong>, dan kabel <strong>GND</strong> kedua ESP32 terhubung bersama (*Common Ground*).
          </div>
        </div>

        <!-- CARD 2: MASTER SENSORS -->
        <div class="hw-card">
          <div class="hw-card-title">
            <div class="icon-title"><span>🌡</span><span>Sensor ESP32 Utama (Master)</span></div>
            <span id="badgeMasterSensors" class="hw-badge ok"><span class="pill-dot"></span>3 Terdeteksi</span>
          </div>

          <div class="hw-device-list">
            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🟣</span><span>Sensor MQ-2 (Gas/Asap)</span></div>
                <div class="hw-item-meta">Analog: GPIO 35 (ADC1) · Digital: GPIO 27</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwMq2" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwMq2" class="hw-val">ADC: --</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>💧</span><span>Sensor Water Level</span></div>
                <div class="hw-item-meta">Analog: GPIO 36 (ADC1 / Pin VP)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwWater" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwWater" class="hw-val">ADC: --</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🔘</span><span>Push Button Manual</span></div>
                <div class="hw-item-meta">Digital In: GPIO 5 (INPUT_PULLUP)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwBtn" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwBtn" class="hw-val">HIGH (Standby)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>📺</span><span>LCD Display 16x2 (I2C)</span></div>
                <div class="hw-item-meta">I2C SDA: GPIO 21 · SCL: GPIO 22</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwLcd" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwLcd" class="hw-val">ACK 0x27 OK</span>
              </div>
            </div>
          </div>

          <div class="troubleshoot-box" style="margin-top:16px;">
            <strong>★ Info Sensor Master:</strong> Sensor MQ-2 dan Water Level menggunakan ADC1 internal ESP32 yang 100% stabil saat WiFi/Hotspot aktif.
          </div>
        </div>

        <!-- CARD 3: ACTUATORS & OUTPUT -->
        <div class="hw-card">
          <div class="hw-card-title">
            <div class="icon-title"><span>⚡</span><span>Aktuator &amp; Output Sinyal</span></div>
            <span id="badgeMasterActuators" class="hw-badge ok"><span class="pill-dot"></span>5 Terdeteksi</span>
          </div>

          <div class="hw-device-list">
            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>⚙</span><span>Driver BTS7960 (Konveyor)</span></div>
                <div class="hw-item-meta">RPWM:4 · LPWM:17 · EN:16 (5 kHz PWM)</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwMotor" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwMotor" class="hw-val">Ch4 &amp; Ch5 OK</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🟡</span><span>Servo 1 (Infeksius)</span></div>
                <div class="hw-item-meta">Pin GPIO 33 · Pulse 544–2400µs</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwSrv1" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwSrv1" class="hw-val">Attached (0°)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>⚫</span><span>Servo 2 (Non-Infeksius)</span></div>
                <div class="hw-item-meta">Pin GPIO 19 · Pulse 544–2400µs</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwSrv2" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwSrv2" class="hw-val">Attached (0°)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>🔴</span><span>Servo 3 (Limbah B3)</span></div>
                <div class="hw-item-meta">Pin GPIO 18 · Pulse 544–2400µs</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwSrv3" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwSrv3" class="hw-val">Attached (0°)</span>
              </div>
            </div>

            <div class="hw-item">
              <div class="hw-item-left">
                <div class="hw-item-name"><span>📢</span><span>Passive Buzzer &amp; RGB LED</span></div>
                <div class="hw-item-meta">Buzzer: GPIO 32 · RGB: 15, 2, 23</div>
              </div>
              <div class="hw-item-right">
                <span id="badgeHwAlarm" class="hw-badge ok">TERDETEKSI</span>
                <span id="valHwAlarm" class="hw-val">Ch6-9 PWM OK</span>
              </div>
            </div>
          </div>
        </div>

      </div><!-- /hw-grid -->
    </div><!-- /card scanner -->

    <!-- MODE OPERASI & KEAMANAN -->
    <div class="section-label">Mode Operasi &amp; Keamanan</div>
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
    <div class="grid-3">
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
        <div class="card-label">Flame Sensor (API) — ESP32-2</div>
        <div class="card-value" id="flameVal" style="color:#ff7043;">—</div>
        <div class="card-sub">DOUT GPIO 13 · Active LOW · Hold 1.5s</div>
        <div id="flamePill" class="pill offline"><span class="pill-dot"></span>Offline</div>
        <div class="bar-wrap"><div class="bar" id="flameBar" style="width:0%;background:var(--danger);"></div></div>
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


    <!-- 🧪 HARDWARE DIAGNOSTIC TEST BUTTONS -->
    <div class="section-label" style="display:flex; justify-content:space-between; align-items:center; margin-top:24px;">
      <span>🧪 Mode Diagnostik &amp; Uji Coba Hardware</span>
      <span style="font-size:11px; color:var(--muted); font-weight:400; text-transform:none;">Simulasi &amp; Pengujian Mandiri Komponen</span>
    </div>

    <div class="card" style="margin-bottom:20px; border-color:rgba(188,140,255,0.25); background:linear-gradient(180deg, rgba(188,140,255,0.03) 0%, var(--surface) 100%);">
      <div style="font-size:12px; color:var(--muted); margin-bottom:14px; line-height:1.5;">
        Gunakan tombol di bawah untuk menguji fungsi aktuator, simulasi lonjakan sensor tanpa mengunci sistem permanen, serta simulasi klasifikasi limbah AI YOLO.
      </div>

      <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap:12px;">
        <!-- Test Buzzer -->
        <button class="test-hw-btn" onclick="testHardware('buzzer')" style="--btn-color: #ffd700;">
          <div class="test-hw-icon">🔊</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Buzzer</div>
            <div class="test-hw-desc">Bunyi beep selama 1 detik</div>
          </div>
        </button>

        <!-- Test Flame Sensor -->
        <button class="test-hw-btn" onclick="testHardware('flame')" style="--btn-color: #f85149;">
          <div class="test-hw-icon">🔥</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Sensor Api</div>
            <div class="test-hw-desc">Simulasi api aktif 3 detik</div>
          </div>
        </button>

        <!-- Test Gas Sensor -->
        <button class="test-hw-btn" onclick="testHardware('gas')" style="--btn-color: #e3b341;">
          <div class="test-hw-icon">💨</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Sensor Gas</div>
            <div class="test-hw-desc">Simulasi lonjakan ADC 4 detik (Aman)</div>
          </div>
        </button>

        <!-- Test Water Sensor -->
        <button class="test-hw-btn" onclick="testHardware('water')" style="--btn-color: #00d9ff;">
          <div class="test-hw-icon">💧</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Sensor Air</div>
            <div class="test-hw-desc">Simulasi air naik 4 detik (Aman)</div>
          </div>
        </button>

        <!-- Test Motor (Maju lalu Mundur) -->
        <button class="test-hw-btn" onclick="testHardware('motor')" style="--btn-color: #3fb950;">
          <div class="test-hw-icon">⚙️</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Motor (Maju-Mundur)</div>
            <div class="test-hw-desc">Maju 2s ➔ Jeda 1s ➔ Mundur 2s</div>
          </div>
        </button>

        <!-- Test YOLO Classification -->
        <button class="test-hw-btn" onclick="testHardware('yolo')" style="--btn-color: #bc8cff;">
          <div class="test-hw-icon">🎯</div>
          <div class="test-hw-content">
            <div class="test-hw-title">Test Deteksi YOLO</div>
            <div class="test-hw-desc">Simulasi sortir limbah &amp; servo</div>
          </div>
        </button>
      </div>
    </div>
  </div><!-- /adminView -->

</main>
</div><!-- /appPage -->

<!-- TOAST -->
<div class="toast" id="toast"></div>

<!-- ═══════════════════════════════════════ ULTRASONIC RADAR MODAL ═══════════════════════════════════════ -->

<!-- ═══════════════════════════════════════ YOLO SIMULATION MODAL ═══════════════════════════════════════ -->
<div id="yoloTestModal" class="radar-modal-overlay" onclick="if(event.target===this) closeYoloTestModal()">
  <div class="radar-modal-card" style="max-width:540px;">
    <div class="radar-modal-header">
      <div class="radar-modal-title">
        <span>🎯</span>
        <div>
          <div>Simulasi Klasifikasi <span>AI YOLO &amp; Pemilahan</span></div>
          <div style="font-size:11px; font-weight:400; color:var(--muted);">Uji coba perintah sortir limbah medis &amp; pergerakan pintu servo</div>
        </div>
      </div>
      <button class="radar-close-btn" onclick="closeYoloTestModal()" title="Tutup">✕</button>
    </div>

    <div style="padding:16px 20px;">
      <div style="font-size:12px; color:var(--muted); margin-bottom:14px; line-height:1.5;">
        Pilih jenis limbah di bawah untuk mengirimkan data hasil deteksi YOLO ke sistem. Pintu servo yang sesuai akan terbuka dan konveyor akan bersiap menyortir limbah:
      </div>

      <div style="display:flex; flex-direction:column; gap:10px;">
        <!-- Infeksius -->
        <button class="yolo-test-item" onclick="runYoloSimulation('Suntikan Bekas', 'Limbah Infeksius', 1)" style="border-left-color:#ffd700;">
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:4px;">
            <strong style="color:#ffd700; font-size:13px;">🟡 Pintu 1: Limbah Infeksius</strong>
            <span class="yolo-badge" style="background:rgba(255,215,0,0.15); color:#ffd700;">Servo 1 (90°)</span>
          </div>
          <div style="font-size:11px; color:var(--text); font-weight:500;">Contoh: Suntikan Bekas / Plester Berdarah</div>
          <div style="font-size:10px; color:var(--muted); margin-top:2px;">Membuka pintu kuning (Infeksius) &amp; mencatat statistik jika melewati US2 Pintu 1.</div>
        </button>

        <!-- Non-Infeksius -->
        <button class="yolo-test-item" onclick="runYoloSimulation('Botol Plastik Obat', 'Limbah Non-Infeksius', 2)" style="border-left-color:#bc8cff;">
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:4px;">
            <strong style="color:#bc8cff; font-size:13px;">🟣 Pintu 2: Limbah Non-Infeksius</strong>
            <span class="yolo-badge" style="background:rgba(188,140,255,0.15); color:#bc8cff;">Servo 2 (90°)</span>
          </div>
          <div style="font-size:11px; color:var(--text); font-weight:500;">Contoh: Botol Plastik / Kain Kasa / Tisu Antiseptik</div>
          <div style="font-size:10px; color:var(--muted); margin-top:2px;">Membuka pintu hitam/ungu (Non-Infeksius) &amp; mencatat statistik jika melewati US2 Pintu 2.</div>
        </button>

        <!-- B3 -->
        <button class="yolo-test-item" onclick="runYoloSimulation('Obat Kadaluarsa / Baterai', 'Limbah B3', 3)" style="border-left-color:#ff7043;">
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:4px;">
            <strong style="color:#ff7043; font-size:13px;">🔴 Pintu 3: Limbah Bahan Berbahaya (B3)</strong>
            <span class="yolo-badge" style="background:rgba(255,112,67,0.15); color:#ff7043;">Servo 3 (90°)</span>
          </div>
          <div style="font-size:11px; color:var(--text); font-weight:500;">Contoh: Obat 1 / Obat 2 / Baterai Alat Medis</div>
          <div style="font-size:10px; color:var(--muted); margin-top:2px;">Membuka pintu merah (B3) &amp; mencatat statistik jika melewati US2 Pintu 3.</div>
        </button>
      </div>
    </div>
  </div>
</div>

<div id="radarModal" class="radar-modal-overlay" onclick="if(event.target===this) closeRadarModal()">
  <div class="radar-modal-card">
    <div class="radar-modal-header">
      <div class="radar-modal-title">
        <span>📡</span>
        <div>
          <div>Monitor Jarak Ultrasonik &amp; <span>Radar Halangan Objek</span></div>
          <div style="font-size:11px; font-weight:400; color:var(--muted);">Visualisasi Real-time 4 Titik Sensor HC-SR04 pada ESP32 ke-2</div>
        </div>
      </div>
      <button class="radar-close-btn" onclick="closeRadarModal()" title="Tutup">✕</button>
    </div>

    <!-- RADAR VISUALIZER VIEW -->
    <div class="radar-screen-container">
      <div style="position:absolute; top:12px; left:16px; font-size:11px; font-weight:700; color:var(--accent); font-family:monospace; display:flex; align-items:center; gap:6px;">
        <span class="blink-dot"></span> LIVE SONAR SWEEP (50 cm Max)
      </div>
      <div style="position:absolute; top:12px; right:16px; font-size:11px; color:var(--muted); font-family:monospace;" id="radarObstacleSummary">
        Halangan: 0 Terdeteksi
      </div>

      <div class="radar-canvas-wrap">
        <!-- SVG Grid Lines & Range Rings -->
        <svg class="radar-grid-svg" viewBox="0 0 320 320">
          <!-- Crosshairs -->
          <line x1="160" y1="0" x2="160" y2="320" stroke="rgba(0,217,255,0.2)" stroke-width="1" stroke-dasharray="3,3"/>
          <line x1="0" y1="160" x2="320" y2="160" stroke="rgba(0,217,255,0.2)" stroke-width="1" stroke-dasharray="3,3"/>
          <line x1="47" y1="47" x2="273" y2="273" stroke="rgba(0,217,255,0.12)" stroke-width="1" stroke-dasharray="2,2"/>
          <line x1="47" y1="273" x2="273" y2="47" stroke="rgba(0,217,255,0.12)" stroke-width="1" stroke-dasharray="2,2"/>

          <!-- Range Rings (5cm, 10cm, 15cm, 20cm, 35cm, 50cm) -->
          <circle cx="160" cy="160" r="28" fill="none" stroke="rgba(248,81,73,0.35)" stroke-width="1.5"/>
          <text x="175" y="136" fill="rgba(248,81,73,0.7)" font-size="8" font-family="monospace">5cm</text>

          <circle cx="160" cy="160" r="56" fill="none" stroke="rgba(227,179,65,0.3)" stroke-width="1"/>
          <text x="175" y="108" fill="rgba(227,179,65,0.7)" font-size="8" font-family="monospace">10cm</text>

          <circle cx="160" cy="160" r="84" fill="none" stroke="rgba(0,217,255,0.25)" stroke-width="1"/>
          <text x="175" y="80" fill="rgba(0,217,255,0.7)" font-size="8" font-family="monospace">15cm</text>

          <circle cx="160" cy="160" r="112" fill="none" stroke="rgba(0,217,255,0.2)" stroke-width="1"/>
          <text x="175" y="52" fill="rgba(0,217,255,0.6)" font-size="8" font-family="monospace">20cm</text>

          <circle cx="160" cy="160" r="150" fill="none" stroke="rgba(0,217,255,0.15)" stroke-width="1"/>
          <text x="175" y="24" fill="rgba(0,217,255,0.5)" font-size="8" font-family="monospace">50cm</text>

          <!-- Sector Direction Badges -->
          <text x="160" y="16" fill="#00d9ff" font-size="9" font-weight="bold" font-family="monospace" text-anchor="middle">▲ US1 (ANOMALI)</text>
          <text x="32" y="154" fill="#ffd700" font-size="9" font-weight="bold" font-family="monospace" text-anchor="middle">◀ PINTU 1</text>
          <text x="160" y="312" fill="#bc8cff" font-size="9" font-weight="bold" font-family="monospace" text-anchor="middle">▼ PINTU 2</text>
          <text x="288" y="154" fill="#ff7043" font-size="9" font-weight="bold" font-family="monospace" text-anchor="middle">PINTU 3 ▶</text>
        </svg>

        <!-- Animated Sonar Sweep Line -->
        <div class="radar-sweep-beam"></div>

        <!-- 4 Live Dynamic Obstacle Blip Dots (Titik Merah / Hijau) -->
        <div id="radarBlip1" class="obstacle-clear-blip" style="top:25px; left:160px;">
          <span class="blip-label" id="radarBlipLabel1">US1: Standby</span>
        </div>

        <div id="radarBlipA" class="obstacle-clear-blip" style="top:160px; left:25px;">
          <span class="blip-label" id="radarBlipLabelA">P1: Standby</span>
        </div>

        <div id="radarBlipB" class="obstacle-clear-blip" style="top:295px; left:160px;">
          <span class="blip-label" id="radarBlipLabelB">P2: Standby</span>
        </div>

        <div id="radarBlipC" class="obstacle-clear-blip" style="top:160px; left:295px;">
          <span class="blip-label" id="radarBlipLabelC">P3: Standby</span>
        </div>

        <!-- Center Hub -->
        <div style="position:absolute; top:50%; left:50%; width:10px; height:10px; background:var(--accent); border-radius:50%; transform:translate(-50%, -50%); box-shadow:0 0 10px var(--accent);"></div>
      </div>
    </div>

    <!-- 4 SENSOR STATIONS LIVE TELEMETRY CARDS -->
    <div class="radar-stations-grid">
      <!-- STATION 1: US1 ANOMALI -->
      <div class="station-card" id="stationCard1">
        <div class="station-header">
          <div class="station-name"><span>📏</span><span>US1 Anomali</span></div>
          <span class="station-status-pill clear" id="stationPill1">BEBAS</span>
        </div>
        <div style="font-size:10px; color:var(--muted); font-family:monospace;">TRIG 25 · ECHO 26</div>
        <div class="station-dist" id="stationDist1">-- <span style="font-size:13px;">cm</span></div>
        <div class="station-bar-track">
          <div class="station-bar-fill" id="stationBar1" style="width:0%; background:var(--ok);"></div>
        </div>
        <div style="font-size:10px; color:var(--muted); margin-top:4px; display:flex; justify-content:space-between;">
          <span>Ambang: 1-20 cm</span>
          <span id="stationDetail1">Jalur Bersih</span>
        </div>
      </div>

      <!-- STATION 2: US2 PINTU A (SERVO 1) -->
      <div class="station-card" id="stationCardA">
        <div class="station-header">
          <div class="station-name"><span style="color:#ffd700;">🟡</span><span>Pintu 1 (Infeksius)</span></div>
          <span class="station-status-pill clear" id="stationPillA">BEBAS</span>
        </div>
        <div style="font-size:10px; color:var(--muted); font-family:monospace;">TRIG 14 · ECHO 32</div>
        <div class="station-dist" id="stationDistA">-- <span style="font-size:13px;">cm</span></div>
        <div class="station-bar-track">
          <div class="station-bar-fill" id="stationBarA" style="width:0%; background:var(--ok);"></div>
        </div>
        <div style="font-size:10px; color:var(--muted); margin-top:4px; display:flex; justify-content:space-between;">
          <span>Ambang: 1-15 cm</span>
          <span id="stationDetailA">Jalur Bersih</span>
        </div>
      </div>

      <!-- STATION 3: US2 PINTU B (SERVO 2) -->
      <div class="station-card" id="stationCardB">
        <div class="station-header">
          <div class="station-name"><span style="color:#bc8cff;">🟣</span><span>Pintu 2 (Non-Inf)</span></div>
          <span class="station-status-pill clear" id="stationPillB">BEBAS</span>
        </div>
        <div style="font-size:10px; color:var(--muted); font-family:monospace;">TRIG 14 · ECHO 33</div>
        <div class="station-dist" id="stationDistB">-- <span style="font-size:13px;">cm</span></div>
        <div class="station-bar-track">
          <div class="station-bar-fill" id="stationBarB" style="width:0%; background:var(--ok);"></div>
        </div>
        <div style="font-size:10px; color:var(--muted); margin-top:4px; display:flex; justify-content:space-between;">
          <span>Ambang: 1-15 cm</span>
          <span id="stationDetailB">Jalur Bersih</span>
        </div>
      </div>

      <!-- STATION 4: US2 PINTU C (SERVO 3) -->
      <div class="station-card" id="stationCardC">
        <div class="station-header">
          <div class="station-name"><span style="color:#ff7043;">🔴</span><span>Pintu 3 (B3)</span></div>
          <span class="station-status-pill clear" id="stationPillC">BEBAS</span>
        </div>
        <div style="font-size:10px; color:var(--muted); font-family:monospace;">TRIG 14 · ECHO 35</div>
        <div class="station-dist" id="stationDistC">-- <span style="font-size:13px;">cm</span></div>
        <div class="station-bar-track">
          <div class="station-bar-fill" id="stationBarC" style="width:0%; background:var(--ok);"></div>
        </div>
        <div style="font-size:10px; color:var(--muted); margin-top:4px; display:flex; justify-content:space-between;">
          <span>Ambang: 1-15 cm</span>
          <span id="stationDetailC">Jalur Bersih</span>
        </div>
      </div>
    </div>

    <!-- HARDWARE CHECK & WIRING INFO -->
    <div style="margin-top:16px; background:rgba(0,0,0,0.3); border:1px solid var(--border); border-radius:10px; padding:12px; font-size:11px; color:var(--muted); line-height:1.6;">
      <strong style="color:var(--text);">💡 Panduan &amp; Tips Sensor HC-SR04:</strong><br>
      • Sensor ultrasonik HC-SR04 <strong>WAJIB diberi tegangan VCC 5V (pin VIN)</strong> pada ESP32 ke-2 (tidak bisa bekerja pada 3.3V).<br>
      • Jika ada halangan dalam jarak ambang (&le; 15–20 cm), <strong>titik radar akan berubah menjadi MERAH MENYALA</strong> dan posisinya akan bergerak mendekat ke titik pusat radar.<br>
      • Pastikan kabel <strong>Common Ground (GND)</strong> antara kedua ESP32 terhubung dengan kencang.
    </div>
  </div>
</div>

<script>
// ═══════════════════════════════════════
//  IP CONFIG & AUTO-DETECTION
// ═══════════════════════════════════════
function getAutoEsp32Host() {
  // 1. Jika dibuka langsung dari browser via web server ESP32 (misal: http://192.168.4.2/ atau http://192.168.4.3/)
  if (window.location.hostname && 
      window.location.hostname !== '' && 
      window.location.hostname !== 'localhost' && 
      window.location.hostname !== '127.0.0.1') {
    return window.location.hostname;
  }
  // 2. Jika dibuka dari file lokal (file://) atau localhost, gunakan IP dari localStorage atau default 192.168.4.2
  return localStorage.getItem('esp32_ip') || '192.168.4.2';
}

let esp32Host = getAutoEsp32Host();

function saveIp() {
  const inp = document.getElementById('esp32IpInput');
  const ip = inp ? inp.value.trim() : '';
  const ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
  if (!ipRegex.test(ip)) {
    const errEl = document.getElementById('ipErr');
    if (errEl) errEl.className = 'login-err show';
    return;
  }
  esp32Host = ip;
  localStorage.setItem('esp32_ip', ip);
  updateLoginTargetIp();
  document.getElementById('ipConfigPage').style.display = 'none';
  document.getElementById('loginPage').style.display = 'flex';
}

function promptChangeIp() {
  const newIp = prompt('Masukkan IP Address ESP32 Smart Bin:', esp32Host);
  if (newIp && newIp.trim()) {
    const ip = newIp.trim();
    const ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
    if (ipRegex.test(ip)) {
      esp32Host = ip;
      localStorage.setItem('esp32_ip', ip);
      updateLoginTargetIp();
      showToast('IP ESP32 diubah ke: ' + ip);
      if (ws) {
        try { ws.close(); } catch(e){}
        ws = null;
      }
      connectWS();
    } else {
      alert('Format IP Address tidak valid! Contoh: 192.168.4.2');
    }
  }
}

function updateLoginTargetIp() {
  const el = document.getElementById('loginTargetIp');
  if (el) el.textContent = esp32Host;
  const inp = document.getElementById('esp32IpInput');
  if (inp) inp.value = esp32Host;
}

// Saat halaman dimuat: sinkronkan host & IP input
window.addEventListener('load', () => {
  esp32Host = getAutoEsp32Host();
  updateLoginTargetIp();
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
    send({ cmd: 'stop' });
    showToast('Sistem dihentikan. Tekan START untuk memulai ulang.');
  } else {
    const m = parseInt(document.getElementById('modeSelect').value);
    send({ cmd: 'start', mode: m });
    showToast('Sistem di-START!');
  }
}

function sendResume() {
  send({ cmd: 'resume' });
  showToast('Safety Lockout direset. Tekan START untuk melanjutkan.');
}

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
  const txt    = document.getElementById('flashStatusText');
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
  if (!dot || !lbl) return;
  if (state === 'connected') {
    dot.className = 'conn-dot online';
    lbl.className = 'conn-label online';
    lbl.textContent = 'Online (' + esp32Host + ')';
  } else if (state === 'connecting') {
    dot.style.background = 'var(--warn)'; dot.className = 'conn-dot';
    lbl.style.color = 'var(--warn)'; lbl.className = 'conn-label';
    lbl.textContent = 'Menghubungkan (' + esp32Host + ')...';
  } else {
    dot.style.background = ''; dot.className = 'conn-dot';
    lbl.style.color = ''; lbl.className = 'conn-label';
    lbl.textContent = 'Offline (' + esp32Host + ')';
  }
}

function connectWS() {
  esp32Host = getAutoEsp32Host();
  setConn('connecting');
  try {
    if (ws) {
      try { ws.close(); } catch(e){}
      ws = null;
    }
    ws = new WebSocket('ws://' + esp32Host + ':81');
    ws.onopen = () => { 
      connected = true; 
      setConn('connected');
      showToast('✓ Terhubung ke ESP32 (' + esp32Host + ')');
    };
    ws.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);
        if (d.type === 'status') {
          updateUI(d);
          updateHardwareUI(d);
        } else if (d.type === 'hardware_scan') {
          updateHardwareUI(d);
          handleHardwareScanResult(d);
        }
      } catch(e){}
    };
    ws.onclose = ws.onerror = () => { 
      connected = false; 
      ws = null; 
      setConn('disconnected'); 
      stopWarmup(); 
      setTimeout(() => {
        // Hanya reconnect otomatis jika sedang di halaman app
        if (document.getElementById('appPage') && document.getElementById('appPage').style.display === 'block') {
          connectWS();
        }
      }, 3000); 
    };
  } catch(err) {
    console.error("WS Error:", err);
    setConn('disconnected');
  }
}

function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

// ═══════════════════════════════════════
//  HARDWARE SCANNER & DIAGNOSTICS
// ═══════════════════════════════════════
let isScanningHw = false;
let lastScanTimestamp = null;

function triggerHardwareScan() {
  if (isScanningHw) return;
  isScanningHw = true;

  const btn = document.getElementById('hwScanBtn');
  const label = document.getElementById('scanBtnLabel');
  const pBox = document.getElementById('scanProgressBox');
  const pFill = document.getElementById('scanProgressFill');
  const pText = document.getElementById('scanStepText');

  if (btn) btn.classList.add('scanning');
  if (label) label.textContent = 'MEMINDAI HARDWARE...';
  if (pBox) pBox.style.display = 'block';
  if (pFill) pFill.style.width = '0%';
  if (pText) pText.textContent = 'Memulai probe sirkuit hardware...';

  // Kirim perintah scan hardware ke ESP32 Utama
  send({ cmd: 'scan_hardware' });

  // Animasi Step Progress Pemindaian
  const steps = [
    { pct: 20, text: 'Memeriksa ADC Sensor Master (MQ-2 Gas, Water Level, Button)...' },
    { pct: 45, text: 'Memindai bus I2C Display LCD 16x2 (Address 0x27)...' },
    { pct: 70, text: 'Memverifikasi sinyal PWM Aktuator (BTS7960, Servos 1-3, Buzzer, RGB)...' },
    { pct: 90, text: 'Menguji link UART TX/RX & Ping Probe ke ESP32 ke-2...' },
    { pct: 100, text: 'Analisis telemetri selesai! Mengompilasi status perangkat...' }
  ];

  let stepIdx = 0;
  const scanInterval = setInterval(() => {
    if (stepIdx < steps.length) {
      if (pFill) pFill.style.width = steps[stepIdx].pct + '%';
      if (pText) pText.textContent = steps[stepIdx].text;
      stepIdx++;
    } else {
      clearInterval(scanInterval);
      setTimeout(() => {
        isScanningHw = false;
        if (btn) btn.classList.remove('scanning');
        if (label) label.textContent = 'PINDAI ULANG HARDWARE';
        if (pBox) pBox.style.display = 'none';

        const now = new Date();
        lastScanTimestamp = now.toLocaleTimeString('id-ID');
        const timeEl = document.getElementById('hwSummaryTime');
        if (timeEl) timeEl.textContent = lastScanTimestamp;

        showToast('✓ Pemindaian Hardware Selesai!');
      }, 600);
    }
  }, 320);
}

function handleHardwareScanResult(d) {
  const countEl = document.getElementById('hwSummaryCount');
  if (countEl && d.total_detected !== undefined) {
    countEl.textContent = d.total_detected + ' / 12 Perangkat';
  }
}

function updateHardwareUI(d) {
  let detectedCount = 0;
  const totalCount = 12;

  // Update Live Ultrasonic Radar Visualizer
  updateUltrasonicRadarUI(d);

  // 1. ESP32-2 & UART Link
  const esp2Online = !!(d.esp2_online !== undefined ? d.esp2_online : (d.esp2_last_rx_ms !== undefined ? d.esp2_last_rx_ms < 3500 : false));
  const badgeEsp2 = document.getElementById('badgeEsp2Main');
  const summaryEsp2 = document.getElementById('hwSummaryEsp2');
  const uartFlow = document.getElementById('uartFlowLine');
  const badgeRx = document.getElementById('badgeUartRx');
  const valRx = document.getElementById('valUartRx');
  const badgeTx = document.getElementById('badgeUartTx');
  const valTx = document.getElementById('valUartTx');

  if (esp2Online) {
    detectedCount += 4; // ESP32-2 node, Flame, US1, US2
    if (badgeEsp2) { badgeEsp2.className = 'hw-badge ok'; badgeEsp2.innerHTML = '<span class="pill-dot"></span>ONLINE &amp; TERDETEKSI'; }
    if (summaryEsp2) { summaryEsp2.textContent = 'ONLINE (UART2)'; summaryEsp2.style.color = 'var(--ok)'; }
    if (uartFlow) { uartFlow.classList.remove('offline'); }
    if (badgeRx) { badgeRx.className = 'hw-badge ok'; badgeRx.textContent = 'STREAMING'; }
    const pkts = d.esp2_rx_pkts !== undefined ? d.esp2_rx_pkts : '--';
    const lastMs = d.esp2_last_rx_ms !== undefined ? Math.round(d.esp2_last_rx_ms) : 0;
    if (valRx) { valRx.textContent = pkts + ' pkt (' + (lastMs < 1000 ? lastMs + 'ms' : (lastMs/1000).toFixed(1) + 's') + ' lalu)'; }
    if (badgeTx) { badgeTx.className = 'hw-badge ok'; badgeTx.textContent = 'PONG OK'; }
    if (valTx) { valTx.textContent = '9600 Baud 8N1'; }
  } else {
    if (badgeEsp2) { badgeEsp2.className = 'hw-badge fail'; badgeEsp2.innerHTML = '<span class="pill-dot"></span>TERPUTUS (OFFLINE)'; }
    if (summaryEsp2) { summaryEsp2.textContent = 'OFFLINE'; summaryEsp2.style.color = 'var(--danger)'; }
    if (uartFlow) { uartFlow.classList.add('offline'); }
    if (badgeRx) { badgeRx.className = 'hw-badge fail'; badgeRx.textContent = 'NO SIGNAL'; }
    if (valRx) { valRx.textContent = '0 pkt / Disconnected'; }
    if (badgeTx) { badgeTx.className = 'hw-badge warn'; badgeTx.textContent = 'WAITING'; }
    if (valTx) { valTx.textContent = 'Menunggu ESP32-2'; }
  }

  // ESP32-2 Sensors
  const badgeFlame = document.getElementById('badgeHwFlame');
  const valFlame = document.getElementById('valHwFlame');
  if (badgeFlame && valFlame) {
    if (esp2Online) {
      if (d.flameAlert2) {
        badgeFlame.className = 'hw-badge fail pulse'; badgeFlame.textContent = 'API TERDETEKSI!';
        valFlame.textContent = 'BAHAYA (LOW)'; valFlame.style.color = 'var(--danger)';
      } else {
        badgeFlame.className = 'hw-badge ok'; badgeFlame.textContent = 'TERDETEKSI';
        valFlame.textContent = 'Aman (Normal)'; valFlame.style.color = 'var(--accent)';
      }
    } else {
      badgeFlame.className = 'hw-badge fail'; badgeFlame.textContent = 'OFFLINE';
      valFlame.textContent = 'Tidak Ada Sinyal'; valFlame.style.color = 'var(--muted)';
    }
  }

  const badgeUs1 = document.getElementById('badgeHwUs1');
  const valUs1 = document.getElementById('valHwUs1');
  if (badgeUs1 && valUs1) {
    if (esp2Online) {
      badgeUs1.className = 'hw-badge ok'; badgeUs1.textContent = 'TERDETEKSI';
      const d1 = (d.d1 !== undefined && d.d1 < 400) ? d.d1.toFixed(1) + ' cm' : (d.anomalyCount ? 'Aktif (' + d.anomalyCount + 'x)' : 'Standby (>8cm)');
      valUs1.textContent = d1;
    } else {
      badgeUs1.className = 'hw-badge fail'; badgeUs1.textContent = 'OFFLINE';
      valUs1.textContent = '-- cm';
    }
  }

  const badgeUs2 = document.getElementById('badgeHwUs2');
  const valUs2 = document.getElementById('valHwUs2');
  if (badgeUs2 && valUs2) {
    if (esp2Online) {
      badgeUs2.className = 'hw-badge ok'; badgeUs2.textContent = 'TERDETEKSI';
      const dA = (d.dA !== undefined && d.dA < 400) ? d.dA.toFixed(0) : '-';
      const dB = (d.dB !== undefined && d.dB < 400) ? d.dB.toFixed(0) : '-';
      const dC = (d.dC !== undefined && d.dC < 400) ? d.dC.toFixed(0) : '-';
      valUs2.textContent = 'A:' + dA + ' B:' + dB + ' C:' + dC + ' cm';
    } else {
      badgeUs2.className = 'hw-badge fail'; badgeUs2.textContent = 'OFFLINE';
      valUs2.textContent = 'Pintu A/B/C Offline';
    }
  }

  // Master Sensors
  // MQ-2
  const badgeMq2 = document.getElementById('badgeHwMq2');
  const valMq2 = document.getElementById('valHwMq2');
  if (badgeMq2 && valMq2) {
    detectedCount++;
    badgeMq2.className = 'hw-badge ok'; badgeMq2.textContent = 'TERDETEKSI';
    valMq2.textContent = 'ADC: ' + (d.gasADC !== undefined ? d.gasADC : '--') + (d.mq2Ready ? ' (Siap)' : ' (Warmup)');
  }

  // Water
  const badgeWater = document.getElementById('badgeHwWater');
  const valWater = document.getElementById('valHwWater');
  if (badgeWater && valWater) {
    detectedCount++;
    badgeWater.className = 'hw-badge ok'; badgeWater.textContent = 'TERDETEKSI';
    valWater.textContent = 'ADC: ' + (d.waterADC !== undefined ? d.waterADC : '--') + (d.waterAlert ? ' (BANJIR)' : ' (Aman)');
  }

  // Button
  const badgeBtn = document.getElementById('badgeHwBtn');
  const valBtn = document.getElementById('valHwBtn');
  if (badgeBtn && valBtn) {
    detectedCount++;
    badgeBtn.className = 'hw-badge ok'; badgeBtn.textContent = 'TERDETEKSI';
    valBtn.textContent = d.btnPressed ? 'LOW (Pressed)' : 'HIGH (Standby)';
  }

  // LCD
  const badgeLcd = document.getElementById('badgeHwLcd');
  const valLcd = document.getElementById('valHwLcd');
  if (badgeLcd && valLcd) {
    const lcdOk = (d.lcd_detected !== undefined) ? d.lcd_detected : true;
    if (lcdOk) {
      detectedCount++;
      badgeLcd.className = 'hw-badge ok'; badgeLcd.textContent = 'TERDETEKSI';
      valLcd.textContent = 'I2C ACK 0x27 OK';
    } else {
      badgeLcd.className = 'hw-badge warn'; badgeLcd.textContent = 'NO ACK';
      valLcd.textContent = 'Cek Pin SDA21/SCL22';
    }
  }

  // Actuators
  // BTS7960
  detectedCount++; // Motor BTS7960
  const badgeMotor = document.getElementById('badgeHwMotor');
  const valMotor = document.getElementById('valHwMotor');
  if (badgeMotor && valMotor) {
    badgeMotor.className = 'hw-badge ok'; badgeMotor.textContent = 'TERDETEKSI';
    valMotor.textContent = (d.motorState ? d.motorState.toUpperCase() : 'STOP') + ' (' + (d.motorSpeed || 200) + ')';
  }

  // Servos 1, 2, 3
  detectedCount += 3;
  const valS1 = document.getElementById('valHwSrv1');
  if (valS1) valS1.textContent = 'Sudut: ' + (d.servo1 !== undefined ? d.servo1 : 0) + '°';
  const valS2 = document.getElementById('valHwSrv2');
  if (valS2) valS2.textContent = 'Sudut: ' + (d.servo2 !== undefined ? d.servo2 : 0) + '°';
  const valS3 = document.getElementById('valHwSrv3');
  if (valS3) valS3.textContent = 'Sudut: ' + (d.servo3 !== undefined ? d.servo3 : 0) + '°';

  // Buzzer & RGB
  detectedCount++; // Buzzer & RGB
  const badgeAlarm = document.getElementById('badgeHwAlarm');
  const valAlarm = document.getElementById('valHwAlarm');
  if (badgeAlarm && valAlarm) {
    badgeAlarm.className = 'hw-badge ok'; badgeAlarm.textContent = 'TERDETEKSI';
    valAlarm.textContent = (d.gasAlert || d.waterAlert || d.flameAlert2) ? 'ALARM AKTIF' : 'Standby (Ch6-9 PWM)';
  }

  // Summary count
  const summaryEl = document.getElementById('hwSummaryCount');
  if (summaryEl) {
    summaryEl.textContent = detectedCount + ' / ' + totalCount + ' Perangkat';
    summaryEl.style.color = (detectedCount === totalCount) ? 'var(--ok)' : (detectedCount > 8 ? 'var(--warn)' : 'var(--danger)');
  }
}

// ═══════════════════════════════════════
//  WASTE & SENSOR STATISTICS
// ═══════════════════════════════════════
let wasteStats = JSON.parse(localStorage.getItem('wasteStats')) || {};
let sensorStats = JSON.parse(localStorage.getItem('sensorStats')) || { gas: 0, water: 0, flame: 0 };
let prevServoActive = 0;
let prevAlerts = { gas: false, water: false, flame: false };
let prevUs2Door   = 0;
let lastYoloName  = 'Tidak Ada';
let lastYoloCat   = '—';
let lastYoloServo = 0;
let sortPopupTimer;

function showSortPopup(msg) {
  const p = document.getElementById('sortPopup');
  p.textContent = msg; p.classList.add('show');
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
  if (confirm('Hapus seluruh data riwayat limbah tersortir?')) {
    wasteStats = {}; localStorage.removeItem('wasteStats'); renderStats();
  }
}
function resetSensorStats() {
  if (confirm('Hapus seluruh data riwayat analisis sensor?')) {
    sensorStats = { gas: 0, water: 0 }; localStorage.removeItem('sensorStats'); renderStats();
  }
}

function renderStats() {
  const listEl = document.getElementById('wasteList');
  const svgCont = document.getElementById('svgContainer');
  const legendCont = document.getElementById('chartLegend');
  listEl.innerHTML = ''; svgCont.innerHTML = ''; legendCont.innerHTML = '';

  const keys = Object.keys(wasteStats);
  if (keys.length > 0 && typeof wasteStats[keys[0]] === 'number') {
    wasteStats = {}; localStorage.removeItem('wasteStats');
  }

  if (Object.keys(wasteStats).length === 0) {
    listEl.innerHTML = '<div style="color:var(--muted);text-align:center;padding:20px;">Belum ada data limbah.</div>';
  } else {
    const bgColors = ['#00d9ff','#3fb950','#ff7043','#bc8cff','#ffd700','#f85149'];
    let colorIdx = 0, categoryTotals = [], grandTotal = 0;
    for (const cat in wasteStats) {
      let catTotal = 0, itemsHtml = '';
      for (const name in wasteStats[cat]) {
        const count = wasteStats[cat][name]; catTotal += count;
        itemsHtml += `<div class="category-item"><span>${name}</span><span>${count}</span></div>`;
      }
      grandTotal += catTotal;
      const catColor = bgColors[colorIdx % bgColors.length];
      categoryTotals.push({ label: cat, val: catTotal, color: catColor });
      const catDiv = document.createElement('div');
      catDiv.className = 'category-group';
      catDiv.innerHTML = `<div class="category-head" style="border-left:4px solid ${catColor}">${cat}</div>${itemsHtml}<div class="category-total"><span>Total</span><span>${catTotal}</span></div>`;
      listEl.appendChild(catDiv); colorIdx++;
    }
    if (grandTotal > 0) {
      let svgHtml = '<svg viewBox="0 0 32 32" class="svg-pie">', legendHtml = '', cum = 0;
      categoryTotals.forEach(item => {
        const pct = item.val / grandTotal;
        svgHtml += `<circle r="16" cx="16" cy="16" fill="transparent" stroke="${item.color}" stroke-width="32" stroke-dasharray="${pct*100} 100" stroke-dashoffset="${-cum*100}"></circle>`;
        legendHtml += `<div class="legend-item"><div class="legend-color" style="background:${item.color}"></div>${item.label}</div>`;
        cum += pct;
      });
      svgCont.innerHTML = svgHtml + '</svg>'; legendCont.innerHTML = legendHtml;
    }
  }

  // Sensor Line Chart
  const sList = document.getElementById('sensorList');
  const sSvg = document.getElementById('sensorSvgContainer');
  sList.innerHTML = ''; sSvg.innerHTML = '';
  const sData = [
    { label: 'Gas/Asap', val: sensorStats.gas||0, color: '#bc8cff' },
    { label: 'Air Berlebih', val: sensorStats.water||0, color: '#00d9ff' },
    { label: 'Api / Flame', val: sensorStats.flame||0, color: '#ff7043' }
  ];
  let maxVal = Math.max(5, ...sData.map(d => d.val));
  sData.forEach(item => {
    sList.innerHTML += `<div class="sensor-item"><div style="display:flex;align-items:center;"><span class="sensor-dot" style="background:${item.color}"></span>${item.label}</div><strong style="color:${item.color};font-size:16px;">${item.val}</strong></div>`;
  });
  const svgW = 200, svgH = 100, gap = svgW / 2;
  const pts = sData.map((d,i) => { const y = svgH - ((d.val/maxVal)*(svgH-25))-12; return `${i*gap},${y}`; });
  const circlesHtml = sData.map((d,i) => {
    const y = svgH - ((d.val/maxVal)*(svgH-25))-12;
    return `<circle cx="${i*gap}" cy="${y}" r="4" fill="${d.color}" stroke="var(--surface2)" stroke-width="1.5"/><text x="${i*gap}" y="${y-10}" fill="${d.color}" font-size="11" text-anchor="middle" font-weight="bold">${d.val}</text>`;
  }).join('');
  sSvg.innerHTML = `<svg viewBox="-15 0 ${svgW+30} ${svgH}" class="svg-line"><polyline points="${pts.join(' ')}" fill="none" stroke="rgba(255,255,255,0.15)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>${circlesHtml}</svg>`;
}

// ═══════════════════════════════════════
//  UPDATE UI
// ═══════════════════════════════════════
function updateUI(d) {
  // ── ALERTS ──
  const alerts = [];
  if (d.gasAlert)    alerts.push('GAS/ASAP');
  if (d.waterAlert)  alerts.push('AIR BERLEBIH');
  if (d.flameAlert2) alerts.push('API / FLAME');

  // Sensor counter with 5s cooldown
  const now_ms = Date.now();
  const COOLDOWN = 5000;
  if (!window._lastSensorCount) window._lastSensorCount = { gas: 0, water: 0, flame: 0 };
  if (d.gasAlert && !prevAlerts.gas && (now_ms-(window._lastSensorCount.gas||0)) > COOLDOWN) {
    sensorStats.gas = (sensorStats.gas||0)+1; window._lastSensorCount.gas = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  if (d.waterAlert && !prevAlerts.water && (now_ms-(window._lastSensorCount.water||0)) > COOLDOWN) {
    sensorStats.water = (sensorStats.water||0)+1; window._lastSensorCount.water = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  if (d.flameAlert2 && !prevAlerts.flame && (now_ms-(window._lastSensorCount.flame||0)) > COOLDOWN) {
    sensorStats.flame = (sensorStats.flame||0)+1; window._lastSensorCount.flame = now_ms;
    localStorage.setItem('sensorStats', JSON.stringify(sensorStats));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
  }
  prevAlerts.gas = !!d.gasAlert; prevAlerts.water = !!d.waterAlert; prevAlerts.flame = !!d.flameAlert2;

  const banner = document.getElementById('alertBanner');
  if (alerts.length) { banner.classList.add('active'); document.getElementById('alertText').textContent = '⚠ BAHAYA: ' + alerts.join(' + ') + '!'; }
  else { banner.classList.remove('active'); }

  // ── YOLO DETECTION (shared data) ──
  const wasteName  = d.wasteName || 'Tidak Ada';
  const wasteCat   = d.wasteCat  || '—';
  const servoNames = { 1: '▸ Servo 1 — Infeksius (Kantong Kuning)', 2: '▸ Servo 2 — Non-Infeksius (Kantong Hitam)', 3: '▸ Servo 3 — B3 (Kantong Merah)' };
  const servoText  = (d.servoActive && d.servoActive > 0 && wasteName !== 'Tidak Ada')
                     ? (servoNames[d.servoActive] || '▸ Servo ' + d.servoActive) : null;

  // Simpan data YOLO saat servo sedang membuka
  if (d.servoActive > 0 && wasteName !== 'Tidak Ada') {
    lastYoloName  = wasteName;
    lastYoloCat   = wasteCat !== '—' ? wasteCat : 'Lainnya';
    lastYoloServo = d.servoActive;
  }
  prevServoActive = d.servoActive || 0;

  // Konfirmasi sortir: US2 mendeteksi limbah melewati pintu (rising edge)
  const US2_THRESH = 8;
  const dA_s = (d.dA !== undefined) ? d.dA : 999;
  const dB_s = (d.dB !== undefined) ? d.dB : 999;
  const dC_s = (d.dC !== undefined) ? d.dC : 999;
  const currentUs2 = (dA_s <= US2_THRESH) ? 1 : (dB_s <= US2_THRESH) ? 2 : (dC_s <= US2_THRESH) ? 3 : 0;

  if (currentUs2 > 0 && prevUs2Door === 0 && lastYoloName !== 'Tidak Ada') {
    const doorNames = { 1: 'Pintu 1 (Infeksius)', 2: 'Pintu 2 (Non-Infeksius)', 3: 'Pintu 3 (B3)' };
    const keys = Object.keys(wasteStats);
    if (keys.length > 0 && typeof wasteStats[keys[0]] === 'number') wasteStats = {};
    if (!wasteStats[lastYoloCat]) wasteStats[lastYoloCat] = {};
    if (!wasteStats[lastYoloCat][lastYoloName]) wasteStats[lastYoloCat][lastYoloName] = 0;
    wasteStats[lastYoloCat][lastYoloName]++;
    localStorage.setItem('wasteStats', JSON.stringify(wasteStats));
    showSortPopup('✓ "' + lastYoloName + '" masuk ' + (doorNames[currentUs2] || 'Pintu ' + currentUs2));
    if (document.getElementById('sidebar').classList.contains('show')) renderStats();
    lastYoloName = 'Tidak Ada'; lastYoloCat = '—'; lastYoloServo = 0;
  }
  prevUs2Door = currentUs2;

  // User live view
  if (document.getElementById('userView').style.display !== 'none') {
    document.getElementById('liveWasteName').textContent = wasteName;
    document.getElementById('liveWasteCat').textContent = 'Kategori: ' + wasteCat;
    const lsi = document.getElementById('liveServoInfo');
    if (servoText) { lsi.textContent = servoText; lsi.style.display = 'inline-block'; }
    else { lsi.style.display = 'none'; }

    const chipGas   = document.getElementById('chipGas');
    const chipWater = document.getElementById('chipWater');
    const chipMotor = document.getElementById('chipMotor');
    chipGas.className   = 'live-sensor-chip' + (d.gasAlert ? ' alert' : '');
    chipGas.textContent = '🟣 Gas: ' + (d.gasADC !== undefined ? d.gasADC : '—');
    chipWater.className = 'live-sensor-chip' + (d.waterAlert ? ' alert' : '');
    chipWater.textContent = '💧 Air: ' + (d.waterADC !== undefined ? d.waterADC : '—');
    const chipFlame = document.getElementById('chipFlame');
    if (chipFlame) {
      chipFlame.className = 'live-sensor-chip' + (d.flameAlert2 ? ' alert' : '');
      chipFlame.textContent = '🔥 Api: ' + (d.flameAlert2 ? 'TERDETEKSI!' : (d.esp2_online ? 'Aman' : 'Offline'));
    }
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

    // Flame (ESP32-2)
    const fv = document.getElementById('flameVal');
    const fb = document.getElementById('flameBar');
    const fp = document.getElementById('flamePill');
    if (fv && fb && fp) {
      if (!d.esp2_online) {
        fv.textContent = 'Offline'; fv.style.color = 'var(--muted)';
        fp.className = 'pill offline'; fp.innerHTML = '<span class="pill-dot"></span>ESP32-2 Offline';
        fb.style.width = '0%';
      } else if (d.flameAlert2) {
        fv.textContent = '🔥 API TERDETEKSI!'; fv.style.color = 'var(--danger)';
        fp.className = 'pill danger'; fp.innerHTML = '<span class="pill-dot"></span>BAHAYA — API';
        fb.style.width = '100%';
      } else {
        fv.textContent = 'Tidak Ada Api'; fv.style.color = 'var(--ok)';
        fp.className = 'pill ok'; fp.innerHTML = '<span class="pill-dot"></span>Aman (Normal)';
        fb.style.width = '0%';
      }
    }

    // Button
    const bp = document.getElementById('btnPill');
    bp.className = 'btn-state ' + (d.btnPressed ? 'pressed' : 'released');
    bp.innerHTML = '<span class="pill-dot"></span>' + (d.btnPressed ? 'Pressed' : 'Released');

    // Motor
    motorState = d.motorState || 'stop';
    if (d.motorSpeed !== undefined) {
      motorSpeed = d.motorSpeed;
      const speedSlider = document.getElementById('motorSpeed');
      if (document.activeElement !== speedSlider) {
        speedSlider.value = motorSpeed;
      }
      document.getElementById('motorSpeedVal').textContent = motorSpeed;
      document.getElementById('motorSpeedPct').textContent = Math.round(motorSpeed / 255 * 100) + '%';
      updateMotorPresetHighlight(motorSpeed);
    }
    updateMotorUI();

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
function motorCmd(dir) {
  if (dir === 'stop') {
    send({ cmd: 'stop' });
    showToast('Motor & Sistem dihentikan.');
  } else {
    send({ cmd: 'motor', dir: dir, speed: motorSpeed });
  }
}

let speedSendTimer = null;

function motorPreset(v) {
  v = parseInt(v);
  motorSpeed = v;
  document.getElementById('motorSpeed').value = v;
  document.getElementById('motorSpeedVal').textContent = v;
  document.getElementById('motorSpeedPct').textContent = Math.round(v / 255 * 100) + '%';
  updateMotorPresetHighlight(v);
  clearTimeout(speedSendTimer);
  send({ cmd: 'motor', dir: motorState, speed: v });
}

function updateMotorPresetHighlight(v) {
  const btns = document.querySelectorAll('.motor-card .preset-btn');
  MOTOR_PRESETS.forEach((p, i) => { btns[i].className = 'preset-btn' + (p === v ? ' active' : ''); });
}

function speedInput(v) {
  v = parseInt(v);
  motorSpeed = v;
  document.getElementById('motorSpeedVal').textContent = v;
  document.getElementById('motorSpeedPct').textContent = Math.round(v / 255 * 100) + '%';
  updateMotorPresetHighlight(v);
  clearTimeout(speedSendTimer);
  speedSendTimer = setTimeout(() => {
    send({ cmd: 'motor', dir: motorState, speed: v });
  }, 50);
}
function speedSend(v) {
  v = parseInt(v);
  motorSpeed = v;
  clearTimeout(speedSendTimer);
  send({ cmd: 'motor', dir: motorState, speed: v });
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

// ═══════════════════════════════════════
//  ULTRASONIC LIVE RADAR & OBSTACLE MONITOR
// ═══════════════════════════════════════
function openRadarModal() {
  document.getElementById('radarModal').classList.add('show');
}
function closeRadarModal() {
  document.getElementById('radarModal').classList.remove('show');
}

function updateUltrasonicRadarUI(d) {
  if (!d) return;

  const d1 = (d.d1 !== undefined && d.d1 > 0) ? d.d1 : 999.0;
  const dA = (d.dA !== undefined && d.dA > 0) ? d.dA : 999.0;
  const dB = (d.dB !== undefined && d.dB > 0) ? d.dB : 999.0;
  const dC = (d.dC !== undefined && d.dC > 0) ? d.dC : 999.0;

  let obstacleCount = 0;

  // Helper untuk memetakan jarak (0-50 cm) ke koordinat lingkaran radar (pusat 160, 160)
  function getPos(angleDeg, distCm) {
    const clampedDist = Math.min(Math.max(distCm, 3), 48); // batas 3cm - 48cm
    const maxRadius = 140; // px
    const r = (clampedDist / 50) * maxRadius;
    const rad = (angleDeg - 90) * (Math.PI / 180);
    const x = 160 + r * Math.cos(rad);
    const y = 160 + r * Math.sin(rad);
    return { x: Math.round(x), y: Math.round(y) };
  }

  // 1. SENSOR 1 (US1 Anomali) - Arah: Atas (0 deg)
  const isObs1 = (d1 >= 1 && d1 <= 20);
  if (isObs1) obstacleCount++;
  const pos1 = getPos(0, (d1 < 400) ? d1 : 48);
  const blip1 = document.getElementById('radarBlip1');
  const label1 = document.getElementById('radarBlipLabel1');
  const card1 = document.getElementById('stationCard1');
  const dist1 = document.getElementById('stationDist1');
  const bar1 = document.getElementById('stationBar1');
  const pill1 = document.getElementById('stationPill1');
  const det1 = document.getElementById('stationDetail1');

  if (blip1) {
    blip1.style.left = pos1.x + 'px';
    blip1.style.top = pos1.y + 'px';
    blip1.className = isObs1 ? 'obstacle-blip' : 'obstacle-clear-blip';
    if (label1) label1.textContent = 'US1: ' + ((d1 < 400) ? d1.toFixed(1) + 'cm' : 'Clear');
  }
  if (card1 && dist1 && bar1 && pill1) {
    if (isObs1) {
      card1.className = 'station-card obstacle-active';
      pill1.className = 'station-status-pill danger';
      pill1.textContent = 'TERHALANG!';
      bar1.style.background = 'var(--danger)';
      bar1.style.width = Math.max(10, Math.min(100, Math.round(100 - (d1 / 20 * 100)))) + '%';
      if (det1) det1.textContent = '⚠ Objek Terdeteksi';
    } else {
      card1.className = 'station-card';
      pill1.className = 'station-status-pill clear';
      pill1.textContent = 'BEBAS';
      bar1.style.background = 'var(--ok)';
      bar1.style.width = ((d1 < 400) ? Math.min(100, Math.round(d1 / 50 * 100)) : 0) + '%';
      if (det1) det1.textContent = 'Jalur Bersih';
    }
    dist1.innerHTML = (d1 < 400) ? d1.toFixed(1) + ' <span style="font-size:13px;">cm</span>' : '∞ <span style="font-size:13px;">cm</span>';
  }

  // 2. SENSOR 2 (US2 Pintu A - Infeksius) - Arah: Kiri (270 deg)
  const isObsA = (dA >= 1 && dA <= 15);
  if (isObsA) obstacleCount++;
  const posA = getPos(270, (dA < 400) ? dA : 48);
  const blipA = document.getElementById('radarBlipA');
  const labelA = document.getElementById('radarBlipLabelA');
  const cardA = document.getElementById('stationCardA');
  const distA = document.getElementById('stationDistA');
  const barA = document.getElementById('stationBarA');
  const pillA = document.getElementById('stationPillA');
  const detA = document.getElementById('stationDetailA');

  if (blipA) {
    blipA.style.left = posA.x + 'px';
    blipA.style.top = posA.y + 'px';
    blipA.className = isObsA ? 'obstacle-blip' : 'obstacle-clear-blip';
    if (labelA) labelA.textContent = 'P1: ' + ((dA < 400) ? dA.toFixed(1) + 'cm' : 'Clear');
  }
  if (cardA && distA && barA && pillA) {
    if (isObsA) {
      cardA.className = 'station-card obstacle-active';
      pillA.className = 'station-status-pill danger';
      pillA.textContent = 'TERHALANG!';
      barA.style.background = 'var(--danger)';
      barA.style.width = Math.max(10, Math.min(100, Math.round(100 - (dA / 15 * 100)))) + '%';
      if (detA) detA.textContent = '⚠ Objek di Pintu 1';
    } else {
      cardA.className = 'station-card';
      pillA.className = 'station-status-pill clear';
      pillA.textContent = 'BEBAS';
      barA.style.background = 'var(--ok)';
      barA.style.width = ((dA < 400) ? Math.min(100, Math.round(dA / 50 * 100)) : 0) + '%';
      if (detA) detA.textContent = 'Pintu Bebas';
    }
    distA.innerHTML = (dA < 400) ? dA.toFixed(1) + ' <span style="font-size:13px;">cm</span>' : '∞ <span style="font-size:13px;">cm</span>';
  }

  // 3. SENSOR 3 (US2 Pintu B - Non-Infeksius) - Arah: Bawah (180 deg)
  const isObsB = (dB >= 1 && dB <= 15);
  if (isObsB) obstacleCount++;
  const posB = getPos(180, (dB < 400) ? dB : 48);
  const blipB = document.getElementById('radarBlipB');
  const labelB = document.getElementById('radarBlipLabelB');
  const cardB = document.getElementById('stationCardB');
  const distB = document.getElementById('stationDistB');
  const barB = document.getElementById('stationBarB');
  const pillB = document.getElementById('stationPillB');
  const detB = document.getElementById('stationDetailB');

  if (blipB) {
    blipB.style.left = posB.x + 'px';
    blipB.style.top = posB.y + 'px';
    blipB.className = isObsB ? 'obstacle-blip' : 'obstacle-clear-blip';
    if (labelB) labelB.textContent = 'P2: ' + ((dB < 400) ? dB.toFixed(1) + 'cm' : 'Clear');
  }
  if (cardB && distB && barB && pillB) {
    if (isObsB) {
      cardB.className = 'station-card obstacle-active';
      pillB.className = 'station-status-pill danger';
      pillB.textContent = 'TERHALANG!';
      barB.style.background = 'var(--danger)';
      barB.style.width = Math.max(10, Math.min(100, Math.round(100 - (dB / 15 * 100)))) + '%';
      if (detB) detB.textContent = '⚠ Objek di Pintu 2';
    } else {
      cardB.className = 'station-card';
      pillB.className = 'station-status-pill clear';
      pillB.textContent = 'BEBAS';
      barB.style.background = 'var(--ok)';
      barB.style.width = ((dB < 400) ? Math.min(100, Math.round(dB / 50 * 100)) : 0) + '%';
      if (detB) detB.textContent = 'Pintu Bebas';
    }
    distB.innerHTML = (dB < 400) ? dB.toFixed(1) + ' <span style="font-size:13px;">cm</span>' : '∞ <span style="font-size:13px;">cm</span>';
  }

  // 4. SENSOR 4 (US2 Pintu C - B3) - Arah: Kanan (90 deg)
  const isObsC = (dC >= 1 && dC <= 15);
  if (isObsC) obstacleCount++;
  const posC = getPos(90, (dC < 400) ? dC : 48);
  const blipC = document.getElementById('radarBlipC');
  const labelC = document.getElementById('radarBlipLabelC');
  const cardC = document.getElementById('stationCardC');
  const distC = document.getElementById('stationDistC');
  const barC = document.getElementById('stationBarC');
  const pillC = document.getElementById('stationPillC');
  const detC = document.getElementById('stationDetailC');

  if (blipC) {
    blipC.style.left = posC.x + 'px';
    blipC.style.top = posC.y + 'px';
    blipC.className = isObsC ? 'obstacle-blip' : 'obstacle-clear-blip';
    if (labelC) labelC.textContent = 'P3: ' + ((dC < 400) ? dC.toFixed(1) + 'cm' : 'Clear');
  }
  if (cardC && distC && barC && pillC) {
    if (isObsC) {
      cardC.className = 'station-card obstacle-active';
      pillC.className = 'station-status-pill danger';
      pillC.textContent = 'TERHALANG!';
      barC.style.background = 'var(--danger)';
      barC.style.width = Math.max(10, Math.min(100, Math.round(100 - (dC / 15 * 100)))) + '%';
      if (detC) detC.textContent = '⚠ Objek di Pintu 3';
    } else {
      cardC.className = 'station-card';
      pillC.className = 'station-status-pill clear';
      pillC.textContent = 'BEBAS';
      barC.style.background = 'var(--ok)';
      barC.style.width = ((dC < 400) ? Math.min(100, Math.round(dC / 50 * 100)) : 0) + '%';
      if (detC) detC.textContent = 'Pintu Bebas';
    }
    distC.innerHTML = (dC < 400) ? dC.toFixed(1) + ' <span style="font-size:13px;">cm</span>' : '∞ <span style="font-size:13px;">cm</span>';
  }

  // Summary status text pada radar
  const summaryEl = document.getElementById('radarObstacleSummary');
  if (summaryEl) {
    summaryEl.innerHTML = (obstacleCount > 0)
      ? '<span style="color:var(--danger);font-weight:bold;">⚠ ' + obstacleCount + ' HALANGAN TERDETEKSI</span>'
      : '<span style="color:var(--ok);">✓ Semua Jalur Bebas</span>';
  }
}

[1, 2, 3].forEach(id => rotateNeedle(id, 0));


// ═══════════════════════════════════════
//  HARDWARE DIAGNOSTIC TEST FUNCTIONS
// ═══════════════════════════════════════
function testHardware(target) {
  if (target === 'yolo') {
    openYoloTestModal();
    return;
  }
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    showToast('WebSocket belum terhubung ke ESP32!', 'warn');
    return;
  }
  if (target === 'gas') {
    showToast('💨 Simulasi Sensor Gas aktif (4 detik)...', 'info');
  } else if (target === 'water') {
    showToast('💧 Simulasi Sensor Air aktif (4 detik)...', 'info');
  } else if (target === 'buzzer') {
    showToast('🔊 Test Buzzer aktif (1 detik)...', 'info');
  } else if (target === 'flame') {
    showToast('🔥 Simulasi Sensor Api aktif (3 detik)...', 'info');
  } else if (target === 'motor') {
    showToast('⚙️ Test Motor (Maju 2s ➔ Jeda 1s ➔ Mundur 2s)...', 'info');
  }
  ws.send(JSON.stringify({ cmd: "test", target: target }));
}

function openYoloTestModal() {
  const m = document.getElementById('yoloTestModal');
  if (m) m.classList.add('show');
}

function closeYoloTestModal() {
  const m = document.getElementById('yoloTestModal');
  if (m) m.classList.remove('show');
}

function runYoloSimulation(name, category, servoId) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    showToast('WebSocket belum terhubung ke ESP32!', 'warn');
    return;
  }
  showToast(`🎯 Simulasi YOLO: "${name}" (${category}) ➔ Servo ${servoId}`, 'ok');
  lastYoloName = name;
  lastYoloCat  = category;
  ws.send(JSON.stringify({
    cmd: "servo",
    id: servoId,
    angle: 90,
    waste: name,
    cat: category
  }));
  closeYoloTestModal();
}

window.onload = function() {
  document.getElementById('loginPage').style.display = 'flex';
  document.getElementById('ipConfigPage').style.display = 'none';
  connectCamWs();
};
</script>
</body>
</html>

)====";

// ============================================================
//  OBJECTS
// ============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
WebServer         server(80);
WebSocketsServer  webSocket(81);

Servo servo1, servo2, servo3;
int   servoAngle1 = 0, servoAngle2 = 0, servoAngle3 = 0;

// ============================================================
//  STATE VARIABLES
// ============================================================
struct RGBColor { uint8_t r, g, b; };
const RGBColor COL_OFF     = {0,0,0};
const RGBColor COL_GAS_A   = {80,0,80};
const RGBColor COL_GAS_B   = {0,0,0};
const RGBColor COL_WATER_A = {0,0,80};
const RGBColor COL_WATER_B = {0,0,0};
const RGBColor COL_FLAME_A = {80,20, 0};  // Orange — api
const RGBColor COL_FLAME_B = {0,  0, 0};

bool   gasAlert    = false;
bool   waterAlert  = false;
int    lastGasVal  = 0;
bool   lastGasD0   = false;
int    lastWaterVal = 0;

// ── ESP32 ke-2 UART State ─────────────────────────────────────
bool     flameAlert2    = false;       // Api dari ESP32 ke-2
bool     us1WasActive   = false;       // Debounce rising edge US1
uint8_t  us2LastDoor    = 0;           // Pintu US2 terakhir aktif
bool     us2WasActive   = false;       // Debounce rising edge US2
uint32_t anomalyCount   = 0;           // Total limbah anomali (US1)
uint32_t sortConfirm[4] = {0,0,0,0};  // Konfirmasi sortir per servo [1-3]
int      lastSortedServo = 0;          // Servo yang terakhir aktif (untuk atribusi US2)
uint32_t lastSortedAt   = 0;           // Waktu servo terakhir menutup
String   uart2Buffer    = "";          // Buffer parsing JSON dari Serial2
uint32_t uart2LastRxTime = 0;          // Waktu paket terakhir diterima dari ESP32-2
uint32_t uart2PacketCount = 0;         // Total paket valid diterima dari ESP32-2
float    esp32_2_us1_dist = 999.0f;    // Jarak sensor anomali US1 (cm)
float    esp32_2_us2_distA = 999.0f;   // Jarak sortir pintu A (cm)
float    esp32_2_us2_distB = 999.0f;   // Jarak sortir pintu B (cm)
float    esp32_2_us2_distC = 999.0f;   // Jarak sortir pintu C (cm)
bool     lcdDetected    = true;        // Status respons bus I2C LCD

// MQ-2 Baseline Calibration
bool     mq2WarmupDone    = false;
uint32_t mq2WarmupStart   = 0;
uint32_t mq2LastRead      = 0;
int      mq2Baseline      = 0;
long     mq2BaselineSum   = 0;
int      mq2BaselineCount = 0;
int      mq2ThreshOn      = 1200;
int      mq2ThreshOff     = 900;
uint32_t gasDoutLowSince  = 0;

uint32_t waterLastRead  = 0;

String   motorState  = "stop";
int      motorSpeed  = MOTOR_SPEED_RUN;
uint32_t motorKickUntil    = 0;  // reserved (tidak dipakai di BTS7960)
uint32_t motorAutoReKickAt = 0;  // reserved
uint32_t motorReKickUntil  = 0;  // reserved

bool     rgbBlinking   = false;
RGBColor rgbBlink1     = COL_OFF;
RGBColor rgbBlink2     = COL_OFF;
bool     rgbBlinkState = false;
uint32_t rgbBlinkTimer = 0;

bool     btnLastState  = HIGH;
bool     btnPressed    = false;
uint32_t btnDebounce   = 0;

bool     lcdClearPending = false;
uint32_t lcdClearTimer   = 0;

uint32_t lastWsBroadcast = 0;
uint32_t lastDebugPrint  = 0;

bool     safetyLockout = false;
bool     systemStarted = false;
int      conveyorMode  = 0;

uint32_t testGasUntil    = 0;
uint32_t testWaterUntil  = 0;
uint32_t testBuzzerUntil = 0;
uint32_t testFlameUntil  = 0;
int      testMotorStep   = 0;
uint32_t testMotorUntil  = 0;

uint32_t lessEnergyStopAt = 0;

String lastWasteName     = "Tidak Ada";
String lastWasteCategory = "-";
int    lastActiveServo   = 0;

#define SERVO_DELAY_1 100   // ms - Infeksius (Servo 1)
#define SERVO_DELAY_2 300   // ms - Non-Infeksius (Servo 2)
#define SERVO_DELAY_3 1200   // ms - B3 (Servo 3)
#define SERVO_HOLD_MS 1500   // ms - Tahan 500ms pada sudut 90° lalu kembali ke 0°

enum ServoStage {
  SERVO_IDLE = 0,
  SERVO_WAITING_OPEN,
  SERVO_HOLDING_OPEN
};

struct ServoJob {
  ServoStage stage;
  uint32_t   openAt;
  uint32_t   closeAt;
  int        targetAngle;
  String     waste;
  String     cat;
};
ServoJob servoJobs[4];

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void handleGasSensor(uint32_t now);
void handleWaterSensor(uint32_t now);
void handleButton(uint32_t now);
void handleRGB(uint32_t now);
void handleUART2(uint32_t now);       // Baca data JSON dari ESP32 ke-2
void performHardwareScan();           // Pindai status keberadaan semua sensor & aktuator
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

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SMART FACTORY BTS7960 (IBT-2 43A) BOOTING ===");

  // UART2: Terima data sensor dari ESP32 ke-2 (Flame + Ultrasonik)
  Serial2.begin(UART2_BAUD, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);
  Serial.printf("[UART2] Serial2 aktif. RX=GPIO%d (\u2190 ESP32-2 TX GPIO17)\n", PIN_UART2_RX);

  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_BUTTON,     INPUT_PULLUP);
  pinMode(PIN_GAS_AOUT,   INPUT);
  pinMode(PIN_GAS_DOUT,   INPUT_PULLUP);
  pinMode(PIN_WATER_AOUT, INPUT);

  // Motor DC BTS7960 Setup
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW); // Standby awal

  ledcAttachChannel(PIN_MOTOR_RPWM, PWM_MOTOR_FREQ, PWM_MOTOR_RES, 4);
  ledcAttachChannel(PIN_MOTOR_LPWM, PWM_MOTOR_FREQ, PWM_MOTOR_RES, 5);
  stopMotor();

  Serial.println("[BTS7960] Driver aktif. RPWM=GPIO4 (Ch4), LPWM=GPIO17 (Ch5), EN=GPIO16");

  // RGB LED (Channel 6, 7, 8)
  ledcAttachChannel(PIN_RGB_R, PWM_FREQ, PWM_RES, 6);
  ledcAttachChannel(PIN_RGB_G, PWM_FREQ, PWM_RES, 7);
  ledcAttachChannel(PIN_RGB_B, PWM_FREQ, PWM_RES, 8);
  setRGBOff();

  analogReadResolution(12);

  // Buzzer (Channel 9 agar tidak bentrok dengan Timer Servo)
  ledcAttachChannel(PIN_BUZZER, 2000, 8, 9);
  ledcWrite(PIN_BUZZER, 0);

  // ========== SERVO SETUP ==========
  // Alokasikan Timer 0 & 1 khusus untuk 50Hz Servo (mendukung hingga 8 servo).
  // Timer 2 & 3 disisakan untuk PWM Motor (5000Hz) dan RGB/Buzzer agar tidak bentrok frekuensi.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);

  servo1.attach(PIN_SERVO1, SERVO_MIN_US, SERVO_MAX_US); delay(20);
  servo2.attach(PIN_SERVO2, SERVO_MIN_US, SERVO_MAX_US); delay(20);
  servo3.attach(PIN_SERVO3, SERVO_MIN_US, SERVO_MAX_US); delay(20);

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  delay(200);

  Serial.print("[Servo] servo1 attached? "); Serial.println(servo1.attached() ? "YES" : "NO");
  Serial.print("[Servo] servo2 attached? "); Serial.println(servo2.attached() ? "YES" : "NO");
  Serial.print("[Servo] servo3 attached? "); Serial.println(servo3.attached() ? "YES" : "NO");

  // LCD
  Wire.begin(21, 22);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0, 0); lcd.print("BTS7960 SmartBin");
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

  // HTTP API untuk Python YOLO
  server.on("/api", HTTP_GET, []() {
    if (server.hasArg("cmd")) {
      String cmd = server.arg("cmd");
      
      if (cmd == "servo") {
        int id    = server.arg("id").toInt();
        int angle = server.hasArg("angle") ? server.arg("angle").toInt() : 90;
        String w  = server.hasArg("waste") ? server.arg("waste") : lastWasteName;
        String c  = server.hasArg("cat")   ? server.arg("cat")   : lastWasteCategory;

        if (id >= 1 && id <= 3) {
          if (angle > 0 && (safetyLockout || !systemStarted)) {
            Serial.printf("[HTTP] Servo %d -> 90 deg DIABAIKAN (Lockout/Belum Start)\n", id);
          } else if (angle > 0) {
            uint32_t delayMs = 0;
            if      (id == 1) delayMs = SERVO_DELAY_1;
            else if (id == 2) delayMs = SERVO_DELAY_2;
            else if (id == 3) delayMs = SERVO_DELAY_3;

            servoJobs[id].stage       = SERVO_WAITING_OPEN;
            servoJobs[id].openAt      = millis() + delayMs;
            servoJobs[id].targetAngle = 90;
            servoJobs[id].waste       = w;
            servoJobs[id].cat         = c;

            Serial.printf("[HTTP] Servo %d dijadwalkan: Delay %dms -> Buka ke 90° -> Tahan %dms -> Kembali ke 0° (Objek: %s)\n",
                          id, delayMs, SERVO_HOLD_MS, w.c_str());

            if (conveyorMode == 1 && systemStarted && !safetyLockout && !gasAlert && !waterAlert) {
              startConveyor();
              lessEnergyStopAt = millis() + 5000;
              Serial.println("[HTTP] Less Energy: Motor ON untuk 5 detik (ada objek)");
            }
          } else {
            servoJobs[id].stage = SERVO_IDLE;
            if      (id == 1) { servo1.write(0); servoAngle1 = 0; }
            else if (id == 2) { servo2.write(0); servoAngle2 = 0; }
            else if (id == 3) { servo3.write(0); servoAngle3 = 0; }
            if (id == lastActiveServo) { lastActiveServo = 0; }
            lastWasteName     = "Tidak Ada";
            lastWasteCategory = "-";
            Serial.printf("[HTTP] Servo %d ditutup ke 0°\n", id);
          }
        }
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "stop") {
        systemStarted = false;
        stopMotor();
        lessEnergyStopAt = 0;
        for (int i = 1; i <= 3; i++) {
          servoJobs[i].stage = SERVO_IDLE;
        }
        servo1.write(0); servoAngle1 = 0;
        servo2.write(0); servoAngle2 = 0;
        servo3.write(0); servoAngle3 = 0;
        lastActiveServo   = 0;
        lastWasteName     = "Tidak Ada";
        lastWasteCategory = "-";
        Serial.println("[HTTP] Sistem di-STOP");
        server.send(200, "text/plain", "OK");
        updateLCD();
        broadcastStatus();
      }
      else if (cmd == "machine") {
        String onStr = server.arg("on");
        if (onStr.equalsIgnoreCase("true")) {
          systemStarted = true;
          startConveyor();
          Serial.println("[HTTP] Mesin ON (kick-start)");
        } else {
          systemStarted = false;
          stopMotor();
          Serial.println("[HTTP] Mesin OFF");
        }
        server.send(200, "text/plain", "OK");
      }
      else if (cmd == "test") {
        String target = server.arg("target");
        if (target == "buzzer") {
          ledcWrite(PIN_BUZZER, 180);
          testBuzzerUntil = millis() + 1000;
          Serial.println("[HTTP TEST] Buzzer test aktif (1 detik)");
        } else if (target == "flame") {
          testFlameUntil = millis() + 3000;
          flameAlert2 = true;
          Serial.println("[HTTP TEST] Simulasi sensor api aktif (3 detik)");
        } else if (target == "gas") {
          testGasUntil = millis() + 4000;
          Serial.println("[HTTP TEST] Simulasi sensor gas aktif 4 detik");
        } else if (target == "water") {
          testWaterUntil = millis() + 4000;
          Serial.println("[HTTP TEST] Simulasi sensor air aktif 4 detik");
        } else if (target == "motor") {
          int spd = (motorSpeed > 0) ? motorSpeed : 180;
          setMotor("fwd", spd);
          testMotorStep  = 1;
          testMotorUntil = millis() + 2000;
          Serial.println("[HTTP TEST] Motor test: Maju 2s -> Jeda 1s -> Mundur 2s");
        }
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "motor") {
        String dir = server.arg("dir");
        int spd = server.hasArg("speed") ? server.arg("speed").toInt() : motorSpeed;
        if (dir == "stop") {
          stopMotor();
        } else {
          setMotor(dir, spd);
        }
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "start" || cmd == "start_system") {
        systemStarted = true;
        if (server.hasArg("mode")) conveyorMode = server.arg("mode").toInt();
        if (conveyorMode == 0) startConveyor();
        else stopMotor();
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "set_mode") {
        if (server.hasArg("mode")) conveyorMode = server.arg("mode").toInt();
        systemStarted = false;
        stopMotor();
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else if (cmd == "resume") {
        safetyLockout = false;
        gasAlert = false;
        waterAlert = false;
        flameAlert2 = false;
        systemStarted = true;
        if (conveyorMode == 0) startConveyor();
        server.send(200, "text/plain", "OK");
        broadcastStatus();
      }
      else {
        server.send(400, "text/plain", "Unknown command");
      }
      broadcastStatus();
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

  Serial.println("[SYSTEM] Smart Factory BTS7960 Siap.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();
  webSocket.loop();
  server.handleClient();

  handleRGB(now);
  handleGasSensor(now);
  handleWaterSensor(now);
  handleButton(now);
  handleUART2(now);   // Baca data JSON dari ESP32 ke-2 (Flame + Ultrasonik)

  // Hapus lastSortedServo jika sudah melebihi timeout
  if (lastSortedServo > 0 && lastSortedAt > 0 && (now - lastSortedAt) >= UART2_SORT_TIMEOUT) {
    lastSortedServo = 0;
  }

  // --- HARDWARE TEST HANDLER ---
  if (testBuzzerUntil > 0) {
    if (now >= testBuzzerUntil) { ledcWrite(PIN_BUZZER, 0); testBuzzerUntil = 0; }
  }
  if (testFlameUntil > 0) {
    if (now >= testFlameUntil) {
      testFlameUntil = 0;
      flameAlert2 = false;
      stopBuzzer();
      setRGBOff();
      Serial.println("[TEST] Simulasi sensor api selesai.");
      broadcastStatus();
    }
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

  // --- SERVO SCHEDULED JOBS (0° -> 90° -> tunggu 0.4s -> 0°) ---
  for (int i = 1; i <= 3; i++) {
    if (servoJobs[i].stage == SERVO_WAITING_OPEN && now >= servoJobs[i].openAt) {
      int a = (servoJobs[i].targetAngle > 0) ? servoJobs[i].targetAngle : 90;
      if      (i == 1) { servo1.write(a); servoAngle1 = a; }
      else if (i == 2) { servo2.write(a); servoAngle2 = a; }
      else if (i == 3) { servo3.write(a); servoAngle3 = a; }

      lastWasteName     = servoJobs[i].waste;
      lastWasteCategory = servoJobs[i].cat;
      lastActiveServo   = i;
      Serial.printf("[SERVO] Servo %d dibuka ke %d° (%s) -> Tahan %dms\n", i, a, lastWasteName.c_str(), SERVO_HOLD_MS);

      servoJobs[i].stage   = SERVO_HOLDING_OPEN;
      servoJobs[i].closeAt = now + SERVO_HOLD_MS;

      updateLCD();
      broadcastStatus();
    }
    else if (servoJobs[i].stage == SERVO_HOLDING_OPEN && now >= servoJobs[i].closeAt) {
      if      (i == 1) { servo1.write(0); servoAngle1 = 0; }
      else if (i == 2) { servo2.write(0); servoAngle2 = 0; }
      else if (i == 3) { servo3.write(0); servoAngle3 = 0; }

      servoJobs[i].stage = SERVO_IDLE;
      if (i == lastActiveServo) { lastActiveServo = 0; }
      // Catat servo yang baru menutup agar US2 bisa atribusikan kategori sortir
      lastSortedServo   = i;
      lastSortedAt      = now;
      lastWasteName     = "Tidak Ada";
      lastWasteCategory = "-";
      Serial.printf("[SERVO] Servo %d selesai -> Kembali ke 0°\n", i);

      updateLCD();
      broadcastStatus();
    }
  }

  if (lcdClearPending && (now - lcdClearTimer >= LCD_CLEAR_DELAY)) {
    lcdClearPending = false;
    updateLCD();
  }

  if (conveyorMode == 1 && lessEnergyStopAt > 0 && now >= lessEnergyStopAt) {
    if (motorState != "stop") {
      stopMotor();
      Serial.println("[BTS7960] Less Energy: Waktu 5 detik habis, motor stop.");
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
    Serial.printf("  MOTOR (BTS7960) : %s  | Spd: %d\n", motorState.c_str(), motorSpeed);
    Serial.printf("======================\n");
  }
}

// ============================================================
//  PUSH BUTTON
// ============================================================
void handleButton(uint32_t now) {
  bool reading = digitalRead(PIN_BUTTON);

  if (reading != btnLastState) {
    btnDebounce = now;
  }

  if ((now - btnDebounce) >= BTN_DEBOUNCE_MS) {
    bool currentlyPressed = (reading == LOW);

    if (currentlyPressed && !btnPressed) {
      btnPressed = true;
      if (motorState == "stop" || motorState == "bwd") {
        systemStarted = true;
        setMotor("fwd", motorSpeed);
      } else {
        systemStarted = false;
        stopMotor();
      }
      broadcastStatus();
      Serial.printf("[BTN] Ditekan → Motor: %s | SystemStarted: %d\n", motorState.c_str(), systemStarted);
    } else if (!currentlyPressed && btnPressed) {
      btnPressed = false;
      Serial.println("[BTN] Dilepas");
      broadcastStatus();
    }
  }

  btnLastState = reading;
}

// ============================================================
//  MOTOR DC BTS7960
// ============================================================
void setMotor(String dir, int spd) {
  spd = constrain(spd, 0, 255);
  motorSpeed = spd;
  motorState = dir;

  if (dir == "fwd") {
    digitalWrite(PIN_MOTOR_EN, HIGH);
    ledcWrite(PIN_MOTOR_LPWM, 0);
    ledcWrite(PIN_MOTOR_RPWM, spd);
    Serial.printf("[BTS7960] Maju, speed=%d\n", spd);
  } else if (dir == "bwd") {
    digitalWrite(PIN_MOTOR_EN, HIGH);
    ledcWrite(PIN_MOTOR_RPWM, 0);
    ledcWrite(PIN_MOTOR_LPWM, spd);
    Serial.printf("[BTS7960] Mundur, speed=%d\n", spd);
  } else {
    stopMotor();
    return;
  }
  updateLCD();
  broadcastStatus();  // langsung broadcast agar slider di web langsung update
}

void stopMotor() {
  ledcWrite(PIN_MOTOR_RPWM, 0);
  ledcWrite(PIN_MOTOR_LPWM, 0);
  digitalWrite(PIN_MOTOR_EN, LOW); // Matikan enable untuk efisiensi & proteksi
  motorState = "stop";
  motorKickUntil = 0;
  motorReKickUntil = 0;
  Serial.println("[BTS7960] Stop");
  updateLCD();
}

// ============================================================
//  UART2 HANDLER — Terima data JSON dari ESP32 ke-2
//  Format masuk: {"f":0,"u1":0,"u2":0,"d1":...,"dA":...,"dB":...,"dC":...,"up":...}\n
//  f  = flame (1=api), u1 = US1 anomali (1=ada), u2 = US2 door (0-3)
// ============================================================
void handleUART2(uint32_t now) {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      uart2Buffer.trim();
      if (uart2Buffer.length() > 2) {
        StaticJsonDocument<384> doc2;
        DeserializationError err = deserializeJson(doc2, uart2Buffer);
        if (!err) {
          uart2LastRxTime = now;
          uart2PacketCount++;

          if (uart2PacketCount <= 5 || uart2PacketCount % 40 == 0) {
            Serial.printf("[UART2] RX #%u: %s\n", uart2PacketCount, uart2Buffer.c_str());
          }

          bool    newFlame = ((int)doc2["f"]  == 1);
          bool    newU1    = ((int)doc2["u1"] == 1);
          uint8_t newU2    = (uint8_t)((int)doc2["u2"]);

          if (doc2.containsKey("d1")) esp32_2_us1_dist  = (float)doc2["d1"];
          if (doc2.containsKey("dA")) esp32_2_us2_distA = (float)doc2["dA"];
          if (doc2.containsKey("dB")) esp32_2_us2_distB = (float)doc2["dB"];
          if (doc2.containsKey("dC")) esp32_2_us2_distC = (float)doc2["dC"];

          // ── Flame Alert ──────────────────────────────────
          if (newFlame && !flameAlert2) {
            flameAlert2   = true;
            safetyLockout = true;
            stopMotor();
            ledcWrite(PIN_BUZZER, 128);
            setRGBBlink(COL_FLAME_A, COL_FLAME_B);
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("!! API / FIRE !!");
            lcd.setCursor(0, 1); lcd.print("Safety Lockout  ");
            Serial.println("[UART2] FLAME ALERT! API terdeteksi dari ESP32-2. Safety Lockout aktif.");
            broadcastStatus();
          } else if (!newFlame && flameAlert2) {
            flameAlert2 = false;
            if (!gasAlert && !waterAlert) {
              stopBuzzer(); setRGBOff();
              lcd.clear(); lcd.setCursor(0, 0); lcd.print("Api padam/aman");
              lcdClearPending = true; lcdClearTimer = now;
            }
            Serial.println("[UART2] Api padam/tidak terdeteksi.");
            broadcastStatus();
          }

          // ── US1: Limbah Anomali (rising edge) ────────────
          if (newU1 && !us1WasActive) {
            anomalyCount++;
            Serial.printf("[UART2] US1: Limbah anomali #%u terdeteksi!\n", anomalyCount);
            broadcastStatus();
          }
          us1WasActive = newU1;

          // ── US2: Konfirmasi Sortir (rising edge) ─────────
          if (newU2 > 0 && !us2WasActive) {
            // Atribusikan ke servo yang paling relevan
            int sid = (lastSortedServo > 0) ? lastSortedServo
                    : (lastActiveServo  > 0) ? lastActiveServo : (int)newU2;
            if (sid >= 1 && sid <= 3) {
              sortConfirm[sid]++;
              Serial.printf("[UART2] US2: Limbah tersortir di Pintu %d (Servo %d) — Total: %u\n",
                            newU2, sid, sortConfirm[sid]);
            }
            broadcastStatus();
          }
          us2WasActive = (newU2 > 0);
          us2LastDoor  = newU2;
        } else {
          Serial.printf("[UART2] JSON Parse Error: %s (Raw: %s)\n", err.c_str(), uart2Buffer.c_str());
        }
      }
      uart2Buffer = "";
    } else if (c != '\r' && uart2Buffer.length() < 256) {
      uart2Buffer += c;
    }
  }
}

// ============================================================
//  HARDWARE SCAN & DIAGNOSTICS
// ============================================================
void performHardwareScan() {
  Serial.println("\n[HW SCAN] ========================================");
  Serial.println("[HW SCAN]   MEMULAI PEMINDAIAN HARDWARE LENGKAP   ");
  Serial.println("[HW SCAN] ========================================");

  // 1. Scan Bus I2C untuk Display LCD 16x2
  Wire.beginTransmission(0x27);
  byte error = Wire.endTransmission();
  if (error == 0) {
    lcdDetected = true;
    Serial.println("[HW SCAN] [✓] LCD 16x2 I2C: Terdeteksi pada address 0x27.");
  } else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) {
      lcdDetected = true;
      Serial.println("[HW SCAN] [✓] LCD 16x2 I2C: Terdeteksi pada address 0x3F.");
    } else {
      lcdDetected = false;
      Serial.println("[HW SCAN] [✖] LCD 16x2 I2C: TIDAK MERESPON (Periksa SDA GPIO21 / SCL GPIO22).");
    }
  }

  // 2. Kirim sinyal probe/ping ke ESP32 ke-2 via UART2 TX (GPIO 25)
  Serial2.println("{\"cmd\":\"ping\"}");
  Serial.printf("[HW SCAN] [✓] Mengirim ping probe ke ESP32-2 via UART TX GPIO%d (9600 baud)...\n", PIN_UART2_TX);

  // 3. Baca Sensor Master (ADC & Digital)
  analogRead(PIN_GAS_AOUT);
  delay(2);
  int gAdc = analogRead(PIN_GAS_AOUT);
  int wAdc = analogRead(PIN_WATER_AOUT);
  int btnVal = digitalRead(PIN_BUTTON);
  Serial.printf("[HW SCAN] [✓] Sensor MQ-2 Gas   : ADC=%d (Baseline=%d, Ready=%d)\n", gAdc, mq2Baseline, mq2WarmupDone ? 1 : 0);
  Serial.printf("[HW SCAN] [✓] Sensor Water Level : ADC=%d (Alert=%d)\n", wAdc, waterAlert ? 1 : 0);
  Serial.printf("[HW SCAN] [✓] Push Button GPIO 5 : %s\n", (btnVal == LOW) ? "PRESSED" : "STANDBY (HIGH)");

  // 4. Aktuator Verification
  Serial.println("[HW SCAN] [✓] Driver Motor BTS7960: RPWM (Ch4), LPWM (Ch5), EN (GPIO16) 5kHz Aktif.");
  Serial.printf("[HW SCAN] [✓] 3x Servo SG90       : S1=%d°, S2=%d°, S3=%d° (Attached 50Hz PWM).\n", servoAngle1, servoAngle2, servoAngle3);
  Serial.println("[HW SCAN] [✓] Buzzer & RGB LED   : Ch6-9 PWM Hardware Timer Siap.");

  // Hitung total perangkat yang berhasil terdeteksi
  bool esp2Online = (millis() - uart2LastRxTime < 3500) && (uart2LastRxTime > 0);
  int detected = 0;
  if (esp2Online) detected += 4; // Node ESP32-2, Flame Sensor, US1 Anomali, US2 Sortir 3 Pintu
  detected += 3;                 // MQ-2, Water Level, Button
  if (lcdDetected) detected++;   // LCD 16x2
  detected += 4;                 // Motor BTS7960, 3x Servo, Buzzer & RGB

  Serial.printf("[HW SCAN] Status ESP32 ke-2: %s (Paket: %u, Terakhir: %lums lalu)\n",
                esp2Online ? "ONLINE & TERHUBUNG" : "OFFLINE / TIDAK ADA SINYAL",
                uart2PacketCount,
                (uart2LastRxTime > 0) ? (millis() - uart2LastRxTime) : 0);
  Serial.printf("[HW SCAN] Ringkasan: %d / 12 Perangkat Berhasil Diverifikasi.\n", detected);
  Serial.println("[HW SCAN] ========================================\n");

  StaticJsonDocument<1024> sdoc;
  sdoc["type"]           = "hardware_scan";
  sdoc["total_detected"] = detected;
  sdoc["esp2_online"]    = esp2Online;
  sdoc["esp2_rx_pkts"]   = uart2PacketCount;
  sdoc["esp2_last_rx_ms"]= (uart2LastRxTime > 0) ? (millis() - uart2LastRxTime) : 99999;
  sdoc["flameAlert2"]    = flameAlert2;
  sdoc["d1"]             = esp32_2_us1_dist;
  sdoc["dA"]             = esp32_2_us2_distA;
  sdoc["dB"]             = esp32_2_us2_distB;
  sdoc["dC"]             = esp32_2_us2_distC;
  sdoc["gasADC"]         = gAdc;
  sdoc["waterADC"]       = wAdc;
  sdoc["mq2Ready"]       = mq2WarmupDone;
  sdoc["btnPressed"]     = (btnVal == LOW);
  sdoc["lcd_detected"]   = lcdDetected;
  sdoc["motorState"]     = motorState;
  sdoc["motorSpeed"]     = motorSpeed;
  sdoc["servo1"]         = servoAngle1;
  sdoc["servo2"]         = servoAngle2;
  sdoc["servo3"]         = servoAngle3;
  sdoc["gasAlert"]       = gasAlert;
  sdoc["waterAlert"]     = waterAlert;

  String out;
  serializeJson(sdoc, out);
  webSocket.broadcastTXT(out);
}

// ============================================================
//  WEBSOCKET EVENT
// ============================================================
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

  // ── Hardware Scan Command ──
  if (strcmp(cmd, "scan_hardware") == 0) {
    performHardwareScan();
    return;
  }

  // ── Servo command ──
  if (strcmp(cmd, "servo") == 0) {
    int id    = doc["id"] | 0;
    int angle = constrain((int)(doc["angle"] | 0), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    if (id >= 1 && id <= 3) {
      if (doc.containsKey("waste")) {
        lastWasteName = doc["waste"].as<String>();
      }
      if (doc.containsKey("cat")) {
        lastWasteCategory = doc["cat"].as<String>();
      }

      if (id == 1) {
        if (!servo1.attached()) servo1.attach(PIN_SERVO1, SERVO_MIN_US, SERVO_MAX_US);
        servoAngle1 = angle;
        servo1.write(angle);
        Serial.printf("[WS] Servo1 → %d° (%s)\n", angle, lastWasteName.c_str());
      }
      else if (id == 2) {
        if (!servo2.attached()) servo2.attach(PIN_SERVO2, SERVO_MIN_US, SERVO_MAX_US);
        servoAngle2 = angle;
        servo2.write(angle);
        delay(10);
        Serial.printf("[WS] Servo2 → %d° (%s)\n", angle, lastWasteName.c_str());
      }
      else if (id == 3) {
        if (!servo3.attached()) servo3.attach(PIN_SERVO3, SERVO_MIN_US, SERVO_MAX_US);
        servoAngle3 = angle;
        servo3.write(angle);
        delay(10);
        Serial.printf("[WS] Servo3 → %d° (%s)\n", angle, lastWasteName.c_str());
      }

      if (angle > 0) {
        lastActiveServo = id;
        // Jika ada nama waste / auto-sortir: jadwalkan auto-close agar tidak macet terbuka terus
        if (doc.containsKey("waste") && lastWasteName != "Tidak Ada") {
          servoJobs[id].stage       = SERVO_HOLDING_OPEN;
          servoJobs[id].closeAt     = millis() + SERVO_HOLD_MS;
          servoJobs[id].targetAngle = angle;
          servoJobs[id].waste       = lastWasteName;
          servoJobs[id].cat         = lastWasteCategory;
        } else {
          servoJobs[id].stage       = SERVO_IDLE;
        }
      } else {
        servoJobs[id].stage = SERVO_IDLE;
        if (id == lastActiveServo) lastActiveServo = 0;
      }
    }

    updateLCD();
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "set_mode") == 0) {
    conveyorMode = doc["mode"] | 0;
    systemStarted = false;
    stopMotor();
    Serial.printf("[WS] Mode diubah ke: %d. Menunggu tombol START.\n", conveyorMode);
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    systemStarted = false;
    stopMotor();
    lessEnergyStopAt = 0;
    for (int i = 1; i <= 3; i++) {
      servoJobs[i].stage = SERVO_IDLE;
    }
    servo1.write(0); servoAngle1 = 0;
    servo2.write(0); servoAngle2 = 0;
    servo3.write(0); servoAngle3 = 0;
    lastActiveServo   = 0;
    lastWasteName     = "Tidak Ada";
    lastWasteCategory = "-";
    Serial.println("[WS] Sistem di-STOP dari web. Motor dimatikan & servo ditutup.");
    updateLCD();
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "start") == 0 || strcmp(cmd, "start_system") == 0) {
    if (!safetyLockout && !gasAlert && !waterAlert) {
      systemStarted = true;
      if (doc.containsKey("mode")) {
        conveyorMode = doc["mode"] | 0;
      }
      Serial.printf("[WS] Sistem di-START dari web (Mode: %d).\n", conveyorMode);
      if (conveyorMode == 0) {
        startConveyor();
        Serial.println("[BTS7960] Keep Going: Konveyor mulai berjalan.");
      } else {
        stopMotor();
        lessEnergyStopAt = 0;
        Serial.println("[BTS7960] Less Energy: Stand-by.");
      }
    } else {
      Serial.println("[WS] START ditolak: alert bahaya aktif atau safety lockout!");
    }
    broadcastStatus();
    return;
  }

  if (strcmp(cmd, "resume") == 0) {
    safetyLockout = false;
    gasAlert      = false;
    waterAlert    = false;
    flameAlert2   = false;   // Reset alert api dari ESP32 ke-2
    systemStarted = true;
    
    Serial.println("[WS] Safety Lockout direset. Sistem dilanjutkan.");
    
    if (conveyorMode == 0) {
      startConveyor();
      Serial.println("[BTS7960] Keep Going: Konveyor mulai berjalan (Resume).");
    } else {
      stopMotor();
      lessEnergyStopAt = 0;
      Serial.println("[BTS7960] Less Energy: Stand-by (Resume).");
    }
    
    broadcastStatus();
    return;
  }

  // ── Motor command (manual dir) ──
  if (strcmp(cmd, "motor") == 0) {
    const char* dir = doc["dir"] | "stop";
    int spd = doc.containsKey("speed") ? constrain((int)doc["speed"], 0, 255) : motorSpeed;

    if ((gasAlert || waterAlert) && strcmp(dir, "stop") != 0) {
      Serial.println("[BTS7960] Ditolak — ada alert bahaya aktif!");
      broadcastStatus();
      return;
    }

    if (strcmp(dir, "stop") == 0) {
      stopMotor();
      motorSpeed = spd;  // simpan speed meski motor stop
      Serial.printf("[BTS7960] Motor Stop. Kecepatan tersimpan: %d\n", motorSpeed);
      broadcastStatus();
    } else {
      setMotor(String(dir), spd);  // setMotor sudah broadcastStatus di dalamnya
    }
    return;
  }

  // ── Set Speed (ubah kecepatan tanpa ubah arah) ──
  // Digunakan oleh slider kecepatan di web agar tidak tergantung motorState client
  if (strcmp(cmd, "set_speed") == 0) {
    int spd = constrain((int)(doc["speed"] | motorSpeed), 0, 255);
    motorSpeed = spd;
    if (motorState == "fwd") {
      ledcWrite(PIN_MOTOR_RPWM, spd);
      Serial.printf("[BTS7960] Speed diubah (Maju): %d\n", spd);
    } else if (motorState == "bwd") {
      ledcWrite(PIN_MOTOR_LPWM, spd);
      Serial.printf("[BTS7960] Speed diubah (Mundur): %d\n", spd);
    } else {
      Serial.printf("[BTS7960] Speed tersimpan: %d (Motor Stop)\n", spd);
    }
    updateLCD();
    broadcastStatus();
    return;
  }

  // ── Test command ──
  if (strcmp(cmd, "test") == 0) {
    const char* target = doc["target"] | "";
    if (strcmp(target, "buzzer") == 0) {
      // Test buzzer: 1 detik bunyi, lalu stop otomatis
      ledcWrite(PIN_BUZZER, 180);
      testBuzzerUntil = millis() + 1000;
      Serial.println("[TEST] Buzzer test aktif (1 detik)");
    } else if (strcmp(target, "flame") == 0) {
      // Test flame: simulasi api aktif 3 detik
      testFlameUntil = millis() + 3000;
      flameAlert2 = true;
      Serial.println("[TEST] Simulasi sensor api aktif (3 detik)");
    } else if (strcmp(target, "gas") == 0) {
      // Test gas: HANYA simulasi nilai ADC untuk display — TIDAK set gasAlert/safetyLockout
      testGasUntil = millis() + 4000;
      Serial.println("[TEST] Simulasi sensor gas aktif 4 detik (display only, no alarm).");
    } else if (strcmp(target, "water") == 0) {
      // Test water: HANYA simulasi nilai ADC untuk display — TIDAK set waterAlert/safetyLockout
      testWaterUntil = millis() + 4000;
      Serial.println("[TEST] Simulasi sensor air aktif 4 detik (display only, no alarm).");
    } else if (strcmp(target, "motor") == 0) {
      // Test motor: maju 2s → stop 1s → mundur 2s → stop
      int spd = (motorSpeed > 0) ? motorSpeed : 180;
      setMotor("fwd", spd);
      testMotorStep  = 1;
      testMotorUntil = millis() + 2000;
      Serial.println("[TEST] Motor test: Maju 2s -> Jeda 1s -> Mundur 2s");
    }
    broadcastStatus();
    return;
  }
}

// ============================================================
//  STATUS JSON
// ============================================================
String buildStatusJson() {
  StaticJsonDocument<1024> doc;
  doc["type"]          = "status";
  doc["gasAlert"]      = gasAlert;
  doc["waterAlert"]    = waterAlert;
  doc["flameAlert2"]   = flameAlert2;      // Api dari ESP32 ke-2
  doc["gasADC"]        = lastGasVal;
  doc["waterADC"]      = lastWaterVal;
  doc["servo1"]        = servoAngle1;
  doc["servo2"]        = servoAngle2;
  doc["servo3"]        = servoAngle3;
  doc["wasteName"]     = lastWasteName;
  doc["wasteCat"]      = lastWasteCategory;
  doc["servoActive"]   = lastActiveServo;
  doc["mq2Ready"]      = mq2WarmupDone;
  doc["mq2Baseline"]   = mq2Baseline;
  doc["mq2ThreshOn"]   = mq2ThreshOn;
  doc["motorState"]    = motorState;
  doc["motorSpeed"]    = motorSpeed;
  doc["btnPressed"]    = btnPressed;
  doc["mode"]          = conveyorMode;
  doc["safetyLockout"] = safetyLockout;
  doc["systemStarted"] = systemStarted;
  doc["anomalyCount"]  = anomalyCount;     // Limbah anomali (US1 ESP32-2)
  doc["sortConfirm1"]  = sortConfirm[1];  // Konfirmasi sortir Servo 1
  doc["sortConfirm2"]  = sortConfirm[2];  // Konfirmasi sortir Servo 2
  doc["sortConfirm3"]  = sortConfirm[3];  // Konfirmasi sortir Servo 3

  // ── Telemetri Hardware & UART ESP32-2 ──
  doc["esp2_online"]    = (millis() - uart2LastRxTime < 3500) && (uart2LastRxTime > 0);
  doc["esp2_last_rx_ms"]= (uart2LastRxTime > 0) ? (millis() - uart2LastRxTime) : 99999;
  doc["esp2_rx_pkts"]   = uart2PacketCount;
  doc["lcd_detected"]   = lcdDetected;
  doc["d1"]             = esp32_2_us1_dist;
  doc["dA"]             = esp32_2_us2_distA;
  doc["dB"]             = esp32_2_us2_distB;
  doc["dC"]             = esp32_2_us2_distC;
  doc["us2Door"]        = (int)us2LastDoor;  // Pintu US2 aktif saat ini (0=none, 1=A, 2=B, 3=C)

  String out;
  serializeJson(doc, out);
  return out;
}

void broadcastStatus() {
  String json = buildStatusJson();
  webSocket.broadcastTXT(json);
}

// ============================================================
//  GAS SENSOR MQ-2 (KALIBRASI DINAMIS & FIXED DEBOUNCE)
// ============================================================
void handleGasSensor(uint32_t now) {
  if (!mq2WarmupDone) {
    uint32_t elapsed = now - mq2WarmupStart;
    
    if (elapsed > 3000) {
      analogRead(PIN_GAS_AOUT);
      delayMicroseconds(200);
      mq2BaselineSum += analogRead(PIN_GAS_AOUT);
      mq2BaselineCount++;
    }

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
    if (mq2BaselineCount > 0) {
      mq2Baseline = (int)(mq2BaselineSum / mq2BaselineCount);
    } else {
      mq2Baseline = 500;
    }
    mq2ThreshOn  = max(mq2Baseline + MQ2_DELTA_ON, 900);
    mq2ThreshOff = max(mq2Baseline + MQ2_DELTA_OFF, 650);

    Serial.println("\n[MQ-2] ========================================");
    Serial.printf("[MQ-2] Warm-up Selesai & Terkalibrasi!\n");
    Serial.printf("[MQ-2] Baseline Udara Bersih : %d\n", mq2Baseline);
    Serial.printf("[MQ-2] Ambang Batas Bahaya   : %d\n", mq2ThreshOn);
    Serial.printf("[MQ-2] Ambang Batas Normal   : %d\n", mq2ThreshOff);
    Serial.println("[MQ-2] ========================================\n");
    updateLCD();
    return;
  }

  if (now - mq2LastRead < MQ2_READ_INTERVAL) return;
  mq2LastRead = now;

  if (testGasUntil > 0) {
    if (now < testGasUntil) {
      // Mode simulasi: paksa nilai ADC tinggi untuk DISPLAY saja — tidak trigger gasAlert
      lastGasVal = 3800;
      lastGasD0  = false;   // jangan aktifkan D0 agar tidak masuk logika alarm
      gasDoutLowSince = 0;
    } else {
      // Simulasi selesai: reset nilai, jangan tinggalkan jejak alarm
      testGasUntil = 0;
      lastGasD0    = false;
      gasDoutLowSince = 0;
      gasAlert     = false; // pastikan tidak ada alarm tersisa
      Serial.println("[TEST] Gas sensor test selesai. Kembali ke pembacaan normal.");
    }
    broadcastStatus();
    return; // skip logika alarm selama test
  }

  analogRead(PIN_GAS_AOUT);
  delayMicroseconds(200);
  long sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(PIN_GAS_AOUT);
    delayMicroseconds(100);
  }
  lastGasVal = (int)(sum / 4);

  bool rawD0 = (digitalRead(PIN_GAS_DOUT) == LOW);
  if (rawD0) {
    if (gasDoutLowSince == 0) {
      gasDoutLowSince = now;
    }
  } else {
    gasDoutLowSince = 0;
  }
  // D0 harus stabil LOW minimal 1200ms agar kebal hembusan gas korek sesaat
  lastGasD0 = rawD0 && (gasDoutLowSince > 0) && (now - gasDoutLowSince >= 1200);

  // Cross-inhibition: jika Flame Alert aktif (ada api), tahan alarm gas/asap agar tidak dobel/salah deteksi
  bool isDanger = !flameAlert2 && (lastGasD0 || (lastGasVal >= mq2ThreshOn) || (lastGasVal >= MQ2_ABS_MAX_ON));
  bool isSafe   = (!lastGasD0) && (lastGasVal <= mq2ThreshOff);

  if (!gasAlert && isDanger) {
    gasAlert = true;
    safetyLockout = true;
    stopMotor();
    ledcWrite(PIN_BUZZER, 128);
    setRGBBlink(COL_GAS_A, COL_GAS_B);
    lcd.clear(); 
    lcd.setCursor(0, 0); lcd.print("!! BAHAYA ASAP!!");
    lcd.setCursor(0, 1); lcd.print("ADC:"); lcd.print(lastGasVal);
    Serial.printf("[MQ-2 ALARM] 🚨 BAHAYA ASAP TERDETEKSI! ADC: %d (Thresh: %d), D0: %d\n",
      lastGasVal, mq2ThreshOn, lastGasD0);
  } else if (gasAlert && isSafe) {
    gasAlert = false;
    Serial.printf("[MQ-2 NORMAL] 🟢 Udara kembali bersih. ADC: %d\n", lastGasVal);
    if (!waterAlert) {
      stopBuzzer(); 
      setRGBOff();
      lcd.clear(); 
      lcd.setCursor(0, 0); lcd.print("Udara bersih.");
      lcdClearPending = true; 
      lcdClearTimer = now;
    }
  }
}

// ============================================================
//  WATER LEVEL SENSOR
// ============================================================
void handleWaterSensor(uint32_t now) {
  if (!mq2WarmupDone) return;
  if (now - waterLastRead < WATER_READ_INTERVAL) return;
  waterLastRead = now;

  if (testWaterUntil > 0) {
    if (now < testWaterUntil) {
      // Mode simulasi: paksa nilai ADC tinggi untuk DISPLAY saja — tidak trigger waterAlert
      lastWaterVal = 3500;
    } else {
      // Simulasi selesai: reset nilai, pastikan tidak ada alarm tersisa
      testWaterUntil = 0;
      waterAlert     = false;
      Serial.println("[TEST] Water sensor test selesai. Kembali ke pembacaan normal.");
    }
    broadcastStatus();
    return; // skip logika alarm selama test
  }

  analogRead(PIN_WATER_AOUT);
  delay(2);
  lastWaterVal = analogRead(PIN_WATER_AOUT);

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
    if (!gasAlert) {
      stopBuzzer(); setRGBOff();
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Air aman.");
      lcdClearPending = true; lcdClearTimer = now;
    }
  }
}

// ============================================================
//  BUZZER
// ============================================================
void stopBuzzer() {
  ledcWrite(PIN_BUZZER, 0);
}

// ============================================================
//  START CONVEYOR
// ============================================================
void startConveyor() {
  setMotor("fwd", motorSpeed);
  Serial.printf("[BTS7960] Konveyor mulai berjalan (Speed: %d)\n", motorSpeed);
}

// ============================================================
//  RGB LED
// ============================================================
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

// ============================================================
//  LCD
// ============================================================
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
