<div align="center">

# 🏥 AIoT Medical Waste Sorting Systems

**Sistem Deteksi & Pemilah Limbah Medis Cerdas Berbasis AI Computer Vision & IoT**

[![ESP32](https://img.shields.io/badge/Microcontroller-ESP32-blue?style=for-the-badge&logo=espressif)](https://www.espressif.com/)
[![YOLOv8](https://img.shields.io/badge/AI_Model-YOLOv8-yellow?style=for-the-badge)](https://ultralytics.com/)
[![OpenCV](https://img.shields.io/badge/Library-OpenCV-green?style=for-the-badge&logo=opencv)](https://opencv.org/)
[![Python](https://img.shields.io/badge/Language-Python_3-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Arduino](https://img.shields.io/badge/Language-Arduino_C++-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)

</div>

---

## 📌 Deskripsi Proyek

Proyek ini adalah purwarupa (prototipe) **Sistem Pemilah Limbah Medis Otomatis** yang mengintegrasikan *Computer Vision* berbasis AI dengan mikrokontroler ESP32. Sistem dirancang untuk mendeteksi berbagai jenis limbah medis secara *real-time* menggunakan kamera dan model YOLOv8, lalu secara otomatis memisahkannya ke dalam kategori yang tepat:

| Kategori | Tempat | Warna Kantong |
|---|---|---|
| 🟡 Limbah Infeksius | Gerbang Servo 1 | Kantong **KUNING** |
| ⚫ Limbah Non-Infeksius | Gerbang Servo 2 | Kantong **HITAM** |
| 🔴 Limbah B3 (Berbahaya) | Gerbang Servo 3 | Kantong **MERAH** |

Sistem juga dilengkapi sensor keamanan untuk mencegah risiko bahaya di area pembuangan limbah.

---

## ✨ Fitur Utama

### 🤖 AI Computer Vision
- **YOLOv8** untuk deteksi & klasifikasi limbah medis secara *real-time*
- **Face Detection (Haarcascade)**: *Conveyor* menyala otomatis saat operator terdeteksi di depan mesin, dan berhenti saat operator pergi
- Mendukung model **Object Detection** maupun **Classification**
- Throttle inferensi untuk menjaga performa kamera tetap mulus

### ⚙️ Sistem Pemilahan Otomatis
- **Motor DC + L298N Driver**: Penggerak sabuk *conveyor*
- **3x Motor Servo**: Membuka gerbang pemilah ke jalur yang sesuai
- Logika *hold-and-release* — servo terbuka selama 3 detik setelah objek terdeteksi
- Kendali motor dari web panel dengan slider kecepatan (0–255 / PWM)

### 🚨 Sensor Keselamatan & Peringatan Dini
| Sensor | Pin | Fungsi |
|---|---|---|
| MQ-2 Gas/Asap | GPIO 26 (Analog) | Deteksi kebocoran gas / asap |
| Flame Sensor | GPIO 34 (Analog), GPIO 13 (Digital) | Deteksi api |
| Water Level | GPIO 14 (Analog) | Deteksi luapan cairan |

**Fail-Safe Mechanism:**
- Motor *conveyor* berhenti otomatis saat bahaya terdeteksi
- **Passive Buzzer** berbunyi dengan frekuensi 2kHz (nyaring!)
- **RGB LED** berkedip dengan warna berbeda tiap jenis bahaya
- Tampilan LCD 16x2 menampilkan notifikasi bahaya

### 🌐 Web Dashboard Control Panel
Antarmuka web modern diakses langsung via browser — tidak perlu instal aplikasi apapun.

**Mode User (Tanpa Password):**
- Tampilan *Live View* status sistem
- Status sensor gas, air, api, dan motor secara real-time

**Mode Admin (`Admin` / `Admin123`):**
- Kontrol penuh Motor DC (Maju / Mundur / Stop + Speed Slider)
- Kontrol manual semua Servo (0°–180°)
- Monitoring ADC sensor secara detail
- **Hardware Diagnostics (Test Mode):** Tes semua *hardware* langsung dari web:
  - 🎯 Simulasi Deteksi YOLO
  - 🔊 Test Buzzer (1 detik)
  - ⚙️ Test Motor Driver (urutan otomatis: Maju → Stop → Mundur)
  - 💨 Test Sensor Gas / Air / Api

---

## 🗺️ Peta Pin ESP32

```
┌─────────────────────┬──────────┬─────────────────────────┐
│ Komponen            │ Pin ESP32│ Catatan                 │
├─────────────────────┼──────────┼─────────────────────────┤
│ LCD I2C SDA         │ 21       │                         │
│ LCD I2C SCL         │ 22       │                         │
│ Passive Buzzer      │ 32       │ tone() / noTone()       │
│ Push Button         │  5       │ INPUT_PULLUP            │
│ Servo 1 (Infeksius) │ 33       │ 50Hz PWM                │
│ Servo 2 (Non-Inf.)  │ 19       │ 50Hz PWM                │
│ Servo 3 (B3)        │ 18       │ 50Hz PWM                │
│ MQ-2 AOUT           │ 26       │ Analog                  │
│ Water Level AOUT    │ 14       │ Analog (ADC1_CH6)       │
│ Flame Sensor AOUT   │ 34       │ Analog                  │
│ Flame Sensor DOUT   │ 13       │ Digital                 │
│ RGB LED - Red       │ 15       │ PWM (ledcAttach)        │
│ RGB LED - Green     │  2       │ PWM (ledcAttach)        │
│ RGB LED - Blue      │ 23       │ PWM (ledcAttach)        │
│ L298N IN1           │  4       │ Arah motor              │
│ L298N IN2           │ 17       │ Arah motor              │
│ L298N ENA           │ 16       │ PWM kecepatan           │
└─────────────────────┴──────────┴─────────────────────────┘
```

---

## 💻 Struktur Proyek

```text
📦 Code and all/
 ┣ 📂 Python/
 ┃ ┣ 📜 main_yolo.py          ← Script utama: streaming kamera + inferensi YOLO + kirim perintah ke ESP32
 ┃ ┣ 📜 train_yolo.py         ← Script untuk melatih model YOLOv8 kustom
 ┃ ┣ 📜 label_manual.py       ← Tool untuk labeling data manual
 ┃ ┗ 📂 waste_model/
 ┃    ┗ 📂 weights/
 ┃       ┗ 📜 best.pt         ← Bobot model AI hasil training (tidak di-commit ke Git)
 ┣ 📂 Esp32/
 ┃ ┗ 📜 Esp32.ino             ← Firmware ESP32 utama (Web Server, WebSocket, Sensor, Motor, Servo)
 ┣ 📂 Backup/
 ┃ ┣ 📜 Esp32_BACKUP.ino      ← Backup firmware ESP32
 ┃ ┗ 📜 Esp32CAM_BACKUP.ino  ← Firmware ESP32-CAM (TCP streaming video)
 ┗ 📜 README.md               ← Dokumentasi ini
```

---

## ⚙️ Dependensi & Library

### Arduino / ESP32 (Arduino IDE)
| Library | Keterangan |
|---|---|
| `ESP32Servo` | Kontrol servo pada ESP32 |
| `WebSocketsServer` (by Markus Sattler) | Server WebSocket untuk komunikasi real-time |
| `ArduinoJson` | Parsing & serialisasi data JSON |
| `LiquidCrystal_I2C` | Kontrol LCD I2C 16x2 |
| `WiFi`, `WebServer` | Built-in library Arduino ESP32 |

> **Board:** `esp32` by Espressif Systems (v3.x direkomendasikan)

### Python
```bash
pip install ultralytics opencv-python websocket-client numpy
```
| Library | Versi | Keterangan |
|---|---|---|
| `ultralytics` | ≥8.0 | Framework YOLOv8 |
| `opencv-python` | ≥4.5 | Pemrosesan gambar & video |
| `websocket-client` | ≥1.0 | Komunikasi WebSocket ke ESP32-CAM |
| `numpy` | ≥1.21 | Manipulasi array gambar |

---

## 🚀 Panduan Penggunaan

### Langkah 1: Setup ESP32 & ESP32-CAM
1. Install **Arduino IDE** dan tambahkan board ESP32 (URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`).
2. Install semua library yang dibutuhkan dari Library Manager.
3. Buka `Esp32/Esp32.ino`, ubah nilai berikut:
   ```cpp
   #define WIFI_SSID   "NamaWiFiAnda"
   #define WIFI_PASS   "PasswordWiFiAnda"
   ```
4. *Upload* firmware ke ESP32. Catat **IP Address** yang muncul di Serial Monitor.
5. *Upload* firmware kamera (`Esp32CAM_BACKUP.ino`) ke ESP32-CAM.

### Langkah 2: Sesuaikan IP di Script Python
Buka `Python/main_yolo.py` dan sesuaikan:
```python
CAM_IP       = "192.168.x.x"   # IP Address ESP32-CAM
SMART_IP     = "192.168.x.x"   # IP Address ESP32 Utama
```

### Langkah 3: Jalankan Program
```bash
cd "Python"
python main_yolo.py
```

**Kontrol Keyboard:**
| Key | Fungsi |
|---|---|
| `S` | Screenshot manual frame saat ini |
| `A` | Toggle mode Auto-Capture untuk mengumpulkan dataset |
| `Q` | Keluar dari program |

### Langkah 4: Akses Web Dashboard
Buka browser dan masuk ke: **`http://<IP-ESP32>/`**

- **Login User**: Klik "Masuk sebagai Pengguna"
- **Login Admin**: Tab Admin → Username: `Admin` | Password: `Admin123`

---

## 🎓 Cara Melatih Model AI Sendiri

1. Kumpulkan gambar sampel limbah menggunakan mode **Auto-Capture** (`A` key) di `main_yolo.py`.
2. Label gambar menggunakan `label_manual.py` atau tools seperti **Roboflow**.
3. Latih model:
   ```bash
   python train_yolo.py
   ```
4. Bobot model terbaik akan tersimpan di `waste_model/weights/best.pt`.

---

## 📡 Arsitektur Sistem

```
┌─────────────┐    TCP Streaming    ┌──────────────┐
│  ESP32-CAM  │ ─────────────────► │  Laptop PC   │
│  (Kamera)   │                    │  main_yolo.py│
└─────────────┘                    │  YOLOv8 AI   │
                                   └──────┬───────┘
                                          │ HTTP GET /api
                                          │ (cmd=servo, cmd=machine)
                                          ▼
┌─────────────────────────────────────────────────────┐
│               ESP32 Utama (Main Controller)          │
│  ┌──────────┐  ┌───────────┐  ┌─────────────────┐  │
│  │ 3x Servo │  │ Motor DC  │  │  Sensor Suite   │  │
│  │(Pemilah) │  │(Conveyor) │  │ MQ2, Flame, Air │  │
│  └──────────┘  └───────────┘  └─────────────────┘  │
│  ┌──────────┐  ┌───────────┐  ┌─────────────────┐  │
│  │ LCD 16x2 │  │ RGB LED   │  │ Passive Buzzer  │  │
│  └──────────┘  └───────────┘  └─────────────────┘  │
│                      ▲                              │
│               WebSocket (Port 81)                   │
│                      │                              │
│              ┌───────────────┐                      │
│              │  Web Browser  │                      │
│              │  (Dashboard)  │                      │
│              └───────────────┘                      │
└─────────────────────────────────────────────────────┘
```

---

## 🤝 Kontribusi

Proyek ini dikembangkan untuk kebutuhan kompetisi **CNC HIMTIKA**. Silakan lakukan *fork* dan modifikasi sesuai kebutuhan!

---

> *Proyek ini dibangun untuk mengatasi permasalahan penyortiran limbah medis secara manual yang berisiko tinggi bagi petugas kebersihan, dengan memberikan sistem nirsentuh cerdas yang responsif.*
