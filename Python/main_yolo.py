"""
=============================================================
  main_yolo.py  —  Sistem Deteksi Limbah Medis (YOLOv8-cls)
=============================================================
  Jalankan train_yolo.py dulu sebelum file ini.

  Keyboard:
    S  = Screenshot manual
    A  = Toggle auto-capture
    F  = Toggle senter (flash) ON/OFF
    Q  = Keluar
=============================================================
"""

import socket
import cv2
import numpy as np
import struct
import websocket
import json
import time
import threading
import os
import urllib.request
import urllib.parse
from collections import deque
from datetime import datetime

# ─── Konfigurasi IP ──────────────────────────────────────────────────────────
CAM_IP       = "192.168.4.4"
CAM_TCP_PORT = 80
CAM_WS_PORT  = 81

SMART_IP      = "192.168.4.2"
SMART_WS_PORT = 81

# ─── Path ─────────────────────────────────────────────────────────────────────
BASE_DIR       = r"C:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Python"
SCREENSHOT_DIR = os.path.join(BASE_DIR, "training_data")
MODEL_PATH     = os.path.join(BASE_DIR, "waste_model", "weights", "best.pt")

os.makedirs(SCREENSHOT_DIR, exist_ok=True)

# ─── Threshold confidence ─────────────────────────────────────────────────────
# Turunkan ke 0.45 agar objek kecil/samar (seperti Plester, Tisu) bisa terdeteksi.
CONF_THRESHOLD = 0.45

# ─── Mapping kelas -> info tampilan ──────────────────────────────────────────
WASTE_INFO = {
    "Obat 1": {
        "kategori": "Limbah B3",
        "warna":    (0, 0, 255),        # Merah
        "aksi":     "Kantong MERAH",
    },
    "Obat 2": {
        "kategori": "Limbah B3",
        "warna":    (0, 0, 255),        # Merah
        "aksi":     "Kantong MERAH",
    },
    "Plester": {
        "kategori": "Limbah Infeksius",
        "warna":    (0, 165, 255),      # Oranye
        "aksi":     "Kantong KUNING",
    },
    "Kain Kasa": {
        "kategori": "Limbah Non-Infeksius",
        "warna":    (0, 200, 0),        # Hijau
        "aksi":     "Kantong HITAM",
    },
    "Kain kasa": {
        "kategori": "Limbah Non-Infeksius",
        "warna":    (0, 200, 0),        # Hijau
        "aksi":     "Kantong HITAM",
    },
    "Tisu Antiseptik": {
        "kategori": "Limbah Non-Infeksius",
        "warna":    (0, 200, 0),        # Hijau
        "aksi":     "Kantong HITAM",
    },
}

def get_waste_info(name):
    """Lookup WASTE_INFO dengan fallback case-insensitive."""
    # Hilangkan underscore (seperti "Kain_kasa" -> "Kain kasa") agar bisa dicocokkan
    name_clean = name.replace("_", " ")
    if name_clean in WASTE_INFO:
        return WASTE_INFO[name_clean]
    name_lower = name_clean.lower()
    for key, val in WASTE_INFO.items():
        if key.lower() == name_lower:
            return val
    return {
        "kategori": "Tidak Dikenal",
        "warna":    (128, 128, 128),
        "aksi":     "-",
    }

# ─── Antrian frame ────────────────────────────────────────────────────────────
frame_queue = deque(maxlen=2)
stop_event  = threading.Event()


# ════════════════════════════════════════════════════════════════════════════════
#  Load model YOLO classification
# ════════════════════════════════════════════════════════════════════════════════
def load_yolo_model():
    try:
        from ultralytics import YOLO
        if not os.path.exists(MODEL_PATH):
            print(f"[YOLO] Model belum ada: {MODEL_PATH}")
            print("[YOLO] Jalankan train_yolo.py dulu!")
            return None
        model = YOLO(MODEL_PATH)
        print(f"[YOLO] Model dimuat: {MODEL_PATH}")
        print(f"[YOLO] Kelas: {list(model.names.values())}")
        return model
    except ImportError:
        print("[YOLO] Install dulu: pip install ultralytics")
        return None


# ════════════════════════════════════════════════════════════════════════════════
#  Kamera TCP
# ════════════════════════════════════════════════════════════════════════════════
def check_network_reachable(ip):
    """
    Cek apakah subnet dari IP tujuan bisa dicapai.
    Jika tidak, tampilkan panduan troubleshooting.
    """
    import subprocess, platform
    cmd = ["ping", "-n" if platform.system() == "Windows" else "-c", "1", "-w" if platform.system() == "Windows" else "-W", "1000", ip]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=5)
        return result.returncode == 0
    except Exception:
        return False


def connect_cam_tcp(ip, port):
    # ── Pre-check: apakah PC sudah di jaringan yang benar? ──
    if not check_network_reachable(ip):
        print(f"""\n{'='*55}
[ERROR] NETWORK UNREACHABLE — {ip} tidak bisa dijangkau!
{'='*55}
Pastikan:
  1. PC/laptop sudah terhubung ke WiFi 'Absolute Solver'
     (bukan WiFi lain!)
  2. ESP32-CAM sudah menyala & sudah connect ke WiFi
     (lihat Serial Monitor Arduino)
  3. Hotspot 'Absolute Solver' aktif
  4. Matikan VPN jika sedang aktif
{'='*55}\n""")
    # Tetap coba connect meski ping gagal (ping bisa diblock firewall)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((ip, port))
    sock.settimeout(15)
    return sock


def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        try:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        except socket.timeout:
            return None
    return data


def receiver_thread(cam_sock):
    while not stop_event.is_set():
        len_bytes = recv_exact(cam_sock, 4)
        if not len_bytes:
            print("[CAM] Gagal baca panjang -> putus")
            break
        frame_len = struct.unpack('>I', len_bytes)[0]
        if frame_len == 0 or frame_len > 200_000:
            print(f"[CAM] Ukuran frame aneh: {frame_len}, skip")
            continue
        jpeg_data = recv_exact(cam_sock, frame_len)
        if not jpeg_data:
            print("[CAM] Data frame tidak lengkap -> putus")
            break
        frame_queue.append(jpeg_data)


# ════════════════════════════════════════════════════════════════════════════════
#  Screenshot
# ════════════════════════════════════════════════════════════════════════════════
def save_screenshot(frame, label="manual"):
    folder = os.path.join(SCREENSHOT_DIR, label)
    os.makedirs(folder, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = os.path.join(folder, f"{ts}.jpg")
    cv2.imwrite(path, frame)
    print(f"[SCREENSHOT] {path}")
    return path


# ════════════════════════════════════════════════════════════════════════════════
#  Inferensi YOLO (Mendukung Detection & Classification)
# ════════════════════════════════════════════════════════════════════════════════
def run_yolo_inference(model, frame):
    """
    Return:
      detections: list of dict {"name": str, "conf": float, "box": (x1,y1,x2,y2)} 
                  (kosong jika classification atau tidak ada deteksi)
      cls_name: str (nama kelas jika classification, None jika detection)
      cls_conf: float
    """
    results = model.predict(
        source  = frame,
        verbose = False,
        imgsz   = 320,
    )

    if not results:
        return [], None, 0.0

    result = results[0]
    
    detections = []
    cls_name = None
    cls_conf = 0.0

    # Cek jika model adalah Object Detection (punya atribut boxes yang tidak None)
    if result.boxes is not None:
        for box in result.boxes:
            conf = float(box.conf[0])
            if conf < CONF_THRESHOLD:
                continue
            cls_idx = int(box.cls[0])
            name = model.names[cls_idx]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append({
                "name": name,
                "conf": conf,
                "box": (x1, y1, x2, y2)
            })
            
    # Cek jika model adalah Classification (punya atribut probs yang tidak None)
    elif result.probs is not None:
        probs = result.probs
        top1_idx = int(probs.top1)
        top1_conf = float(probs.top1conf)
        if top1_conf >= CONF_THRESHOLD:
            cls_name = model.names[top1_idx]
            cls_conf = top1_conf

    return detections, cls_name, cls_conf


# ════════════════════════════════════════════════════════════════════════════════
#  Gambar overlay hasil klasifikasi di frame
# ════════════════════════════════════════════════════════════════════════════════
def draw_classification_overlay(frame, name, conf):
    """
    Tampilkan hasil klasifikasi sebagai banner di tengah bawah frame.
    Jika name=None, tampilkan 'Tidak ada objek dikenal'.
    """
    h, w = frame.shape[:2]

    if name is None:
        # Banner abu-abu: tidak ada deteksi
        cv2.rectangle(frame, (0, h - 60), (w, h - 30), (40, 40, 40), -1)
        cv2.putText(
            frame, "Tidak ada objek dikenal",
            (10, h - 38),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1
        )
        return frame

    info  = get_waste_info(name)
    color = info["warna"]

    # ── Banner bawah: nama objek + confidence ──
    cv2.rectangle(frame, (0, h - 90), (w, h - 55), (20, 20, 20), -1)
    label = f"{name}  ({conf:.0%})"
    cv2.putText(
        frame, label,
        (10, h - 62),
        cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2
    )

    # ── Banner bawah: kategori + aksi ──
    cv2.rectangle(frame, (0, h - 55), (w, h - 25), (30, 30, 30), -1)
    sub = f"{info['kategori']}  |  {info['aksi']}"
    cv2.putText(
        frame, sub,
        (10, h - 32),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1
    )

    # ── Border tepi frame sesuai warna kategori ──
    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, 4)

    return frame


def draw_yolo_overlay(frame, detections, cls_name, cls_conf):
    h, w = frame.shape[:2]

    # Jika ada deteksi objek (Object Detection)
    if detections:
        for det in detections:
            name = det["name"]
            conf = det["conf"]
            x1, y1, x2, y2 = det["box"]

            info = get_waste_info(name)
            color = info["warna"]

            # Gambar Bounding Box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            
            # Label
            label1 = f"{name} {conf:.0%}"
            label2 = f"{info['kategori']} | {info['aksi']}"
            
            (lw1, lh1), _ = cv2.getTextSize(label1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            (lw2, lh2), _ = cv2.getTextSize(label2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            lw = max(lw1, lw2)
            
            # Background untuk teks
            # Pastikan teks tidak keluar batas atas frame
            y_bg = max(y1, 40)
            cv2.rectangle(frame, (x1, y_bg - 40), (x1 + lw + 10, y_bg), color, -1)
            
            # Teks Putih
            cv2.putText(frame, label1, (x1 + 5, y_bg - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, label2, (x1 + 5, y_bg - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        return frame
        
    # Jika mode klasifikasi
    elif cls_name is not None:
        return draw_classification_overlay(frame, cls_name, cls_conf)
        
    # Jika tidak ada objek (baik classification maupun detection kosong)
    else:
        return draw_classification_overlay(frame, None, 0.0)

# ════════════════════════════════════════════════════════════════════════════════
#  Main loop
# ════════════════════════════════════════════════════════════════════════════════
def main():
    auto_capture = False
    flash_state  = False  # Senter default OFF — nyala hanya jika tekan F

    print("="*58)
    print("  DETEKSI LIMBAH MEDIS — YOLOv8 Classification")
    print("="*58)
    print("  S = Screenshot  |  A = Auto-capture  |  F = Flash  |  Q = Keluar")
    print("  ► Flash (senter) OFF secara default. Tekan F untuk menyalakan.")
    print("  ► Motor OFF secara default. Tekan START di web dashboard.")
    print("="*58)

    yolo_model = load_yolo_model()

    while True:
        cam_sock = cam_ws = sf_ws = recv_t = None
        stop_event.clear()
        frame_queue.clear()

        try:
            print("[CAM] Menghubungkan TCP...")
            cam_sock = connect_cam_tcp(CAM_IP, CAM_TCP_PORT)
            print("[CAM] TCP OK.")

            cam_ws = websocket.WebSocket()
            cam_ws.settimeout(3)
            try:
                cam_ws.connect(f"ws://{CAM_IP}:{CAM_WS_PORT}")
                print("[CAM] WS senter OK.")
            except Exception as e:
                print(f"[CAM] WS senter gagal: {e}")

            print(f"[SF] Smart Bin target HTTP: http://{SMART_IP}/api")
            
            def send_flash(state):
                def _do_send():
                    try:
                        cam_ws.send(json.dumps({"cmd": "flash", "state": 1 if state else 0}))
                    except Exception:
                        pass
                threading.Thread(target=_do_send, daemon=True).start()

            def send_sf(cmd_dict):
                """Kirim perintah ke Smart Bin secara HTTP GET (Anti-Drop/Teknik Drone)."""
                def _do_send():
                    try:
                        # Convert dict ke parameter URL
                        # Contoh: {"cmd": "servo", "id": 1, "angle": 90} -> cmd=servo&id=1&angle=90
                        query = urllib.parse.urlencode(cmd_dict)
                        url = f"http://{SMART_IP}/api?{query}"
                        print(f"[SF] HTTP GET: {url}")
                        
                        # Timeout sangat singkat agar respons cepat, tidak perlu koneksi persistent
                        req = urllib.request.urlopen(url, timeout=2.0)
                        resp = req.read()
                        req.close()
                        print(f"[SF] Berhasil!")
                    except Exception as e:
                        print(f"\n[SF] GAGAL HTTP KE SMART BIN: {e}")
                        print(f"[SF] PASTIKAN SMART_IP '{SMART_IP}' SUDAH SESUAI DENGAN IP BARU DARI SERIAL MONITOR!\n")
                
                # Kirim pakai thread fire-and-forget agar kamera sama sekali tidak nge-lag
                threading.Thread(target=_do_send, daemon=True).start()

            # Flash (senter): sinkron ke state terakhir (default OFF, bisa berubah saat reconnect)
            send_flash(flash_state)
            print(f"[CAM] Senter {'ON' if flash_state else 'OFF'} (sinkron state terakhir).")

            # Motor TIDAK di-start otomatis — tunggu tombol START di web dashboard
            print("[SYSTEM] Motor standby. Tekan tombol START di web dashboard untuk mulai.")

            recv_t = threading.Thread(target=receiver_thread, args=(cam_sock,), daemon=True)
            recv_t.start()

            face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            )
            face_active = False

            # Throttle YOLO: jalankan max tiap 0.15 detik
            last_yolo_time  = 0
            YOLO_INTERVAL   = 0.15
            last_cls_name   = None   # hasil YOLO classification
            last_cls_conf   = 0.0
            last_detections = []     # hasil YOLO detection
            
            last_active_servo = None
            servo_hold_time   = 0.0
            last_servo_change_time = 0.0
            SERVO_HOLD_DELAY  = 3.0

            while recv_t.is_alive():
                if not frame_queue:
                    time.sleep(0.005)
                    continue

                jpeg_data = frame_queue.pop()
                np_arr    = np.frombuffer(jpeg_data, np.uint8)
                frame     = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # ── Deteksi wajah (DIMATIKAN agar tidak memberatkan CPU / Lag) ──
                # gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # faces = face_cascade.detectMultiScale(gray, 1.1, 5)
                faces = [] # Disable face detection for performance

                if len(faces) > 0:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 200, 0), 2)
                    face_txt = "WAJAH TERDETEKSI"
                    face_col = (0, 220, 220)
                    if not face_active:
                        # send_sf({"cmd": "machine", "on": True}) # Dihapus agar motor berjalan terus
                        # send_sf({"cmd": "servo", "id": 1, "angle": 90}) # Dihapus agar tidak konflik dengan servo pemilah
                        face_active = True
                else:
                    face_txt = "" # Dikosongkan agar UI lebih bersih
                    face_col = (0, 0, 200)
                    if face_active:
                        # send_sf({"cmd": "machine", "on": False}) # Dihapus agar motor berjalan terus
                        # send_sf({"cmd": "servo", "id": 1, "angle": 0})
                        face_active = False

                # ── YOLO inference (di-throttle) ─────────────────────
                now = time.time()
                if yolo_model and (now - last_yolo_time) >= YOLO_INTERVAL:
                    last_detections, last_cls_name, last_cls_conf = run_yolo_inference(yolo_model, frame)
                    last_yolo_time = now

                    # ── Kontrol Servo Berdasarkan Kategori Limbah ──
                    detected_category = None
                    if last_detections:
                        obj_name = last_detections[0]["name"]
                        detected_category = get_waste_info(obj_name).get("kategori")
                        print(f"[YOLO] Deteksi objek: '{obj_name}' -> Kategori: '{detected_category}'")
                    elif last_cls_name:
                        detected_category = get_waste_info(last_cls_name).get("kategori")
                        print(f"[YOLO] Klasifikasi: '{last_cls_name}' -> Kategori: '{detected_category}'")

                    target_servo = None
                    if detected_category == "Limbah Infeksius":
                        target_servo = 1
                    elif detected_category == "Limbah Non-Infeksius":
                        target_servo = 2
                    elif detected_category == "Limbah B3":
                        target_servo = 3
                        
                    if target_servo is not None:
                        # Set timer untuk menahan servo terbuka
                        servo_hold_time = now + SERVO_HOLD_DELAY
                        
                        if target_servo != last_active_servo:
                            # ── ANTI-FLICKER: Abaikan jika model berganti-ganti tebakan dalam waktu singkat (2 detik) ──
                            if last_active_servo is None or (now - last_servo_change_time) > 2.0:
                                # Kembalikan servo sebelumnya ke 0 derajat
                                if last_active_servo is not None:
                                    send_sf({"cmd": "servo", "id": last_active_servo, "angle": 0})
                                    print(f"[SERVO] Servo {last_active_servo} ditutup.")
                                
                                # Menentukan nama objek yang terdeteksi
                                detected_name = obj_name if last_detections else last_cls_name
                                
                                # Buka servo target ke 90 derajat & sertakan nama objek serta kategori ke web dashboard
                                send_sf({"cmd": "servo", "id": target_servo, "angle": 90, "waste": detected_name, "cat": detected_category})
                                print(f"[SERVO] Kategori '{detected_category}' terdeteksi -> Servo {target_servo} dibuka.")
                                last_active_servo = target_servo
                                last_servo_change_time = now
                            else:
                                print(f"[YOLO] Flickering terdeteksi. Mengabaikan perubahan ke Servo {target_servo}.")
                    else:
                        # Jika tidak ada yang terdeteksi, tutup setelah delay habis
                        if last_active_servo is not None and now > servo_hold_time:
                            send_sf({"cmd": "servo", "id": last_active_servo, "angle": 0, "waste": "Tidak Ada", "cat": "-"})
                            print(f"[SERVO] Tidak ada deteksi selama {SERVO_HOLD_DELAY}s -> Servo {last_active_servo} ditutup.")
                            last_active_servo = None

                # ── Gambar overlay ───────────────────────────────────
                if yolo_model:
                    frame = draw_yolo_overlay(frame, last_detections, last_cls_name, last_cls_conf)

                # ── Status wajah (pojok kiri atas) ───────────────────
                cv2.putText(
                    frame, face_txt, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, face_col, 2
                )

                # ── Info jika YOLO belum aktif ────────────────────────
                if not yolo_model:
                    cv2.putText(
                        frame, "YOLO BELUM AKTIF -- jalankan train_yolo.py",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 80, 255), 1
                    )

                # ── Auto-capture ──────────────────────────────────────
                if auto_capture:
                    if last_detections:
                        label_cap = last_detections[0]["name"].replace(" ", "_")
                    elif last_cls_name:
                        label_cap = last_cls_name.replace(" ", "_")
                    else:
                        label_cap = "unknown"
                    save_screenshot(frame, label=label_cap)
                    cv2.putText(
                        frame, "AUTO-CAPTURE ON", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
                    )

                # ── Panduan keyboard ──────────────────────────────────
                cv2.putText(
                    frame, "S:Screenshot  A:Auto  Q:Quit",
                    (10, frame.shape[0] - 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1
                )

                # ── Perbesar tampilan (2× dari resolusi asli kamera 320x240) ──
                display_frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
                cv2.imshow("Deteksi Limbah Medis", display_frame)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    raise KeyboardInterrupt

                elif key == ord('s'):
                    raw_arr   = np.frombuffer(jpeg_data, np.uint8)
                    raw_frame = cv2.imdecode(raw_arr, cv2.IMREAD_COLOR)
                    save_screenshot(raw_frame if raw_frame is not None else frame, label="manual")

                elif key == ord('a'):
                    auto_capture = not auto_capture
                    print(f"[AUTO-CAPTURE] {'ON' if auto_capture else 'OFF'}")

                elif key == ord('f'):
                    flash_state = not flash_state
                    send_flash(flash_state)
                    print(f"[FLASH] Senter {'ON' if flash_state else 'OFF'}")

        except KeyboardInterrupt:
            print("Keluar.")
            break

        except Exception as e:
            print(f"Error: {e}")

        finally:
            stop_event.set()
            try:
                if cam_ws:
                    cam_ws.send(json.dumps({"cmd": "flash", "state": 0}))
                    flash_state = False  # Reset state agar setelah reconnect flash tetap OFF
                    print("[CAM] Senter OFF (koneksi putus).")
            except Exception:
                pass
            if cam_sock: cam_sock.close()
            if cam_ws:
                try: cam_ws.close()
                except Exception: pass
            if recv_t and recv_t.is_alive():
                recv_t.join(timeout=2)

        print("[RETRY] Menunggu 3 detik sebelum reconnect...")
        time.sleep(3)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
