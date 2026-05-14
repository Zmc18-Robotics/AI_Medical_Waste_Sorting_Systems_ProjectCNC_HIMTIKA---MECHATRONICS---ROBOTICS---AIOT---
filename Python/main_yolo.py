"""
=============================================================
  main_yolo.py  —  Sistem Deteksi Limbah Medis (YOLOv8-cls)
=============================================================
  Jalankan train_yolo.py dulu sebelum file ini.

  Keyboard:
    S  = Screenshot manual
    A  = Toggle auto-capture
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
from collections import deque
from datetime import datetime

# ─── Konfigurasi IP ──────────────────────────────────────────────────────────
CAM_IP       = "192.168.8.136"
CAM_TCP_PORT = 80
CAM_WS_PORT  = 81

SMART_IP      = "192.168.8.199"
SMART_WS_PORT = 81

# ─── Path ─────────────────────────────────────────────────────────────────────
BASE_DIR       = r"C:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Python"
SCREENSHOT_DIR = os.path.join(BASE_DIR, "training_data")
MODEL_PATH     = os.path.join(BASE_DIR, "waste_model", "weights", "best.pt")

os.makedirs(SCREENSHOT_DIR, exist_ok=True)

# ─── Threshold confidence ─────────────────────────────────────────────────────
# Model classification memberi 1 prediksi per frame (bukan per objek).
# Turunkan ke 0.40 jika objek sering tidak terdeteksi.
CONF_THRESHOLD = 0.50

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
def connect_cam_tcp(ip, port):
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
#  Inferensi YOLO classification (BERBEDA dari detection!)
#
#  Model classification (yolov8n-cls) tidak menghasilkan bounding box.
#  Dia mengklasifikasikan SELURUH frame -> 1 prediksi teratas per frame.
#  Output: result.probs  (bukan result.boxes)
# ════════════════════════════════════════════════════════════════════════════════
def run_yolo_cls(model, frame):
    """
    Jalankan classification pada frame.
    Return:
      detected_name : str  nama kelas terdeteksi, atau None jika di bawah threshold
      confidence    : float
      frame         : frame dengan overlay info
    """
    results = model.predict(
        source  = frame,
        verbose = False,
        imgsz   = 320,
    )

    if not results:
        return None, 0.0, frame

    result = results[0]

    # result.probs adalah Probs object
    # .top1       -> index kelas dengan prob tertinggi
    # .top1conf   -> confidence-nya (tensor)
    # model.names -> dict {index: nama_kelas}
    probs = result.probs
    if probs is None:
        return None, 0.0, frame

    top1_idx  = int(probs.top1)
    top1_conf = float(probs.top1conf)
    top1_name = model.names[top1_idx]

    # Hanya tampilkan jika confidence melewati threshold
    if top1_conf < CONF_THRESHOLD:
        return None, top1_conf, frame

    return top1_name, top1_conf, frame


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

    info  = WASTE_INFO.get(name, {
        "kategori": "Tidak Dikenal",
        "warna":    (128, 128, 128),
        "aksi":     "-",
    })
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


# ════════════════════════════════════════════════════════════════════════════════
#  Main loop
# ════════════════════════════════════════════════════════════════════════════════
def main():
    auto_capture = False

    print("=" * 50)
    print("  DETEKSI LIMBAH MEDIS — YOLOv8 Classification")
    print("=" * 50)
    print("  S = Screenshot  |  A = Auto-capture  |  Q = Keluar")
    print("=" * 50)

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
            cam_ws.connect(f"ws://{CAM_IP}:{CAM_WS_PORT}")
            print("[CAM] WS senter OK.")

            sf_ws = websocket.WebSocket()
            sf_ws.connect(f"ws://{SMART_IP}:{SMART_WS_PORT}")
            print("[SF] WS OK.")

            def send_flash(state):
                try:
                    if cam_ws.sock and cam_ws.sock.connected:
                        cam_ws.send(json.dumps({"cmd": "flash", "state": 1 if state else 0}))
                except Exception:
                    pass

            def send_sf(cmd_dict):
                try:
                    if sf_ws.sock and sf_ws.sock.connected:
                        sf_ws.send(json.dumps(cmd_dict))
                except Exception:
                    pass

            send_flash(True)
            print("[CAM] Senter ON.")

            recv_t = threading.Thread(target=receiver_thread, args=(cam_sock,), daemon=True)
            recv_t.start()

            face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            )
            face_active = False

            # Throttle YOLO: jalankan max tiap 0.15 detik
            last_yolo_time  = 0
            YOLO_INTERVAL   = 0.15
            last_name       = None   # hasil YOLO terakhir (ditampilkan antar interval)
            last_conf       = 0.0

            while recv_t.is_alive():
                if not frame_queue:
                    time.sleep(0.005)
                    continue

                jpeg_data = frame_queue.pop()
                np_arr    = np.frombuffer(jpeg_data, np.uint8)
                frame     = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # ── Deteksi wajah ────────────────────────────────────
                gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.1, 5)

                if len(faces) > 0:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 200, 0), 2)
                    face_txt = "WAJAH TERDETEKSI"
                    face_col = (0, 220, 220)
                    if not face_active:
                        send_sf({"cmd": "machine", "on": True})
                        send_sf({"cmd": "servo", "id": 1, "angle": 90})
                        face_active = True
                else:
                    face_txt = "TIDAK ADA WAJAH"
                    face_col = (0, 0, 200)
                    if face_active:
                        send_sf({"cmd": "machine", "on": False})
                        send_sf({"cmd": "servo", "id": 1, "angle": 0})
                        face_active = False

                # ── YOLO classification (di-throttle) ────────────────
                now = time.time()
                if yolo_model and (now - last_yolo_time) >= YOLO_INTERVAL:
                    last_name, last_conf, frame = run_yolo_cls(yolo_model, frame)
                    last_yolo_time = now

                # ── Gambar overlay klasifikasi ────────────────────────
                if yolo_model:
                    frame = draw_classification_overlay(frame, last_name, last_conf)

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
                    label_cap = last_name.replace(" ", "_") if last_name else "unknown"
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

                cv2.imshow("Deteksi Limbah Medis", frame)

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

        except KeyboardInterrupt:
            print("Keluar.")
            break

        except Exception as e:
            print(f"Error: {e}")

        finally:
            stop_event.set()
            try:
                if cam_ws and cam_ws.sock and cam_ws.sock.connected:
                    cam_ws.send(json.dumps({"cmd": "flash", "state": 0}))
                    print("[CAM] Senter OFF.")
            except Exception:
                pass
            if cam_sock: cam_sock.close()
            if cam_ws:   cam_ws.close()
            if sf_ws:    sf_ws.close()
            if recv_t and recv_t.is_alive():
                recv_t.join(timeout=2)

        print("[RETRY] Menunggu 3 detik sebelum reconnect...")
        time.sleep(3)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()