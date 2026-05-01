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

CAM_IP       = "192.168.8.136"
CAM_TCP_PORT = 80
CAM_WS_PORT  = 81

SMART_IP      = "192.168.8.199"
SMART_WS_PORT = 81

# ─── Folder screenshot untuk training AI ────────────────
SCREENSHOT_DIR = r"C:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Python\training_data"
os.makedirs(SCREENSHOT_DIR, exist_ok=True)

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

frame_queue = deque(maxlen=2)
stop_event  = threading.Event()


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


def save_screenshot(frame, label="manual"):
    """
    Simpan screenshot ke folder training_data.
    Sub-folder otomatis dibuat berdasarkan label:
      training_data/manual/   ← screenshot manual (tekan S)
      training_data/face/     ← otomatis saat wajah terdeteksi (tekan A)
      training_data/no_face/  ← otomatis saat tidak ada wajah (tekan A)
    Nama file: YYYYMMDD_HHMMSS_mmm.jpg
    """
    folder = os.path.join(SCREENSHOT_DIR, label)
    os.makedirs(folder, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename  = os.path.join(folder, f"{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"[SCREENSHOT] Disimpan: {filename}")
    return filename


def main():
    global frame_queue

    auto_capture = False  # mode auto-capture berdasarkan status wajah

    print("=" * 45)
    print("  Kontrol keyboard:")
    print("  S          = Screenshot manual")
    print("  A          = Toggle auto-capture (face/no_face)")
    print("  Q          = Keluar")
    print("=" * 45)

    while True:
        cam_sock = None
        cam_ws   = None
        sf_ws    = None
        recv_t   = None
        stop_event.clear()
        frame_queue.clear()

        try:
            # ── Koneksi TCP kamera ──────────────────────────────────
            print("[CAM] Menghubungkan TCP...")
            cam_sock = connect_cam_tcp(CAM_IP, CAM_TCP_PORT)
            print("[CAM] TCP OK.")

            # ── Koneksi WebSocket senter ────────────────────────────
            cam_ws = websocket.WebSocket()
            cam_ws.connect(f"ws://{CAM_IP}:{CAM_WS_PORT}")
            print("[CAM] WS senter OK.")

            # ── Koneksi WebSocket SmartFan/Robot ────────────────────
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

            # ── Senter ON ───────────────────────────────────────────
            send_flash(True)
            print("[CAM] Senter ON.")

            # ── Mulai thread penerima frame ─────────────────────────
            recv_t = threading.Thread(target=receiver_thread, args=(cam_sock,), daemon=True)
            recv_t.start()

            face_active = False

            # ── Loop utama: deteksi + tampilkan ────────────────────
            while recv_t.is_alive():
                if not frame_queue:
                    time.sleep(0.005)
                    continue

                jpeg_data = frame_queue.pop()

                np_arr = np.frombuffer(jpeg_data, np.uint8)
                frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # ── Deteksi wajah ───────────────────────────────────
                gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.1, 5)

                if len(faces) > 0:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    status = "FACE"
                    if not face_active:
                        send_sf({"cmd": "machine", "on": True})
                        send_sf({"cmd": "servo", "id": 1, "angle": 90})
                        face_active = True
                    if auto_capture:
                        save_screenshot(frame, label="face")
                else:
                    status = "NO FACE"
                    if face_active:
                        send_sf({"cmd": "machine", "on": False})
                        send_sf({"cmd": "servo", "id": 1, "angle": 0})
                        face_active = False
                    if auto_capture:
                        save_screenshot(frame, label="no_face")

                # ── Overlay teks status ─────────────────────────────
                cv2.putText(
                    frame, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0) if len(faces) > 0 else (0, 0, 255), 2
                )

                # Tampilkan indikator auto-capture
                if auto_capture:
                    cv2.putText(
                        frame, "AUTO-CAPTURE ON", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2
                    )

                # Tampilkan panduan kontrol di layar
                cv2.putText(frame, "S:Screenshot  A:Auto  Q:Quit", (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow("Deteksi", frame)

                # ── Keyboard ────────────────────────────────────────
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    raise KeyboardInterrupt

                elif key == ord('s'):
                    # Screenshot manual — simpan frame tanpa kotak deteksi
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