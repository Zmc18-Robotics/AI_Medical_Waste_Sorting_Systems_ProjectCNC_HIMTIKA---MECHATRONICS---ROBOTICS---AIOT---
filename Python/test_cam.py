import socket
import struct
import time
import sys
import os
from datetime import datetime
import cv2
import numpy as np

# Konfigurasi Default ESP32-CAM
# (Bisa diubah langsung di sini atau lewat argumen terminal: python test_cam.py 192.168.4.2)
CAM_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.2"
CAM_TCP_PORT = 80
CAM_WS_PORT = 81

# Coba import websocket untuk kontrol senter/flash (opsional)
try:
    import websocket
    import json
    HAS_WEBSOCKET = True
except ImportError:
    HAS_WEBSOCKET = False

def recv_exact(sock, n):
    """Membaca data persis sejumlah n bytes dari socket TCP."""
    data = bytearray()
    while len(data) < n:
        try:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        except socket.timeout:
            return None
        except Exception:
            return None
    return bytes(data)

def connect_camera(ip, port, timeout=5):
    """Membuka koneksi socket TCP ke ESP32-CAM."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect((ip, port))
    # Setelah koneksi berhasil, set timeout baca data frame
    sock.settimeout(3.0)
    return sock

def main():
    print("=" * 60)
    print(f"[*] REALTIME STREAM VIEWER ESP32-CAM")
    print(f"[*] Target IP   : {CAM_IP}")
    print(f"[*] Port Video  : {CAM_TCP_PORT}")
    print("=" * 60)
    print("Kontrol:")
    print("  - Tekan 'q' atau 'ESC' : Keluar")
    print("  - Tekan 's'            : Simpan tangkapan layar (Screenshot)")
    print("  - Tekan 'f'            : Toggle Senter / Flash (jika didukung)")
    print("=" * 60)

    # Inisialisasi WebSocket Flash (jika tersedia)
    ws_flash = None
    flash_state = False
    if HAS_WEBSOCKET:
        try:
            ws_flash = websocket.WebSocket()
            ws_flash.settimeout(1.5)
            ws_flash.connect(f"ws://{CAM_IP}:{CAM_WS_PORT}")
            print("[*] WebSocket Flash terhubung!")
        except Exception:
            ws_flash = None

    sock = None
    try:
        print(f"\n[1] Menghubungkan ke kamera ({CAM_IP}:{CAM_TCP_PORT})...")
        sock = connect_camera(CAM_IP, CAM_TCP_PORT)
        print("    -> [SUKSES] Terhubung! Memulai streaming realtime...\n")
    except socket.timeout:
        print(f"    -> [GAGAL] Timeout! Kamera {CAM_IP} tidak merespons.")
        print("       Pastikan PC terhubung ke WiFi hotspot yang sama dengan ESP32-CAM.")
        return
    except ConnectionRefusedError:
        print(f"    -> [GAGAL] Connection Refused pada {CAM_IP}:{CAM_TCP_PORT}!")
        print("       Kamera mungkin belum siap atau server TCP kamera belum berjalan.")
        return
    except Exception as e:
        print(f"    -> [GAGAL] Error saat koneksi: {e}")
        return

    # Inisialisasi variabel untuk perhitungan FPS
    prev_time = time.time()
    fps = 0.0
    frame_count = 0

    window_name = f"ESP32-CAM Stream ({CAM_IP})"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            # 1. Baca 4 byte header untuk ukuran file JPEG
            len_bytes = recv_exact(sock, 4)
            if not len_bytes:
                print("\n[!] Koneksi terputus atau tidak menerima header frame.")
                break

            frame_len = struct.unpack('>I', len_bytes)[0]
            if frame_len <= 0 or frame_len > 1_000_000:
                print(f"[!] Ukuran frame tidak valid: {frame_len} bytes, melewati...")
                continue

            # 2. Baca data JPEG sesuai ukuran frame
            jpeg_data = recv_exact(sock, frame_len)
            if not jpeg_data or len(jpeg_data) != frame_len:
                print("\n[!] Frame tidak lengkap atau koneksi terputus.")
                break

            # 3. Decode JPEG ke format gambar OpenCV
            np_arr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            # Perhitungan FPS
            curr_time = time.time()
            time_diff = curr_time - prev_time
            if time_diff > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / time_diff) if fps > 0 else (1.0 / time_diff)
            prev_time = curr_time
            frame_count += 1

            # Tampilkan informasi pada frame (Overlay OSD)
            h, w, _ = frame.shape
            info_text = f"{w}x{h} | {fps:.1f} FPS"
            cv2.putText(frame, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

            # Tampilkan frame
            cv2.imshow(window_name, frame)

            # Tangani keyboard event (1 ms delay untuk realtime)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' atau ESC
                print("\n[*] Menghentikan stream atas permintaan pengguna.")
                break
            elif key == ord('s'):  # Simpan screenshot
                os.makedirs("screenshots", exist_ok=True)
                filename = f"screenshots/cam_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[*] Screenshot disimpan: {filename}")
            elif key == ord('f'):  # Toggle Flash
                if ws_flash:
                    try:
                        flash_state = not flash_state
                        ws_flash.send(json.dumps({"cmd": "flash", "state": 1 if flash_state else 0}))
                        print(f"[*] Senter/Flash: {'ON' if flash_state else 'OFF'}")
                    except Exception as e:
                        print(f"[!] Gagal mengirim perintah flash: {e}")

    except KeyboardInterrupt:
        print("\n[*] Stream dihentikan (Ctrl+C).")
    except Exception as e:
        print(f"\n[!] Terjadi kesalahan saat streaming: {e}")
    finally:
        if sock:
            sock.close()
        if ws_flash:
            try:
                ws_flash.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
        print("[*] Selesai. Socket dan window ditutup.")

if __name__ == "__main__":
    main()
