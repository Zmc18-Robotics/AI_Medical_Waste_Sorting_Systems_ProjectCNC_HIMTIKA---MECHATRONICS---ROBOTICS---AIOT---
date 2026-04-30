# ============================================
# Python : Terima Gambar & Kendalikan ESP32
# ============================================
import socket
import cv2
import numpy as np
import time
import struct

# ===== KONFIGURASI =====
CAM_IP = "192.168.8.136"    # Ganti dengan IP ESP32-CAM
CAM_PORT = 80

OUTPUT_IP = "192.168.8.128" # Ganti dengan IP ESP32 biasa
OUTPUT_PORT = 8080

# Load Haar Cascade untuk deteksi wajah
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

def connect_socket(ip, port, timeout=5):
    """Buat koneksi TCP dengan timeout."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect((ip, port))
    return sock

def recv_exact(sock, n):
    """Terima tepat n byte dari socket."""
    data = b""
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def main():
    # Koneksi ke ESP32-CAM
    print("Menghubungkan ke ESP32-CAM...")
    try:
        cam_sock = connect_socket(CAM_IP, CAM_PORT)
        print("Terhubung ke ESP32-CAM")
    except Exception as e:
        print(f"Gagal konek CAM: {e}")
        return

    # Koneksi ke ESP32 output
    print("Menghubungkan ke ESP32 output...")
    try:
        out_sock = connect_socket(OUTPUT_IP, OUTPUT_PORT)
        print("Terhubung ke ESP32 output")
    except Exception as e:
        print(f"Gagal konek output: {e}")
        cam_sock.close()
        return

    face_detected_last = False
    try:
        while True:
            # Baca 4 byte panjang frame
            len_bytes = recv_exact(cam_sock, 4)
            if not len_bytes:
                print("Koneksi CAM terputus")
                break
            frame_len = struct.unpack('>I', len_bytes)[0]

            # Baca data JPEG
            jpeg_data = recv_exact(cam_sock, frame_len)
            if not jpeg_data:
                print("Data frame tidak lengkap")
                break

            # Decode JPEG menjadi gambar OpenCV
            np_arr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # ---------- Deteksi Wajah ----------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 5)

            if len(faces) > 0:
                # Gambar kotak di setiap wajah
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                status = "DETEKSI"
                if not face_detected_last:
                    # Wajah baru terdeteksi -> kirim LED_ON
                    out_sock.sendall(b"LED_ON\n")
                    face_detected_last = True
            else:
                status = "TIDAK ADA"
                if face_detected_last:
                    # Wajah hilang -> kirim LED_OFF
                    out_sock.sendall(b"LED_OFF\n")
                    face_detected_last = False

            # Tampilkan status di layar
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0) if len(faces) > 0 else (0, 0, 255), 2)

            cv2.imshow("ESP32-CAM Deteksi", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("Dihentikan oleh pengguna")
    finally:
        cam_sock.close()
        out_sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()