"""
=============================================================
  firebase_service.py — Modul Penghubung Firebase Realtime DB
=============================================================
  Sistem Pemilah Limbah Medis Berbasis IoT & AI
  Endpoint: /
=============================================================
"""

import os
import threading
from datetime import datetime

DATABASE_URL = "-"

_is_initialized = False

def get_db():
    """Mengembalikan objek firebase_admin.db."""
    from firebase_admin import db
    return db

def init_firebase(custom_url=None):
    """
    Inisialisasi Firebase Admin SDK menggunakan serviceAccountKey.json.
    Mendukung auto-detect lokasi file kunci privat.
    """
    global _is_initialized
    if _is_initialized:
        return True

    try:
        import firebase_admin
        from firebase_admin import credentials, db
    except ImportError:
        print("[FIREBASE] Library 'firebase-admin' belum terpasang. Jalankan: pip install firebase-admin")
        return False

    # Cari file serviceAccountKey.json di beberapa lokasi yang mungkin
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    candidate_paths = [
        os.path.join(curr_dir, "serviceAccountKey.json"),
        os.path.join(curr_dir, "Firebase admin SDK", "serviceAccountKey.json"),
        "serviceAccountKey.json"
    ]

    key_path = None
    for p in candidate_paths:
        if os.path.exists(p):
            key_path = p
            break

    if not key_path:
        print("[FIREBASE] ERROR: 'serviceAccountKey.json' tidak ditemukan!")
        print(f"[FIREBASE] Telah dicari di: {candidate_paths}")
        return False

    target_url = custom_url or DATABASE_URL

    try:
        # Hindari inisialisasi ganda jika app sudah ada
        if not firebase_admin._apps:
            cred = credentials.Certificate(key_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': target_url
            })
        _is_initialized = True
        print(f"[FIREBASE] Berhasil terhubung ke Firebase RTDB: {target_url}")
        
        # Inisialisasi struktur awal jika belum ada
        _init_default_structure()
        return True
    except Exception as e:
        print(f"[FIREBASE] Gagal inisialisasi Firebase: {e}")
        return False


def _init_default_structure():
    """Memastikan field stats dan realtime_status tersedia jika database masih kosong."""
    def _task():
        try:
            db = get_db()
            stats_ref = db.reference('stats')
            curr_stats = stats_ref.get()
            if curr_stats is None:
                stats_ref.set({
                    'limbah_b3': 0,
                    'limbah_infeksius': 0,
                    'limbah_non_infeksius': 0,
                    'total_terolah': 0,
                    'total_terdeteksi_yolo': 0,
                    'total_tersortir_sukses': 0,
                    'total_anomali': 0,
                    'pintu_1_infeksius': 0,
                    'pintu_2_non_infeksius': 0,
                    'pintu_3_b3': 0
                })
                print("[FIREBASE] Struktur awal 'stats' dibuat di database.")
        except Exception as e:
            print(f"[FIREBASE] Gagal cek struktur awal: {e}")

    threading.Thread(target=_task, daemon=True).start()


def push_yolo_detection(item_name, category, servo_id, confidence=1.0):
    """
    Mencatat data deteksi awal dari YOLO ke Firebase:
    - Menambah counter total_terdeteksi_yolo di stats
    - Mengupdate status realtime bahwa limbah sedang menuju sensor/pintu pemilah
    """
    if not _is_initialized:
        return

    def _task():
        try:
            db = get_db()
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 1. Update realtime status deteksi
            db.reference('realtime_status').update({
                'last_detected_item': item_name,
                'last_detected_category': category,
                'last_target_servo': int(servo_id),
                'detected_confidence': float(round(confidence, 2)),
                'detection_status': "Menuju Pemilah (Konveyor)",
                'last_detected_at': now_str
            })

            # 2. Increment Counter YOLO Terdeteksi
            yolo_ref = db.reference('stats/total_terdeteksi_yolo')
            yolo_ref.transaction(lambda current: (current or 0) + 1)

            print(f"[FIREBASE] [YOLO] Terdeteksi -> {item_name} ({category}) | Menuju Pintu {servo_id} (+1 Terdeteksi)")
        except Exception as e:
            print(f"[FIREBASE] Gagal update deteksi YOLO: {e}")

    threading.Thread(target=_task, daemon=True).start()


def push_waste_sorted_verified(item_name, category, servo_id, door_id, distance_cm=0.0, confidence=1.0):
    """
    KONFIRMASI ULTRASONIK US2:
    Dipanggil saat Sensor Ultrasonik US2 mendeteksi limbah berhasil melewati/jatuh ke pintu pemilah.
    - Mengupdate statistik kategori, total_tersortir_sukses, dan total_terolah
    - Mengupdate realtime_status dengan verifikasi ultrasonik
    - Menambahkan entri log sortir terverifikasi
    """
    if not _is_initialized:
        return

    def _task():
        try:
            db = get_db()
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            door_name = f"Pintu {door_id}"
            if door_id == 1: door_name += " (Infeksius)"
            elif door_id == 2: door_name += " (Non-Infeksius)"
            elif door_id == 3: door_name += " (B3)"

            # 1. Update status terkini (realtime_status)
            db.reference('realtime_status').update({
                'last_item': item_name,
                'last_category': category,
                'last_servo': int(servo_id),
                'confidence': float(round(confidence, 2)),
                'status': f"BERHASIL TERSORTIR - {door_name}",
                'verified_by': f"Ultrasonik US2 {door_name}",
                'ultrasonic_cm': float(round(distance_cm, 1)) if distance_cm > 0 else 0.0,
                'last_sorted_at': now_str
            })

            # 2. Increment Counter Statistik
            key_map = {
                "Limbah B3": "limbah_b3",
                "Limbah Infeksius": "limbah_infeksius",
                "Limbah Non-Infeksius": "limbah_non_infeksius"
            }
            cat_key = key_map.get(category, "lainnya")

            # Update counter per kategori
            cat_ref = db.reference(f'stats/{cat_key}')
            cat_ref.transaction(lambda current: (current or 0) + 1)

            # Update counter per pintu
            door_keys = {1: 'pintu_1_infeksius', 2: 'pintu_2_non_infeksius', 3: 'pintu_3_b3'}
            if door_id in door_keys:
                d_ref = db.reference(f'stats/{door_keys[door_id]}')
                d_ref.transaction(lambda current: (current or 0) + 1)

            # Update counter total tersortir sukses & total terolah
            sukses_ref = db.reference('stats/total_tersortir_sukses')
            sukses_ref.transaction(lambda current: (current or 0) + 1)

            total_ref = db.reference('stats/total_terolah')
            total_ref.transaction(lambda current: (current or 0) + 1)

            # 3. Catat ke riwayat pemilahan (logs)
            db.reference('logs').push({
                'item': item_name,
                'category': category,
                'servo': int(servo_id),
                'door': int(door_id),
                'confidence': float(round(confidence, 2)),
                'distance_cm': float(round(distance_cm, 1)) if distance_cm > 0 else None,
                'status': "BERHASIL_TERSORTIR",
                'verified_by': f"Ultrasonik US2 Pintu {door_id}",
                'timestamp': now_str
            })

            print(f"[FIREBASE] [US2 VERIFIKASI] [OK] {item_name} ({category}) | Pintu {door_id} ({distance_cm:.1f} cm) -> BERHASIL TERSORTIR (+1 Counter)")
        except Exception as e:
            print(f"[FIREBASE] Gagal mengupdate data sortir terverifikasi: {e}")

    threading.Thread(target=_task, daemon=True).start()


def push_waste_anomaly(distance_cm=0.0, details="Limbah Anomali / Tidak Tersortir (Melewati Konveyor)"):
    """
    DETEKSI ANOMALI ULTRASONIK US1:
    Dipanggil saat Sensor Ultrasonik US1 mendeteksi limbah anomali yang lolos/tidak masuk pintu pemilah.
    - Menambah counter total_anomali di stats
    - Mengupdate realtime_status dengan peringatan anomali
    - Menambahkan entri ke logs dan node anomalies
    """
    if not _is_initialized:
        return

    def _task():
        try:
            db = get_db()
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 1. Update realtime status anomali
            db.reference('realtime_status').update({
                'status': "ANOMALI: Limbah Tidak Tersortir",
                'verified_by': "Ultrasonik US1 (Sensor Anomali)",
                'last_anomaly_cm': float(round(distance_cm, 1)),
                'last_anomaly_at': now_str
            })

            # 2. Increment Counter Anomali
            anomali_ref = db.reference('stats/total_anomali')
            anomali_ref.transaction(lambda current: (current or 0) + 1)

            # 3. Catat ke logs umum
            db.reference('logs').push({
                'item': "Limbah Anomali",
                'category': "Anomali (Gagal Sortir)",
                'servo': 0,
                'distance_cm': float(round(distance_cm, 1)),
                'status': "GAGAL_TERSORTIR_ANOMALI",
                'verified_by': "Ultrasonik US1 (Anomali)",
                'details': details,
                'timestamp': now_str
            })

            # 4. Catat ke node khusus anomalies
            db.reference('anomalies').push({
                'type': "ANOMALI_TIDAK_TERSORTIR",
                'distance_cm': float(round(distance_cm, 1)),
                'details': details,
                'timestamp': now_str
            })

            print(f"[FIREBASE] [US1 ANOMALI] [!] Limbah Anomali terdeteksi ({distance_cm:.1f} cm) -> GAGAL TERSORTIR (+1 Anomali)")
        except Exception as e:
            print(f"[FIREBASE] Gagal mengupdate data anomali: {e}")

    threading.Thread(target=_task, daemon=True).start()


def update_ultrasonic_telemetry(d1, dA, dB, dC, us1_active, us2_door):
    """Update live radar dan jarak ultrasonik realtime ke Firebase."""
    if not _is_initialized:
        return

    def _task():
        try:
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            get_db().reference('sensor_telemetry/ultrasonic').set({
                'us1_anomali_cm': float(round(d1, 1)),
                'us2_pintuA_infeksius_cm': float(round(dA, 1)),
                'us2_pintuB_noninfeksius_cm': float(round(dB, 1)),
                'us2_pintuC_b3_cm': float(round(dC, 1)),
                'us1_active': bool(us1_active),
                'us2_door': int(us2_door),
                'last_updated': now_str
            })
        except Exception as e:
            pass

    threading.Thread(target=_task, daemon=True).start()


def push_waste_sorted(item_name, category, servo_id, confidence=1.0):
    """
    Fungsi kompatibilitas mundur: Meneruskan ke push_waste_sorted_verified.
    """
    push_waste_sorted_verified(item_name, category, servo_id, door_id=servo_id, distance_cm=0.0, confidence=confidence)


def update_system_status(status_dict):
    """Update status sistem (motor, conveyor, camera, status deteksi)."""
    if not _is_initialized:
        return

    def _task():
        try:
            get_db().reference('system_status').update(status_dict)
        except Exception as e:
            pass

    threading.Thread(target=_task, daemon=True).start()
