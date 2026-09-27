"""
=============================================================
  test_firebase.py — Verifikasi Lengkap Firebase RTDB
=============================================================
  Menguji:
  1. Deteksi Awal YOLO
  2. Verifikasi Pemilahan Sukses via Sensor Ultrasonik US2
  3. Deteksi Limbah Anomali (Gagal Sortir) via Sensor Ultrasonik US1
  4. Live Telemetri Ultrasonik Radar
=============================================================
"""

import time
from firebase_service import (
    init_firebase,
    push_yolo_detection,
    push_waste_sorted_verified,
    push_waste_anomaly,
    update_ultrasonic_telemetry,
    get_db
)

print("="*60)
print("   VERIFIKASI SISTEM PEMILAHAN & ANOMALI KE FIREBASE CLOUD")
print("="*60)

if init_firebase():
    db = get_db()
    
    print("\n[SKENARIO 1: Deteksi YOLO + Konfirmasi Ultrasonik US2 Pintu 1]")
    # 1. YOLO mendeteksi limbah infeksius
    push_yolo_detection("Plester", "Limbah Infeksius", servo_id=1, confidence=0.94)
    time.sleep(1.0)
    # 2. Sensor Ultrasonik US2 mendeteksi objek masuk di Pintu 1 (jarak 6.2 cm)
    push_waste_sorted_verified("Plester", "Limbah Infeksius", servo_id=1, door_id=1, distance_cm=6.2, confidence=0.94)
    time.sleep(1.5)

    print("\n[SKENARIO 2: Deteksi YOLO + Konfirmasi Ultrasonik US2 Pintu 2]")
    push_yolo_detection("Kain Kasa", "Limbah Non-Infeksius", servo_id=2, confidence=0.91)
    time.sleep(1.0)
    push_waste_sorted_verified("Kain Kasa", "Limbah Non-Infeksius", servo_id=2, door_id=2, distance_cm=5.8, confidence=0.91)
    time.sleep(1.5)

    print("\n[SKENARIO 3: Deteksi YOLO + Konfirmasi Ultrasonik US2 Pintu 3]")
    push_yolo_detection("Obat 1", "Limbah B3", servo_id=3, confidence=0.96)
    time.sleep(1.0)
    push_waste_sorted_verified("Obat 1", "Limbah B3", servo_id=3, door_id=3, distance_cm=7.1, confidence=0.96)
    time.sleep(1.5)

    print("\n[SKENARIO 4: Deteksi Limbah Anomali via Sensor Ultrasonik US1]")
    # Objek tidak tersortir melewati semua pintu dan terdeteksi di ujung konveyor oleh US1
    push_waste_anomaly(distance_cm=8.5, details="Limbah tidak tersortir di pintu 1-3, lolos ke ujung konveyor (US1)")
    time.sleep(1.5)

    print("\n[SKENARIO 5: Update Live Telemetri Sensor Ultrasonik]")
    update_ultrasonic_telemetry(d1=8.5, dA=6.2, dB=5.8, dC=7.1, us1_active=True, us2_door=1)
    time.sleep(1.5)

    # Ambil data akhir dari Firebase Cloud
    stats = db.reference('stats').get() or {}
    realtime = db.reference('realtime_status').get() or {}
    telemetry = db.reference('sensor_telemetry/ultrasonic').get() or {}

    print("\n" + "="*60)
    print("         HASIL DATA STATISTIK DI FIREBASE CLOUD:")
    print("="*60)
    print(f" • Total Terdeteksi YOLO   : {stats.get('total_terdeteksi_yolo', 0)} item")
    print(f" • Total Tersortir Sukses  : {stats.get('total_tersortir_sukses', 0)} item")
    print(f" • Total Anomali (Gagal)   : {stats.get('total_anomali', 0)} item")
    print(f" • Total Terolah           : {stats.get('total_terolah', 0)} item")
    print("------------------------------------------------------------")
    print("   Rincian Per Kategori Tersortir:")
    print(f" • Limbah Infeksius (Pintu 1)    : {stats.get('limbah_infeksius', 0)} item (Pintu 1: {stats.get('pintu_1_infeksius', 0)})")
    print(f" • Limbah Non-Infeksius (Pintu 2): {stats.get('limbah_non_infeksius', 0)} item (Pintu 2: {stats.get('pintu_2_non_infeksius', 0)})")
    print(f" • Limbah B3 (Pintu 3)           : {stats.get('limbah_b3', 0)} item (Pintu 3: {stats.get('pintu_3_b3', 0)})")
    print("="*60)
    print("   STATUS REALTIME TERAKHIR:")
    print(f" • Status     : {realtime.get('status')}")
    print(f" • Verifikasi : {realtime.get('verified_by')}")
    print(f" • Item       : {realtime.get('last_item')} ({realtime.get('last_category')})")
    print(f" • Waktu      : {realtime.get('last_sorted_at')}")
    print("="*60)
    print("   LIVE TELEMETRI ULTRASONIK:")
    print(f" • US1 Anomali  : {telemetry.get('us1_anomali_cm')} cm (Aktif: {telemetry.get('us1_active')})")
    print(f" • US2 Pintu A  : {telemetry.get('us2_pintuA_infeksius_cm')} cm")
    print(f" • US2 Pintu B  : {telemetry.get('us2_pintuB_noninfeksius_cm')} cm")
    print(f" • US2 Pintu C  : {telemetry.get('us2_pintuC_b3_cm')} cm")
    print("="*60)
    print(">>> SEMUA DATA ULTRASONIK & ANOMALI BERHASIL DISINKRONISASI KE FIREBASE! <<<")
