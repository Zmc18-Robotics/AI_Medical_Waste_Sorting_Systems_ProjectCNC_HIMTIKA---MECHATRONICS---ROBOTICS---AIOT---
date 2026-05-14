"""
=============================================================
  train_yolo.py  —  Training YOLOv8 untuk Deteksi Limbah Medis
=============================================================
  Jalankan SEKALI sebelum main_yolo.py:
      python train_yolo.py

  Install dulu (satu kali):
      pip install ultralytics opencv-python numpy pillow

  Struktur Datasheet yang diharapkan:
    Datasheet/
      Limbah B3/
        Obat 1/        ← tiap subfolder = 1 kelas
        Obat 2/
      Limbah Infeksius/
        Plester/
      Limbah Non-Infeksius/
        Kain kasa/
        Tisu Antiseptik/
=============================================================
"""

import os
import shutil
import random
from pathlib import Path

# ─── Path konfigurasi ────────────────────────────────────────────────────────
BASE_DIR      = r"C:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Python"
DATASHEET_DIR = os.path.join(BASE_DIR, "Datasheet")
YOLO_DATA_DIR = os.path.join(BASE_DIR, "yolo_dataset")   # dataset sementara
MODEL_OUT_DIR = os.path.join(BASE_DIR, "waste_model")    # hasil training

TRAIN_RATIO = 0.80
EPOCHS      = 50
IMG_SIZE    = 320
BATCH_SIZE  = 8

# WAJIB pakai yolov8n-cls.pt (bukan yolov8n.pt) untuk classification
MODEL_BASE  = "yolov8n-cls.pt"

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ════════════════════════════════════════════════════════════════════════════════
def scan_classes(datasheet_dir):
    """Scan subfolder 2 level -> dict {nama_kelas: [list path gambar]}"""
    classes = {}
    datasheet_path = Path(datasheet_dir)

    if not datasheet_path.exists():
        raise FileNotFoundError(
            f"Folder Datasheet tidak ditemukan:\n  {datasheet_dir}\n"
            "Pastikan path BASE_DIR sudah benar."
        )

    for kategori_dir in sorted(datasheet_path.iterdir()):
        if not kategori_dir.is_dir():
            continue
        for kelas_dir in sorted(kategori_dir.iterdir()):
            if not kelas_dir.is_dir():
                continue
            gambar_list = [
                str(p) for p in kelas_dir.iterdir()
                if p.suffix.lower() in IMG_EXTS
            ]
            if not gambar_list:
                print(f"  [!] Kosong (tidak ada gambar): {kelas_dir}")
                continue
            classes[kelas_dir.name] = gambar_list
            print(f"  Kelas '{kelas_dir.name}' ({kategori_dir.name}): {len(gambar_list)} gambar")

    return classes


# ════════════════════════════════════════════════════════════════════════════════
def build_dataset(classes, out_dir):
    """
    Buat struktur YOLO classification:
      out_dir/train/<kelas>/gambar...
      out_dir/val/<kelas>/gambar...
    """
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)

    for kelas, gambar_list in classes.items():
        random.shuffle(gambar_list)
        split    = max(1, int(len(gambar_list) * TRAIN_RATIO))
        trn_imgs = gambar_list[:split]
        val_imgs = gambar_list[split:] or [gambar_list[-1]]

        for img_path in trn_imgs:
            dst = os.path.join(out_dir, "train", kelas)
            os.makedirs(dst, exist_ok=True)
            shutil.copy2(img_path, dst)

        for img_path in val_imgs:
            dst = os.path.join(out_dir, "val", kelas)
            os.makedirs(dst, exist_ok=True)
            shutil.copy2(img_path, dst)

    print(f"\n[DATASET] Folder dataset dibuat: {out_dir}")


# ════════════════════════════════════════════════════════════════════════════════
def main():
    print("=" * 55)
    print("  TRAINING YOLOv8-cls -- Deteksi Limbah Medis")
    print("=" * 55)

    # Cek ultralytics
    try:
        from ultralytics import YOLO
        print("[OK] ultralytics terinstall.")
    except ImportError:
        print("\n[ERROR] ultralytics belum terinstall!")
        print("Jalankan:\n    pip install ultralytics\n")
        return

    # Scan kelas
    print(f"\n[SCAN] Membaca dataset dari:\n  {DATASHEET_DIR}\n")
    try:
        classes = scan_classes(DATASHEET_DIR)
    except FileNotFoundError as e:
        print(f"\n[ERROR] {e}")
        return

    if not classes:
        print("\n[ERROR] Tidak ada kelas ditemukan!")
        print("Pastikan subfolder Datasheet berisi file gambar.")
        return

    total_imgs = sum(len(v) for v in classes.values())
    print(f"\n[INFO] Total kelas  : {len(classes)}")
    print(f"[INFO] Nama kelas   : {list(classes.keys())}")
    print(f"[INFO] Total gambar : {total_imgs}")

    if total_imgs < 10:
        print("\n[WARNING] Gambar sangat sedikit -- akurasi mungkin rendah.")
        print("Disarankan minimal 30 gambar per kelas.")

    # Build dataset
    print("\n[DATASET] Menyiapkan folder dataset...")
    random.seed(42)
    build_dataset(classes, YOLO_DATA_DIR)

    # Training
    # Gunakan yolov8n-cls.pt -> sudah otomatis task=classify
    # JANGAN tambahkan parameter task= di sini, itu yang menyebabkan error
    print(f"\n[TRAIN] Memulai training...")
    print(f"  Model   : {MODEL_BASE}  (classification)")
    print(f"  Epochs  : {EPOCHS}")
    print(f"  ImgSize : {IMG_SIZE}")
    print(f"  Batch   : {BATCH_SIZE}")
    print(f"  Output  : {MODEL_OUT_DIR}\n")

    model = YOLO(MODEL_BASE)

    model.train(
        data     = YOLO_DATA_DIR,
        epochs   = EPOCHS,
        imgsz    = IMG_SIZE,
        batch    = BATCH_SIZE,
        project  = BASE_DIR,
        name     = "waste_model",
        exist_ok = True,
        patience = 15,
        verbose  = True,
    )

    # Cari hasil
    print("\n" + "=" * 55)
    print("  TRAINING SELESAI!")
    print("=" * 55)

    best_pt = os.path.join(MODEL_OUT_DIR, "weights", "best.pt")
    if os.path.exists(best_pt):
        print(f"  Model: {best_pt}")
        print("\n  Sekarang jalankan: python main_yolo.py")
    else:
        print("  Mencari best.pt...")
        for root, dirs, files in os.walk(BASE_DIR):
            for f in files:
                if f == "best.pt":
                    found = os.path.join(root, f)
                    print(f"  Ditemukan: {found}")
                    print("  Update MODEL_PATH di main_yolo.py sesuai path ini.")
    print("=" * 55)


if __name__ == "__main__":
    main()