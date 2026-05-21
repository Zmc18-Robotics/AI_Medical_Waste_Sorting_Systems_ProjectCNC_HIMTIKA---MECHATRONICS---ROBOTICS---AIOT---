"""
=============================================================
  train_yolo.py  —  Training YOLOv8 untuk Deteksi Limbah Medis
=============================================================
  VERSI BARU: Object Detection (Menggunakan Bounding Box)
  
  Jalankan label_manual.py TERLEBIH DAHULU untuk memberi
  kotak (bounding box) pada gambar di folder Datasheet.

  Jalankan file ini:
      python train_yolo.py
=============================================================
"""

import os
import shutil
import random
import yaml
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

# Model Object Detection (bukan -cls.pt)
MODEL_BASE  = "yolov8n.pt"

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def scan_classes(datasheet_dir):
    """Membaca folder Datasheet dan mencari gambar yang SUDAH memiliki file .txt"""
    classes_dict = {}
    datasheet_path = Path(datasheet_dir)

    if not datasheet_path.exists():
        raise FileNotFoundError(f"Folder Datasheet tidak ditemukan: {datasheet_dir}")

    # Kumpulkan nama kelas
    for kategori_dir in sorted(datasheet_path.iterdir()):
        if not kategori_dir.is_dir(): continue
        for kelas_dir in sorted(kategori_dir.iterdir()):
            if not kelas_dir.is_dir(): continue
            classes_dict[kelas_dir.name] = str(kelas_dir)

    class_names = sorted(list(classes_dict.keys()))
    
    # Kumpulkan file yang siap train
    valid_images = {name: [] for name in class_names}
    
    for class_name in class_names:
        class_dir = classes_dict[class_name]
        for img_name in os.listdir(class_dir):
            if os.path.splitext(img_name)[1].lower() not in IMG_EXTS:
                continue
                
            img_path = os.path.join(class_dir, img_name)
            txt_path = os.path.splitext(img_path)[0] + ".txt"
            
            if os.path.exists(txt_path):
                valid_images[class_name].append((img_path, txt_path))

    return class_names, valid_images


def build_dataset(class_names, valid_images, out_dir):
    """Membuat struktur dataset Object Detection untuk YOLOv8"""
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)

    # Buat direktori
    for split in ["train", "val"]:
        os.makedirs(os.path.join(out_dir, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(out_dir, split, "labels"), exist_ok=True)

    total_train = 0
    total_val = 0

    for class_name, img_txt_pairs in valid_images.items():
        if not img_txt_pairs:
            continue
            
        random.shuffle(img_txt_pairs)
        split = max(1, int(len(img_txt_pairs) * TRAIN_RATIO))
        trn_pairs = img_txt_pairs[:split]
        val_pairs = img_txt_pairs[split:] or [img_txt_pairs[-1]]

        for img_path, txt_path in trn_pairs:
            shutil.copy2(img_path, os.path.join(out_dir, "train", "images"))
            shutil.copy2(txt_path, os.path.join(out_dir, "train", "labels"))
            total_train += 1

        for img_path, txt_path in val_pairs:
            shutil.copy2(img_path, os.path.join(out_dir, "val", "images"))
            shutil.copy2(txt_path, os.path.join(out_dir, "val", "labels"))
            total_val += 1

    print(f"\n[DATASET] Dataset Detection dibuat di: {out_dir}")
    print(f"  Train: {total_train} gambar")
    print(f"  Val  : {total_val} gambar")
    
    # Buat data.yaml
    yaml_path = os.path.join(out_dir, "data.yaml")
    data_yaml = {
        "path": out_dir,
        "train": "train/images",
        "val": "val/images",
        "nc": len(class_names),
        "names": class_names
    }
    with open(yaml_path, "w") as f:
        yaml.dump(data_yaml, f, sort_keys=False)
        
    return yaml_path


def main():
    print("=" * 55)
    print("  TRAINING YOLOv8 -- Object Detection")
    print("=" * 55)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] ultralytics belum terinstall! Jalankan: pip install ultralytics")
        return

    print(f"\n[SCAN] Mencari gambar berlabel di:\n  {DATASHEET_DIR}\n")
    try:
        class_names, valid_images = scan_classes(DATASHEET_DIR)
    except FileNotFoundError as e:
        print(f"\n[ERROR] {e}")
        return

    total_labeled = sum(len(pairs) for pairs in valid_images.values())
    
    print(f"[INFO] Kelas ditemukan : {class_names}")
    print(f"[INFO] Gambar berlabel : {total_labeled} gambar")
    
    if total_labeled == 0:
        print("\n[ERROR] TIDAK ADA GAMBAR BERLABEL (.txt)!")
        print("Silakan jalankan 'python label_manual.py' terlebih dahulu untuk")
        print("memberi kotak bounding box pada gambar di folder Datasheet.")
        return

    # Build dataset
    yaml_path = build_dataset(class_names, valid_images, YOLO_DATA_DIR)

    # Training
    print(f"\n[TRAIN] Memulai training Object Detection...")
    print(f"  Model   : {MODEL_BASE}")
    print(f"  Epochs  : {EPOCHS}")
    print(f"  ImgSize : {IMG_SIZE}")
    print(f"  Batch   : {BATCH_SIZE}\n")

    model = YOLO(MODEL_BASE)

    model.train(
        data     = yaml_path,
        epochs   = EPOCHS,
        imgsz    = IMG_SIZE,
        batch    = BATCH_SIZE,
        project  = BASE_DIR,
        name     = "waste_model",
        exist_ok = True,
        patience = 15,
        verbose  = True,
    )

    print("\n" + "=" * 55)
    print("  TRAINING SELESAI!")
    print("=" * 55)
    
    best_pt = os.path.join(MODEL_OUT_DIR, "weights", "best.pt")
    if os.path.exists(best_pt):
        print(f"  Model siap: {best_pt}")
        print("\n  Sekarang Anda bisa menjalankan: python main_yolo.py")
    else:
        print("  Selesai, silakan cek folder waste_model/weights/")
    print("=" * 55)


if __name__ == "__main__":
    main()
