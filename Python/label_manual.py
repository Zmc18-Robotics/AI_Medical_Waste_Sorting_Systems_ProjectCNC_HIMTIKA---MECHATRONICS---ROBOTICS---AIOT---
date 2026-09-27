"""
label_manual.py - Tool untuk menggambar bounding box secara manual (Non-Roboflow)
================================================================================
Gunakan program ini untuk memberi kotak pada gambar-gambar di folder Datasheet.
1. Jalankan program ini.
2. Tarik garis (drag) menggunakan mouse untuk membuat kotak.
3. Tekan SPASI atau ENTER untuk menyimpan.
4. Ulangi sampai semua gambar berlabel.
"""
import os
import cv2
from pathlib import Path

BASE_DIR = r"C:\Users\Muhammad Zidane A\Documents\PTN\HIMTIKA\Lomba CNC\Code and all\Python"
DATASHEET_DIR = os.path.join(BASE_DIR, "Datasheet")

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

def scan_classes(datasheet_dir):
    classes = {}
    datasheet_path = Path(datasheet_dir)
    if not datasheet_path.exists():
        return classes

    for kategori_dir in sorted(datasheet_path.iterdir()):
        if not kategori_dir.is_dir(): continue
        for kelas_dir in sorted(kategori_dir.iterdir()):
            if not kelas_dir.is_dir(): continue
            classes[kelas_dir.name] = str(kelas_dir)
            
    return classes

def main():
    print("=" * 60)
    print("          MANUAL LABELING (NON-ROBOFLOW)")
    print("=" * 60)
    print(" PETUNJUK:")
    print(" 1. Gunakan mouse (klik & tahan) untuk menggambar kotak.")
    print(" 2. Tekan ENTER atau SPASI setelah kotak digambar untuk Simpan.")
    print(" 3. Tekan C jika kotak salah & ingin mengulang.")
    print(" 4. Tekan ESC jika ingin melewati (skip) foto tanpa label.")
    print("=" * 60)

    classes_dict = scan_classes(DATASHEET_DIR)
    if not classes_dict:
        print(f"Folder {DATASHEET_DIR} tidak ditemukan atau kosong.")
        return

    class_names = sorted(list(classes_dict.keys()))
    print("Kelas yang ditemukan:", class_names)
    
    # Buat map ID
    class_to_id = {name: idx for idx, name in enumerate(class_names)}

    for class_name in class_names:
        class_dir = classes_dict[class_name]
        class_id = class_to_id[class_name]
        print(f"\n--- Memproses kelas: {class_name} (ID: {class_id}) ---")
        
        for img_name in os.listdir(class_dir):
            ext = os.path.splitext(img_name)[1].lower()
            if ext not in IMG_EXTS:
                continue
                
            img_path = os.path.join(class_dir, img_name)
            txt_path = os.path.splitext(img_path)[0] + ".txt"
            
            if os.path.exists(txt_path):
                print(f"  [SKIP] Sudah ada label: {img_name}")
                continue
                
            img = cv2.imread(img_path)
            if img is None:
                continue
                
            h, w = img.shape[:2]
            scale = 1.0
            if max(h, w) > 800:
                scale = 800 / max(h, w)
                display_img = cv2.resize(img, (int(w * scale), int(h * scale)))
            else:
                display_img = img.copy()

            window_name = f"Gambar: {img_name} | KELAS: {class_name}"
            
            print(f"  Menampilkan: {img_name}... (Gambarlah kotak)")
            roi = cv2.selectROI(window_name, display_img, showCrosshair=True, fromCenter=False)
            cv2.destroyWindow(window_name)
            
            if roi == (0, 0, 0, 0):
                print(f"  --> Dilewati (Skipped).")
                continue
                
            x_disp, y_disp, w_disp, h_disp = roi
            
            x = int(x_disp / scale)
            y = int(y_disp / scale)
            box_w = int(w_disp / scale)
            box_h = int(h_disp / scale)
            
            x_center = (x + box_w / 2.0) / w
            y_center = (y + box_h / 2.0) / h
            norm_w = box_w / w
            norm_h = box_h / h
            
            # Format YOLO: class x_center y_center width height
            line = f"{class_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}\n"
            with open(txt_path, "w") as f:
                f.write(line)
            
            print(f"  --> Tersimpan: {txt_path}")

    print("\nProses labeling selesai! Sekarang Anda bisa menjalankan train_yolo.py")

if __name__ == "__main__":
    main()
