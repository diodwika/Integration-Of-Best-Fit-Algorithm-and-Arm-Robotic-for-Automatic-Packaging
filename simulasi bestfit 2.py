import cv2
import numpy as np

scale_mm_per_pixel = 0.146
lower_kaleng = np.array([0, 0, 122])
upper_kaleng = np.array([110, 85, 255])
lower_wadah = np.array([122, 63, 119])
upper_wadah = np.array([160, 124, 255])

ruang_kosong = []
posisi_kaleng_diletakkan = []
jumlah_kaleng = 0

cap = cv2.VideoCapture(1)

deteksi_siap = True  # Awalnya deteksi kaleng aktif

def is_overlap(k1, k2):
    return not (
        k1["x"] + k1["width"] <= k2["x"] or
        k2["x"] + k2["width"] <= k1["x"] or
        k1["y"] + k1["height"] <= k2["y"] or
        k2["y"] + k2["height"] <= k1["y"]
    )

# ✅ Diubah: Urutkan ruang kosong dari kiri ke kanan, lalu atas ke bawah
def find_best_fit(ruang_kosong, w, h):
    ruang_kosong_urut = sorted(ruang_kosong, key=lambda r: (r["x"], r["y"]))  # Urut kiri ke kanan, atas ke bawah
    for ruang in ruang_kosong_urut:
        calon = {"x": ruang["x"], "y": ruang["y"], "width": w, "height": h}
        if not any(is_overlap(calon, k) for k in posisi_kaleng_diletakkan):
            if w <= ruang["width"] and h <= ruang["height"]:
                return ruang
    return None

def update_ruang_kosong(ruang_kosong, area):
    hasil = []
    for ruang in ruang_kosong:
        if is_overlap(ruang, area):
            kanan = {
                "x": area["x"] + area["width"],
                "y": ruang["y"],
                "width": (ruang["x"] + ruang["width"]) - (area["x"] + area["width"]),
                "height": ruang["height"]
            }
            bawah = {
                "x": ruang["x"],
                "y": area["y"] + area["height"],
                "width": ruang["width"],
                "height": (ruang["y"] + ruang["height"]) - (area["y"] + area["height"])
            }
            if kanan["width"] > 0 and kanan["height"] > 0:
                hasil.append(kanan)
            if bawah["width"] > 0 and bawah["height"] > 0:
                hasil.append(bawah)
        else:
            hasil.append(ruang)
    return hasil

kaleng_terdeteksi = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_kaleng = cv2.inRange(hsv, lower_kaleng, upper_kaleng)
    mask_wadah = cv2.inRange(hsv, lower_wadah, upper_wadah)

    cnts_wadah, _ = cv2.findContours(mask_wadah, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts_wadah:
        c = max(cnts_wadah, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        if not ruang_kosong:
            ruang_kosong.append({"x": x, "y": y, "width": w, "height": h})

    if deteksi_siap:
        cnts_kaleng, _ = cv2.findContours(mask_kaleng, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts_kaleng:
            c_kaleng = max(cnts_kaleng, key=cv2.contourArea)
            xk, yk, wk, hk = cv2.boundingRect(c_kaleng)

            best_fit = find_best_fit(ruang_kosong, wk, hk)
            if best_fit:
                kaleng_terdeteksi = {
                    "x": best_fit["x"],
                    "y": best_fit["y"],
                    "width": wk,
                    "height": hk
                }
                print("Kaleng terdeteksi. Tekan '1' untuk konfirmasi penempatan.")
                deteksi_siap = False

    # Menunggu konfirmasi dari tombol keyboard
    key = cv2.waitKey(1) & 0xFF
    if key == ord('1') and kaleng_terdeteksi:
        posisi_kaleng_diletakkan.append(kaleng_terdeteksi)
        ruang_kosong = update_ruang_kosong(ruang_kosong, kaleng_terdeteksi)
        jumlah_kaleng += 1
        print(f"Kaleng ke-{jumlah_kaleng} ditempatkan di ({kaleng_terdeteksi['x']}, {kaleng_terdeteksi['y']})")
        kaleng_terdeteksi = None
        deteksi_siap = True

    elif key == ord('q'):
        break

    # Visualisasi
    for k in posisi_kaleng_diletakkan:
        cv2.rectangle(frame, (k["x"], k["y"]), (k["x"] + k["width"], k["y"] + k["height"]), (0, 255, 0), 2)
    for r in ruang_kosong:
        cv2.rectangle(frame, (r["x"], r["y"]), (r["x"] + r["width"], r["y"] + r["height"]), (255, 0, 0), 1)
    if kaleng_terdeteksi:
        cv2.rectangle(frame, (kaleng_terdeteksi["x"], kaleng_terdeteksi["y"]),
                      (kaleng_terdeteksi["x"] + kaleng_terdeteksi["width"],
                       kaleng_terdeteksi["y"] + kaleng_terdeteksi["height"]),
                      (0, 255, 255), 2)

    cv2.imshow("Simulasi Best Fit Kamera", frame)

cap.release()
cv2.destroyAllWindows()
