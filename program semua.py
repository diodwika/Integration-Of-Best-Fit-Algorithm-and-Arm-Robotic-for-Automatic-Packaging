import cv2
import numpy as np
import time
import paho.mqtt.client as mqtt

# Konfigurasi MQTT
MQTT_BROKER = "192.168.207.178"
MQTT_PORT = 1883
MQTT_TOPIC_SEND = "esp32/coordinates"
MQTT_TOPIC_RECEIVE = "esp32/commands"
MQTT_TOPIC_ERROR = "esp32/error"
MQTT_TOPIC_REQUEST_CORRECTION = "esp32/request_correction"
MQTT_TOPIC_ACKNOWLEDGE_STATUS = "esp32/acknowledge_status"

# Variabel global
last_can_coordinates = None
last_placement_coordinates = None
send_can_coordinates = False
kaleng_ditempatkan = False
error_threshold_x = 5  # Threshold error dalam cm
error_threshold_y = 10  # Threshold error dalam cm
current_target = None  # 'can' atau 'placement'
check_error = False  # Flag untuk memulai deteksi error
current_error_data = None  # Menyimpan error data saat ini

# Rentang warna HSV untuk kuning cerah (end effector)
lower_yellow = np.array([90, 30, 150])
upper_yellow = np.array([115, 300, 255])

# Variabel untuk sistem penempatan baru
ruang_kosong = []
posisi_kaleng_diletakkan = []
jumlah_kaleng = 0
mask_area = None


# Callback MQTT
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(MQTT_TOPIC_RECEIVE)
    client.subscribe(MQTT_TOPIC_REQUEST_CORRECTION)


def on_message(client, userdata, msg):
    global send_can_coordinates, current_target, check_error

    message = msg.payload.decode()
    print(f"Received message on topic {msg.topic}: {message}")

    if msg.topic == MQTT_TOPIC_RECEIVE:
        if message == "1":
            if last_placement_coordinates is not None:
                client.publish(MQTT_TOPIC_SEND, f"Placement: {last_placement_coordinates}")
                print(f"Sent placement coordinates: {last_placement_coordinates}")
                current_target = 'placement'
        elif message == "2":
            send_can_coordinates = True
            current_target = 'can'

    elif msg.topic == MQTT_TOPIC_REQUEST_CORRECTION:
        if message == "error handling":
            print("Memulai deteksi error...")
            check_error = True  # Aktifkan pengecekan error


# Inisialisasi MQTT Client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()


# Fungsi deteksi objek
def detect_object(frame, lower_hsv, upper_hsv, label, mask_area=None):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    if mask_area is not None:
        mask = cv2.bitwise_and(mask, mask, mask=cv2.bitwise_not(mask_area))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        if label == "Kaleng":
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Center: ({center_x}, {center_y})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            area = cv2.contourArea(largest_contour)
            cv2.putText(frame, f"Area: {area:.2f} px", (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            return w, h, center_x, center_y

        elif label == "Wadah":
            cv2.drawContours(frame, [largest_contour], -1, (255, 0, 0), 2)
            coords = np.column_stack(np.where(mask > 0))

            if coords.size > 0:
                top_left = tuple(coords[np.argmin(coords[:, 0] + coords[:, 1])][::-1])
                cv2.circle(frame, top_left, 5, (0, 255, 255), -1)
                cv2.putText(frame, f"Top-Left: {top_left}", (top_left[0] + 10, top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                x, y, w, h = cv2.boundingRect(largest_contour)
                return top_left, w, h

    return None


# Fungsi deteksi end effector
def detect_end_effector(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
            cv2.putText(frame, f"End Effector: ({cx}, {cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            return cx, cy

    return None, None


# Fungsi koreksi posisi
def position_correction(frame, end_effector_x, end_effector_y, target_x, target_y, scale_mm_per_pixel):
    if end_effector_x is None or end_effector_y is None:
        return None, None, False

    try:
        cv2.line(frame, (int(end_effector_x), int(end_effector_y)), (int(target_x), int(target_y)), (0, 0, 255), 2)
    except Exception as e:
        print("Gagal menggambar garis", e)

    error_x = target_x - end_effector_x
    error_y = target_y - end_effector_y

    error_x_mm = error_x * scale_mm_per_pixel
    error_y_mm = error_y * scale_mm_per_pixel

    error_x_sign = "+" if error_x_mm >= 0 else ""
    error_y_sign = "+" if error_y_mm >= 0 else ""
    cv2.putText(frame, f"Error X: {error_x_sign}{error_x_mm:.2f} mm", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Error Y: {error_y_sign}{error_y_mm:.2f} mm", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    direction_x = "kiri" if error_x_mm < 0 else "kanan"
    direction_y = "bawah" if error_y_mm < 0 else "atas"
    cv2.putText(frame, f"Gerak X: {direction_x}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(frame, f"Gerak Y: {direction_y}", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    within_threshold = (abs(error_x_mm) < error_threshold_x)

    return error_x_mm, error_y_mm, within_threshold


# Fungsi transformasi koordinat
def transform_2d_to_3d(x_pixel, y_pixel, scale_mm_per_pixel):
    x_mm = x_pixel * scale_mm_per_pixel
    y_mm = y_pixel * scale_mm_per_pixel
    z_mm = 18
    return x_mm, y_mm, z_mm


# ✅ Fungsi baru: Cek overlap
def is_overlap(k1, k2):
    return not (
            k1["x"] + k1["width"] <= k2["x"] or
            k2["x"] + k2["width"] <= k1["x"] or
            k1["y"] + k1["height"] <= k2["y"] or
            k2["y"] + k2["height"] <= k1["y"]
    )


# ✅ Fungsi baru: Best fit yang diubah
def find_best_fit(ruang_kosong, w, h):
    ruang_kosong_urut = sorted(ruang_kosong, key=lambda r: (r["x"], r["y"]))  # Urut kiri ke kanan, atas ke bawah
    for ruang in ruang_kosong_urut:
        calon = {"x": ruang["x"], "y": ruang["y"], "width": w, "height": h}
        if not any(is_overlap(calon, k) for k in posisi_kaleng_diletakkan):
            if w <= ruang["width"] and h <= ruang["height"]:
                return ruang
    return None


# ✅ Fungsi baru: Update ruang kosong
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


# Inisialisasi kamera dan variabel
cap = cv2.VideoCapture(1)
scale_mm_per_pixel = 0.142
lower_hsv_kaleng = np.array([0, 0, 60])
upper_hsv_kaleng = np.array([124, 50, 255])
lower_hsv_wadah = np.array([118, 84, 80])
upper_hsv_wadah = np.array([160, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    mirrored_frame = frame

    # Deteksi end effector (warna kuning)
    ee_x, ee_y = detect_end_effector(mirrored_frame)

    # Deteksi kaleng dan wadah
    kaleng_data = detect_object(mirrored_frame, lower_hsv_kaleng, upper_hsv_kaleng, "Kaleng", mask_area)
    wadah_data = detect_object(mirrored_frame, lower_hsv_wadah, upper_hsv_wadah, "Wadah")

    if wadah_data:
        top_left, wadah_width, wadah_height = wadah_data

        if not ruang_kosong:
            ruang_kosong.append({"x": top_left[0], "y": top_left[1], "width": wadah_width, "height": wadah_height})

        if kaleng_data is not None:
            w, h, center_x, center_y = kaleng_data
            kaleng_3d = transform_2d_to_3d(center_x, center_y, scale_mm_per_pixel)
            last_can_coordinates = kaleng_3d

            if not kaleng_ditempatkan:
                client.publish(MQTT_TOPIC_SEND, f"Can: {last_can_coordinates}")
                print(f"Sent can coordinates: {last_can_coordinates}")
                kaleng_ditempatkan = True
                current_target = 'can'

            # Hanya periksa error jika ada permintaan "error handling"
            if check_error and ee_x is not None and ee_y is not None:
                if current_target == 'can':
                    target_x, target_y = center_x, center_y
                elif current_target == 'placement':
                    if best_fit:
                        target_x = best_fit["x"] + (w / 2)
                        target_y = best_fit["y"] + (h / 2)
                    else:
                        continue  # Skip jika tidak ada best_fit

                error_x_mm, error_y_mm, within_threshold = position_correction(
                    mirrored_frame, ee_x, ee_y, target_x, target_y, scale_mm_per_pixel)

                if error_x_mm is not None and error_y_mm is not None:
                    if within_threshold:
                        if current_target == 'can':
                            client.publish(MQTT_TOPIC_ACKNOWLEDGE_STATUS, "ok 1")
                        elif current_target == 'placement':
                            client.publish(MQTT_TOPIC_ACKNOWLEDGE_STATUS, "ok 2")
                            # Konfirmasi kaleng telah diletakkan
                            posisi_kaleng_diletakkan.append({
                                "x": best_fit["x"],
                                "y": best_fit["y"],
                                "width": w,
                                "height": h
                            })
                            jumlah_kaleng += 1

                            # Update mask area
                            if mask_area is None:
                                mask_area = np.zeros_like(mirrored_frame[:, :, 0])
                            cv2.rectangle(mask_area,
                                          (int(best_fit["x"]), int(best_fit["y"])),
                                          (int(best_fit["x"] + w), int(best_fit["y"] + h)),
                                          255, -1)

                            # Update ruang kosong
                            ruang_kosong = update_ruang_kosong(ruang_kosong, {
                                "x": best_fit["x"],
                                "y": best_fit["y"],
                                "width": w,
                                "height": h
                            })

                            print(f"Kaleng ke-{jumlah_kaleng} ditempatkan di ({best_fit['x']}, {best_fit['y']})")
                            kaleng_ditempatkan = False  # Reset untuk deteksi kaleng baru

                        print("Posisi sesuai threshold, mengirim 'ok'")
                    else:
                        client.publish(MQTT_TOPIC_ERROR, f"Error: {error_x_mm:.2f}, {error_y_mm:.2f}")
                        print(f"Mengirim error: X={error_x_mm:.2f}, Y={error_y_mm:.2f}")

                    check_error = False  # Matikan pengecekan sampai ada permintaan baru

            best_fit = find_best_fit(ruang_kosong, w, h)

            if best_fit:
                ref_x_new = best_fit["x"] + (w / 2)
                ref_y_new = best_fit["y"] + (h / 2)
                ref_point_new = (int(ref_x_new), int(ref_y_new))

                titik_peletakan_3d = transform_2d_to_3d(ref_x_new, ref_y_new, scale_mm_per_pixel)
                last_placement_coordinates = titik_peletakan_3d

                cv2.circle(mirrored_frame, ref_point_new, 5, (0, 255, 255), -1)
                cv2.line(mirrored_frame, (center_x, center_y), ref_point_new, (0, 255, 0), 2)

                # Koreksi posisi untuk penempatan (jika aktif)
                if check_error and current_target == 'placement' and ee_x is not None and ee_y is not None:
                    error_x_mm, error_y_mm, within_threshold = position_correction(
                        mirrored_frame, ee_x, ee_y, ref_x_new, ref_y_new, scale_mm_per_pixel)

                    if error_x_mm is not None and error_y_mm is not None:
                        if within_threshold:
                            client.publish(MQTT_TOPIC_ACKNOWLEDGE_STATUS, "ok 2")
                            # Konfirmasi kaleng telah diletakkan
                            posisi_kaleng_diletakkan.append({
                                "x": best_fit["x"],
                                "y": best_fit["y"],
                                "width": w,
                                "height": h
                            })
                            jumlah_kaleng += 1

                            # Update mask area
                            if mask_area is None:
                                mask_area = np.zeros_like(mirrored_frame[:, :, 0])
                            cv2.rectangle(mask_area,
                                          (int(best_fit["x"]), int(best_fit["y"])),
                                          (int(best_fit["x"] + w), int(best_fit["y"] + h)),
                                          255, -1)

                            # Update ruang kosong
                            ruang_kosong = update_ruang_kosong(ruang_kosong, {
                                "x": best_fit["x"],
                                "y": best_fit["y"],
                                "width": w,
                                "height": h
                            })

                            print(f"Kaleng ke-{jumlah_kaleng} ditempatkan di ({best_fit['x']}, {best_fit['y']})")
                            kaleng_ditempatkan = False  # Reset untuk deteksi kaleng baru

                            print("Posisi penempatan sesuai threshold, mengirim 'ok 2'")
                        else:
                            client.publish(MQTT_TOPIC_ERROR, f"Error: {error_x_mm:.2f}, {error_y_mm:.2f}")
                            print(f"Mengirim error penempatan: X={error_x_mm:.2f}, Y={error_y_mm:.2f}")

                        check_error = False  # Matikan pengecekan

        if send_can_coordinates and last_can_coordinates is not None:
            client.publish(MQTT_TOPIC_SEND, f"Can: {last_can_coordinates}")
            print(f"Sent new can coordinates: {last_can_coordinates}")
            send_can_coordinates = False

        # Visualisasi posisi kaleng yang sudah diletakkan
        for k in posisi_kaleng_diletakkan:
            cv2.rectangle(mirrored_frame,
                          (k["x"], k["y"]),
                          (k["x"] + k["width"], k["y"] + k["height"]),
                          (0, 255, 0), 2)

    cv2.imshow("Deteksi Objek dengan Koreksi Posisi", mirrored_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
client.loop_stop()