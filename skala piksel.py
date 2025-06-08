import cv2
import numpy as np

# Ukuran asli objek dalam cm
actual_width_cm = 16
actual_height_cm = 16

# HSV Range dari objek
lower_hsv_wadah = np.array([125, 50, 80])
upper_hsv_wadah = np.array([160, 255, 255])

# Buka webcam
cap = cv2.VideoCapture(1)

# Buat jendela dengan ukuran yang bisa diatur
#cv2.namedWindow("Object Detection & Pixel Scale", cv2.WINDOW_NORMAL)
#cv2.resizeWindow("Object Detection & Pixel Scale", 1200, 600)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Konversi ke HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Buat mask untuk objek berdasarkan HSV
    mask = cv2.inRange(hsv, lower_hsv_wadah, upper_hsv_wadah)

    # Temukan kontur dari objek
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # filter area kecil
            x, y, w, h = cv2.boundingRect(contour)

            # Hitung skala piksel
            cm_per_pixel_width = actual_width_cm / w
            cm_per_pixel_height = actual_height_cm / h
            avg_cm_per_pixel = (cm_per_pixel_width + cm_per_pixel_height) / 2

            # Gambar bounding box dan info
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"W: {w}px, H: {h}px", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Scale: {avg_cm_per_pixel:.3f} cm/pixel", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Tampilkan hasil
    cv2.imshow("Object Detection & Pixel Scale", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Bersihkan
cap.release()
cv2.destroyAllWindows()
