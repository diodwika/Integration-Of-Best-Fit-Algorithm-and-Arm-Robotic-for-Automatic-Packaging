import cv2
import numpy as np
from PIL import Image

# Buka gambar
image = cv2.imread('C:/Users/PC/OneDrive/Pictures/Screenshots/Screenshot 2025-06-07 165504.png')
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Hitung rentang HSV
h_min = hsv_image[:, :, 0].min()
h_max = hsv_image[:, :, 0].max()
s_min = hsv_image[:, :, 1].min()
s_max = hsv_image[:, :, 1].max()
v_min = hsv_image[:, :, 2].min()
v_max = hsv_image[:, :, 2].max()

print("Hue:", h_min, "-", h_max)
print("Saturation:", s_min, "-", s_max)
print("Value:", v_min, "-", v_max)
