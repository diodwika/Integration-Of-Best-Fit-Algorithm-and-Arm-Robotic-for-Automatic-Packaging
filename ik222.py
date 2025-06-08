import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Panjang segmen lengan
l1 = 19.5  # cm
l2 = 12.0  # cm
l3 = 25.0  # cm

# Posisi awal robot
robot_x = 60.0  # cm
robot_y = 0.0   # cm
robot_z = 15.5  # cm (base setinggi 10 cm)

def calculate_angles(x, y, z):
    try:
        # Hitung theta_1 (rotasi pada poros Z)
        theta_1 = np.arctan2(y - robot_y, x - robot_x)
        theta_1 = np.degrees(theta_1) % 180

        # Hitung jarak horizontal a dan b
        a = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
        b = a - l3

        # Hitung jarak vertikal, dikurangi tinggi base (10 cm)
        z_offset = z - robot_z

        # Hitung jarak c (hipotenusa pada bidang vertikal)
        c = np.sqrt(b**2 + z_offset**2)

        # Validasi apakah target dapat dijangkau
        if c > l1 + l2 or c < abs(l1 - l2):
            raise ValueError("Target di luar jangkauan lengan robot.")

        # Hitung sudut alpha1 dan alpha2 untuk theta_2
        alpha_1 = np.arctan2(z_offset, b)
        alpha_2 = np.arccos((l1**2 + c**2 - l2**2) / (2 * l1 * c))
        theta_2 = alpha_1 + alpha_2

        # Hitung theta_3 (sudut antara l1 dan l2)
        theta_3 = (np.arccos((l1**2 + l2**2 - c**2) / (2 * l1 * l2))) + np.pi

        # Hitung beta1 dan beta2 untuk theta_4
        beta_1 = np.arccos((l2**2 + c**2 - l1**2) / (2 * l2 * c))
        beta_2 = np.arctan2(b, z_offset)
        theta_4 = beta_1 + beta_2 + np.radians(90) + np.pi

        # Konversi sudut ke derajat
        theta_2 = np.degrees(theta_2)
        theta_3 = np.degrees(theta_3)
        theta_4 = np.degrees(theta_4)

        # Normalisasi sudut
        theta_2 = (theta_2 + 180) % 360 - 180
        theta_3 = (theta_3 + 180) % 360 - 180
        theta_4 = (theta_4 + 180) % 360 - 180

        return theta_1, theta_2, theta_3, theta_4

    except ValueError as e:
        print(f"Error: {e}")
        return None, None, None, None

def plot_arm(x, y, z, theta_1, theta_2, theta_3, theta_4):
    # Konversi sudut ke radian
    theta_1, theta_2, theta_3, theta_4 = map(np.radians, [theta_1, theta_2, theta_3, theta_4])

    # Posisi awal
    x0, y0, z0 = robot_x, robot_y, robot_z

    # Hitung posisi tiap joint
    x1, y1, z1 = x0, y0, z0
    x2 = x1 + l1 * np.cos(theta_2) * np.cos(theta_1)
    y2 = y1 + l1 * np.cos(theta_2) * np.sin(theta_1)
    z2 = z1 + l1 * np.sin(theta_2)

    x3 = x2 + l2 * np.cos(theta_2 + theta_3) * np.cos(theta_1)
    y3 = y2 + l2 * np.cos(theta_2 + theta_3) * np.sin(theta_1)
    z3 = z2 + l2 * np.sin(theta_2 + theta_3)

    x4 = x3 + l3 * np.cos(theta_2 + theta_3 + theta_4) * np.cos(theta_1)
    y4 = y3 + l3 * np.cos(theta_2 + theta_3 + theta_4) * np.sin(theta_1)
    z4 = z3 + l3 * np.sin(theta_2 + theta_3 + theta_4)

    # Plot lengan robot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([x0, x2, x3, x4], [y0, y2, y3, y4], [z0, z2, z3, z4], 'bo-', linewidth=2)
    ax.scatter([x, x4], [y, y4], [z, z4], c=['red', 'green'], label=['Target', 'End-Effector'])

    # Set batasan dan label plot
    ax.set_xlim([0, 120])
    ax.set_ylim([-60, 60])
    ax.set_zlim([0, 60])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Simulasi Lengan Robot dengan Base 10 cm')
    plt.legend()
    plt.show()

# Input koordinat target
x = float(input("Masukkan koordinat x: "))
y = float(input("Masukkan koordinat y: "))
z = float(input("Masukkan koordinat z: "))

# Validasi koordinat target
if x < 0 or x > 120 or y < -60 or y > 60 or z < 0:
    print("Error: Koordinat target berada di luar area kerja yang ditentukan.")
else:
    theta_1, theta_2, theta_3, theta_4 = calculate_angles(x, y, z)
    if theta_1 is not None:
        print(f"theta_1: {theta_1:.2f}°")
        print(f"theta_2: {theta_2:.2f}°")
        print(f"theta_3: {theta_3:.2f}°")
        print(f"theta_4: {theta_4:.2f}°")
        plot_arm(x, y, z, theta_1, theta_2, theta_3, theta_4)
