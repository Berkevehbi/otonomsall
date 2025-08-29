import cv2
import numpy as np

print("Dönüşüm matrisi oluşturuluyor...")

# Adım 2'de senin seçtiğin ve doğruladığımız kaynak noktaları
src_points = np.float32([
    [329, 699],  # Sol Alt
    [1278, 670],  # Sağ Alt
    [776, 418],  # Sağ Üst
    [603, 418]   # Sol Üst
])

# Kuşbakışı görüntümüzün hedef boyutu ve köşe noktaları.
# Bu boyutları daha sonra ihtiyaca göre değiştirebiliriz.
output_width = 640
output_height = 480

dst_points = np.float32([
    [0, output_height - 1],               # Sol Alt
    [output_width - 1, output_height - 1],# Sağ Alt
    [output_width - 1, 0],                # Sağ Üst
    [0, 0]                                # Sol Üst
])

# Perspektif dönüşüm matrisini (M) hesapla
M = cv2.getPerspectiveTransform(src_points, dst_points)

# Bu matrisi daha sonra kullanmak üzere '.npy' formatında kaydet
np.save("ipm_matrix.npy", M)

print(f"Başarılı! Dönüşüm matrisi 'ipm_matrix.npy' olarak kaydedildi.")
print("Kaydedilen Matris:")
print(M)
