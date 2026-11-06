import cv2
import numpy as np
import json
import time

# 1. Video Kaynağı (Pazartesi yüklenecek video için 'video.mp4', canlı test için 0)
VIDEO_SOURCE = 0  # Canlı webcam testi için 0, video dosyası için "ardy_video.mp4"

cap = cv2.VideoCapture(VIDEO_SOURCE)

# Ardy karakterinin baskın rengine göre HSV filtresi (Varsayılan: Yeşil/Mavi tonlar)
# Pazartesi günü çekilen videodaki kıyafet/model rengine göre bu aralıklar güncellenir.
LOWER_HSV = np.array([35, 50, 50])   # HSV Alt Sınır
UPPER_HSV = np.array([85, 255, 255]) # HSV Üst Sınır

def pixel_to_world_coords(center_x, center_y, frame_w, frame_h):
    """
    Görüntüdeki piksel merkezini (x, y), C++ DLL'imizdeki GOTO hedefi
    olan 3D dünya koordinatlarına (X, Y, Z) dönüştürür.
    """
    # Ekran ortasını 0,0 kabul eden normalize koordinat (-1.0 ile +1.0 arası)
    norm_x = (center_x - (frame_w / 2.0)) / (frame_w / 2.0)
    norm_y = (center_y - (frame_h / 2.0)) / (frame_h / 2.0)
    
    # Derinlik ve düzlem ölçekleme (Simülasyon haritasına göre katsayılar ayarlanır)
    world_x = norm_x * 6.0          # X ekseni (sağ/sol)
    world_y = 0.0                   # Zemin seviyesi
    world_z = (1.0 - norm_y) * 8.0  # Z ekseni (ileri/geri derinlik)
    
    return round(world_x, 2), round(world_y, 2), round(world_z, 2)

print("OpenCV Ardy Nesne Takip Modülü Başlatıldı. Çıkış için 'q' tuşuna basın.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Video yayını sonlandı veya dosya okunamadı.")
        break

    frame_h, frame_w = frame.shape[:2]

    # Görüntü İşleme Adımları: BGR -> HSV -> Blur -> Maske
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(blurred, LOWER_HSV, UPPER_HSV)

    # Gürültü Temizleme (Erosion & Dilation)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Konturları (Nesne Sınırlarını) Bul
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    target_pos = None

    if contours:
        # Alanı en büyük olan konturu (hedef Ardy karakterini) seç
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 400: # Yanlış tespitleri engellemek için alan eşiği
            x, y, w, h = cv2.boundingRect(c)
            center_x = x + (w // 2)
            center_y = y + (h // 2)

            # 3D Dünya koordinatını hesapla
            wx, wy, wz = pixel_to_world_coords(center_x, center_y, frame_w, frame_h)
            target_pos = {"x": wx, "y": wy, "z": wz}

            # Görselleştirme: Hedef karakteri yeşil kutu içine al ve koordinat yaz
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Ardy Target -> X:{wx} Z:{wz}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Bu hedef koordinatı C++ SDK tarafındaki GOTO mekanizmasına gönderilecek veri yapısıdır
            # Örnek Çıktı: {"x": 2.4, "y": 0.0, "z": 5.1}

    # Ekran çıktısını göster
    cv2.imshow("Ardy Real-Time Tracker (OpenCV)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()