````markdown
## 🚀 Kullanım Senaryoları ve ROS 2 Komutları

### **1. Docker İmajını Derle**
```bash
sudo docker build -t robotaksi_ws:latest .
````

* Tüm ROS 2 workspace ve bağımlılıkları tek komutta imaj olarak hazırlanır.

### **2. Konteyneri Çalıştır**

```bash
docker run -it --rm \
  --name ros2-container \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  --network bridge \
  --privileged \
  robotaksi_ws:latest bash

docker run -it --rm --name ros2-container --gpus all --privileged --network bridge robotaksi_ws:latest bash
```

* X11 görüntü aktarımı, GPU erişimi ve tam araç haberleşmesi için optimize edilmiş başlatma komutu.

---

### **3. Tüm Sistemi Otonom Modda Başlatmak**

```bash
ros2 launch robotaxi_bringup autonomous_drive.launch.py
```

* Tüm ana node’ları (donanım arayüzü, perception, planlama, kontrol) **tek komutla başlatır**.
* Gerçek araç otonom sürüş ve şerit takibi için ana entrypoint.

---

### **4. Modülleri Tek Tek Manuel Başlatmak**

#### **Donanım Arayüzü**

```bash
ros2 run robotaxi_hw_interface hw_interface_node
```

* Motor, direksiyon ve fren komutlarını gerçek araca iletir.
* Otomatik başlatma sekansı ile aracı sürüşe hazırlar.

#### **Şerit ve Obje Algılama (Perception - YOLOP)**

```bash
ros2 run robotaxi_perception perception_node
```

* Kameradan gelen görüntüden şerit maskesi ve drivable area çıkarır.

#### **Yol Planlayıcı**

```bash
ros2 run robotaxi_planning path_planner_node
```

* Şerit maskesinden araç ofsetini ve eğrilik yarıçapını hesaplar.
* PID node’u için `/planning/lane_offset_m` topic’ine veri yollar.

#### **PID Kontrolcü**

```bash
ros2 run robotaxi_control pid_controller_node
```

* Planlayıcıdan gelen ofsete göre aracı şeritte ortalar ve hız/steer komutu üretir.

#### **Klavye ile Manuel Sürüş (Teleop)**

```bash
ros2 run robotaxi_control teleop_keyboard_node
```

* Klavyeden (w/a/s/d/space/i) komutları ile aracı elle kontrol etmeni sağlar.

#### **Trafik Levhası Tanıma**

```bash
ros2 run yolo_traffic_sign_detector yolo_traffic_sign_detector
```

* YOLOv8 modeliyle trafik işaretlerini tespit eder ve sonuçları işaretli olarak yeni bir topic’e yayınlar.

#### **Test Videosu ile Sanal Kamera Yayını**

```bash
ros2 run robotaxi_perception test_video_publisher
```

* Bir video dosyasını, sanki canlı kamera gibi belirli bir topic’e yayınlar.
* Gerçek kamera olmadan perception ve downstream node’ları test etmek için kullanılır.

