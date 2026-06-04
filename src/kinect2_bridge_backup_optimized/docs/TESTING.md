# kinect2_bridge — Guía de pruebas

## Requisitos previos

- Kinect v2 conectada por USB **3.0** (puerto azul, 5000M en `lsusb`)
- libfreenect2 instalado en `/usr/local`
- ROS 2 Humble con `cv_bridge`, `image_transport`, `rqt_image_view`

---

## 1. Compilar

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select kinect2_bridge
source install/setup.bash
```

Salida esperada: `Finished <<< kinect2_bridge` sin errores (solo warnings de OpenCV — ver sección 6).

---

## 2. Lanzar el nodo

```bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py
```

Con color reducido activado (útil para visual servoing a mayor FPS):

```bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py publish_resized_color:=true
```

Con pipeline CPU (debug, muy lento):

```bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py pipeline:=cpu
```

Parámetros sueltos vía `--ros-args`:

```bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py \
  --ros-args -p timeout_ms:=2000 -p publish_ir:=false
```

---

## 3. Verificar tópicos publicados

```bash
ros2 topic list | grep kinect2
```

Salida esperada (todos los streams activos):

```
/kinect2/color/camera_info
/kinect2/color/image_raw
/kinect2/color/image_raw/compressed       ← transport plugin, OK
/kinect2/color/image_raw/compressedDepth  ← solo aparece si hay suscriptor
/kinect2/color/image_resized              ← solo si publish_resized_color:=true
/kinect2/depth/camera_info
/kinect2/depth/image_raw
/kinect2/ir/camera_info
/kinect2/ir/image_raw
```

---

## 4. Medir FPS

```bash
# En terminales separadas, con el nodo corriendo:
ros2 topic hz /kinect2/depth/image_raw
ros2 topic hz /kinect2/ir/image_raw
ros2 topic hz /kinect2/color/image_raw
```

### Resultados esperados tras la optimización

| Tópico | Antes (hilo único) | Después (hilos separados) |
|--------|--------------------|--------------------------|
| depth  | ~13–15 Hz          | ~25–30 Hz                |
| IR     | ~13–15 Hz          | ~25–30 Hz                |
| color  | ~10–11 Hz          | ~15–20 Hz                |

> **Por qué color es más lento:** el decode JPEG de color usa TurboJPEG (~36 ms/frame ≈ 27 Hz máx teórico). Depth e IR usan OpenGL GPU (~7–9 ms ≈ 30 Hz). Con hilos separados, el lento no bloquea al rápido.

---

## 5. Visualizar en rqt_image_view

```bash
rqt_image_view
```

### Tópicos seguros para abrir

| Tópico | Tipo | Notas |
|--------|------|-------|
| `/kinect2/color/image_raw` | BGR8 1920×1080 | |
| `/kinect2/color/image_resized` | BGR8 640×360 | más liviano para visualizar |
| `/kinect2/ir/image_raw` | 16UC1 512×424 | imagen infrarroja |
| `/kinect2/depth/image_raw` | 32FC1 512×424 | valores en metros |

### Tópico que NO abrir (error conocido)

```
/kinect2/depth/image_raw/compressed   ← ERROR
```

Error que produce:
```
Compressed Image Transport - JPEG compression requires 8/16-bit color format
input format is: 32FC1
```

**Causa:** `image_transport` intenta comprimir depth con JPEG, pero 32FC1 no es soportado.
**Solución correcta para depth comprimido:** usar el transport `compressedDepth` (no `compressed`):

```bash
ros2 topic echo /kinect2/depth/image_raw/compressedDepth \
  --no-arr sensor_msgs/msg/CompressedImage
```

---

## 6. Warnings de OpenCV (cosméticos)

Al compilar aparece:

```
libopencv_imgcodecs.so.4.5d may conflict with libopencv_imgcodecs.so.408
```

**Causa:** `cv_bridge` de ROS 2 apt fue compilado con OpenCV 4.5 (con debug: `4.5d`),
mientras que `find_package(OpenCV)` en el sistema enlaza con OpenCV 4.0.8 (`408`).
Ambas versiones conviven en el proceso.

**Impacto real:** ninguno observado. Las API de OpenCV 4.x son compatibles hacia atrás
y las funciones que usamos (`cvtColor`, `convertTo`, `resize`) no cambiaron su ABI.

**Fix definitivo** (si fuera necesario): compilar `cv_bridge` desde fuente apuntando
al mismo OpenCV que el sistema, o forzar `set(OpenCV_DIR ...)` antes de `find_package`.

---

## 7. Parámetros del nodo

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `pipeline` | string | `opengl` | `opengl` (GPU, recomendado) o `cpu` |
| `publish_color` | bool | `true` | Publica color 1920×1080 |
| `publish_depth` | bool | `true` | Publica depth 32FC1 metros |
| `publish_ir` | bool | `true` | Publica IR 16UC1 |
| `publish_resized_color` | bool | `false` | Publica color reducido |
| `resized_width` | int | `640` | Ancho del color reducido |
| `resized_height` | int | `360` | Alto del color reducido |
| `color_frame_id` | string | `kinect2_color_optical_frame` | |
| `depth_frame_id` | string | `kinect2_depth_optical_frame` | |
| `ir_frame_id` | string | `kinect2_ir_optical_frame` | |
| `timeout_ms` | int | `1000` | Timeout de waitForNewFrame en ms |

---

## 8. Configuración de QoS para suscriptores custom

El nodo publica con `SensorDataQoS` (`best_effort`, `volatile`, `keep_last 5`).
Los suscriptores deben usar un perfil compatible:

```python
# Python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)
self.sub = self.create_subscription(Image, '/kinect2/color/image_raw', cb, qos)
```

```cpp
// C++
auto qos = rclcpp::SensorDataQoS();
sub_ = create_subscription<sensor_msgs::msg::Image>(
  "/kinect2/color/image_raw", qos, cb);
```

---

## 9. Archivos del paquete

```
kinect2_bridge/
├── src/
│   ├── kinect2_bridge_node.cpp          ← nodo principal (Fase 2)
│   └── kinect2_bridge_node.cpp.bak_phase2  ← backup intermedio
├── launch/
│   └── kinect2_bridge.launch.py
├── docs/
│   └── TESTING.md                       ← este archivo
├── CMakeLists.txt
└── package.xml
```
