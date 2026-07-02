# Especificación para Claude Code en Jetson: LLM + Kinect v2 + Brazo robótico 4 GDL

**Objetivo:** transformar el workspace ROS 2 actual de la Jetson para que un LLM pueda usar la información 3D de la Kinect y ordenar tareas de alto nivel al brazo robótico de 4 grados de libertad, sin que el LLM controle motores directamente.

**Workspace esperado:** `/home/franco/ros2_ws`

**Distro ROS 2 esperada según la documentación actual:** Humble. No obstante, el agente debe verificar `echo $ROS_DISTRO` antes de compilar o ejecutar.

**Regla central de arquitectura:**

> El LLM no debe publicar en `/joint_commands`, `/target_position` ni `/gripper_command` directamente. El LLM solo genera planes JSON de alto nivel. ROS 2 ejecuta el movimiento mediante nodos deterministas, validaciones de seguridad y cinemática inversa.

---

## 1. Contexto del sistema actual

El workspace actual contiene al menos estos paquetes:

```text
/home/franco/ros2_ws/src/
├── kinect2_bridge
├── kinect2_bridge_backup_optimized
├── object_3d_detector
└── teleop_brazo
```

### 1.1 `kinect2_bridge`

Driver C++ para Kinect v2 basado en `libfreenect2`. Publica:

```text
/kinect2/color/image_raw
/kinect2/depth/image_raw
/kinect2/ir/image_raw
/kinect2/color/camera_info
/kinect2/depth/camera_info
/kinect2/ir/camera_info
```

La documentación indica que está optimizado para Jetson usando pipeline OpenGL.

### 1.2 `object_3d_detector`

Contiene el nodo principal:

```text
object_3d_detector/yolo_depth_to_point.py
```

Este nodo hace actualmente:

```text
Kinect RGB + Kinect depth + camera_info
        ↓
YOLOv11 o detección HSV de rojo
        ↓
centro 2D del objeto
        ↓
escalado aproximado RGB → depth
        ↓
mediana de profundidad
        ↓
desproyección pin-hole a 3D
        ↓
offsets cámara → robot
        ↓
workspace limiter
        ↓
/target_position
```

### 1.3 `teleop_brazo`

Contiene control manual por consola y publica comandos a:

```text
/joint_commands
/target_position
/gripper_command
```

Debe conservarse para pruebas manuales.

---

## 2. Problema a resolver

Actualmente el nodo de visión publica directamente en `/target_position`. Eso es útil para depuración, pero no es correcto para un brazo controlado por LLM, porque:

1. La visión puede fluctuar frame a frame.
2. El brazo podría perseguir ruido de profundidad.
3. El LLM no tendría una representación simbólica de la escena.
4. No existe una capa clara de seguridad entre percepción y movimiento.
5. No hay separación limpia entre percepción, planificación, ejecución y control.

El objetivo es migrar a esta arquitectura:

```text
Kinect v2
  ↓
kinect2_bridge
  ↓
object_3d_detector
  ↓
/perception/objects_camera
  ↓
camera_to_base_node + TF2
  ↓
/perception/objects_base
  ↓
scene_state_node
  ↓
/scene_state JSON
  ↓
llm_agent_node o entrada humana/Claude API
  ↓
/llm_plan JSON
  ↓
task_executor_node
  ↓
/arm/request_target_position + /arm/request_gripper_command
  ↓
safety_guard_node
  ↓
/target_position + /gripper_command
  ↓
controlador cartesiano / IK / micro-ROS
  ↓
servos del brazo
```

---

## 3. Principios obligatorios

### 3.1 El LLM solo planifica

Permitido para el LLM:

```json
{
  "task": "pick_and_place",
  "object": {
    "class_name": "red",
    "selection": "largest_reachable"
  },
  "place_zone": "drop_zone_a"
}
```

Prohibido para el LLM:

```json
{
  "joint_1": 1.2,
  "joint_2": -0.4,
  "servo_pwm": 1500
}
```

El LLM no debe generar:

- ángulos articulares,
- PWM,
- velocidades en tiempo real,
- coordenadas inventadas,
- comandos directos a `/joint_commands`,
- comandos directos a `/target_position`,
- comandos directos a `/gripper_command`.

### 3.2 El sistema debe usar PBVS, no IBVS

Para este brazo de 4 GDL con Kinect fija en configuración eye-to-hand, usar:

```text
pixel + depth → punto 3D en cámara → TF2 a base_link → objetivo cartesiano → IK
```

No usar control visual directo por píxeles con el LLM.

### 3.3 Todo punto 3D debe llevar frame

No usar `geometry_msgs/msg/Point` para percepción. Usar:

```text
geometry_msgs/msg/PointStamped
```

Motivo: debe saberse si el punto está en `kinect2_depth_optical_frame`, `base_link`, `world`, etc.

### 3.4 Nada de clipping silencioso para autonomía

Si un objeto está fuera del workspace, no mover el punto al borde automáticamente.

Correcto:

```text
objeto fuera del workspace → reachable=false, reason="outside_workspace"
```

Incorrecto para autonomía:

```text
objeto fuera del workspace → modificar coordenada al límite y mover el brazo de todos modos
```

El clipping puede conservarse solo en modo debug manual.

### 3.5 El brazo real debe empezar desarmado

Agregar parámetros globales de seguridad:

```text
dry_run:=true
autonomous_enable:=false
hardware_armed:=false
```

El sistema solo debe publicar comandos reales a `/target_position` y `/gripper_command` si:

```text
dry_run == false
autonomous_enable == true
hardware_armed == true
```

Para pruebas en RViz o consola, permitir `dry_run:=true` y registrar lo que se habría enviado.

---

## 4. Paquetes nuevos a crear

Crear estos paquetes dentro de `/home/franco/ros2_ws/src`:

```text
brazo_interfaces
brazo_ai
```

---

# FASE 0 — Inspección y backup

Claude Code debe ejecutar primero:

```bash
cd /home/franco/ros2_ws
pwd
echo $ROS_DISTRO
find src -maxdepth 2 -type f | sort | sed -n '1,200p'
ros2 pkg list | grep -E 'kinect|object|teleop|brazo' || true
```

Si el workspace tiene Git:

```bash
git status
git checkout -b feature/llm-kinect-brazo || true
```

Si no tiene Git:

```bash
mkdir -p /home/franco/ros2_ws_backup_$(date +%Y%m%d_%H%M%S)
cp -a /home/franco/ros2_ws/src /home/franco/ros2_ws_backup_$(date +%Y%m%d_%H%M%S)/
```

No ejecutar comandos destructivos como:

```bash
rm -rf /home/franco/ros2_ws/src
rm -rf /home/franco/ros2_ws
```

Solo está permitido limpiar `build`, `install` y `log` después de guardar cambios y cuando sea necesario recompilar limpio.

---

# FASE 1 — Crear `brazo_interfaces`

## 1.1 Crear paquete

```bash
cd /home/franco/ros2_ws/src
ros2 pkg create brazo_interfaces \
  --build-type ament_cmake \
  --dependencies std_msgs geometry_msgs action_msgs rosidl_default_generators
```

## 1.2 Crear mensajes

Crear:

```text
brazo_interfaces/msg/Object3D.msg
brazo_interfaces/msg/Object3DArray.msg
brazo_interfaces/msg/TaskPlan.msg
brazo_interfaces/msg/ArmCommandStatus.msg
brazo_interfaces/action/PickObject.action
brazo_interfaces/action/MoveToPoint.action
```

### `msg/Object3D.msg`

```text
std_msgs/Header header

string object_id
string class_name
float32 confidence

geometry_msgs/Point point

float32 depth_m
int32 u_color
int32 v_color
int32 u_depth
int32 v_depth

float32 bbox_area
bool reachable
string reason
```

### `msg/Object3DArray.msg`

```text
std_msgs/Header header
brazo_interfaces/Object3D[] objects
```

### `msg/TaskPlan.msg`

```text
std_msgs/Header header
string task
string object_id
string class_name
string selection
string place_zone
string raw_json
bool valid
string reason
```

### `msg/ArmCommandStatus.msg`

```text
std_msgs/Header header
string state
bool busy
bool success
string message
geometry_msgs/PointStamped current_target
```

### `action/PickObject.action`

```text
string class_name
string object_id
string selection
string place_zone
---
bool success
string message
---
string state
geometry_msgs/PointStamped current_target
```

### `action/MoveToPoint.action`

```text
geometry_msgs/PointStamped target
float32 tolerance_m
float32 timeout_s
---
bool success
string message
---
geometry_msgs/PointStamped current_pose
float32 error_m
```

## 1.3 Actualizar `CMakeLists.txt`

Agregar generación de interfaces:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object3D.msg"
  "msg/Object3DArray.msg"
  "msg/TaskPlan.msg"
  "msg/ArmCommandStatus.msg"
  "action/PickObject.action"
  "action/MoveToPoint.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

## 1.4 Actualizar `package.xml`

Asegurar dependencias:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>action_msgs</depend>
```

## 1.5 Compilar fase 1

```bash
cd /home/franco/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select brazo_interfaces
source install/setup.bash
ros2 interface show brazo_interfaces/msg/Object3D
ros2 interface show brazo_interfaces/action/PickObject
```

---

# FASE 2 — Refactor de `object_3d_detector/yolo_depth_to_point.py`

## 2.1 Objetivo

El nodo de visión debe publicar percepción, no movimiento.

Antes:

```text
yolo_depth_to_point.py → /target_position
```

Después:

```text
yolo_depth_to_point.py → /perception/objects_camera
                         /perception/selected_object_camera
                         /yolo/annotated_image
```

`/target_position` solo se permite con parámetro de debug:

```text
publish_direct_target:=true
```

Por defecto:

```text
publish_direct_target:=false
```

## 2.2 Parámetros a agregar o normalizar

```python
self.declare_parameter("target_class", "red")
self.declare_parameter("model_path", "yolo11n.pt")
self.declare_parameter("confidence", 0.45)
self.declare_parameter("device", "cpu")
self.declare_parameter("depth_window", 9)
self.declare_parameter("min_depth", 0.20)
self.declare_parameter("max_depth", 4.0)
self.declare_parameter("publish_direct_target", False)
self.declare_parameter("publish_all_objects", True)
self.declare_parameter("camera_frame", "kinect2_depth_optical_frame")
self.declare_parameter("debug_annotations", True)
```

En Jetson, usar `device:=cpu` por defecto para evitar problemas de memoria CUDA con YOLO. Luego se puede probar `cuda` manualmente.

## 2.3 Publicadores requeridos

```python
from geometry_msgs.msg import PointStamped, Point
from brazo_interfaces.msg import Object3D, Object3DArray

self.objects_camera_pub = self.create_publisher(
    Object3DArray,
    "/perception/objects_camera",
    10
)

self.selected_camera_pub = self.create_publisher(
    PointStamped,
    "/perception/selected_object_camera",
    10
)

self.debug_target_pub = self.create_publisher(
    Point,
    "/target_position",
    10
)
```

## 2.4 Mantener entradas actuales

Mantener suscripciones con QoS de sensor:

```text
/kinect2/color/image_raw
/kinect2/depth/image_raw
/kinect2/depth/camera_info
```

## 2.5 Salida principal

Cada objeto detectado debe convertirse a `Object3D` con:

```text
header.frame_id = camera_frame
class_name = target_class o clase YOLO
confidence = confianza del detector
point = punto 3D en frame de cámara, no de robot
reachable = true solo si la profundidad es válida
reason = "ok" o motivo de rechazo
```

Ejemplo lógico:

```python
obj = Object3D()
obj.header.stamp = self.get_clock().now().to_msg()
obj.header.frame_id = self.camera_frame
obj.object_id = f"{class_name}_{i}"
obj.class_name = class_name
obj.confidence = float(conf)
obj.point.x = float(x_cam)
obj.point.y = float(y_cam)
obj.point.z = float(z_cam)
obj.depth_m = float(depth_m)
obj.u_color = int(u_color)
obj.v_color = int(v_color)
obj.u_depth = int(u_depth)
obj.v_depth = int(v_depth)
obj.bbox_area = float(area)
obj.reachable = True
obj.reason = "ok"
```

Publicar arreglo:

```python
arr = Object3DArray()
arr.header.stamp = self.get_clock().now().to_msg()
arr.header.frame_id = self.camera_frame
arr.objects = objects
self.objects_camera_pub.publish(arr)
```

Publicar seleccionado:

```python
selected = PointStamped()
selected.header = arr.header
selected.point = objects[0].point
self.selected_camera_pub.publish(selected)
```

## 2.6 Prohibición en modo autónomo

No aplicar offsets `x_robot = z_cam + offset` dentro del detector como ruta principal.

No aplicar workspace limiter dentro del detector para tomar decisiones de brazo.

El detector solo debe saber si la profundidad es válida. La alcanzabilidad real se decide después de transformar a `base_link`.

## 2.7 Compatibilidad temporal

Conservar el viejo comportamiento solo si:

```python
publish_direct_target == True
```

En ese caso se puede seguir publicando `Point` a `/target_position` para debug.

---

# FASE 3 — Crear paquete `brazo_ai`

## 3.1 Crear paquete

```bash
cd /home/franco/ros2_ws/src
ros2 pkg create brazo_ai \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs brazo_interfaces
```

## 3.2 Estructura esperada

```text
brazo_ai/
├── brazo_ai/
│   ├── __init__.py
│   ├── camera_to_base_node.py
│   ├── scene_state_node.py
│   ├── llm_agent_node.py
│   ├── task_executor_node.py
│   ├── safety_guard_node.py
│   └── manual_plan_node.py
├── launch/
│   └── llm_kinect_brazo.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

## 3.3 Actualizar `setup.py`

Agregar entry points:

```python
entry_points={
    'console_scripts': [
        'camera_to_base_node = brazo_ai.camera_to_base_node:main',
        'scene_state_node = brazo_ai.scene_state_node:main',
        'llm_agent_node = brazo_ai.llm_agent_node:main',
        'task_executor_node = brazo_ai.task_executor_node:main',
        'safety_guard_node = brazo_ai.safety_guard_node:main',
        'manual_plan_node = brazo_ai.manual_plan_node:main',
    ],
},
```

---

# FASE 4 — `camera_to_base_node.py`

## 4.1 Objetivo

Transformar objetos de:

```text
kinect2_depth_optical_frame
```

a:

```text
base_link
```

usando TF2.

## 4.2 Suscripciones y publicaciones

Entrada:

```text
/perception/objects_camera    brazo_interfaces/msg/Object3DArray
```

Salida:

```text
/perception/objects_base      brazo_interfaces/msg/Object3DArray
/perception/selected_object_base geometry_msgs/msg/PointStamped
```

## 4.3 Parámetros

```python
self.declare_parameter("base_frame", "base_link")
self.declare_parameter("camera_frame", "kinect2_depth_optical_frame")
self.declare_parameter("workspace_x_min", 0.05)
self.declare_parameter("workspace_x_max", 0.35)
self.declare_parameter("workspace_y_min", -0.25)
self.declare_parameter("workspace_y_max", 0.25)
self.declare_parameter("workspace_z_min", 0.02)
self.declare_parameter("workspace_z_max", 0.35)
self.declare_parameter("object_stale_timeout", 0.75)
```

Ajustar los límites al brazo real después de medir.

## 4.4 Comportamiento

Por cada `Object3D` recibido:

1. Crear `PointStamped` en frame de cámara.
2. Transformar con TF2 a `base_link`.
3. Copiar el objeto y reemplazar `point` por coordenadas en base.
4. Evaluar workspace.
5. Si está dentro: `reachable=true, reason="ok"`.
6. Si está fuera: `reachable=false, reason="outside_workspace"`.
7. Publicar arreglo completo.
8. Publicar como seleccionado el primer objeto reachable, priorizando mayor área/confianza.

## 4.5 Si TF no existe

No publicar comandos al brazo. Loguear:

```text
No transform from kinect2_depth_optical_frame to base_link. Calibrate TF first.
```

---

# FASE 5 — Calibración TF cámara → base

El sistema necesita esta transformación:

```text
base_link ← kinect2_depth_optical_frame
```

Comando base:

```bash
ros2 run tf2_ros static_transform_publisher \
  X Y Z ROLL PITCH YAW \
  base_link kinect2_depth_optical_frame
```

Los valores `X Y Z ROLL PITCH YAW` deben medirse físicamente. No inventarlos.

## Procedimiento recomendado

1. Fijar la Kinect rígidamente.
2. Definir el origen `base_link` en la base del brazo.
3. Colocar un objeto rojo en puntos conocidos dentro del workspace.
4. Medir esos puntos respecto a la base.
5. Compararlos con `/perception/selected_object_camera`.
6. Ajustar la transformación hasta que `/perception/selected_object_base` coincida con la medición real.

Puntos de prueba sugeridos:

```text
P1 = x 0.18, y  0.00, z 0.05
P2 = x 0.22, y  0.08, z 0.05
P3 = x 0.22, y -0.08, z 0.05
P4 = x 0.28, y  0.00, z 0.08
P5 = x 0.15, y  0.10, z 0.10
```

---

# FASE 6 — `scene_state_node.py`

## 6.1 Objetivo

Convertir tópicos ROS en un JSON semántico que el LLM pueda leer.

## 6.2 Entradas

```text
/perception/objects_base        brazo_interfaces/msg/Object3DArray
/joint_states                   sensor_msgs/msg/JointState opcional
/gripper_state                  std_msgs/msg/Bool opcional
/arm_state                      std_msgs/msg/String opcional
```

## 6.3 Salida

```text
/scene_state                    std_msgs/msg/String
```

## 6.4 JSON esperado

```json
{
  "timestamp": 123456.78,
  "frames": {
    "base": "base_link",
    "camera": "kinect2_depth_optical_frame"
  },
  "robot": {
    "state": "idle",
    "busy": false,
    "gripper": "unknown",
    "hardware_armed": false
  },
  "objects": [
    {
      "id": "red_0",
      "class": "red",
      "confidence": 0.98,
      "reachable": true,
      "reason": "ok",
      "position_base": {
        "x": 0.22,
        "y": -0.04,
        "z": 0.06
      }
    }
  ],
  "zones": {
    "home": {"x": 0.16, "y": 0.0, "z": 0.20},
    "drop_zone_a": {"x": 0.18, "y": -0.15, "z": 0.12},
    "drop_zone_b": {"x": 0.18, "y": 0.15, "z": 0.12}
  },
  "allowed_tasks": [
    "observe_scene",
    "pick_object",
    "pick_and_place",
    "open_gripper",
    "close_gripper",
    "go_home",
    "abort"
  ]
}
```

## 6.5 Frecuencia

Publicar a 2 Hz o cuando cambie la escena. No publicar a 30 Hz para el LLM.

---

# FASE 7 — `llm_agent_node.py`

## 7.1 Objetivo

Recibir instrucciones humanas y `scene_state`, y publicar planes JSON en `/llm_plan`.

## 7.2 Modo MVP sin API

Implementar primero un modo determinista sin conexión a API:

Entrada:

```text
/user_command std_msgs/msg/String
/scene_state  std_msgs/msg/String
```

Salida:

```text
/llm_plan std_msgs/msg/String
```

Si el usuario escribe:

```text
agarra el objeto rojo
agarra rojo
pick red
```

publicar:

```json
{
  "task": "pick_and_place",
  "object": {
    "class_name": "red",
    "selection": "largest_reachable"
  },
  "place_zone": "drop_zone_a"
}
```

Si no hay objeto reachable en `scene_state`, publicar:

```json
{
  "task": "abort",
  "reason": "no_reachable_object"
}
```

## 7.3 Modo LLM real opcional

Preparar estructura para API, pero no asumir claves ni internet. Usar parámetro:

```text
use_llm_api:=false
```

Si `use_llm_api:=true`, leer la API key desde variable de entorno. No hardcodear claves.

```bash
ANTHROPIC_API_KEY
```

## 7.4 Prompt de sistema interno del LLM

```text
Eres un planificador de alto nivel para un brazo robótico de 4 grados de libertad con Kinect v2.

Reglas obligatorias:
- No controles articulaciones.
- No generes ángulos.
- No generes PWM.
- No publiques en /joint_commands.
- No publiques directamente en /target_position.
- No publiques directamente en /gripper_command.
- No inventes coordenadas de objetos.
- Usa solo los objetos presentes en scene_state.
- Si no hay objeto visible y reachable=true, responde abort.
- Si el objeto solicitado tiene reachable=false, responde abort.
- Devuelve exclusivamente JSON válido.

Acciones permitidas:
- observe_scene
- pick_object
- pick_and_place
- open_gripper
- close_gripper
- go_home
- abort

Formato preferido:
{
  "task": "pick_and_place",
  "object": {
    "class_name": "red",
    "selection": "largest_reachable"
  },
  "place_zone": "drop_zone_a"
}
```

---

# FASE 8 — `manual_plan_node.py`

## 8.1 Objetivo

Permitir pruebas sin LLM:

```bash
ros2 run brazo_ai manual_plan_node --ros-args -p task:=pick_red
```

O por tópico:

```bash
ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1
```

El nodo debe publicar en `/llm_plan` el JSON correcto.

---

# FASE 9 — `task_executor_node.py`

## 9.1 Objetivo

Ejecutar planes de alto nivel de forma determinista.

Entradas:

```text
/llm_plan                     std_msgs/msg/String
/perception/objects_base      brazo_interfaces/msg/Object3DArray
/scene_state                  std_msgs/msg/String
```

Salidas hacia safety guard:

```text
/arm/request_target_position  geometry_msgs/msg/PointStamped
/arm/request_gripper_command  std_msgs/msg/Bool
/arm/command_status           brazo_interfaces/msg/ArmCommandStatus
```

No publicar directo a `/target_position` salvo en modo legacy explícito.

## 9.2 Parámetros

```python
self.declare_parameter("pregrasp_dz", 0.08)
self.declare_parameter("grasp_dz", 0.015)
self.declare_parameter("lift_dz", 0.12)
self.declare_parameter("move_wait_s", 2.0)
self.declare_parameter("gripper_wait_s", 1.0)
self.declare_parameter("default_place_zone", "drop_zone_a")
self.declare_parameter("dry_run", True)
```

## 9.3 Máquina de estados para `pick_and_place`

```text
IDLE
  ↓
LOAD_PLAN
  ↓
VALIDATE_PLAN
  ↓
SELECT_OBJECT
  ↓
VALIDATE_OBJECT_REACHABLE
  ↓
OPEN_GRIPPER
  ↓
MOVE_PREGRASP
  ↓
MOVE_GRASP
  ↓
CLOSE_GRIPPER
  ↓
LIFT_OBJECT
  ↓
MOVE_TO_PLACE_ZONE
  ↓
OPEN_GRIPPER
  ↓
RETREAT
  ↓
GO_HOME
  ↓
DONE
```

## 9.4 Selección de objeto

Si el plan dice:

```json
"selection": "largest_reachable"
```

seleccionar el objeto con:

```text
class_name == solicitado
reachable == true
mayor bbox_area o mayor confidence
```

## 9.5 Secuencia cartesiana

Para objeto:

```text
obj = (x, y, z)
```

Calcular:

```text
pregrasp = (x, y, z + pregrasp_dz)
grasp    = (x, y, z + grasp_dz)
lift     = (x, y, z + lift_dz)
place    = drop_zone_a/drop_zone_b
home     = zona home
```

Publicar cada punto como `PointStamped` en `base_link`.

## 9.6 Espera de movimiento

MVP:

```python
time.sleep(move_wait_s)
```

Mejor posterior:

```text
esperar /arm_motion_done o acción MoveToPoint
```

## 9.7 Abortos

Abortar si:

- JSON inválido.
- tarea no permitida.
- objeto no encontrado.
- objeto `reachable=false`.
- safety guard rechaza objetivo.
- `hardware_armed=false` cuando se intenta brazo real.

---

# FASE 10 — `safety_guard_node.py`

## 10.1 Objetivo

Validar comandos antes de que lleguen al brazo real.

Entradas:

```text
/arm/request_target_position  geometry_msgs/msg/PointStamped
/arm/request_gripper_command  std_msgs/msg/Bool
/emergency_stop               std_msgs/msg/Bool
```

Salidas:

```text
/target_position              geometry_msgs/msg/Point
/gripper_command              std_msgs/msg/Bool
/arm/command_status           brazo_interfaces/msg/ArmCommandStatus
```

## 10.2 Parámetros

```python
self.declare_parameter("dry_run", True)
self.declare_parameter("autonomous_enable", False)
self.declare_parameter("hardware_armed", False)
self.declare_parameter("base_frame", "base_link")
self.declare_parameter("workspace_x_min", 0.05)
self.declare_parameter("workspace_x_max", 0.35)
self.declare_parameter("workspace_y_min", -0.25)
self.declare_parameter("workspace_y_max", 0.25)
self.declare_parameter("workspace_z_min", 0.03)
self.declare_parameter("workspace_z_max", 0.35)
self.declare_parameter("max_step_m", 0.12)
```

## 10.3 Reglas de validación

Rechazar si:

```text
frame_id != base_link
x fuera de límites
y fuera de límites
z fuera de límites
emergency_stop == true
salto cartesiano > max_step_m
hardware_armed == false y dry_run == false
autonomous_enable == false y dry_run == false
```

Si `dry_run=true`:

- No publicar a `/target_position` ni `/gripper_command`.
- Publicar estado/log con lo que se habría enviado.

Si `dry_run=false`, `autonomous_enable=true`, `hardware_armed=true`:

- Convertir `PointStamped.point` a `geometry_msgs/msg/Point`.
- Publicar a `/target_position`.
- Publicar gripper a `/gripper_command`.

---

# FASE 11 — Launch file

Crear:

```text
brazo_ai/launch/llm_kinect_brazo.launch.py
```

Debe lanzar:

```text
camera_to_base_node
scene_state_node
llm_agent_node o manual_plan_node
task_executor_node
safety_guard_node
```

No lanzar Kinect ni controlador del brazo en este launch si ya existen launch files separados. Permitir integrarlo después.

Parámetros por defecto seguros:

```text
dry_run:=true
autonomous_enable:=false
hardware_armed:=false
target_class:=red
```

Ejemplo:

```bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py dry_run:=true
```

Para brazo real, exigir explícitamente:

```bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py \
  dry_run:=false \
  autonomous_enable:=true \
  hardware_armed:=true
```

---

# FASE 12 — Pruebas incrementales

## 12.1 Compilar todo

```bash
cd /home/franco/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

## 12.2 Probar Kinect

Terminal 1:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/franco/ros2_ws
source install/setup.bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py
```

Terminal 2:

```bash
ros2 topic hz /kinect2/color/image_raw
ros2 topic hz /kinect2/depth/image_raw
ros2 topic echo /kinect2/depth/camera_info --once
```

## 12.3 Probar detector sin mover brazo

```bash
ros2 run object_3d_detector yolo_depth_to_point \
  --ros-args \
  -p target_class:=red \
  -p publish_direct_target:=false \
  -p device:=cpu
```

Verificar:

```bash
ros2 topic echo /perception/objects_camera
ros2 topic echo /perception/selected_object_camera
```

## 12.4 Publicar TF provisional

Usar valores medidos. Ejemplo ficticio, no final:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.00 0.00 0.50 0.00 0.00 0.00 \
  base_link kinect2_depth_optical_frame
```

## 12.5 Probar transformación a base

```bash
ros2 run brazo_ai camera_to_base_node
ros2 topic echo /perception/objects_base
ros2 topic echo /perception/selected_object_base
```

## 12.6 Probar scene state

```bash
ros2 run brazo_ai scene_state_node
ros2 topic echo /scene_state
```

## 12.7 Probar plan sin LLM

```bash
ros2 run brazo_ai manual_plan_node --ros-args -p task:=pick_red
ros2 topic echo /llm_plan
```

O:

```bash
ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1
```

## 12.8 Probar executor en dry run

```bash
ros2 run brazo_ai task_executor_node --ros-args -p dry_run:=true
ros2 run brazo_ai safety_guard_node --ros-args -p dry_run:=true
```

Verificar logs. No debe moverse el brazo.

## 12.9 Probar safety guard con comando manual

```bash
ros2 topic pub /arm/request_target_position geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.18, y: 0.0, z: 0.12}}" -1
```

En `dry_run=true`, no debe publicar a `/target_position`.

## 12.10 Probar brazo real solo cuando todo lo anterior funcione

Lanzar safety con flags explícitos:

```bash
ros2 run brazo_ai safety_guard_node \
  --ros-args \
  -p dry_run:=false \
  -p autonomous_enable:=true \
  -p hardware_armed:=true
```

Luego enviar un punto seguro alto:

```bash
ros2 topic pub /arm/request_target_position geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.18, y: 0.0, z: 0.20}}" -1
```

---

# FASE 13 — Integración con el controlador existente del brazo

El sistema actual usa:

```text
/target_position     geometry_msgs/msg/Point
/gripper_command     std_msgs/msg/Bool
/joint_commands      sensor_msgs/msg/JointState
```

Mantener esa compatibilidad en la salida final del `safety_guard_node`.

No cambiar todavía el controlador real si ya funciona. Primero insertar la nueva arquitectura por encima.

Flujo final compatible:

```text
LLM → /llm_plan
executor → /arm/request_target_position
safety → /target_position
controlador existente → IK → /joint_commands → micro-ROS/servos
```

---

# FASE 14 — Criterios de éxito

La implementación se considera terminada cuando:

```text
[ ] colcon build compila sin errores.
[ ] yolo_depth_to_point.py ya no publica /target_position por defecto.
[ ] /perception/objects_camera publica Object3DArray.
[ ] /perception/selected_object_camera publica PointStamped con frame_id correcto.
[ ] camera_to_base_node transforma objetos a base_link usando TF2.
[ ] /perception/objects_base contiene reachable=true/false.
[ ] scene_state_node publica JSON válido en /scene_state.
[ ] llm_agent_node o manual_plan_node publica /llm_plan válido.
[ ] task_executor_node ejecuta máquina de estados en dry_run.
[ ] safety_guard_node bloquea comandos si dry_run=true.
[ ] safety_guard_node bloquea comandos fuera del workspace.
[ ] safety_guard_node solo publica a /target_position si hardware_armed=true.
[ ] se puede hacer una demo completa en dry_run: “agarra el objeto rojo”.
[ ] se puede hacer una prueba real con un punto seguro antes de pick-and-place.
```

---

# FASE 15 — Comando final de demo

## Terminal 1 — Kinect

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/franco/ros2_ws
source install/setup.bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py
```

## Terminal 2 — Detector

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/franco/ros2_ws
source install/setup.bash
ros2 run object_3d_detector yolo_depth_to_point \
  --ros-args \
  -p target_class:=red \
  -p publish_direct_target:=false \
  -p device:=cpu
```

## Terminal 3 — TF cámara-base

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run tf2_ros static_transform_publisher \
  X Y Z ROLL PITCH YAW \
  base_link kinect2_depth_optical_frame
```

## Terminal 4 — IA segura en dry-run

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/franco/ros2_ws
source install/setup.bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py dry_run:=true
```

## Terminal 5 — Orden humana

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/franco/ros2_ws
source install/setup.bash
ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1
```

---

# FASE 16 — Prompt que debe usar el LLM de runtime

Este prompt es para el nodo `llm_agent_node` si más adelante se conecta a una API:

```text
Eres un planificador de alto nivel para un brazo robótico de 4 grados de libertad con Kinect v2 en ROS 2.

Recibirás:
1. Una instrucción humana.
2. Un JSON scene_state con objetos detectados, posiciones en base_link, estado del robot y zonas disponibles.

Tu trabajo:
Convertir la instrucción humana en un plan JSON válido.

Reglas obligatorias:
- No controles articulaciones.
- No generes ángulos.
- No generes PWM.
- No generes velocidades.
- No publiques en /joint_commands.
- No publiques en /target_position.
- No publiques en /gripper_command.
- No inventes coordenadas.
- Usa solo objetos presentes en scene_state.
- Usa solo zonas presentes en scene_state.
- Si el objeto no existe, responde abort.
- Si el objeto tiene reachable=false, responde abort.
- Si el robot está busy=true, responde abort o observe_scene.
- Responde exclusivamente JSON. No expliques.

Tareas permitidas:
- observe_scene
- pick_object
- pick_and_place
- open_gripper
- close_gripper
- go_home
- abort

Formato válido para pick and place:
{
  "task": "pick_and_place",
  "object": {
    "class_name": "red",
    "selection": "largest_reachable"
  },
  "place_zone": "drop_zone_a"
}

Formato válido para abort:
{
  "task": "abort",
  "reason": "explicación_corta"
}
```

---

# FASE 17 — Notas importantes para Jetson

1. Usar `device:=cpu` por defecto para YOLO hasta confirmar memoria libre.
2. La detección HSV de rojo es la ruta recomendada para MVP.
3. No mover el brazo real con el LLM hasta que `/perception/objects_base` coincida con mediciones físicas.
4. La calibración `base_link ← kinect2_depth_optical_frame` es crítica.
5. El sistema debe funcionar primero en `dry_run=true`.
6. La Kinect debe estar fija; si se mueve, recalibrar TF.
7. El brazo de 4 GDL sirve para pick-and-place simple con orientación fija de pinza, no para orientación arbitraria 6D.

---

# Resumen ejecutivo para Claude Code

Implementar esta separación:

```text
percepción ≠ movimiento
LLM ≠ controlador
executor = máquina de estados
safety_guard = única puerta al brazo real
```

El resultado mínimo funcional debe permitir:

```text
Usuario: agarra el objeto rojo
        ↓
scene_state contiene red_0 reachable=true
        ↓
llm_agent_node publica plan JSON
        ↓
task_executor_node calcula pregrasp/grasp/lift/place
        ↓
safety_guard_node valida
        ↓
/target_position y /gripper_command mueven el brazo
```

