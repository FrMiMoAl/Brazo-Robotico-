# LLM + Kinect v2 + Brazo robotico 4 GDL — Guia rapida

Implementacion de la arquitectura descrita en `LLM_KINECT_BRAZO_JETSON_SPEC.md`.
Esta guia cubre como compilar, lanzar y probar el sistema, y que falta
calibrar antes de mover el brazo real.

Rama de trabajo: `feature/llm-kinect-brazo`.

## 0. Aviso sobre la ruta del workspace

Este directorio se llama `ros2_ws (copy)` (con espacio y parentesis).
**CMake genera comandos de shell que no escapan correctamente los
parentesis**, lo que rompe `colcon build` para cualquier paquete con
generacion de interfaces Python (p.ej. `brazo_interfaces`) si `build/`,
`install/` y `log/` quedan dentro de esa ruta.

Solucion aplicada (ya configurada, no requiere accion): `build/`,
`install/` y `log/` son **symlinks** a `/home/franco/ros2_ws_copy_out/{build,install,log}`,
fuera de la ruta con parentesis. `colcon build` y `source install/setup.bash`
funcionan de forma normal, sin flags especiales. Si se borran estos
symlinks por error, recrearlos con:

```bash
cd "/home/franco/ros2_ws (copy)"
mkdir -p /home/franco/ros2_ws_copy_out/{build,install,log}
ln -s /home/franco/ros2_ws_copy_out/build build
ln -s /home/franco/ros2_ws_copy_out/install install
ln -s /home/franco/ros2_ws_copy_out/log log
```

Tambien se agrego `COLCON_IGNORE` en `src/kinect2_bridge_backup_optimized/`
(nombre de paquete duplicado con `src/kinect2_bridge`) y en
`jetson-containers/` (repo de terceros con un proyecto CUDA que no es parte
de este workspace ROS 2), para que `colcon build` sin argumentos funcione
sobre todo el workspace. Ningun archivo dentro de esos directorios fue
modificado ni borrado.

## 1. Compilar

```bash
cd "/home/franco/ros2_ws (copy)"
echo $ROS_DISTRO   # debe imprimir: humble
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

## 2. Arquitectura (resumen)

```text
Kinect v2 -> kinect2_bridge -> object_3d_detector (yolo_depth_to_point.py)
    -> /perception/objects_camera (brazo_interfaces/Object3DArray)
    -> camera_to_base_node (TF2)
    -> /perception/objects_base (reachable=true/false segun workspace real)
    -> scene_state_node
    -> /scene_state (JSON)
    -> llm_agent_node (o manual_plan_node)
    -> /llm_plan (JSON)
    -> task_executor_node (maquina de estados)
    -> /arm/request_target_position, /arm/request_gripper_command
    -> safety_guard_node   <-- UNICA puerta hacia el brazo real
    -> /target_position, /gripper_command
    -> controlador existente del brazo (sin cambios)
```

El LLM **nunca** publica en `/joint_commands`, `/target_position` ni
`/gripper_command`. Solo `safety_guard_node` lo hace, y solo si
`dry_run=false`, `autonomous_enable=true` y `hardware_armed=true`.

## 3. Lanzar todo (modo seguro, por defecto)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd "/home/franco/ros2_ws (copy)"
source install/setup.bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py
# equivalente explicito:
ros2 launch brazo_ai llm_kinect_brazo.launch.py \
  dry_run:=true autonomous_enable:=false hardware_armed:=false
```

Esto lanza: `camera_to_base_node`, `scene_state_node`, `llm_agent_node`
(modo determinista, sin API), `task_executor_node` y `safety_guard_node`.
**No** lanza Kinect ni el detector — se corren aparte (terminales
separadas, ver demo completa abajo).

Para usar `manual_plan_node` en vez de `llm_agent_node`:

```bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py use_llm_agent:=false manual_task:=pick_red
```

Para conectar `llm_agent_node` a la API real de Anthropic (opcional):

```bash
export ANTHROPIC_API_KEY=sk-...
ros2 launch brazo_ai llm_kinect_brazo.launch.py use_llm_api:=true
```

## 4. Demo completa (5 terminales)

**Terminal 1 — Kinect:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd "/home/franco/ros2_ws (copy)" && source install/setup.bash
ros2 launch kinect2_bridge kinect2_bridge.launch.py
```

**Terminal 2 — Detector (solo percepcion, no mueve el brazo):**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd "/home/franco/ros2_ws (copy)" && source install/setup.bash
ros2 run object_3d_detector yolo_depth_to_point \
  --ros-args -p target_class:=red -p publish_direct_target:=false -p device:=cpu
```

**Terminal 3 — TF camara->base (ver seccion 6, valores placeholder):**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run tf2_ros static_transform_publisher \
  X Y Z ROLL PITCH YAW base_link kinect2_depth_optical_frame
```

**Terminal 4 — IA segura en dry-run:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd "/home/franco/ros2_ws (copy)" && source install/setup.bash
ros2 launch brazo_ai llm_kinect_brazo.launch.py dry_run:=true
```

**Terminal 5 — Orden humana:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd "/home/franco/ros2_ws (copy)" && source install/setup.bash
ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1
```

## 5. Pruebas paso a paso (sin hardware, validando cada capa)

```bash
# Interfaces
ros2 interface show brazo_interfaces/msg/Object3D
ros2 interface show brazo_interfaces/action/PickObject

# Percepcion (requiere Kinect+detector corriendo, terminales 1-2 arriba)
ros2 topic echo /perception/objects_camera
ros2 topic echo /perception/selected_object_camera

# Transformacion a base_link (requiere TF de terminal 3)
ros2 topic echo /perception/objects_base
ros2 topic echo /perception/selected_object_base

# Estado de escena (JSON para el LLM)
ros2 topic echo /scene_state

# Plan sin LLM
ros2 run brazo_ai manual_plan_node --ros-args -p task:=pick_red
ros2 topic echo /llm_plan
# o por texto:
ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1

# Executor + safety guard en dry_run (no debe moverse el brazo)
ros2 run brazo_ai task_executor_node --ros-args -p dry_run:=true
ros2 run brazo_ai safety_guard_node --ros-args -p dry_run:=true
ros2 topic echo /arm/command_status

# Probar safety_guard con un comando manual (debe bloquear en dry_run)
ros2 topic pub /arm/request_target_position geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.18, y: 0.0, z: 0.12}}" -1
```

### Brazo real (solo despues de validar todo lo anterior y calibrar TF)

```bash
ros2 run brazo_ai safety_guard_node --ros-args \
  -p dry_run:=false -p autonomous_enable:=true -p hardware_armed:=true

# punto seguro alto, lejos de obstaculos, primero:
ros2 topic pub /arm/request_target_position geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.18, y: 0.0, z: 0.20}}" -1
```

## 6. Pendiente: calibrar TF camara -> base (CRITICO)

El sistema usa `base_link <- kinect2_depth_optical_frame` para pasar de
percepcion a coordenadas del brazo. **No se inventaron valores reales.**
Mientras no este calibrado, usar un `static_transform_publisher` con
placeholders y medir:

```bash
ros2 run tf2_ros static_transform_publisher \
  X Y Z ROLL PITCH YAW \
  base_link kinect2_depth_optical_frame
```

`X Y Z` en metros, `ROLL PITCH YAW` en radianes (orden: roll, pitch, yaw
extrinsecos sobre los ejes de `base_link`).

Procedimiento recomendado (ver tambien FASE 5 del spec):

1. Fijar la Kinect rigidamente (si se mueve, hay que recalibrar).
2. Definir el origen de `base_link` en la base fisica del brazo.
3. Colocar un objeto rojo en puntos conocidos dentro del workspace y medir
   su posicion real respecto a `base_link` con una regla/cinta metrica.
4. Comparar esa medicion con lo que publica `/perception/selected_object_camera`
   (posicion en frame de camara) y `/perception/selected_object_base`
   (posicion ya transformada).
5. Ajustar `X Y Z ROLL PITCH YAW` hasta que `/perception/selected_object_base`
   coincida con la medicion real para varios puntos de prueba:

```text
P1 = x 0.18, y  0.00, z 0.05
P2 = x 0.22, y  0.08, z 0.05
P3 = x 0.22, y -0.08, z 0.05
P4 = x 0.28, y  0.00, z 0.08
P5 = x 0.15, y  0.10, z 0.10
```

Una vez calibrado, fijar esos valores en el `static_transform_publisher`
real (o convertirlo en un nodo/launch propio) y en los limites de
workspace (`workspace_x_min/max`, etc. — ver seccion 7) segun el alcance
real del brazo de 4 GDL.

## 7. Parametros de seguridad clave

| Parametro | Nodo(s) | Default | Efecto |
|---|---|---|---|
| `dry_run` | `task_executor_node`, `safety_guard_node` | `true` | En `safety_guard_node`, bloquea TODA publicacion real a `/target_position` y `/gripper_command`. En `task_executor_node` solo afecta el texto de log. |
| `autonomous_enable` | `safety_guard_node` | `false` | Debe ser `true` (junto con `hardware_armed` y `dry_run=false`) para mover el brazo real. |
| `hardware_armed` | `safety_guard_node` (y reflejado informativamente en `scene_state_node`) | `false` | Idem. El brazo arranca siempre desarmado. |
| `workspace_x_min/max`, `y_min/max`, `z_min/max` | `camera_to_base_node`, `safety_guard_node` | placeholders, ver seccion 6 | Limites cartesianos del brazo real; fuera de rango = `reachable=false` o `REJECTED`, nunca clipping silencioso. |
| `max_step_m` | `safety_guard_node`, `task_executor_node` | `0.12` | Salto cartesiano maximo entre comandos consecutivos aceptados. `task_executor_node` parte movimientos mas largos (p.ej. pick -> place) en varios waypoints de este tamano. |

Estos valores se pasan una sola vez desde `llm_kinect_brazo.launch.py` a
todos los nodos que los necesitan, para evitar que queden desincronizados.

## 8. Paquetes

- `brazo_interfaces` (`ament_cmake`): mensajes `Object3D`, `Object3DArray`,
  `TaskPlan`, `ArmCommandStatus` y acciones `PickObject`, `MoveToPoint`.
- `object_3d_detector` (`ament_python`, existente, refactorizado): nodo
  `yolo_depth_to_point` — deteccion HSV roja (MVP) o YOLO, publica
  percepcion en frame de camara. Ya no mueve el brazo por defecto.
- `brazo_ai` (`ament_python`, nuevo): `camera_to_base_node`,
  `scene_state_node`, `llm_agent_node`, `manual_plan_node`,
  `task_executor_node`, `safety_guard_node`, y el launch file.
- `teleop_brazo` (existente, sin cambios): control manual por teclado,
  sigue funcionando igual, publica directo a `/joint_commands`,
  `/target_position`, `/gripper_command` (uso manual, no autonomo).
- `kinect2_bridge` (existente, sin cambios): driver de la Kinect v2.
