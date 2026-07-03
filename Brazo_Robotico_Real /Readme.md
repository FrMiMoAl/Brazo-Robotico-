# 🤖 Componente de Simulación 3D y Control Cinemático (NEMA + Pololu + Servos)

## Integrantes
* Franco Morales
* Joel Sejas
* Samuel Barrios Rocha
* Samuel Guzman

---

## 📝 Descripción del Módulo

[cite_start]Este componente del proyecto final para **IMT-342 Robótica - UCB** [cite: 1, 2] contiene el software de integración, mapeo geométrico y control cinemático desarrollado para operar un brazo robótico físico de 4 Grados de Libertad (DOF) más un actuador final (Gripper). 

A través de este módulo se resuelven de forma unificada tres grandes desafíos del proyecto:
1. **Calibración y Unificación de Orígenes:** El punto de origen absoluto del espacio cartesiano `(0.0, 0.0, 0.0)` se ha fijado en el suelo, exactamente alineado con el eje central del primer motor físico (NEMA). Se eliminaron mediante software las desorientaciones diagonales y desfases lineales residuales provenientes de la exportación del diseño CAD (SolidWorks), logrando que un comando a $0^\circ$ apunte estrictamente al frente en el eje global del mundo.
2. **Suavizado de Trayectorias a 50Hz:** Un nodo puente dinámico (`esp32_to_urdf_bridge.py`) intercepta los tópicos generados por la placa ESP32. Aplica una interpolación lineal cíclica en tiempo real, transformando saltos discretos abruptos en transiciones continuas y amortiguadas dentro de RViz2, protegiendo además los componentes mecánicos del robot contra picos de torque dañinos.
3. **Servidor de Cinemática Inversa Orientado al Hardware:** El nodo `robot_mode_server.py` utiliza el método numérico amortiguado de Levenberg-Marquardt sobre una cadena cinemática corregida. Al recibir una coordenada cartesiana $(X, Y, Z)$ en metros, calcula de inmediato los ángulos de los motores reales e inyecta offsets de compensación fina para contrarrestar la gravedad y desajustes de los eslabones superiores.

---

## 🛠️ Especificaciones de Acoplamiento y Mapeo

Para garantizar un gemelo digital exacto entre el modelo virtual URDF y el hardware real, el puente de comunicación opera bajo la siguiente matriz de asignación y conversión:

| Junta / Eje | Dispositivo Real | Tópico ROS 2 | Rango de Operación | Lógica de Conversión Aplicada |
| :---: | :--- | :--- | :---: | :--- |
| **`j1`** (Junta 1) | Motor a Pasos NEMA | `/motor_nema/target_deg` | $0^\circ \text{ a } 130^\circ$ | Sentido de giro invertido (`* -1.0`) para correspondencia espacial. |
| **`j2`** (Junta 2) | Motor Pololu DC | `/motor_pololu/target_deg` | $-320^\circ \text{ a } 0^\circ$ | Compensación por relación de reducción física de hardware (/2.0). |
| **`j3`** (Junta 3) | Servomotor (Servo 2) | `/servo2/target_deg` | $0^\circ \text{ a } 150^\circ$ | Mapeado lineal completo escalado hacia el rango $0^\circ - 180^\circ$ del URDF. |
| **`j4`** (Junta 4) | Servomotor (Servo 1) | `/servo1/target_deg` | $0^\circ \text{ a } 180^\circ$ | Escalado lineal nativo 1:1. $0$ físico es $0$ virtual, $180$ físico es extremo total. |
| **`gripper`** | Servomotor (Servo 3) | `/servo3/target_deg` | $0^\circ \text{ a } 180^\circ$ | Transformación de rango lineal a apertura de pinzas angulares (`-0.2` a `0.6` rad). |

---

## ⚙️ Requisitos del Sistema y Dependencias

* **Sistema Operativo:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2 Jazzy Jalisco (Instalación Desktop Completa)
* **Librerías de Python:** `math`, `numpy` (Para cálculo de matrices de transformación homogéneas y pseudo-inversas del Jacobiano)

---

## 🚀 Compilación e Instalación

Para construir de forma limpia este paquete e instalar los scripts ejecutables en tu entorno de ROS 2 local, ejecuta los siguientes comandos desde la raíz de tu workspace:

```bash
cd ~/Robotica_class/brazo4_ws
rm -rf build/ brazo4/ install/brazo4/
colcon build --packages-select brazo4
source install/setup.bash
