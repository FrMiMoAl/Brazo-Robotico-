#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

// =====================================================
// PINES USADOS
// =====================================================
#define FC1_PIN 27   // Home Pololu (Final de carrera)
#define FC2_PIN 26   // Home NEMA

// Motor Paso a Paso NEMA 23
#define PUL_PIN 2
#define DIR_PIN 15
#define ENA_PIN 13

// 3 Servomotores
const int SERVO1_PIN = 23; 
const int SERVO2_PIN = 22; 
const int SERVO3_PIN = 21; 

// Puente H - Motor Pololu (Pines de dirección/PWM)
const int IN1_PIN = 18;
const int IN2_PIN = 19;

// Encoder Pololu
const int ENC_A = 33;
const int ENC_B = 32;

#define LED_PIN 2

// =====================================================
// PARÁMETROS DE CONTROL POLOLU
// =====================================================
const float TICKS_PER_DEGREE = 35.56;
const int POTENCIA_MAX_PWM = 255;    // Máximo voltaje permitido
const int POTENCIA_MIN_PWM = 75;     // Mínimo voltaje para vencer la fricción interna y que no zumbe
const int POTENCIA_HOMING_PWM = 170; 
const int TOLERANCIA_TICKS = 2;      // ¡Reducimos la tolerancia a solo 2 ticks para máxima precisión!

// GANANCIA PROPORCIONAL (Kp)
// Ajusta este valor si el motor desacelera muy rápido (subir Kp) o si oscila (bajar Kp)
const float Kp = 1.2; 

// =====================================================
// INSTANCIAS Y VARIABLES MECÁNICAS
// =====================================================
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);
Servo servo1; Servo servo2; Servo servo3;

const float MICROSTEPPING = 3200.0;
const float RELATION_NEMA = 2.0;
const float PASOS_VUELTA_EFECTOR_NEMA = MICROSTEPPING * RELATION_NEMA;

volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// Objetivos de control
float target_nema_deg = 0.0;
float target_pololu_deg = 0.0;
long target_pololu_ticks = 0; 
bool pololu_control_active = false;
bool buscando_home_ros = false; 
unsigned long lastSerialPrintTime = 0;

bool ya_reseteado = false; 

// =====================================================
// MICRO-ROS OBJETOS
// =====================================================
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_subscription_t sub_nema; rcl_subscription_t sub_pololu;
rcl_subscription_t sub_servo1; rcl_subscription_t sub_servo2; rcl_subscription_t sub_servo3;

std_msgs__msg__Float32 msg_nema; std_msgs__msg__Float32 msg_pololu;
std_msgs__msg__Int32 msg_servo1; std_msgs__msg__Int32 msg_servo2; std_msgs__msg__Int32 msg_servo3;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// =====================================================
// INTERRUPCIÓN ENCODER
// =====================================================
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENC_A);
  int LSB = digitalRead(ENC_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;
  lastEncoded = encoded;
}

// =====================================================
// RUTINA DE HOMING PARA EL MOTOR POLOLU (AL INICIAR)
// =====================================================
void ejecutarHomingPololu() {
  Serial.println("\n[HOMING] Iniciando búsqueda de cero para Pololu...");
  
  if (digitalRead(FC1_PIN) == LOW) {
    Serial.println("[HOMING] El motor ya se encuentra en el origen.");
  } else {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, POTENCIA_HOMING_PWM);

    while (digitalRead(FC1_PIN) == HIGH) {
      delay(1); 
    }
  }

  analogWrite(IN1_PIN, 0);
  analogWrite(IN2_PIN, 0);

  noInterrupts();
  encoderCount = 0;
  interrupts();

  target_pololu_ticks = 0;
  target_pololu_deg = 0.0;
  pololu_control_active = false;
  buscando_home_ros = false;
  ya_reseteado = true;

  Serial.println("[HOMING] Pololu calibrado con éxito. Posición reseteada a 0 Ticks.\n");
}

// =====================================================
// CALLBACKS DE MICRO-ROS
// =====================================================
void cb_nema(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_nema_deg = msg->data;
  long pasos_objetivo_limpio = (target_nema_deg / 360.0) * PASOS_VUELTA_EFECTOR_NEMA;
  stepper.moveTo(pasos_objetivo_limpio);
}

void cb_pololu(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_pololu_deg = msg->data;
  
  if (target_pololu_deg == 0.0) {
    target_pololu_ticks = 0;
    pololu_control_active = false; 
    buscando_home_ros = true;      
    ya_reseteado = false;
    Serial.println("\n[ROS 2] Comando 0.00 recibido. Buscando final de carrera físico...");
  } 
  else {
    buscando_home_ros = false;
    target_pololu_ticks = (long)(target_pololu_deg * TICKS_PER_DEGREE);
    pololu_control_active = true;
    ya_reseteado = false; 
    
    Serial.print("\n[ROS 2] Grados objetivo: "); Serial.print(target_pololu_deg);
    Serial.print(" | Equivalente a Ticks: "); Serial.println(target_pololu_ticks);
  }
}

// Control local de velocidad mediante aproximación incremental controlada por tiempo
void cb_servo1(const void * msgin) { 
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; 
  int target = constrain(msg->data, 0, 180);
  static int current_pos = 90; // Posición inicial por defecto
  
  while (current_pos != target) {
    if (current_pos < target) current_pos++;
    else current_pos--;
    servo1.write(current_pos);
    delayMicroseconds(10000); // Retardo sintonizado para simular el 75% de velocidad máxima
  }
}

void cb_servo2(const void * msgin) { 
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; 
  int target = 180 - constrain(msg->data, 0, 180); // Invertir J4 (180 es 0, 0 es 180)
  static int current_pos = 90; // Posición inicial por defecto
  
  while (current_pos != target) {
    if (current_pos < target) current_pos++;
    else current_pos--;
    servo2.write(current_pos);
    delayMicroseconds(10000); // Retardo sintonizado para simular el 75% de velocidad máxima
  }
}

void cb_servo3(const void * msgin) { const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; servo3.write(constrain(msg->data, 0, 180)); }

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200); 
  delay(500);
  Serial.println("=== CONTROL POLOLU CON HOMING INTEGRADO ===");

  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);

  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(500.0);

  pinMode(FC1_PIN, INPUT_PULLUP);
  pinMode(FC2_PIN, INPUT_PULLUP);

  servo1.attach(SERVO1_PIN); servo2.attach(SERVO2_PIN); servo3.attach(SERVO3_PIN);
  servo1.write(90); servo2.write(90); servo3.write(90);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  analogWrite(IN1_PIN, 0);
  analogWrite(IN2_PIN, 0);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  ejecutarHomingPololu();

  char ssid[] = "ZTE_2.4G_WRgugR";
  char psk[]  = "aWV6fWY6";
  char agent_ip[] = "192.168.1.102";
  set_microros_wifi_transports(ssid, psk, agent_ip, 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_multi_motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_nema, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_nema/target_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_pololu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_pololu/target_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_servo1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/servo1/target_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_servo2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/servo2/target_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_servo3, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/servo3/target_deg"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_nema, &msg_nema, &cb_nema, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_pololu, &msg_pololu, &cb_pololu, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo1, &msg_servo1, &cb_servo1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo2, &msg_servo2, &cb_servo2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo3, &msg_servo3, &cb_servo3, ON_NEW_DATA));
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
  stepper.run();

  // === INTERRUPCIÓN POR SOFTWARE: SI TOCA EL FINAL DE CARRERA FIJA A 0 ===
  if (digitalRead(FC1_PIN) == LOW) {
    encoderCount = 0; 
    if (buscando_home_ros) {
      analogWrite(IN1_PIN, 0);
      analogWrite(IN2_PIN, 0);
      buscando_home_ros = false;
      Serial.println("[CORRECTO] Origen físico encontrado por comando ROS. Detenido.");
    }
    if (!ya_reseteado) {
      ya_reseteado = true;
      Serial.println("[RESET] Ticks congelados en 0.");
    }
  } else {
    ya_reseteado = false; 
  }

  long current_ticks;
  noInterrupts(); current_ticks = encoderCount; interrupts();

  // ACCIÓN A: EJECUTAR BÚSQUEDA DEL BOTÓN FÍSICO SI EL COMANDO FUE 0.00
  if (buscando_home_ros) {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, POTENCIA_HOMING_PWM);
  }

  // ACCIÓN B: CONTROL INTELIGENTE PROPORCIONAL DE PWM (REEMPLAZA EL ON/OFF)
  if (pololu_control_active) {
    long error_ticks = target_pololu_ticks - current_ticks;

    if (abs(error_ticks) > TOLERANCIA_TICKS) {
      // Calculamos la potencia calculando el error multiplicado por Kp
      int pwm_calculado = (int)(abs(error_ticks) * Kp);
      
      // Limitamos el PWM para que esté en el rango seguro (ni muy rápido, ni tan bajo que no se mueva)
      int pwm_final = constrain(pwm_calculado, POTENCIA_MIN_PWM, POTENCIA_MAX_PWM);

      if (error_ticks > 0) {
        analogWrite(IN1_PIN, 0);
        analogWrite(IN2_PIN, pwm_final);
      } else {
        analogWrite(IN1_PIN, pwm_final);
        analogWrite(IN2_PIN, 0);
      }
    } else {
      // Frenado dinámico inmediato cuando entra en la tolerancia estricta
      analogWrite(IN1_PIN, 0);
      analogWrite(IN2_PIN, 0);
      pololu_control_active = false;
      Serial.print("[CORRECTO] Objetivo alcanzado. Detenido en Ticks: "); Serial.println(current_ticks);
    }
  }

  // MONITOR SERIAL DE TELEMETRÍA (100 MS)
  if (millis() - lastSerialPrintTime >= 100) {
    lastSerialPrintTime = millis();

    Serial.print("Encoder Ticks: ");
    Serial.print(current_ticks);
    Serial.print(" | Objetivo Ticks: ");
    Serial.print(target_pololu_ticks);
    Serial.print(" | FC1 (Pololu): ");
    Serial.print(digitalRead(FC1_PIN) == LOW ? "PRESIONADO" : "LIBRE");
    Serial.print(" | Estado: ");
    if (buscando_home_ros) Serial.println("Buscando Sensor Home");
    else Serial.println(pololu_control_active ? "En Movimiento" : "DETENIDO");
  }
}