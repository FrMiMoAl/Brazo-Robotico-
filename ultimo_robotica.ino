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
#define FC2_PIN 26   // Home NEMA (Final de carrera)

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
// PARÁMETROS DE CONTROL POLOLU Y NEMA
// =====================================================
const float TICKS_PER_DEGREE = 35.56;
const int POTENCIA_MAX_PWM = 170;    
const int POTENCIA_MIN_PWM = 75;     
const int POTENCIA_HOMING_PWM = 170; 
const int TOLERANCIA_TICKS = 2;      
const float Kp = 1.2; 

// Configuración Mecánica NEMA 23
const float MICROSTEPPING = 3200.0;
const float RELATION_NEMA = 2.0;
const float PASOS_VUELTA_EFECTOR_NEMA = MICROSTEPPING * RELATION_NEMA;
const float VELOCIDAD_HOMING_NEMA = -400.0; // Velocidad constante (negativa para retroceder al switch)

// =====================================================
// INSTANCIAS Y VARIABLES MECÁNICAS
// =====================================================
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);
Servo servo1; Servo servo2; Servo servo3;

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

// Variables de Control Manual Serial
int target_servo1_deg = 90;
int target_servo2_deg = 90;
int target_servo3_deg = 90;
int selected_motor = 0; // 0: NEMA, 1: Pololu, 2: Servo1, 3: Servo2, 4: Servo3

// Variables para Control de Velocidad de Servos
float current_servo1_deg = 90.0;
float current_servo2_deg = 90.0;
float current_servo3_deg = 90.0;
unsigned long lastServoUpdateTime = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // Actualizar cada 20 ms
const float SERVO_SPEED_STEP = 2.0;             // Aprox 100 grados por segundo (mitad de velocidad normal)

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
// RUTINA DE HOMING: MOTOR POLOLU
// =====================================================
void ejecutarHomingPololu() {
  Serial.println("[HOMING] Iniciando búsqueda de cero para Pololu...");
  if (digitalRead(FC1_PIN) == LOW) {
    Serial.println("[HOMING] Pololu ya se encuentra en el origen.");
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
  Serial.println("[HOMING] Pololu calibrado con éxito.\n");
}

// =====================================================
// RUTINA DE HOMING: MOTOR PASO A PASO NEMA 23 (NUEVA)
// =====================================================
void ejecutarHomingNema() {
  Serial.println("[HOMING] Iniciando búsqueda de cero para NEMA 23...");
  
  // Habilitar driver paso a paso (si es lógica invertida usa LOW, si no HIGH)
  digitalWrite(ENA_PIN, LOW); 
  delay(50);

  if (digitalRead(FC2_PIN) == LOW) {
    Serial.println("[HOMING] NEMA 23 ya se encuentra en el origen.");
  } else {
    // Fijamos una velocidad constante negativa para retroceder hacia el switch
    stepper.setSpeed(VELOCIDAD_HOMING_NEMA);

    while (digitalRead(FC2_PIN) == HIGH) {
      stepper.runSpeed(); // Ejecuta pasos continuos sin aceleración de forma rápida
    }
  }

  // Frenar en seco el motor paso a paso
  stepper.stop();
  
  // Establecer la posición actual como el Cero (0) absoluto en pasos
  stepper.setCurrentPosition(0);
  target_nema_deg = 0.0;

  Serial.println("[HOMING] NEMA 23 calibrado con éxito. Posición reseteada a 0 pasos.\n");
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

void cb_servo1(const void * msgin) { const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; target_servo1_deg = constrain(msg->data, 0, 180); }
void cb_servo2(const void * msgin) { const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; target_servo2_deg = constrain(msg->data, 0, 180); }
void cb_servo3(const void * msgin) { const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; target_servo3_deg = constrain(msg->data, 0, 180); }

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200); 
  delay(500);
  Serial.println("=== SISTEMA ROBÓTICO MULTI-MOTOR CON DOBLE HOMING ===");

  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); // Bloqueo inicial

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

  // --- CALIBRACIÓN MECÁNICA INICIAL ---
  ejecutarHomingPololu();
  ejecutarHomingNema();

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
  
  // Mover el motor paso a paso de forma fluida si tiene un objetivo cargado
  stepper.run();

  // === CONTROL MANUAL POR SERIAL ===
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') {
      selected_motor = (selected_motor + 1) % 5;
      Serial.print("[MANUAL] Motor seleccionado: ");
      if (selected_motor == 0) Serial.println("NEMA 23 (Paso a Paso)");
      else if (selected_motor == 1) Serial.println("Pololu (DC con Encoder)");
      else if (selected_motor == 2) Serial.println("Servo 1");
      else if (selected_motor == 3) Serial.println("Servo 2");
      else if (selected_motor == 4) Serial.println("Servo 3");
    }
    else if (c == 'w' || c == 'W') {
      if (selected_motor == 0) {
        target_nema_deg += 5.0;
        long pasos_objetivo_limpio = (target_nema_deg / 360.0) * PASOS_VUELTA_EFECTOR_NEMA;
        stepper.moveTo(pasos_objetivo_limpio);
        Serial.print("[MANUAL] NEMA 23 -> "); Serial.println(target_nema_deg);
      }
      else if (selected_motor == 1) {
        target_pololu_deg += 5.0;
        target_pololu_ticks = (long)(target_pololu_deg * TICKS_PER_DEGREE);
        pololu_control_active = true;
        buscando_home_ros = false;
        ya_reseteado = false;
        Serial.print("[MANUAL] Pololu -> "); Serial.println(target_pololu_deg);
      }
      else if (selected_motor == 2) {
        target_servo1_deg = constrain(target_servo1_deg + 5, 0, 180);
        Serial.print("[MANUAL] Servo 1 -> "); Serial.println(target_servo1_deg);
      }
      else if (selected_motor == 3) {
        target_servo2_deg = constrain(target_servo2_deg + 5, 0, 180);
        Serial.print("[MANUAL] Servo 2 -> "); Serial.println(target_servo2_deg);
      }
      else if (selected_motor == 4) {
        target_servo3_deg = constrain(target_servo3_deg + 5, 0, 180);
        Serial.print("[MANUAL] Servo 3 -> "); Serial.println(target_servo3_deg);
      }
    }
    else if (c == 's' || c == 'S') {
      if (selected_motor == 0) {
        target_nema_deg -= 5.0;
        long pasos_objetivo_limpio = (target_nema_deg / 360.0) * PASOS_VUELTA_EFECTOR_NEMA;
        stepper.moveTo(pasos_objetivo_limpio);
        Serial.print("[MANUAL] NEMA 23 -> "); Serial.println(target_nema_deg);
      }
      else if (selected_motor == 1) {
        target_pololu_deg -= 5.0;
        target_pololu_ticks = (long)(target_pololu_deg * TICKS_PER_DEGREE);
        pololu_control_active = true;
        buscando_home_ros = false;
        ya_reseteado = false;
        Serial.print("[MANUAL] Pololu -> "); Serial.println(target_pololu_deg);
      }
      else if (selected_motor == 2) {
        target_servo1_deg = constrain(target_servo1_deg - 5, 0, 180);
        Serial.print("[MANUAL] Servo 1 -> "); Serial.println(target_servo1_deg);
      }
      else if (selected_motor == 3) {
        target_servo2_deg = constrain(target_servo2_deg - 5, 0, 180);
        Serial.print("[MANUAL] Servo 2 -> "); Serial.println(target_servo2_deg);
      }
      else if (selected_motor == 4) {
        target_servo3_deg = constrain(target_servo3_deg - 5, 0, 180);
        Serial.print("[MANUAL] Servo 3 -> "); Serial.println(target_servo3_deg);
      }
    }
  }

  // === MONITOREO FINAL DE CARRERA POLOLU ===
  if (digitalRead(FC1_PIN) == LOW) {
    encoderCount = 0; 
    if (buscando_home_ros) {
      analogWrite(IN1_PIN, 0);
      analogWrite(IN2_PIN, 0);
      buscando_home_ros = false;
      Serial.println("[CORRECTO] Origen físico Pololu encontrado. Detenido.");
    }
    if (!ya_reseteado) {
      ya_reseteado = true;
    }
  } else {
    ya_reseteado = false; 
  }

  // === SEGURIDAD ADICIONAL NEMA 23 ===
  // Si por accidente choca el fin de carrera del NEMA en operacion normal, detiene el objetivo
  if (digitalRead(FC2_PIN) == LOW && stepper.speed() < 0) {
     stepper.stop();
     stepper.setCurrentPosition(0);
  }

  long current_ticks;
  noInterrupts(); current_ticks = encoderCount; interrupts();

  // ACCIÓN A: BUSCANDO HOME MECÁNICO POLOLU (COMANDO 0.00)
  if (buscando_home_ros) {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, POTENCIA_HOMING_PWM);
  }

  // ACCIÓN B: CONTROL PROPORCIONAL POLOLU
  if (pololu_control_active) {
    long error_ticks = target_pololu_ticks - current_ticks;

    if (abs(error_ticks) > TOLERANCIA_TICKS) {
      int pwm_calculado = (int)(abs(error_ticks) * Kp);
      int pwm_final = constrain(pwm_calculado, POTENCIA_MIN_PWM, POTENCIA_MAX_PWM);

      if (error_ticks > 0) {
        analogWrite(IN1_PIN, 0);
        analogWrite(IN2_PIN, pwm_final);
      } else {
        analogWrite(IN1_PIN, pwm_final);
        analogWrite(IN2_PIN, 0);
      }
    } else {
      analogWrite(IN1_PIN, 0);
      analogWrite(IN2_PIN, 0);
      pololu_control_active = false;
      Serial.print("[CORRECTO] Pololu en Ticks: "); Serial.println(current_ticks);
    }
  }

  // === MOVIMIENTO GRADUAL Y SUAVE DE SERVOS (CONTROL DE VELOCIDAD) ===
  if (millis() - lastServoUpdateTime >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdateTime = millis();
    
    // Servo 1
    if (current_servo1_deg < target_servo1_deg) {
      current_servo1_deg = min(current_servo1_deg + SERVO_SPEED_STEP, (float)target_servo1_deg);
      servo1.write((int)current_servo1_deg);
    } else if (current_servo1_deg > target_servo1_deg) {
      current_servo1_deg = max(current_servo1_deg - SERVO_SPEED_STEP, (float)target_servo1_deg);
      servo1.write((int)current_servo1_deg);
    }
    
    // Servo 2
    if (current_servo2_deg < target_servo2_deg) {
      current_servo2_deg = min(current_servo2_deg + SERVO_SPEED_STEP, (float)target_servo2_deg);
      servo2.write((int)current_servo2_deg);
    } else if (current_servo2_deg > target_servo2_deg) {
      current_servo2_deg = max(current_servo2_deg - SERVO_SPEED_STEP, (float)target_servo2_deg);
      servo2.write((int)current_servo2_deg);
    }
    
    // Servo 3
    if (current_servo3_deg < target_servo3_deg) {
      current_servo3_deg = min(current_servo3_deg + SERVO_SPEED_STEP, (float)target_servo3_deg);
      servo3.write((int)current_servo3_deg);
    } else if (current_servo3_deg > target_servo3_deg) {
      current_servo3_deg = max(current_servo3_deg - SERVO_SPEED_STEP, (float)target_servo3_deg);
      servo3.write((int)current_servo3_deg);
    }
  }

  // MONITOR SERIAL DE TELEMETRÍA (100 MS)
  if (millis() - lastSerialPrintTime >= 100) {
    lastSerialPrintTime = millis();

    Serial.print("Pololu Ticks: "); Serial.print(current_ticks);
    Serial.print(" | NEMA Pos Pasos: "); Serial.print(stepper.currentPosition());
    Serial.print(" | FC1: "); Serial.print(digitalRead(FC1_PIN) == LOW ? "PRES" : "FREE");
    Serial.print(" | FC2: "); Serial.println(digitalRead(FC2_PIN) == LOW ? "PRES" : "FREE");
  }
}