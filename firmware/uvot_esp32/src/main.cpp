// ============================================================================
// UVOT — Firmware de la base móvil (ESP32 + micro-ROS)
// ============================================================================
//
// Una sola base de código para las tres configuraciones de la plataforma.
// La configuración se elige al compilar con ROBOT_TYPE:
//   ROBOT_DIFF (1) diferencial, 2 ruedas
//   ROBOT_SKID (2) skid-steering, 4 ruedas convencionales
//   ROBOT_OMNI (3) omnidireccional, 4 ruedas mecanum
//
// Estructura de ejecución:
//   - esp_timer cada 10 ms  → timer_control_callback(): encoders, EKF,
//                             cinemática inversa y PID de las ruedas.
//   - tarea "inertial"      → lectura de la IMU a 100 Hz, calibración y
//                             re-estimación del bias del giróscopo.
//   - tarea "topics"        → micro-ROS: conexión con el agente, publicación
//                             a 20 Hz (odom, imu, joint_states, period) y
//                             recepción de cmd_vel / cmd_joints.
//
// Convenio de ejes (ROS): x hacia delante, y hacia la izquierda, z hacia arriba.
// Convenio de ruedas:
//   diff : W[0] izquierda, W[1] derecha
//   skid : W[0], W[3] lado izquierdo; W[1], W[2] lado derecho
//   omni : mismo orden que en inverse_kinematics()
// ============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/time.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "esp_timer.h"

#include "rosidl_runtime_c/string.h"
#include <string>
#include <math.h>
#include <algorithm>   // std::sort (calibración del bias por mediana)

#include "esp_system.h"

#include <ESP32Encoder.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

// ============================================================
// Selección de plataforma
// ============================================================
#define ROBOT_DIFF  1
#define ROBOT_SKID  2
#define ROBOT_OMNI  3

#ifndef ROBOT_TYPE
#define ROBOT_TYPE ROBOT_OMNI
#endif

#if ROBOT_TYPE == ROBOT_DIFF
  #include "DiffDriveEKF.h"
#elif ROBOT_TYPE == ROBOT_SKID
  #include "SkidSteerEKF.h"
#elif ROBOT_TYPE == ROBOT_OMNI
  #include "OmniDriveEKF.h"
#else
  #error "ROBOT_TYPE inválido"
#endif

#include <sensor_msgs/msg/time_reference.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#if ROBOT_TYPE == ROBOT_DIFF
  #define WHEEL_NUMBER 2
#else
  #define WHEEL_NUMBER 4
#endif

#define UVOT_FIRMWARE_VERSION "1.0.0"

#if ROBOT_TYPE == ROBOT_DIFF
  #define ROBOT_TYPE_NAME "diff"
#elif ROBOT_TYPE == ROBOT_SKID
  #define ROBOT_TYPE_NAME "skid"
#elif ROBOT_TYPE == ROBOT_OMNI
  #define ROBOT_TYPE_NAME "omni"
#endif

// Estados de la conexión con el agente micro-ROS
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED
};

#ifndef ROBOT_NAMESPACE
#define ROBOT_NAMESPACE "robot"
#endif

std::string ns = ROBOT_NAMESPACE;
std::string topic_name;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// ============================================================
// Pines
// ============================================================
// Motor 1
const int encoder1A  = 15;
const int encoder1B  =  4;
const int motor1Pin1 = 13;
const int motor1Pin2 = 27;

// Motor 2
const int encoder2A  = 17;
const int encoder2B  = 16;
const int motor2Pin1 = 26;
const int motor2Pin2 = 25;

// Motor 3
const int encoder3A  =  5;
const int encoder3B  = 18;
const int motor3Pin1 = 19;
const int motor3Pin2 = 23;

// Motor 4
const int encoder4A  = 34;
const int encoder4B  = 35;
const int motor4Pin1 = 32;
const int motor4Pin2 = 33;

// ============================================================
// Encoders
// ============================================================
ESP32Encoder encoder1, encoder2, encoder3, encoder4;
const float encoderPPR = 900.0f;  // cuentas por vuelta del eje de salida (calibrado)

int64_t previousNanos = 0;        // instante del ciclo de control anterior

// Velocidad medida (pv) y consigna (sp) de cada rueda, en rpm
float pv1 = 0, pv2 = 0, pv3 = 0, pv4 = 0;
float sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;

// Variables de los PID (error, integral, derivada, acción)
float e1 = 0, e2 = 0, e3 = 0, e4 = 0;
float e1_k1 = 0, e2_k1 = 0, e3_k1 = 0, e4_k1 = 0;
float e1_k2 = 0, e2_k2 = 0, e3_k2 = 0, e4_k2 = 0;
float e1i = 0, e2i = 0, e3i = 0, e4i = 0;
float e1d = 0, e2d = 0, e3d = 0, e4d = 0;
float u1 = 0, u2 = 0, u3 = 0, u4 = 0;
float u1_k1 = 0, u2_k1 = 0, u3_k1 = 0, u4_k1 = 0;

// PWM de los drivers (10 bits; pwmHalf = motor parado)
const int pwmFreq = 30000;
const int pwmChannel = 0;
const int pwmBits = 10;
uint32_t pwmFull, pwmHalf;

// Cuentas de encoder actuales y del ciclo anterior
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;
volatile long count1_k1 = 0;
volatile long count2_k1 = 0;
volatile long count3_k1 = 0;
volatile long count4_k1 = 0;

// ============================================================
// Geometría y calibración
// ============================================================
// a: semidistancia longitudinal (centro → eje de ruedas)
// b: semidistancia transversal  (centro → línea media de las ruedas)
// Medidas por bordes (exterior e interior) y promediadas.
#if ROBOT_TYPE == ROBOT_OMNI
const float a = 0.08125f; // batalla 16.25 cm
const float b = 0.105f;   // vía 21 cm          → a + b = 0.18625
const float r = 0.04f;    // rueda mecanum Ø 80 mm
#elif ROBOT_TYPE == ROBOT_SKID
const float a = 0.0825f;  // batalla 16.5 cm (no interviene en la cinemática)
const float b = 0.104f;   // vía 20.75 cm (geométrica; la cinemática usa B_SKID)
const float r = 0.0325f;  // rueda convencional Ø 65 mm
#else  // ROBOT_DIFF
const float a = 0.0f;     // ruedas sobre el eje
const float b = 0.11f;    // vía 22 cm, verificada: encoders y giróscopo coinciden (< 1 %)
const float r = 0.0325f;  // rueda convencional Ø 65 mm
#endif

// --- Solo skid-steering ---
// Semiancho de vía EFECTIVO: b_eff = χ·b. Las ruedas de un skid deslizan
// lateralmente al girar, y el centro instantáneo de rotación de cada lado
// queda más lejos que la rueda (modelo ICR). χ depende del suelo, del
// neumático y de la relación a/b; medido en el laboratorio: χ = 0.17/0.104 ≈ 1.64.
// Recalibrar en otro suelo con un giro de 360° en lazo abierto:
//   B_SKID_nuevo = B_SKID * 360 / ángulo_real_girado
const float B_SKID = 0.17f;

// Corrección de radio efectivo por lado: izquierdo r(1+T), derecho r(1-T).
// Compensa una asimetría mecánica entre lados. Depende del montaje: con el
// montaje de noviembre (v13–v19) era 0.012 (las ruedas izquierdas avanzaban
// ~2 % más por vuelta); tras volver a montar el skid, con 0.012 el robot
// giraba ~9° a la izquierda en 2 m, lo mismo que produce el propio
// SIDE_TRIM sin asimetría que compensar → 0.
// Recalibrar tras cada montaje: con una recta de 2 m, cada 0.001 de
// SIDE_TRIM gira el robot ≈ 0.7° (positivo = hacia la izquierda).
const float SIDE_TRIM = 0.0f;

unsigned int period_us = 10000;   // periodo del lazo de control (10 ms)

// --- Feedforward por rueda (motor identificado en lazo abierto) ---
// Modelo estático: ω = FF_K·(|u| − FF_U0), u = ciclo de trabajo en [-1, 1].
// u_ff = signo(ω_ref)·(FF_U0 + |ω_ref|/FF_K); el PI solo corrige el resto.
// Solo con |ω_ref| > FF_W_MIN para no vibrar con el robot parado.
#if ROBOT_TYPE == ROBOT_OMNI
const bool  FF_ON = true;
const float FF_U0 = 0.15f;    // zona muerta (fricción dinámica), suelo
const float FF_K  = 21.0f;    // rad/s por unidad de PWM, suelo
#elif ROBOT_TYPE == ROBOT_SKID
const bool  FF_ON = true;     // identificado en el suelo (nov. 2026)
const float FF_U0 = 0.14f;    // zona muerta del modelo, suelo
const float FF_K  = 21.0f;    // rad/s por unidad de PWM, suelo
#else  // ROBOT_DIFF
const bool  FF_ON = true;     // identificado en el suelo (nov. 2026)
const float FF_U0 = 0.16f;    // zona muerta del modelo, suelo
const float FF_K  = 21.0f;    // rad/s por unidad de PWM, suelo
#endif
const float FF_W_MIN = 0.1f;  // rad/s

// --- Protección de rueda bloqueada ---
// Si una rueda recibe más de STALL_U del PWM y gira a menos de STALL_W
// durante STALL_N ciclos (1 s), se corta su PWM: un motor bloqueado con el PI
// empujando se calienta y se daña. La rueda queda desactivada hasta que su
// consigna vuelva a cero (robot parado). En joint_states, 'effort' publica el
// PWM aplicado a cada rueda en [-1, 1]: una rueda bloqueada aparece con
// effort = 0 aunque tenga consigna.
const float STALL_U = 0.6f;     // fracción de PWM
const float STALL_W = 0.2f;     // rad/s
const int   STALL_N = 100;      // ciclos de 10 ms
bool stalled[4] = {false, false, false, false};
int  stall_cnt[4] = {0, 0, 0, 0};

float stall_guard(int i, float u, float w_meas)
{
  if (stalled[i]) return 0.0f;
  if (fabsf(u) > STALL_U * float(pwmHalf) && fabsf(w_meas) < STALL_W) {
    if (++stall_cnt[i] > STALL_N) {
      stalled[i] = true;
      return 0.0f;
    }
  } else {
    stall_cnt[i] = 0;
  }
  return u;
}
float fc = 1000.0f;

// Ganancias PI (sobre velocidad de rueda en rpm; salida en cuentas de PWM)
// Ajuste IMC con el motor identificado en el suelo (omni, nov. 2026):
// τ ≈ 0.105 s → Ti = τ (cancelación polo-cero); τ_cl = 0.5·τ → Kp ≈ 5.
float Kp = 5.0f;
float Ti = 0.1f;
float Td = 0.0f;     // sin acción derivativa: el lazo de velocidad es un PI
float Ki;
float Kd;

// Estado estimado del robot (marco odom)
float x, y, z;
float vx, vy, wz;
float ax, ay;

// ============================================================
// IMU
// ============================================================
volatile float wz_bias = 0.0f;            // bias del giróscopo z (rad/s)
xyzFloat acc_off = {0.0f, 0.0f, 0.0f};    // offsets de autoOffsets(), cuentas crudas
xyzFloat gyr_off = {0.0f, 0.0f, 0.0f};
volatile bool imu_ready = false;          // calibración terminada → arranca el EKF
volatile bool gyro_bias_ready = false;    // wz_bias ya se aplica a las lecturas
volatile bool ekf_ready = false;
volatile bool robot_stationary = false;   // ruedas quietas y sin consigna (lo calcula el control)

float wx_imu, wy_imu, wz_imu;   // rad/s, ejes del robot
float ax_imu, ay_imu, az_imu;   // m/s², az sin gravedad
float mx_imu, my_imu, mz_imu;   // magnetómetro

// Offsets (no usados actualmente)
float oyaw, owz, oax, oay;

// Velocidades de rueda: medidas (Wd) y consigna (Wi), en rad/s
float Wd[WHEEL_NUMBER];
float Wi[WHEEL_NUMBER];
float Vd[3];

float dT = 0.01f;           // periodo real medido del último ciclo (s)
uint32_t count = 0;         // ciclos sin recibir comando
uint32_t countMax = 200;    // timeout de comando: 200 ciclos de 10 ms = 2 s

// Frames
char frame_id[30] = "odom";
char child_frame_id[30] = "base_footprint";
char imu_frame_id[30] = "base_link";
char frame_id_ns[30];
char child_frame_id_ns[30];
char imu_frame_id_ns[30];

// Consigna de cmd_vel
float vx_r, vy_r, wz_r;

// Valores de la IMU en unidades de la librería (g, °/s, µT)
xyzFloat gValue;
xyzFloat gyr;
xyzFloat magValue;
float temp;
float resultantG;

// Orientación
float qw, qx, qy, qz;
float roll, pitch, yaw;
float heading, heading0;

volatile bool ros_connected = false;   // sesión con el agente establecida
volatile bool ros_kinematics = true;   // true: consigna por cmd_vel; false: por cmd_joints
// Modo PWM directo (lazo abierto, para identificar los motores): si cmd_joints
// trae 'effort', cada valor es el ciclo de trabajo de una rueda en [-1, 1] y el
// PID queda desactivado. Sale del modo al llegar un cmd_vel, un cmd_joints sin
// effort o por el timeout de 2 s sin comandos.
volatile bool pwm_mode = false;
volatile float pwm_duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ============================================================
// EKF por plataforma
// ============================================================
// Q: ruido de proceso. R: ruido de medida (cuanto menor, más se confía).
// En las tres configuraciones el giróscopo tiene mucho peso (R pequeña):
// es la única medida de wz que no se ve afectada por el deslizamiento.

#if ROBOT_TYPE == ROBOT_DIFF

// Estado: [x, y, θ, v, w, a]
const float Q_ekf[6][6] = {
  {1e-6f, 0,     0,     0,     0,     0},
  {0,     1e-6f, 0,     0,     0,     0},
  {0,     0,     1e-6f, 0,     0,     0},
  {0,     0,     0,     1e-4f, 0,     0},
  {0,     0,     0,     0,     1e-4f, 0},
  {0,     0,     0,     0,     0,     1e-3f}
};

// Medidas: [ω_der, ω_izq, ax_imu, ay_imu, wz_imu]  (orden de DiffDriveEKF)
const float R_ekf[5][5] = {
  {0.0025f, 0,        0,        0,        0},
  {0,       0.0025f,  0,        0,        0},
  {0,       0,        0.0025f,  0,        0},
  {0,       0,        0,        0.1000f,  0},
  {0,       0,        0,        0,        2.5e-5f}
};

const float x0_ekf[6] = {0, 0, 0, 0, 0, 0};

const float P0_ekf[6][6] = {
  {1e-2f, 0,     0,     0,     0,     0},
  {0,     1e-2f, 0,     0,     0,     0},
  {0,     0,     1e-3f, 0,     0,     0},
  {0,     0,     0,     1e-1f, 0,     0},
  {0,     0,     0,     0,     1e-1f, 0},
  {0,     0,     0,     0,     0,     1e-1f}
};

// DiffDriveEKF usa "b" como distancia TOTAL entre ruedas (su modelo es
// ω = (v ± 0.5·b·w)/r), mientras que aquí b es la semidistancia: se pasa 2·b.
DiffDriveEKF ekf((float)period_us * 1e-6f, r, 2.0f * b, Q_ekf, R_ekf, x0_ekf, P0_ekf);

#elif ROBOT_TYPE == ROBOT_SKID

// Estado: [x, y, θ, v, w, a]
const float Q_ekf[6][6] = {
  {1e-6f, 0,     0,     0,     0,     0},
  {0,     1e-6f, 0,     0,     0,     0},
  {0,     0,     1e-6f, 0,     0,     0},
  {0,     0,     0,     1e-4f, 0,     0},
  {0,     0,     0,     0,     2e-4f, 0},
  {0,     0,     0,     0,     0,     2e-3f}
};

// Medidas: [ω_der, ω_izq, ax_imu, ay_imu, wz_imu]
const float R_ekf[5][5] = {
  {0.0050f, 0,        0,        0,        0},
  {0,       0.0050f,  0,        0,        0},
  {0,       0,        1000.0f,  0,        0},       // ax prácticamente ignorado
  {0,       0,        0,        1000.0f,  0},       // ay prácticamente ignorado
  {0,       0,        0,        0,        5.0e-5f}  // wz_imu
};

const float x0_ekf[6] = {0, 0, 0, 0, 0, 0};

const float P0_ekf[6][6] = {
  {1e-2f, 0,     0,     0,     0,     0},
  {0,     1e-2f, 0,     0,     0,     0},
  {0,     0,     1e-3f, 0,     0,     0},
  {0,     0,     0,     1e-1f, 0,     0},
  {0,     0,     0,     0,     1e-1f, 0},
  {0,     0,     0,     0,     0,     1e-1f}
};

SkidSteerEKF ekf((float)period_us * 1e-6f, r, B_SKID, Q_ekf, R_ekf, x0_ekf, P0_ekf);

#elif ROBOT_TYPE == ROBOT_OMNI

// Estado: [x, y, θ, vx_body, vy_body, wz, ax_body, ay_body]
const float Q_ekf[8][8] = {
  {1e-6f, 0,     0,     0,     0,     0,     0,     0},
  {0,     1e-6f, 0,     0,     0,     0,     0,     0},
  {0,     0,     1e-6f, 0,     0,     0,     0,     0},
  {0,     0,     0,     1e-4f, 0,     0,     0,     0},
  {0,     0,     0,     0,     1e-4f, 0,     0,     0},
  {0,     0,     0,     0,     0,     1e-4f, 0,     0},
  {0,     0,     0,     0,     0,     0,     1e-3f, 0},
  {0,     0,     0,     0,     0,     0,     0,     1e-3f}
};

// Medidas: [ω1, ω2, ω3, ω4, wz_imu, ax_imu, ay_imu]
// wz_imu con 5e-5 (antes 1e-3): con 1e-3 la odometría ignoraba el giro
// que producen los rodillos al deslizar y que los encoders no ven.
const float R_ekf[7][7] = {
  {0.0025f, 0,        0,        0,        0,        0,        0},
  {0,       0.0025f,  0,        0,        0,        0,        0},
  {0,       0,        0.0025f,  0,        0,        0,        0},
  {0,       0,        0,        0.0025f,  0,        0,        0},
  {0,       0,        0,        0,        5.0e-5f,  0,        0},
  {0,       0,        0,        0,        0,        0.0500f,  0},
  {0,       0,        0,        0,        0,        0,        0.0500f}
};

const float x0_ekf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

const float P0_ekf[8][8] = {
  {1e-2f, 0,     0,     0,     0,     0,     0,     0},
  {0,     1e-2f, 0,     0,     0,     0,     0,     0},
  {0,     0,     1e-3f, 0,     0,     0,     0,     0},
  {0,     0,     0,     1e-1f, 0,     0,     0,     0},
  {0,     0,     0,     0,     1e-1f, 0,     0,     0},
  {0,     0,     0,     0,     0,     1e-1f, 0,     0},
  {0,     0,     0,     0,     0,     0,     1e-1f, 0},
  {0,     0,     0,     0,     0,     0,     0,     1e-1f}
};

OmniDriveEKF ekf((float)period_us * 1e-6f, r, a, b, Q_ekf, R_ekf, x0_ekf, P0_ekf);

#endif

// ============================================================
// Objetos ROS
// ============================================================
rcl_publisher_t publisher_period;
sensor_msgs__msg__TimeReference period;
rcl_publisher_t publisher_odom;
nav_msgs__msg__Odometry odom;
rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu imu;
rcl_publisher_t publisher_joints;
sensor_msgs__msg__JointState joints;

rcl_subscription_t subscriber_twist;
geometry_msgs__msg__Twist cmd_vel;
rcl_subscription_t subscriber_joints;
sensor_msgs__msg__JointState cmd_joints;

rcl_publisher_t publisher_robot_info;
std_msgs__msg__String robot_info;

uint8_t robot_info_counter = 0;

rcl_allocator_t allocator_topics;
rclc_support_t support_topics;
rcl_node_t node_topics;
rclc_executor_t executor_topics;
rcl_timer_t timer_publish;

esp_timer_handle_t timer_control_handle;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ============================================================
// Utilidades
// ============================================================
void error_loop() {
  while (1) {
    delay(100);
  }
}

float unwrap(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

// ============================================================
// Cinemática
// ============================================================
// Directa: velocidades de rueda W (rad/s) → velocidad del robot V = [vx, vy, wz]
void direct_kinematics(float* W, float* V) {
#if ROBOT_TYPE == ROBOT_OMNI
  // Mecanum: los rodillos a 45° absorben la componente lateral, por eso
  // el término de giro es a + b (exacto, sin calibración).
  V[0] = (W[0] + W[1] + W[2] + W[3]) * r / 4.0f;
  V[1] = (W[0] - W[1] + W[2] - W[3]) * r / 4.0f;
  V[2] = (-W[0] + W[1] + W[2] - W[3]) * r / (4.0f * (a + b));
#elif ROBOT_TYPE == ROBOT_SKID
  // Skid: igual que un diferencial con semiancho efectivo B_SKID
  V[0] = (W[0] + W[1] + W[2] + W[3]) * r / 4.0f;
  V[1] = 0.0f;
  V[2] = (W[1] + W[2] - W[3] - W[0]) * r / (4.0f * B_SKID);
#else
  // Diferencial: semiancho geométrico b (las ruedas están en el eje, no deslizan)
  V[0] = (W[0] + W[1]) * r / 2.0f;
  V[1] = 0.0f;
  V[2] = (W[1] - W[0]) * r / (2.0f * b);
#endif
}

// Inversa: consigna (vx, vy, wz) → velocidades de rueda W (rad/s)
void inverse_kinematics(float vx_cmd, float vy_cmd, float wz_cmd, float* W) {
#if ROBOT_TYPE == ROBOT_OMNI
  W[0] = (vx_cmd + vy_cmd - (a + b) * wz_cmd) / r;
  W[1] = (vx_cmd - vy_cmd + (a + b) * wz_cmd) / r;
  W[2] = (vx_cmd + vy_cmd + (a + b) * wz_cmd) / r;
  W[3] = (vx_cmd - vy_cmd - (a + b) * wz_cmd) / r;
#elif ROBOT_TYPE == ROBOT_SKID
  (void)vy_cmd;
  // Lado izquierdo con radio r(1+T) y derecho con r(1-T)
  W[0] = (vx_cmd - B_SKID * wz_cmd) / (r * (1.0f + SIDE_TRIM));
  W[1] = (vx_cmd + B_SKID * wz_cmd) / (r * (1.0f - SIDE_TRIM));
  W[2] = W[1];
  W[3] = W[0];
#else
  (void)vy_cmd;
  W[0] = (vx_cmd - b * wz_cmd) / r;
  W[1] = (vx_cmd + b * wz_cmd) / r;
#endif
}

void compute_euler_angles(float qw_, float qx_, float qy_, float qz_) {
  double sinr_cosp = 2 * (qw_ * qx_ + qy_ * qz_);
  double cosr_cosp = 1 - 2 * (qx_ * qx_ + qy_ * qy_);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = std::sqrt(1 + 2 * (qw_ * qy_ - qx_ * qz_));
  double cosp = std::sqrt(1 - 2 * (qw_ * qy_ - qx_ * qz_));
  pitch = 2 * std::atan2(sinp, cosp) - PI / 2;

  double siny_cosp = 2 * (qw_ * qz_ + qx_ * qy_);
  double cosy_cosp = 1 - 2 * (qy_ * qy_ + qz_ * qz_);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

void reset_pid_integral()
{
  e1i = 0.0f;
  e2i = 0.0f;
  e3i = 0.0f;
  e4i = 0.0f;
}

void compute_quaternion(float roll_, float pitch_, float yaw_) {
  if (!isfinite(roll_))  roll_  = 0.0f;
  if (!isfinite(pitch_)) pitch_ = 0.0f;
  if (!isfinite(yaw_))   yaw_   = 0.0f;

  float cr = cos(roll_ * 0.5f);
  float sr = sin(roll_ * 0.5f);
  float cp = cos(pitch_ * 0.5f);
  float sp = sin(pitch_ * 0.5f);
  float cy = cos(yaw_ * 0.5f);
  float sy = sin(yaw_ * 0.5f);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

float compute_compass_heading(void) {
  float h = atan2f(mx_imu, my_imu);
  if (h > PI) h -= 2.0f * PI;
  else if (h < -PI) h += 2.0f * PI;
  else if (h < 0) h += 2.0f * PI;
  return h;
}

// ============================================================
// EKF
// ============================================================
void ekf_set_initial_state() {
#if ROBOT_TYPE == ROBOT_OMNI
  ekf.x[0] = x;
  ekf.x[1] = y;
  ekf.x[2] = yaw;
  ekf.x[3] = 0.0f; // vx_body
  ekf.x[4] = 0.0f; // vy_body
  ekf.x[5] = 0.0f; // wz
  ekf.x[6] = 0.0f; // ax_body
  ekf.x[7] = 0.0f; // ay_body
#else
  ekf.x[0] = x;
  ekf.x[1] = y;
  ekf.x[2] = yaw;
  ekf.x[3] = 0.0f; // v
  ekf.x[4] = 0.0f; // w
  ekf.x[5] = 0.0f; // a
#endif
}

// Copia el estado del EKF a las variables globales.
// Pose (x, y, yaw) en el marco odom; velocidades y aceleraciones en el marco
// del robot (base_footprint), como pide nav_msgs/Odometry: el twist se expresa
// en child_frame_id. Nav2 y robot_localization lo interpretan así.
void ekf_copy_to_global_state() {
#if ROBOT_TYPE == ROBOT_OMNI
  float px = ekf.x[0];
  float py = ekf.x[1];
  float th = ekf.x[2];
  float vx_body = ekf.x[3];
  float vy_body = ekf.x[4];
  float wz_est  = ekf.x[5];
  float ax_body = ekf.x[6];
  float ay_body = ekf.x[7];

  x = px;
  y = py;
  yaw = th;

  vx = vx_body;
  vy = vy_body;
  wz = wz_est;

  ax = ax_body;
  ay = ay_body;
#else
  float px = ekf.x[0];
  float py = ekf.x[1];
  float th = ekf.x[2];
  float v_est  = ekf.x[3];
  float w_est  = ekf.x[4];
  float a_est  = ekf.x[5];

  x = px;
  y = py;
  yaw = th;

  vx = v_est;      // diff y skid: sin velocidad lateral en el marco del robot
  vy = 0.0f;
  wz = w_est;

  ax = a_est;
  ay = 0.0f;
#endif
}

void ekf_predict_step(float dt_now) {
  ekf.dt = dt_now;
  ekf.predict();
}

// Corrección del EKF con encoders e IMU.
// En las tres configuraciones se aplica ZUPT: con las ruedas quietas y sin
// consigna el robot no puede girar, así que se usa wz = 0 en lugar del
// giróscopo. Así un bias residual no hace girar la odometría en parado.
void ekf_update_step() {
#if ROBOT_TYPE == ROBOT_OMNI
  float wz_meas = robot_stationary ? 0.0f : wz_imu;

  float z_meas[7] = {
    Wd[0], Wd[1], Wd[2], Wd[3], wz_meas, ax_imu, ay_imu
  };

  bool valid_meas = true;
  for (int i = 0; i < 7; i++) {
    if (!isfinite(z_meas[i])) {
      valid_meas = false;
      break;
    }
  }
  if (valid_meas) ekf.update(z_meas);

#elif ROBOT_TYPE == ROBOT_SKID
  // Velocidades de cada lado llevadas al radio nominal r del modelo,
  // con la misma corrección por lado que la cinemática inversa.
  float omegaL = 0.5f * (Wd[0] + Wd[3]) * (1.0f + SIDE_TRIM);
  float omegaR = 0.5f * (Wd[1] + Wd[2]) * (1.0f - SIDE_TRIM);

  // Si el giróscopo da un valor imposible o muy distinto de lo que dicen
  // los encoders, en esta iteración se usa el wz de los encoders.
  float wz_enc  = r * (omegaR - omegaL) / (2.0f * B_SKID);
  float wz_meas = wz_imu;
  if (!isfinite(wz_meas) ||
      fabsf(wz_meas) > 3.0f ||
      fabsf(wz_meas - wz_enc) > 0.8f) {
    wz_meas = wz_enc;
  }

  if (robot_stationary) {
    wz_meas = 0.0f;   // ZUPT
  }

  float z_meas[5] = {
    omegaR, omegaL, ax_imu, ay_imu, wz_meas
  };

  bool valid_meas = true;
  for (int i = 0; i < 5; i++) {
    if (!isfinite(z_meas[i])) {
      valid_meas = false;
      break;
    }
  }
  if (valid_meas) ekf.update(z_meas);

#else
  float wz_meas = robot_stationary ? 0.0f : wz_imu;   // ZUPT

  // DiffDriveEKF espera [ω_der, ω_izq, ...]; Wd[0] es la izquierda y Wd[1]
  // la derecha. Con el orden cambiado los encoders "veían" el giro con el
  // signo contrario y el EKF subestimaba wz un 17 %.
  float z_meas[5] = {
    Wd[1], Wd[0], ax_imu, ay_imu, wz_meas
  };

  bool valid_meas = true;
  for (int i = 0; i < 5; i++) {
    if (!isfinite(z_meas[i])) {
      valid_meas = false;
      break;
    }
  }
  if (valid_meas) ekf.update(z_meas);
#endif
}

// ============================================================
// IMU
// ============================================================

// Lee ACCEL_OUT..GYRO_OUT (0x3B–0x48, 14 bytes) en una sola transacción I²C
// comprobando cada paso. Las funciones de lectura de la librería no detectan
// errores de I²C y, si una transacción falla, devuelven bytes basura (picos
// de más de 100 °/s que contaminaban la calibración del bias).
static bool mpu_burst_read(uint8_t *buf) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);                                   // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14, (uint8_t)true) != 14)
    return false;
  for (int i = 0; i < 14; i++) {
    int v = Wire.read();
    if (v < 0) return false;
    buf[i] = (uint8_t)v;
  }
  return true;
}

// Entero de 16 bits big-endian
static inline int16_t be16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void init_imu(void) {
  Wire.begin();   // 100 kHz y timeout por defecto

  // La librería se usa solo para configurar el sensor y obtener los offsets
  myMPU9250.init();
  myMPU9250.initMagnetometer();
  myMPU9250.autoOffsets();
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);          // ~5 Hz de ancho de banda
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  // Offsets en cuentas crudas (rangos 2G y 250 °/s → factor de rango 1)
  acc_off = myMPU9250.getAccOffsets();
  gyr_off = myMPU9250.getGyrOffsets();

  delay(200);

  heading0 = 0.0f;
}

// Lee la IMU y actualiza las variables *_imu.
// Devuelve false si la lectura I²C falla; en ese caso se conservan los
// últimos valores buenos.
bool read_imu(void) {
  uint8_t buf[14];
  if (!mpu_burst_read(buf)) {
    return false;
  }

  // Conversión a unidades físicas (g y °/s)
  gValue.x = (be16(buf + 0)  - acc_off.x) / 16384.0f;
  gValue.y = (be16(buf + 2)  - acc_off.y) / 16384.0f;
  gValue.z = (be16(buf + 4)  - acc_off.z) / 16384.0f;
  temp     =  be16(buf + 6) / 333.87f + 21.0f;
  gyr.x    = (be16(buf + 8)  - gyr_off.x) / 131.072f;
  gyr.y    = (be16(buf + 10) - gyr_off.y) / 131.072f;
  gyr.z    = (be16(buf + 12) - gyr_off.z) / 131.072f;

  magValue   = myMPU9250.getMagValues();   // solo se usa para heading
  resultantG = myMPU9250.getResultantG(gValue);

  // Filtro de plausibilidad del giróscopo z. Con el filtro interno de 5 Hz,
  // un salto de más de 30 °/s entre lecturas de 10 ms no es físicamente
  // posible: se descarta y se mantiene el valor anterior. Tras 3 descartes
  // seguidos se acepta, para no quedarse bloqueado ante un cambio real.
  static float   gz_prev   = 0.0f;
  static bool    have_prev = false;
  static uint8_t rejects   = 0;
  if (have_prev && fabsf(gyr.z - gz_prev) > 30.0f && rejects < 3) {
    rejects++;
    gyr.z = gz_prev;
  } else {
    gz_prev   = gyr.z;
    rejects   = 0;
    have_prev = true;
  }

  // Paso a ejes del robot y unidades SI
  ax_imu =  gValue.x * 9.81f;
  ay_imu = -gValue.y * 9.81f;
  az_imu = (gValue.z - 1.0f) * 9.81f;

  wx_imu =  gyr.x * PI / 180.0f;
  wy_imu = -gyr.y * PI / 180.0f;
  float wz_raw = -gyr.z * PI / 180.0f;

  // Hasta que termina la calibración se usa la lectura sin corregir
  wz_imu = gyro_bias_ready ? (wz_raw - wz_bias) : wz_raw;

  mx_imu = magValue.x;
  my_imu = magValue.y;
  mz_imu = magValue.z;

  heading = compute_compass_heading() - heading0;

  return true;
}

// ============================================================
// Encoders
// ============================================================
void init_encoders(void) {
  encoder1.attachFullQuad(encoder1A, encoder1B);
  encoder2.attachFullQuad(encoder2A, encoder2B);
#if WHEEL_NUMBER == 4
  encoder3.attachFullQuad(encoder3A, encoder3B);
  encoder4.attachFullQuad(encoder4A, encoder4B);
#endif

  encoder1.clearCount();
  encoder2.clearCount();
#if WHEEL_NUMBER == 4
  encoder3.clearCount();
  encoder4.clearCount();
#endif
}

void read_encoders(void) {
  count1 = encoder1.getCount();
  count2 = encoder2.getCount();
#if WHEEL_NUMBER == 4
  count3 = encoder3.getCount();
  count4 = encoder4.getCount();
#endif
}

// ============================================================
// Publicadores
// ============================================================
void publish_odom(int64_t time_ns) {
  odom.header.stamp.sec = floor(time_ns / 1e9);
  odom.header.stamp.nanosec = time_ns - odom.header.stamp.sec * 1e9;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0f;

  compute_quaternion(0.0f, 0.0f, yaw);

  odom.pose.pose.orientation.x = qx;
  odom.pose.pose.orientation.y = qy;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  for (int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = 0.0;
    odom.twist.covariance[i] = 0.0;
  }

  // Covarianza de pose: x, y, θ del EKF; z, roll, pitch no se estiman
  odom.pose.covariance[0]  = ekf.P[0][0];
  odom.pose.covariance[7]  = ekf.P[1][1];
  odom.pose.covariance[35] = ekf.P[2][2];

  odom.pose.covariance[14] = 1e3;
  odom.pose.covariance[21] = 1e3;
  odom.pose.covariance[28] = 1e3;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0.0f;
  odom.twist.twist.angular.x = 0.0f;
  odom.twist.twist.angular.y = 0.0f;
  odom.twist.twist.angular.z = wz;

  odom.twist.covariance[0] = 0.05;
#if ROBOT_TYPE == ROBOT_OMNI
  odom.twist.covariance[7] = 0.05;
#else
  odom.twist.covariance[7] = 1e3;   // diff/skid no tienen velocidad lateral
#endif
  odom.twist.covariance[35] = 0.01;

  RCSOFTCHECK(rcl_publish(&publisher_odom, &odom, NULL));
}

void publish_imu(int64_t time_ns) {
  imu.header.stamp.sec = floor(time_ns / 1e9);
  imu.header.stamp.nanosec = time_ns - imu.header.stamp.sec * 1e9;

  imu.angular_velocity.x = wx_imu;
  imu.angular_velocity.y = wy_imu;
  imu.angular_velocity.z = wz_imu;

  imu.linear_acceleration.x = ax_imu;
  imu.linear_acceleration.y = ay_imu;
  imu.linear_acceleration.z = az_imu;

  compute_quaternion(roll, pitch, heading);

  imu.orientation.x = qx;
  imu.orientation.y = qy;
  imu.orientation.z = qz;
  imu.orientation.w = qw;

  imu.orientation_covariance[0] = -1.0;   // orientación no fiable (convenio ROS)
  imu.orientation_covariance[4] = 0.0;
  imu.orientation_covariance[8] = 0.0;

  imu.angular_velocity_covariance[0] = 0.01;
  imu.angular_velocity_covariance[4] = 0.01;
  imu.angular_velocity_covariance[8] = 0.01;

  imu.linear_acceleration_covariance[0] = 0.1;
  imu.linear_acceleration_covariance[4] = 0.1;
  imu.linear_acceleration_covariance[8] = 0.1;

  RCSOFTCHECK(rcl_publish(&publisher_imu, &imu, NULL));
}

// Periodo real del lazo de control
void publish_period(int64_t time_ns) {
  period.header.stamp.sec = floor(time_ns / 1e9);
  period.header.stamp.nanosec = time_ns - period.header.stamp.sec * 1e9;
  period.time_ref.sec = 0;
  period.time_ref.nanosec = uint32_t(dT * 1e9);

  RCSOFTCHECK(rcl_publish(&publisher_period, &period, NULL));
}

// Velocidad medida de cada rueda (rad/s)
void publish_joints(int64_t time_ns) {
  joints.header.stamp.sec = floor(time_ns / 1e9);
  joints.header.stamp.nanosec = time_ns - joints.header.stamp.sec * 1e9;

  joints.velocity.data[0] = Wd[0];
  joints.velocity.data[1] = Wd[1];
  // PWM aplicado (saturado) en [-1, 1]
  joints.effort.data[0] = constrain(u1 / float(pwmHalf), -1.0f, 1.0f);
  joints.effort.data[1] = constrain(u2 / float(pwmHalf), -1.0f, 1.0f);
#if WHEEL_NUMBER == 4
  joints.velocity.data[2] = Wd[2];
  joints.velocity.data[3] = Wd[3];
  joints.effort.data[2] = constrain(u3 / float(pwmHalf), -1.0f, 1.0f);
  joints.effort.data[3] = constrain(u4 / float(pwmHalf), -1.0f, 1.0f);
#endif

  RCSOFTCHECK(rcl_publish(&publisher_joints, &joints, NULL));
}

// Mensaje fijo con namespace, tipo y número de ruedas. Se construye una sola
// vez al conectar. No formatear texto aquí cada segundo: un snprintf con
// floats en esta tarea desbordaba la pila y provocaba reinicios por panic.
void publish_robot_info() {
  RCSOFTCHECK(rcl_publish(&publisher_robot_info, &robot_info, NULL));
}

// Se ejecuta cada 50 ms desde el timer del executor (tarea "topics")
void publish_msgs() {
  if (!ros_connected) return;

  int64_t time_ns = rmw_uros_epoch_nanos();
  if (time_ns <= 0) {
    time_ns = esp_timer_get_time() * 1000LL;
  }
  publish_period(time_ns);
  publish_imu(time_ns);
  publish_joints(time_ns);
  publish_odom(time_ns);

  robot_info_counter++;
  if (robot_info_counter >= 20) {   // 1 Hz
    publish_robot_info();
    robot_info_counter = 0;
  }
}

// ============================================================
// Callback cmd_vel
// ============================================================
void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
      (const geometry_msgs__msg__Twist *)msgin;

  bool was_stopped =
      fabsf(vx_r) < 1e-4f &&
      fabsf(vy_r) < 1e-4f &&
      fabsf(wz_r) < 1e-4f;

  bool starting =
      fabsf(msg->linear.x) > 1e-4f ||
      fabsf(msg->linear.y) > 1e-4f ||
      fabsf(msg->angular.z) > 1e-4f;

  // Al arrancar desde parado se vacían los integradores del PID
  if (was_stopped && starting) {
    reset_pid_integral();
  }

  if (pwm_mode) reset_pid_integral();
  pwm_mode = false;

  vx_r = msg->linear.x;
  vy_r = msg->linear.y;
  wz_r = msg->angular.z;

  ros_kinematics = true;
  count = 0;   // reinicia el timeout de comando
}

// ============================================================
// Callback cmd_joints (consigna directa por rueda, rad/s)
// ============================================================
void joints_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  if (msg->velocity.size < WHEEL_NUMBER || msg->velocity.data == NULL) {
    return;
  }

  if (msg->effort.size >= WHEEL_NUMBER && msg->effort.data != NULL) {
    // Modo PWM directo
    for (int i = 0; i < WHEEL_NUMBER; i++) {
      pwm_duty[i] = constrain((float)msg->effort.data[i], -1.0f, 1.0f);
      Wi[i] = 0.0f;
    }
    pwm_mode = true;
  } else {
    for (int i = 0; i < WHEEL_NUMBER; i++) {
      Wi[i] = msg->velocity.data[i];
    }
    if (pwm_mode) reset_pid_integral();
    pwm_mode = false;
  }

  ros_kinematics = false;
  count = 0;
}

// ============================================================
// Lazo de control (esp_timer, cada 10 ms)
// ============================================================
void timer_control_callback(void* arg) {
  (void)arg;

  // Primer ciclo tras terminar la calibración de la IMU
  if (!ekf_ready) {
    if (!imu_ready) {
      return;
    }

    x = 0.0f;
    y = 0.0f;
    yaw = 0.0f;
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;
    ax = 0.0f;
    ay = 0.0f;

    ekf_set_initial_state();

    // Arranque limpio del lazo de velocidad: se toman las cuentas actuales
    // como referencia. Si las ruedas se movieron mientras la IMU calibraba,
    // el primer ciclo vería un salto enorme y el PID saturaría al revés.
    read_encoders();
    count1_k1 = count1;
    count2_k1 = count2;
    count3_k1 = count3;
    count4_k1 = count4;
    pv1 = pv2 = pv3 = pv4 = 0.0f;
    e1_k1 = e2_k1 = e3_k1 = e4_k1 = 0.0f;
    reset_pid_integral();

    previousNanos = esp_timer_get_time() * 1000LL;
    ekf_ready = true;
    return;
  }

  // Periodo real con el reloj monotónico del ESP32 (el reloj sincronizado
  // con el agente puede saltar al resincronizar)
  int64_t currentNanos = esp_timer_get_time() * 1000LL;

  dT = float(currentNanos - previousNanos) * 1e-9f;
  previousNanos = currentNanos;

  if (dT <= 0.0f || dT > 0.1f || !isfinite(dT)) {
    dT = (float)period_us * 1e-6f;
  }

  // --- EKF: predicción ---
  ekf_predict_step(dT);

  // --- Velocidad de las ruedas (rpm), filtrada con un paso bajo ---
  read_encoders();

  float alpha = 0.5f;
  pv1 = pv1 - alpha * (pv1 - float(count1 - count1_k1) / dT * (60.0f / encoderPPR));
  pv2 = pv2 - alpha * (pv2 - float(count2 - count2_k1) / dT * (60.0f / encoderPPR));
#if WHEEL_NUMBER == 4
  pv3 = pv3 - alpha * (pv3 - float(count3 - count3_k1) / dT * (60.0f / encoderPPR));
  pv4 = pv4 - alpha * (pv4 - float(count4 - count4_k1) / dT * (60.0f / encoderPPR));
#endif

  count1_k1 = count1;
  count2_k1 = count2;
#if WHEEL_NUMBER == 4
  count3_k1 = count3;
  count4_k1 = count4;
#endif

  // rpm → rad/s
  Wd[0] = pv1 * 2.0f * PI / 60.0f;
  Wd[1] = pv2 * 2.0f * PI / 60.0f;
#if WHEEL_NUMBER == 4
  Wd[2] = pv3 * 2.0f * PI / 60.0f;
  Wd[3] = pv4 * 2.0f * PI / 60.0f;
#endif

  // --- EKF: corrección ---
  ekf_update_step();
  ekf_copy_to_global_state();

  // --- Timeout de comando: sin cmd_vel/cmd_joints durante 2 s → parar ---
  count++;
  if (count > countMax) {
    pwm_mode = false;
    vx_r = 0.0f;
    vy_r = 0.0f;
    wz_r = 0.0f;
    Wi[0] = 0.0f;
    Wi[1] = 0.0f;
#if WHEEL_NUMBER == 4
    Wi[2] = 0.0f;
    Wi[3] = 0.0f;
#endif
  }

#if ROBOT_TYPE != ROBOT_OMNI
  vy_r = 0.0f;   // diff y skid no pueden moverse de lado
#endif

  // --- Consigna de cada rueda ---
  if (ros_kinematics == true)
    inverse_kinematics(vx_r, vy_r, wz_r, Wi);

  sp1 = Wi[0] * 60.0f / (2.0f * PI);
  sp2 = Wi[1] * 60.0f / (2.0f * PI);
#if WHEEL_NUMBER == 4
  sp3 = Wi[2] * 60.0f / (2.0f * PI);
  sp4 = Wi[3] * 60.0f / (2.0f * PI);
#endif

  // --- Robot parado: ruedas quietas y sin consigna ---
  // Lo usan el ZUPT del EKF y la re-estimación del bias (tarea inertial).
  {
    bool stationary = true;
    for (int i = 0; i < WHEEL_NUMBER; i++) {
      if (fabsf(Wd[i]) > 0.05f || fabsf(Wi[i]) > 1e-4f) {
        stationary = false;
        break;
      }
    }
    robot_stationary = stationary;
  }

  // --- PI de velocidad de cada rueda (+ feedforward) ---
  // Anti-windup: el término integral aporta como mucho medio rango de PWM.
  const float i_max = float(pwmHalf) / Ki;

  // Rueda bloqueada: se rearma cuando su consigna vuelve a cero
  for (int i = 0; i < WHEEL_NUMBER; i++) {
    bool ref_zero = pwm_mode ? (fabsf(pwm_duty[i]) < 1e-3f) : (fabsf(Wi[i]) < FF_W_MIN);
    if (stalled[i] && ref_zero) {
      stalled[i] = false;
      stall_cnt[i] = 0;
    }
  }

  float ff[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (FF_ON) {
    for (int i = 0; i < WHEEL_NUMBER; i++) {
      float w = Wi[i];
      if (fabsf(w) > FF_W_MIN) {
        ff[i] = copysignf(FF_U0 + fabsf(w) / FF_K, w) * float(pwmHalf);
      }
    }
  }

  //-------------------- PID Motor 1 --------------------------
  e1 = sp1 - pv1;
  e1i = constrain(e1i + e1 * dT, -i_max, i_max);
  e1d = (e1 - e1_k1) / dT;
  u1 = pwm_mode ? pwm_duty[0] * float(pwmHalf)
            : ff[0] + Kp * e1 + Ki * e1i + Kd * e1d;
  u1 = stall_guard(0, u1, Wd[0]);
  if (stalled[0]) e1i = 0.0f;
  u1_k1 = u1;
  e1_k2 = e1_k1;
  e1_k1 = e1;

  // PWM complementario: pwmHalf = parado; saturación a pwmFull / 0
  if (u1 > float(pwmHalf)) {
    ledcWrite(motor1Pin1, pwmFull);
    ledcWrite(motor1Pin2, 0);
  } else if (u1 < -float(pwmHalf)) {
    ledcWrite(motor1Pin1, 0);
    ledcWrite(motor1Pin2, pwmFull);
  } else {
    ledcWrite(motor1Pin1, pwmHalf + int32_t(u1));
    ledcWrite(motor1Pin2, pwmHalf - int32_t(u1));
  }

  //-------------------- PID Motor 2 --------------------------
  e2 = sp2 - pv2;
  e2i = constrain(e2i + e2 * dT, -i_max, i_max);
  e2d = (e2 - e2_k1) / dT;
  u2 = pwm_mode ? pwm_duty[1] * float(pwmHalf)
            : ff[1] + Kp * e2 + Ki * e2i + Kd * e2d;
  u2 = stall_guard(1, u2, Wd[1]);
  if (stalled[1]) e2i = 0.0f;
  u2_k1 = u2;
  e2_k2 = e2_k1;
  e2_k1 = e2;

  if (u2 > float(pwmHalf)) {
    ledcWrite(motor2Pin1, pwmFull);
    ledcWrite(motor2Pin2, 0);
  } else if (u2 < -float(pwmHalf)) {
    ledcWrite(motor2Pin1, 0);
    ledcWrite(motor2Pin2, pwmFull);
  } else {
    ledcWrite(motor2Pin1, pwmHalf + int32_t(u2));
    ledcWrite(motor2Pin2, pwmHalf - int32_t(u2));
  }

#if WHEEL_NUMBER == 4
  //-------------------- PID Motor 3 --------------------------
  e3 = sp3 - pv3;
  e3i = constrain(e3i + e3 * dT, -i_max, i_max);
  e3d = (e3 - e3_k1) / dT;
  u3 = pwm_mode ? pwm_duty[2] * float(pwmHalf)
            : ff[2] + Kp * e3 + Ki * e3i + Kd * e3d;
  u3 = stall_guard(2, u3, Wd[2]);
  if (stalled[2]) e3i = 0.0f;
  u3_k1 = u3;
  e3_k2 = e3_k1;
  e3_k1 = e3;

  if (u3 > float(pwmHalf)) {
    ledcWrite(motor3Pin1, pwmFull);
    ledcWrite(motor3Pin2, 0);
  } else if (u3 < -float(pwmHalf)) {
    ledcWrite(motor3Pin1, 0);
    ledcWrite(motor3Pin2, pwmFull);
  } else {
    ledcWrite(motor3Pin1, pwmHalf + int32_t(u3));
    ledcWrite(motor3Pin2, pwmHalf - int32_t(u3));
  }

  //-------------------- PID Motor 4 --------------------------
  e4 = sp4 - pv4;
  e4i = constrain(e4i + e4 * dT, -i_max, i_max);
  e4d = (e4 - e4_k1) / dT;
  u4 = pwm_mode ? pwm_duty[3] * float(pwmHalf)
            : ff[3] + Kp * e4 + Ki * e4i + Kd * e4d;
  u4 = stall_guard(3, u4, Wd[3]);
  if (stalled[3]) e4i = 0.0f;
  u4_k1 = u4;
  e4_k2 = e4_k1;
  e4_k1 = e4;

  if (u4 > float(pwmHalf)) {
    ledcWrite(motor4Pin1, pwmFull);
    ledcWrite(motor4Pin2, 0);
  } else if (u4 < -float(pwmHalf)) {
    ledcWrite(motor4Pin1, 0);
    ledcWrite(motor4Pin2, pwmFull);
  } else {
    ledcWrite(motor4Pin1, pwmHalf + int32_t(u4));
    ledcWrite(motor4Pin2, pwmHalf - int32_t(u4));
  }
#endif

  // En modo PWM el PID no actúa: integradores a cero para volver sin saltos
  if (pwm_mode) reset_pid_integral();
}

// ============================================================
// Timer de publicación (executor de micro-ROS, cada 50 ms)
// ============================================================
void timer_publish_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  publish_msgs();
}

// ============================================================
// micro-ROS: creación de entidades
// ============================================================
bool create_entities()
{
  ros_connected = true;

  allocator_topics = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support_topics, 0, NULL, &allocator_topics));
  RCCHECK(rclc_node_init_default(&node_topics, "uvot_base", ns.c_str(), &support_topics));

  // Sincroniza el reloj con el agente (para los stamps de los mensajes)
  RCSOFTCHECK(rmw_uros_sync_session(1000));

  previousNanos = esp_timer_get_time() * 1000LL;

  // --- Publicadores (QoS fiable) ---
  topic_name = "period";
  RCCHECK(rclc_publisher_init_default(
    &publisher_period,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, TimeReference),
    topic_name.c_str()));

  topic_name = "odom";
  RCCHECK(rclc_publisher_init_default(
    &publisher_odom,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    topic_name.c_str()));

  topic_name = "imu";
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    topic_name.c_str()));

  topic_name = "joint_states";
  RCCHECK(rclc_publisher_init_default(
    &publisher_joints,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    topic_name.c_str()));

  topic_name = "robot_info";
  RCCHECK(rclc_publisher_init_default(
    &publisher_robot_info,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    topic_name.c_str()));

  // Publicación fiable sin bloquear. Por defecto rcl_publish espera hasta
  // 1000 ms la confirmación del agente; con tráfico entrante (cmd_vel) a veces
  // la espera se agotaba y todos los topics se paraban ~1 s. Con 5 ms la
  // publicación vuelve enseguida y la sesión sigue gestionando los reenvíos.
  // Se mantienen fiables porque un suscriptor fiable (robot_state_publisher,
  // Nav2, scripts) no recibe de un publicador best effort.
  const int PUB_TIMEOUT_MS = 5;
  RCSOFTCHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher_period),     PUB_TIMEOUT_MS));
  RCSOFTCHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher_odom),       PUB_TIMEOUT_MS));
  RCSOFTCHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher_imu),        PUB_TIMEOUT_MS));
  RCSOFTCHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher_joints),     PUB_TIMEOUT_MS));
  RCSOFTCHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher_robot_info), PUB_TIMEOUT_MS));

  std::string info =
    std::string("namespace=") + ns +
    ";type=" + ROBOT_TYPE_NAME +
    ";wheels=" + std::to_string(WHEEL_NUMBER);

  robot_info.data = micro_ros_string_utilities_set(robot_info.data, info.c_str());

  // --- Suscripciones (best effort: perder un comando suelto a 20 Hz no
  //     importa, y un publicador fiable se conecta sin problema) ---
  topic_name = "cmd_vel";
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber_twist,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    topic_name.c_str()));

  topic_name = "cmd_joints";
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber_joints,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    topic_name.c_str()));

  // --- Timer de publicación a 20 Hz y executor ---
  const unsigned int timer_publish_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer_publish,
    &support_topics,
    RCL_MS_TO_NS(timer_publish_timeout),
    timer_publish_callback));

  RCCHECK(rclc_executor_init(&executor_topics, &support_topics.context, 3, &allocator_topics));
  RCCHECK(rclc_executor_add_timer(&executor_topics, &timer_publish));
  RCCHECK(rclc_executor_add_subscription(&executor_topics, &subscriber_twist, &cmd_vel, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_topics, &subscriber_joints, &cmd_joints, &joints_callback, ON_NEW_DATA));

  return true;
}

// ============================================================
// Tarea "topics": conexión con el agente y executor
// ============================================================
void topics(void *pvParameters)
{
  (void)pvParameters;

  AgentState state = WAITING_AGENT;

  while (1)
  {
    switch (state)
    {
      case WAITING_AGENT:
        if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK)
          state = AGENT_AVAILABLE;
        break;

      case AGENT_AVAILABLE:
        if (create_entities())
          state = AGENT_CONNECTED;
        else
          state = WAITING_AGENT;
        break;

      case AGENT_CONNECTED:
      {
        // Comprobación de la conexión: un ping por segundo y reinicio solo
        // tras 3 fallos seguidos (un paquete perdido no reinicia el robot).
        static uint32_t last_ping_ms = 0;
        static uint8_t  consecutive_fail = 0;

        uint32_t now_ms = millis();
        if (now_ms - last_ping_ms >= 1000) {
          last_ping_ms = now_ms;
          if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            consecutive_fail++;
          } else {
            consecutive_fail = 0;
          }
        }

        if (consecutive_fail >= 3) {
          ros_connected = false;
          esp_restart();
        }

        rclc_executor_spin_some(&executor_topics, RCL_MS_TO_NS(10));
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// Tarea "inertial": IMU a 100 Hz
// ============================================================
void inertial(void *pvParameters) {
  (void)pvParameters;

  init_imu();

  // Estabilización inicial (1 s)
  for (int i = 0; i < 100; i++) {
    read_imu();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Calibración del bias del giróscopo z: mediana de 300 muestras (3 s),
  // descartando picos. La mediana no se desplaza por unos pocos valores
  // corruptos, a diferencia de la media. El robot debe estar quieto.
  // Si se descartan más de la mitad de las muestras, se repite (hasta 3 veces).
  const int   N_CAL     = 300;
  const float CAL_LIMIT = 0.35f;     // rad/s (~20 °/s): por encima es un pico
  static float cal_buf[N_CAL];       // static: no ocupa pila de la tarea

  float bias = 0.0f;
  for (int attempt = 0; attempt < 3; attempt++) {
    int n = 0;
    for (int i = 0; i < N_CAL; i++) {
      // Aún sin compensar: wz_imu = lectura cruda
      if (read_imu() && isfinite(wz_imu) && fabsf(wz_imu) < CAL_LIMIT) {
        cal_buf[n++] = wz_imu;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (n > N_CAL / 2) {
      std::sort(cal_buf, cal_buf + n);
      bias = cal_buf[n / 2];
      break;
    }
  }

  wz_bias = bias;
  gyro_bias_ready = true;   // a partir de aquí se compensa el bias
  read_imu();
  imu_ready = true;         // permite arrancar el EKF

  // Funcionamiento normal + ZARU (re-estimación del bias en parado).
  // Tras 1 s con el robot quieto, el bias se corrige poco a poco hacia la
  // lectura actual (constante de tiempo ~1 s). Así se compensa la deriva
  // del bias con la temperatura o una calibración inicial imperfecta.
  uint16_t still_count = 0;
  while (1) {
    bool ok = read_imu();

    if (ok && robot_stationary) {
      if (still_count < 1000) still_count++;
    } else {
      still_count = 0;
    }

    if (still_count >= 100 && isfinite(wz_imu) && fabsf(wz_imu) < 1.0f) {
      wz_bias = wz_bias + 0.01f * wz_imu;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  // Encoders
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
#if WHEEL_NUMBER == 4
  pinMode(encoder3A, INPUT);
  pinMode(encoder3B, INPUT);
  pinMode(encoder4A, INPUT);
  pinMode(encoder4B, INPUT);
#endif

  // Motores
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
#if WHEEL_NUMBER == 4
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
#endif

  // PWM
  ledcAttach(motor1Pin1, pwmFreq, pwmBits);
  ledcAttach(motor1Pin2, pwmFreq, pwmBits);
  ledcAttach(motor2Pin1, pwmFreq, pwmBits);
  ledcAttach(motor2Pin2, pwmFreq, pwmBits);
#if WHEEL_NUMBER == 4
  ledcAttach(motor3Pin1, pwmFreq, pwmBits);
  ledcAttach(motor3Pin2, pwmFreq, pwmBits);
  ledcAttach(motor4Pin1, pwmFreq, pwmBits);
  ledcAttach(motor4Pin2, pwmFreq, pwmBits);
#endif

  pwmFull = 1023;
  pwmHalf = 511;

  Ki = Kp / Ti;
  Kd = Kp * Td;

  count = 0;

  x = 0.0f; y = 0.0f; z = 0.0f;
  vx = 0.0f; vy = 0.0f; wz = 0.0f;
  ax = 0.0f; ay = 0.0f;
  roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
  oyaw = 0.0f; owz = 0.0f; oax = 0.0f; oay = 0.0f;
  vx_r = 0.0f; vy_r = 0.0f; wz_r = 0.0f;

  ekf_set_initial_state();
  init_encoders();

  // Frames con namespace
  sprintf(frame_id_ns, "%s/%s", ns.c_str(), frame_id);
  odom.header.frame_id.data = frame_id_ns;
  odom.header.frame_id.capacity = 30;

  sprintf(child_frame_id_ns, "%s/%s", ns.c_str(), child_frame_id);
  odom.child_frame_id.data = child_frame_id_ns;
  odom.child_frame_id.capacity = 30;

  sprintf(imu_frame_id_ns, "%s/%s", ns.c_str(), imu_frame_id);
  imu.header.frame_id.data = imu_frame_id_ns;
  imu.header.frame_id.capacity = 30;

  // Memoria de los mensajes JointState (publicado y recibido)
  static micro_ros_utilities_memory_conf_t conf = {0};

  micro_ros_utilities_memory_rule_t joint_state_rules[] = {
    {"name", WHEEL_NUMBER},
    {"position", WHEEL_NUMBER},
    {"velocity", WHEEL_NUMBER},
    {"effort", WHEEL_NUMBER}
  };

  conf.rules = joint_state_rules;

  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    &joints,
    conf);

  micro_ros_string_utilities_set(joints.header.frame_id, child_frame_id_ns);

  joints.name.size = WHEEL_NUMBER;
  joints.name.data[0] = micro_ros_string_utilities_set(joints.name.data[0], "wheel1_joint");
  joints.name.data[1] = micro_ros_string_utilities_set(joints.name.data[1], "wheel2_joint");
#if WHEEL_NUMBER == 4
  joints.name.data[2] = micro_ros_string_utilities_set(joints.name.data[2], "wheel3_joint");
  joints.name.data[3] = micro_ros_string_utilities_set(joints.name.data[3], "wheel4_joint");
#endif

  joints.position.size = WHEEL_NUMBER;
  joints.velocity.size = WHEEL_NUMBER;
  joints.effort.size = WHEEL_NUMBER;

  micro_ros_utilities_memory_rule_t cmd_joint_state_rules[] = {
    {"name", WHEEL_NUMBER},
    {"position", WHEEL_NUMBER},
    {"velocity", WHEEL_NUMBER},
    {"effort", WHEEL_NUMBER}   // modo PWM directo
  };

  micro_ros_utilities_memory_conf_t cmd_conf = {0};
  cmd_conf.rules = cmd_joint_state_rules;

  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    &cmd_joints,
    cmd_conf
  );

  cmd_joints.name.size = WHEEL_NUMBER;
  cmd_joints.position.size = WHEEL_NUMBER;
  cmd_joints.velocity.size = WHEEL_NUMBER;
  cmd_joints.effort.size = 0;

  // Transporte serie de micro-ROS (el buffer RX se fija antes de begin)
  Serial.setRxBufferSize(2048);
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(200);

  previousNanos = esp_timer_get_time() * 1000LL;

  // Lazo de control cada 10 ms
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_control_callback,
    .arg = NULL,
    .name = "control_timer"
  };

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_control_handle));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer_control_handle, period_us));

  countMax = countMax * 10000 / period_us;

  // Tareas: micro-ROS en el núcleo 0 e IMU en el núcleo 1
  xTaskCreatePinnedToCore(topics, "topics", 8192, NULL, 1, NULL, 0);
  delay(500);

  xTaskCreatePinnedToCore(inertial, "inertial", 6144, NULL, 2, NULL, 1);
  delay(500);
}

void loop() {
  // Todo se gestiona en timers y tareas FreeRTOS
}
