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
// Pines motor 1
const int encoder1A  = 15;
const int encoder1B  =  4;
const int motor1Pin1 = 13;
const int motor1Pin2 = 27;

// Pines motor 2
const int encoder2A  = 17;
const int encoder2B  = 16;
const int motor2Pin1 = 26;
const int motor2Pin2 = 25;

// Pines motor 3
const int encoder3A  =  5;
const int encoder3B  = 18;
const int motor3Pin1 = 19;
const int motor3Pin2 = 23;

// Pines motor 4
const int encoder4A  = 34;
const int encoder4B  = 35;
const int motor4Pin1 = 32;
const int motor4Pin2 = 33;

// ============================================================
// Encoders
// ============================================================
ESP32Encoder encoder1, encoder2, encoder3, encoder4;
uint16_t encoderPPR = 274;  // luego se multiplica por 4

int64_t previousNanos = 0;

// Variables velocidad ruedas
float pv1 = 0, pv2 = 0, pv3 = 0, pv4 = 0;
float sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;

// PID
float e1 = 0, e2 = 0, e3 = 0, e4 = 0;
float e1_k1 = 0, e2_k1 = 0, e3_k1 = 0, e4_k1 = 0;
float e1_k2 = 0, e2_k2 = 0, e3_k2 = 0, e4_k2 = 0;
float e1i = 0, e2i = 0, e3i = 0, e4i = 0;
float e1d = 0, e2d = 0, e3d = 0, e4d = 0;
float u1 = 0, u2 = 0, u3 = 0, u4 = 0;
float u1_k1 = 0, u2_k1 = 0, u3_k1 = 0, u4_k1 = 0;

// PWM
const int pwmFreq = 30000;
const int pwmChannel = 0;
const int pwmBits = 10;
uint32_t pwmFull, pwmHalf;

// Contadores
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;
volatile long count1_k1 = 0;
volatile long count2_k1 = 0;
volatile long count3_k1 = 0;
volatile long count4_k1 = 0;

// Geometría
const float a = 0.08f; //0.1f;
const float b = 0.11f; //0.1f;

#if ROBOT_TYPE == ROBOT_OMNI
const float r = 0.04f;
#else
const float r = 0.035f;
#endif

unsigned int period_us = 10000;
float fc = 1000.0f;

// PID gains
float Kp = 2.0f;
float Ti = 0.1f;
float Td = 0.001f;
float Ki;
float Kd;

// Estado global
float x, y, z;
float vx, vy, wz;
float ax, ay;

// IMU
float wx_imu, wy_imu, wz_imu;
float ax_imu, ay_imu, az_imu;
float mx_imu, my_imu, mz_imu;

// Offsets
float oyaw, owz, oax, oay;

// Velocidades ruedas / robot
float Wd[WHEEL_NUMBER];
float Wi[WHEEL_NUMBER];
float Vd[3];

float dT = 0.01f;
uint32_t count = 0;
uint32_t countMax = 50;

// Frames
char frame_id[30] = "odom";
char child_frame_id[30] = "base_footprint";
char imu_frame_id[30] = "base_link";
char frame_id_ns[30];
char child_frame_id_ns[30];
char imu_frame_id_ns[30];

// Referencias cmd_vel
float vx_r, vy_r, wz_r;

// IMU vars
xyzFloat gValue;
xyzFloat gyr;
xyzFloat magValue;
float temp;
float resultantG;

// Orientación
float qw, qx, qy, qz;
float roll, pitch, yaw;
float heading, heading0;

// Global flag to check ROS connection
volatile bool ros_connected = false;
volatile bool ros_kinematics = true;

// ============================================================
// EKF por plataforma
// ============================================================

#if ROBOT_TYPE == ROBOT_DIFF

const float Q_ekf[6][6] = {
  {1e-6f, 0,     0,     0,     0,     0},
  {0,     1e-6f, 0,     0,     0,     0},
  {0,     0,     1e-6f, 0,     0,     0},
  {0,     0,     0,     1e-4f, 0,     0},
  {0,     0,     0,     0,     1e-4f, 0},
  {0,     0,     0,     0,     0,     1e-3f}
};

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

DiffDriveEKF ekf((float)period_us * 1e-6f, r, b, Q_ekf, R_ekf, x0_ekf, P0_ekf);

#elif ROBOT_TYPE == ROBOT_SKID

const float Q_ekf[6][6] = {
  {1e-6f, 0,     0,     0,     0,     0},
  {0,     1e-6f, 0,     0,     0,     0},
  {0,     0,     1e-6f, 0,     0,     0},
  {0,     0,     0,     1e-4f, 0,     0},
  {0,     0,     0,     0,     2e-4f, 0},
  {0,     0,     0,     0,     0,     2e-3f}
};

const float R_ekf[5][5] = {
  {0.0050f, 0,        0,        0,        0},
  {0,       0.0050f,  0,        0,        0},
  {0,       0,        0.0100f,  0,        0},
  {0,       0,        0,        0.2000f,  0},
  {0,       0,        0,        0,        5.0e-5f}
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

SkidSteerEKF ekf((float)period_us * 1e-6f, r, a + b, Q_ekf, R_ekf, x0_ekf, P0_ekf);

#elif ROBOT_TYPE == ROBOT_OMNI

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

const float R_ekf[7][7] = {
  {0.0025f, 0,        0,        0,        0,        0,        0},
  {0,       0.0025f,  0,        0,        0,        0,        0},
  {0,       0,        0.0025f,  0,        0,        0,        0},
  {0,       0,        0,        0.0025f,  0,        0,        0},
  {0,       0,        0,        0,        1.0e-3f,  0,        0},
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
// ROS objects
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
// Helpers
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

void direct_kinematics(float* W, float* V) {
#if ROBOT_TYPE == ROBOT_OMNI
  V[0] = (W[0] + W[1] + W[2] + W[3]) * r / 4.0f;
  V[1] = (W[0] - W[1] + W[2] - W[3]) * r / 4.0f;
  V[2] = (-W[0] + W[1] + W[2] - W[3]) * r / (4.0f * (a + b));
#elif ROBOT_TYPE == ROBOT_SKID
  V[0] = (W[0] + W[1] + W[2] + W[3]) * r / 4.0f;
  V[1] = 0.0f;
  V[2] = (W[1] + W[2] - W[3] - W[0]) * r / (4.0f * (a + b));
#else
  V[0] = (W[0] + W[1]) * r / 2.0f;
  V[1] = 0.0f;
  V[2] = (W[1] - W[0]) * r / (2.0f * b);
#endif
}

void inverse_kinematics(float vx_cmd, float vy_cmd, float wz_cmd, float* W) {
#if ROBOT_TYPE == ROBOT_OMNI
  W[0] = (vx_cmd + vy_cmd - (a + b) * wz_cmd) / r;
  W[1] = (vx_cmd - vy_cmd + (a + b) * wz_cmd) / r;
  W[2] = (vx_cmd + vy_cmd + (a + b) * wz_cmd) / r;
  W[3] = (vx_cmd - vy_cmd - (a + b) * wz_cmd) / r;
#elif ROBOT_TYPE == ROBOT_SKID
  (void)vy_cmd;
  W[0] = (vx_cmd - (a + b) * wz_cmd) / r;
  W[1] = (vx_cmd + (a + b) * wz_cmd) / r;
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

  vx = vx_body * cosf(th) - vy_body * sinf(th);
  vy = vx_body * sinf(th) + vy_body * cosf(th);
  wz = wz_est;

  ax = ax_body * cosf(th) - ay_body * sinf(th);
  ay = ax_body * sinf(th) + ay_body * cosf(th);
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

  vx = v_est * cosf(th);
  vy = v_est * sinf(th);
  wz = w_est;

  ax = a_est * cosf(th);
  ay = a_est * sinf(th);
#endif
}

void ekf_predict_step(float dt_now) {
  ekf.dt = dt_now;
  ekf.predict();
}

void ekf_update_step() {
#if ROBOT_TYPE == ROBOT_OMNI
  // float z_meas[7] = {
  //   Wd[0], Wd[1], Wd[2], Wd[3], 0.0, 0.0, 0.0
  // };
  float z_meas[7] = {
    Wd[0], Wd[1], Wd[2], Wd[3], wz_imu, ax_imu, ay_imu
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
  float omegaL = 0.5f * (Wd[0] + Wd[3]);
  float omegaR = 0.5f * (Wd[1] + Wd[2]);

  float z_meas[5] = {
    omegaR, omegaL, ax_imu, ay_imu, wz_imu
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
  float z_meas[5] = {
    Wd[0], Wd[1], ax_imu, ay_imu, wz_imu
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
void init_imu(void) {
  Wire.begin();
  myMPU9250.init();
  myMPU9250.initMagnetometer();
  myMPU9250.autoOffsets();
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);

  heading0 = 0.0f;
}

void read_imu(void) {
  gValue     = myMPU9250.getGValues();
  gyr        = myMPU9250.getGyrValues();
  magValue   = myMPU9250.getMagValues();
  temp       = myMPU9250.getTemperature();
  resultantG = myMPU9250.getResultantG(gValue);

  ax_imu = ( gValue.x) * 9.81f;
  ay_imu = (-gValue.y) * 9.81f;
  az_imu = ( gValue.z - 1.0f) * 9.81f;

  wx_imu =  gyr.x * PI / 180.0f;
  wy_imu = -gyr.y * PI / 180.0f;
  wz_imu = -gyr.z * PI / 180.0f;

  mx_imu = magValue.x;
  my_imu = magValue.y;
  mz_imu = magValue.z;

  heading = compute_compass_heading() - heading0;
}

// ============================================================
// Encoders
// ============================================================
void init_encoders(void) {
  encoderPPR = encoderPPR * 4;

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
// Publishers
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
  odom.twist.covariance[7] = 1e3;
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

  imu.orientation_covariance[0] = -1.0;
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

void publish_period(int64_t time_ns) {
  period.header.stamp.sec = floor(time_ns / 1e9);
  period.header.stamp.nanosec = time_ns - period.header.stamp.sec * 1e9;
  period.time_ref.sec = 0;
  period.time_ref.nanosec = uint32_t(dT * 1e9);

  RCSOFTCHECK(rcl_publish(&publisher_period, &period, NULL));
}

void publish_joints(int64_t time_ns) {
  joints.header.stamp.sec = floor(time_ns / 1e9);
  joints.header.stamp.nanosec = time_ns - joints.header.stamp.sec * 1e9;
  
  joints.velocity.data[0] = Wd[0];
  joints.velocity.data[1] = Wd[1];
#if WHEEL_NUMBER == 4
  joints.velocity.data[2] = Wd[2];
  joints.velocity.data[3] = Wd[3];
#endif

  RCSOFTCHECK(rcl_publish(&publisher_joints, &joints, NULL));
}

void publish_robot_info() {
  RCSOFTCHECK(
    rcl_publish(
      &publisher_robot_info,
      &robot_info,
      NULL
    )
  );
}

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
  if (robot_info_counter >= 20) {
    publish_robot_info();
    robot_info_counter = 0;
  }
}

// ============================================================
// Callback cmd_vel
// ============================================================
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  vx_r = msg->linear.x;
  vy_r = msg->linear.y;
  wz_r = msg->angular.z;
  ros_kinematics = true;
  count = 0;
}

// ============================================================
// Callback cmd_joints
// ============================================================
void joints_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  if (msg->velocity.size < WHEEL_NUMBER || msg->velocity.data == NULL) {
    return;
  }

  for (int i = 0; i < WHEEL_NUMBER; i++) {
    Wi[i] = msg->velocity.data[i];
  }

  ros_kinematics = false;
  count = 0;
}

// ============================================================
// Control loop
// ============================================================
void timer_control_callback(void* arg) {
  (void)arg;
  static bool ekf_ready = false;

  if (!ekf_ready) {
    bool ok = isfinite(ax_imu) && isfinite(ay_imu) && isfinite(wz_imu);
    if (!ok) return;
    ekf_ready = true;
  }

  int64_t currentNanos = rmw_uros_epoch_nanos();
  if (currentNanos <= 0) {
      currentNanos = esp_timer_get_time() * 1000LL;
  }

  dT = float(currentNanos - previousNanos) * 1e-9f;
  previousNanos = currentNanos;

  if (dT <= 0.0f || dT > 0.1f || !isfinite(dT)) {
    dT = (float)period_us * 1e-6f;
  }

  ekf_predict_step(dT);

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

  Wd[0] = pv1 * 2.0f * PI / 60.0f;
  Wd[1] = pv2 * 2.0f * PI / 60.0f;
#if WHEEL_NUMBER == 4
  Wd[2] = pv3 * 2.0f * PI / 60.0f;
  Wd[3] = pv4 * 2.0f * PI / 60.0f;
#endif

  ekf_update_step();

#if ROBOT_TYPE == ROBOT_OMNI
  for (int i = 0; i < 8; i++) {
    if (!isfinite(ekf.x[i])) ekf.x[i] = 0.0f;
  }
#else
  for (int i = 0; i < 6; i++) {
    if (!isfinite(ekf.x[i])) ekf.x[i] = 0.0f;
  }
#endif

  ekf_copy_to_global_state();

  count++;
  if (count > countMax) {
    vx_r = 0.0f;
    vy_r = 0.0f;
    wz_r = 0.0f;
    Wi[0] = 0.0f;
    Wi[1] = 0.0f;
    Wi[2] = 0.0f;
    Wi[3] = 0.0f;
  }

#if ROBOT_TYPE != ROBOT_OMNI
  vy_r = 0.0f;
#endif

  if(ros_kinematics == true)
    inverse_kinematics(vx_r, vy_r, wz_r, Wi);

  sp1 = Wi[0] * 60.0f / (2.0f * PI);
  sp2 = Wi[1] * 60.0f / (2.0f * PI);
#if WHEEL_NUMBER == 4
  sp3 = Wi[2] * 60.0f / (2.0f * PI);
  sp4 = Wi[3] * 60.0f / (2.0f * PI);
#endif

  //-------------------- PID Motor 1 --------------------------
  e1 = sp1 - pv1;
  e1i = e1i + e1 * dT;
  e1d = (e1 - e1_k1) / dT;
  u1 = Kp * e1 + Ki * e1i + Kd * e1d;
  u1_k1 = u1;
  e1_k2 = e1_k1;
  e1_k1 = e1;

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
  e2i = e2i + e2 * dT;
  e2d = (e2 - e2_k1) / dT;
  u2 = Kp * e2 + Ki * e2i + Kd * e2d;
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
  e3i = e3i + e3 * dT;
  e3d = (e3 - e3_k1) / dT;
  u3 = Kp * e3 + Ki * e3i + Kd * e3d;
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
  e4i = e4i + e4 * dT;
  e4d = (e4 - e4_k1) / dT;
  u4 = Kp * e4 + Ki * e4i + Kd * e4d;
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
}

// ============================================================
// Publish timer
// ============================================================
void timer_publish_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  publish_msgs();
}

// ============================================================
// Tasks
// ============================================================

bool create_entities()
{
  ros_connected = true;  
  
  allocator_topics = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support_topics, 0, NULL, &allocator_topics));
  RCCHECK(rclc_node_init_default(&node_topics, "uvot_base", ns.c_str(), &support_topics));

  RCSOFTCHECK(rmw_uros_sync_session(1000));
  previousNanos = rmw_uros_epoch_nanos();
  if (previousNanos <= 0)
      previousNanos = esp_timer_get_time() * 1000LL;

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

  std::string info =
    std::string("namespace=") + ns +
    ";type=" + ROBOT_TYPE_NAME +
    ";wheels=" + std::to_string(WHEEL_NUMBER);

  robot_info.data = micro_ros_string_utilities_set(robot_info.data,info.c_str());

  topic_name = "cmd_vel";
  RCCHECK(rclc_subscription_init_default(
    &subscriber_twist,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    topic_name.c_str()));

  topic_name = "cmd_joints";
  RCCHECK(rclc_subscription_init_default(
    &subscriber_joints,
    &node_topics,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    topic_name.c_str()));
  

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
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
                {
                    ros_connected = false;

                    esp_restart();
                }
                else
                {
                    rclc_executor_spin_some(
                        &executor_topics,
                        RCL_MS_TO_NS(10));
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void inertial(void *pvParameters) {
  (void)pvParameters;
  init_imu();

  while (1) {
    read_imu();
    vTaskDelay(10 / portTICK_PERIOD_MS);
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

  sprintf(frame_id_ns, "%s/%s", ns.c_str(), frame_id);
  odom.header.frame_id.data = frame_id_ns;
  odom.header.frame_id.capacity = 30;

  sprintf(child_frame_id_ns, "%s/%s", ns.c_str(), child_frame_id);
  odom.child_frame_id.data = child_frame_id_ns;
  odom.child_frame_id.capacity = 30;

  sprintf(imu_frame_id_ns, "%s/%s", ns.c_str(), imu_frame_id);
  imu.header.frame_id.data = imu_frame_id_ns;
  imu.header.frame_id.capacity = 30;

  static micro_ros_utilities_memory_conf_t conf = {0};

  micro_ros_utilities_memory_rule_t joint_state_rules[] = {
    {"name", WHEEL_NUMBER},
    {"position", WHEEL_NUMBER},
    {"velocity", WHEEL_NUMBER},
    {"effort", 0}
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
  joints.effort.size = 0;

  micro_ros_utilities_memory_rule_t cmd_joint_state_rules[] = {
    {"name", WHEEL_NUMBER},
    {"position", WHEEL_NUMBER},
    {"velocity", WHEEL_NUMBER},
    {"effort", 0}
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

  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(200);

  previousNanos = rmw_uros_epoch_nanos();
  if (previousNanos <= 0) previousNanos = esp_timer_get_time() * 1000LL;

  const esp_timer_create_args_t timer_args = {
    .callback = &timer_control_callback,
    .arg = NULL,
    .name = "control_timer"
  };

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_control_handle));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer_control_handle, period_us));

  countMax = countMax * 10000 / period_us;

  xTaskCreatePinnedToCore(topics, "topics", 4000, NULL, 1, NULL, 0);
  delay(500);

  xTaskCreatePinnedToCore(inertial, "inertial", 4000, NULL, 2, NULL, 1);
  delay(500);
}

void loop() {
  // Todo se gestiona en timers y tareas FreeRTOS
}