#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"

// ================= ULTRASONIC SENSOR =================
#define echoPin 16
#define trigPin 17

// ================= MOTOR & SERVO =====================
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25
#define IS_L 26
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12

// ================= GLOBAL PARAMETERS =================
const uint16_t MIN_PULSE_WIDTH = 500;
const uint16_t MAX_PULSE_WIDTH = 2500;
const uint16_t REFRESH_INTERVAL = 20000;
const uint8_t SERVO_MIN_ANGLE = 40;
const uint8_t SERVO_MAX_ANGLE = 130;
const uint16_t CURRENT_LIMIT_MV = 7000;
const uint8_t PULSES_PER_REV = 4;
volatile uint16_t pulseCount = 0;
volatile float currentRPM = 0.0f;
const uint16_t RPM_CALC_INTERVAL_MS = 600;
const uint32_t RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile uint32_t lastRpmCalcTime = 0;
volatile uint16_t lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;
const float EMA_ALPHA = 0.4f;
float g_filteredRPM = 0.0f;
float RPM_setpoint = 0.0f;
float Kp = 0.98f;
float Ki = 0.6f;
float Kd = 0.05f;
const uint8_t PID_INTERVAL_MS = 10;
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0f;
float previousErrorRPM = 0.0f;
uint32_t lastPIDRunTime = 0;
uint8_t currentPWM = 0;
const float RPM_TO_PWM_SLOPE = 0.05f;
const float RPM_TO_PWM_OFFSET = 4.6f;
const uint16_t MAX_RPM_ESTIMATE = 5000;
bool motorRunning = false;
const uint8_t SERIAL_BUFFER_SIZE = 10;
byte serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialBufferIndex = 0;

int32_t steering_angle = 90;
int32_t received_speed = 0;

// ================== ULTRASONIC FUNCTION ==================
float getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int64_t startTime = esp_timer_get_time();
  while (digitalRead(echoPin) == LOW)
  {
    if ((esp_timer_get_time() - startTime) > 30000) return -1;
  }

  int64_t echoStart = esp_timer_get_time();
  while (digitalRead(echoPin) == HIGH)
  {
    if ((esp_timer_get_time() - echoStart) > 30000) return -1;
  }

  int64_t echoEnd = esp_timer_get_time();
  float duration = (float)(echoEnd - echoStart);
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

// ================== SMOOTH FILTER ==================
float smoothDistance(float newVal)
{
  const int N = 5;
  static float buffer[N];
  static int idx = 0;
  static bool filled = false;
  buffer[idx] = newVal;
  idx = (idx + 1) % N;
  float sum = 0;
  int count = filled ? N : idx;
  for (int i = 0; i < count; i++) sum += buffer[i];
  if (idx == 0) filled = true;
  return (count > 0) ? (sum / count) : newVal;
}

// ==================== INTERRUPT =====================

void IRAM_ATTR rpm_timer_callback(void *arg)
{
  uint32_t currentTime_us = micros();
  uint16_t currentPulseReading;
  noInterrupts();
  currentPulseReading = pulseCount;
  interrupts();

  uint32_t deltaTime_us = currentTime_us - lastRpmCalcTime;
  uint16_t deltaPulses = currentPulseReading - lastRpmCalcPulseCount;
  float calculatedRPM = 0.0f;

  if (deltaTime_us > 0 && deltaPulses > 0)
  {
    float pulses_per_second = (float)deltaPulses * 1000000.0f / (float)deltaTime_us;
    float rps = pulses_per_second / (float)PULSES_PER_REV;
    calculatedRPM = rps * 60.0f;
  }
  else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2)
  {
    if (abs(RPM_setpoint) < 0.1f) calculatedRPM = 0.0f;
  }

  noInterrupts();
  currentRPM = calculatedRPM;
  interrupts();
  lastRpmCalcTime = currentTime_us;
  lastRpmCalcPulseCount = currentPulseReading;
}

// ================== MOTOR CONTROL ==================
void setMotorPWM(int pwmValue)
{
  pwmValue = constrain(pwmValue, 0, 255);
  currentPWM = pwmValue;
  uint16_t raw_IS_R = analogRead(IS_R);
  float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

  if (voltage_IS_R_mV < CURRENT_LIMIT_MV)
  {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    analogWrite(RPWM, pwmValue);
    digitalWrite(LPWM, LOW);
    motorRunning = (pwmValue > 0);
  }
  else
  {
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(RPWM, 0);
    digitalWrite(LPWM, LOW);
    motorRunning = false;
    integral = 0;
    currentPWM = 0;
  }
}

// ================== PID & FILTER ==================
float updateEMA(float val)
{
  static float prev = 0;
  static bool init = false;
  if (!init)
  {
    prev = val;
    init = true;
    return prev;
  }
  float ema = EMA_ALPHA * val + (1.0 - EMA_ALPHA) * prev;
  prev = ema;
  return ema;
}

float calculatePID_RPM_Output(float setpoint, float measured)
{
  float error = setpoint - measured;
  float maxIntegral = (Ki > 0.001f) ? MAX_RPM_ESTIMATE / Ki : 1e9f;
  integral += error * PID_SAMPLE_TIME_S;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float derivative = (error - previousErrorRPM) / PID_SAMPLE_TIME_S;
  previousErrorRPM = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

int transformRPMtoPWM(float effort)
{
  float val = RPM_TO_PWM_SLOPE * effort + RPM_TO_PWM_OFFSET;
  return constrain((int)round(val), 0, 255);
}

// ================== SERVO CONTROL ==================
void control_servo()
{
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  if (now - lastUpdate >= REFRESH_INTERVAL)
  {
    lastUpdate = now;
    uint8_t ang = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    uint16_t pulse = map(ang, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// ================== DISPLAY ==================
void displayData()
{
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200)
  {
    lastPrint = millis();
    Serial.print("SetpointRPM:");
    Serial.print(RPM_setpoint, 2);
    Serial.print(", FilteredRPM:");
    Serial.print(g_filteredRPM, 2);
    Serial.print(", PWM:");
    Serial.print(currentPWM);
    Serial.print(", SteeringAngle:");
    Serial.print(steering_angle);
    Serial.println();
  }
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(IS_R, INPUT);
  pinMode(IS_L, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  analogWrite(LPWM, 0);
  digitalWrite(RPWM, LOW);

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

  esp_timer_create_args_t args = {.callback = &rpm_timer_callback, .name = "rpm_calc"};
  esp_timer_create(&args, &rpm_timer_handle);
  esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
}

// ================== LOOP ==================
void loop()
{
  // --- Nhận dữ liệu serial ---
  while (Serial.available() > 0)
  {
    uint8_t b = Serial.read();
    if (serialBufferIndex == 0 && b != '<') continue;
    if (serialBufferIndex < SERIAL_BUFFER_SIZE)
      serialBuffer[serialBufferIndex++] = b;
    if (serialBufferIndex == SERIAL_BUFFER_SIZE)
    {
      if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>')
        parseSerialData();
      serialBufferIndex = 0;
    }
  }

  // --- Cập nhật RPM ---
  float raw;
  noInterrupts();
  raw = currentRPM;
  interrupts();
  g_filteredRPM = updateEMA(raw);

  // --- PID điều khiển ---
  if (millis() - lastPIDRunTime >= PID_INTERVAL_MS)
  {
    lastPIDRunTime = millis();
    if (abs(RPM_setpoint) < 0.1f)
    {
      integral = 0;
      previousErrorRPM = 0;
    }
    float effort = calculatePID_RPM_Output(RPM_setpoint, g_filteredRPM);
    uint16_t pwm = transformRPMtoPWM(effort);
    setMotorPWM(pwm);
  }

  // --- Điều khiển servo ---
  control_servo();

  // --- Hiển thị thông tin ---
  displayData();

  // --- Giữ khoảng cách an toàn ---
  static uint32_t lastDistTime = 0;
  static float baseSpeed = 0;
  static bool obstacleDetected = false;

  if (millis() - lastDistTime >= 200)
  {
    lastDistTime = millis();
    float d = getDistance();
    if (d > 0) d = smoothDistance(d);

    if (d > 0)
    {
      Serial.print("[DIST] ");
      Serial.print(d, 1);
      Serial.println(" cm");

      if (d > 35)
      {
        if (obstacleDetected)
        {
          RPM_setpoint = baseSpeed;
          obstacleDetected = false;
          Serial.println("Khoảng cách an toàn, trở lại tốc độ ban đầu");
        }
      }
      else if (d > 15 && d <= 35)
      {
        if (!obstacleDetected)
        {
          baseSpeed = RPM_setpoint;
          obstacleDetected = true;
        }
        float scale = (d - 15) / (35 - 15);
        float newSpeed = baseSpeed * scale;
        newSpeed = max(newSpeed, 0.0f);
        RPM_setpoint = newSpeed;

        Serial.print("Giảm tốc: ");
        Serial.print(RPM_setpoint, 1);
        Serial.println(" RPM (giữ khoảng cách an toàn)");
      }
      else if (d <= 15)
      {
        setMotorPWM(0);
        RPM_setpoint = 0;
        Serial.println("STOP! Vật cản quá gần (<15cm)");
      }
    }
  }
}
