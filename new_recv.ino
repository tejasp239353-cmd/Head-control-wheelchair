#include <esp_now.h>
#include <WiFi.h>
#include <vector>
using namespace std;

#define LED_BUILTIN 2  // Built-in LED

// Movement Directions
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

// Motor Definitions
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn;
  int pwmSpeedChannel;
};

// Motor Pins (safe for ESP32)
std::vector<MOTOR_PINS> motorPins = {
  {14, 27, 32, 4},  // RIGHT MOTOR
  {25, 26, 33, 5}   // LEFT MOTOR
};

// Speed & PWM Settings
#define MAX_MOTOR_SPEED 120  // Adjusted for faster response
const int PWMFreq = 1000;
const int PWMResolution = 8;

// ESP-NOW Data Structure
struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
};
PacketData receiverData;

unsigned long lastRecvTime = 0;
#define SIGNAL_TIMEOUT 1000

// ====== Motor Control Function ======
void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  } else if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

// ====== Movement Processing ======
void processCarMovement(int movement) {
  switch (movement) {
    case FORWARD:
      rotateMotor(RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case BACKWARD:
      rotateMotor(RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;
    case LEFT:
      rotateMotor(RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(LEFT_MOTOR, 0);
      break;
    case RIGHT:
      rotateMotor(RIGHT_MOTOR, 0);
      rotateMotor(LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case STOP:
    default:
      rotateMotor(RIGHT_MOTOR, 0);
      rotateMotor(LEFT_MOTOR, 0);
      break;
  }
}

// ====== ESP-NOW Receive Callback ======
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == 0) return;

  memcpy(&receiverData, incomingData, sizeof(receiverData));

  Serial.print("x: "); Serial.print(receiverData.xAxisValue);
  Serial.print(" y: "); Serial.print(receiverData.yAxisValue);
  Serial.print(" z: "); Serial.println(receiverData.zAxisValue);

  int movement = STOP;

  // Adjusted thresholds for smoother MPU control
  if (receiverData.yAxisValue < 100) {
    movement = FORWARD;
  } else if (receiverData.yAxisValue > 155) {
    movement = BACKWARD;
  } else if (receiverData.xAxisValue > 155) {
    movement = RIGHT;
  } else if (receiverData.xAxisValue < 100) {
    movement = LEFT;
  } else {
    movement = STOP;
  }

  if (movement == RIGHT) Serial.println("➡️ Turning RIGHT");
  if (movement == LEFT) Serial.println("⬅️ Turning LEFT");
  if (movement == FORWARD) Serial.println("⬆️ Moving FORWARD");
  if (movement == BACKWARD) Serial.println("⬇️ Moving BACKWARD");

  processCarMovement(movement);
  lastRecvTime = millis();
}

// ====== Setup Pins ======
void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    rotateMotor(i, 0);
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(200);
  WiFi.disconnect(false, true);
  delay(100);

  setUpPinModes();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("✅ ESP-NOW Receiver Ready!");
  digitalWrite(LED_BUILTIN, HIGH);
}

// ====== Loop ======
void loop() {
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    processCarMovement(STOP);
  }
}
