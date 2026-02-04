#include <esp_now.h>
#include <WiFi.h>

// ====== MPU6050 Libraries ======
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ====== MPU Variables ======
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// ====== RECEIVER MAC Address (✅ Updated) ======
uint8_t receiverMacAddress[] = {0xF8, 0xB3, 0xB7, 0x2B, 0xD7, 0xA0};

// ====== Data Structure to Send ======
struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
};
PacketData data;

// ====== ESP-NOW Send Callback ======
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Sent OK" : "❌ Send failed");
}

// ====== MPU Setup Function ======
void setupMPU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("✅ MPU6050 ready!");
  } else {
    Serial.println("❌ MPU6050 init failed");
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);

  // ✅ Clean WiFi + ESP-NOW initialization
  WiFi.mode(WIFI_STA);
  delay(200);
  WiFi.disconnect(false, true);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed, restarting...");
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("✅ ESP-NOW initialized successfully");
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer (Receiver)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("✅ Peer added successfully");
  }

  setupMPU();

  // LED indicator (optional)
  pinMode(2, OUTPUT); // Built-in LED pin (GPIO 2)
}

// ====== Main Loop ======
void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int xAxisValue = constrain(ypr[2] * 180 / M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180 / M_PI, -90, 90);
    int zAxisValue = constrain(ypr[0] * 180 / M_PI, -90, 90);

    data.xAxisValue = map(xAxisValue, -90, 90, 0, 254);
    data.yAxisValue = map(yAxisValue, -90, 90, 0, 254);
    data.zAxisValue = map(zAxisValue, -90, 90, 0, 254);

    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));

    Serial.print("x: "); Serial.print(xAxisValue);
    Serial.print("  y: "); Serial.print(yAxisValue);
    Serial.print("  z: "); Serial.println(zAxisValue);

    digitalWrite(2, !digitalRead(2)); // Blink LED while sending
    delay(50); // ~20 Hz updates
  }
}
