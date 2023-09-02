#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HuskyLensMbed.h> // Include the HuskyLens library

#define ENABLE_WIFI true

#define telemetriaTX 19
#define telemetriaRX 23

#if ENABLE_WIFI == true
#include <OTAUpdate.h>
#include <Telemetry.h>

IPAddress receiversIP(192, 168, 0, 102);
uint16_t receiversPort = 4210;
uint16_t udpPort = 1234;
uint16_t otaPort = 80;

Updater miota(otaPort);
TelemetryManager telemetry(receiversIP, receiversPort);
#endif

#define servoKP 2.5
#define servoKD 10
int prev_setAngle;
int actual_directionError;
int prev_directionError;
int objectiveDirection;

enum e {
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando
};
uint8_t estado = e::Recto;

uint8_t giros = 1;
int8_t turnSense;

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

uint32_t prev_ms_tele = 0;

float lidarAngle;
uint16_t distancesArray[2][360];
volatile bool arrayLecture = false;

bool reading = false;
bool writing = false;

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;
HardwareSerial teleSerial(0);

int directionError(int bearing, int target);

void setSpeed(int speed);
void setSteering(int angle);
void receiveData();
void manageTension(uint8_t tension);

uint16_t getIndex(float angle);
// Angle from 0 to 359
uint16_t readDistance(uint16_t angle);
// Create code for task1
void LidarTaskCode(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);

  setPinModes();

  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

#if ENABLE_WIFI == true
  miota.WiFiInit();
  miota.SetStaticIP(250);
  miota.OTAInit();

  telemetry.StartUDP(udpPort);
  digitalWrite(pinLED_rojo, HIGH);
#endif

  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  // Initialize HuskyLens here
  // For example: HuskyLens.begin(Serial2);
  // (Replace the above comment with your actual HuskyLens initialization code)

  // removed Lidar initialization

  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
      LidarTaskCode,
      "Task1",
      100000,
      NULL,
      10,
      &Task1,
      0);
  delay(500);
  digitalWrite(pinLED_rojo, LOW);
  // start motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  // removed HuskyLens initialization

  delay(500);

  setSpeed(20);
  mimpu.measureFirstMillis();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (commSerial.available()) {
    receiveData();
  }

  mimpu.UpdateAngle();

  static uint32_t prev_ms_direction = millis();
  if (millis() > prev_ms_direction) {
    actual_directionError = constrain(directionError(mimpu.GetAngle(), objectiveDirection), -127, 127);
    int _setAngle = servoKP * actual_directionError + servoKD * (actual_directionError - prev_directionError);
    if (_setAngle != prev_setAngle) {
      setSteering(_setAngle);
      prev_setAngle = _setAngle;
    }
    prev_directionError = actual_directionError;
    prev_ms_direction = millis() + 20;
  }

  if (millis() > prev_ms_tele + 300) {
    teleSerial.write(04);
    uint16_t zi = 0;
    uint16_t pi = 0;
    while (zi < 360) {
      if (zi == pi) {
        teleSerial.write(distancesArray[arrayLecture][zi] >> 8);
        pi++;
      } else {
        teleSerial.write(distancesArray[arrayLecture][zi] & 0x00ff);
        zi++;
      }
    }
    prev_ms_tele = millis();
  }

  static uint16_t i = 0;
  int distance0 = readDistance(i);
  switch (estado) {
  case e::Recto:
    digitalWrite(pinLED_rojo, LOW);
    digitalWrite(pinLED_verde, LOW);
    if (giros == 13) {
      setSpeed(0);
    } else if ((distance0 < 1000) && (distance0 > 300)) {
      digitalWrite(pinLED_rojo, HIGH);
      estado = e::DecidiendoGiro;
    }
    break;
  case e::DecidiendoGiro:
    if (readDistance(90) >= 1000) {
      turnSense = -1;
      objectiveDirection -= 90;
      estado = e::Girando;
    } else if (readDistance(270) >= 1000) {
      turnSense = 1;
      objectiveDirection += 90;
      estado = e::Girando;
    }
    break;
  case e::Girando:
    digitalWrite(pinLED_verde, HIGH);
    if (abs(90 * giros - mimpu.GetAngle() * turnSense) < 10) {
      giros++;
      estado = e::Recto;
    }
    break;
  }
  i++;
  if (i == 5)
    i = 355;
  if (i == 360)
    i = 0;

  // Handle HuskyLens data here
  // Retrieve HuskyLens data and process it as needed
  // Example:
  int objectCount = HuskyLens.count();
  for (int i = 0; i < objectCount; i++) {
    String objectName = HuskyLens.readName(i);
    int objectX = HuskyLens.readX(i);
    int objectY = HuskyLens.readY(i);
    // Process the object data as needed
  }

  // Send data over UDP for object detection and color identification results
  // Modify the code to send HuskyLens data over UDP if needed

  /*
  telemetry.AddData(String(estado));
  for (uint16_t arrayIndex; arrayIndex < 5; arrayIndex++) {
    telemetry.AddData(String(arrayIndex));
    telemetry.AddData(String(readDistance(arrayIndex)));
  }
  for (uint16_t arrayIndex=355; arrayIndex < 360; arrayIndex++) {
    telemetry.AddData(String(arrayIndex));
    telemetry.AddData(String(readDistance(arrayIndex)));
  }
  telemetry.SendData();*/
}





