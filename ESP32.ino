#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_XOUT_H  0x43
#define I2C_SDA 25
#define I2C_SCL 26

const char* ssid = "Wifi name";
const char* password = "Wifi password";

WiFiUDP udp;
unsigned int localPort = 4210;

const int pcLogPort = 4211;
IPAddress pcIP;
bool pcIPSet = false;

char incomingPacket[255];
char replyPacket[] = "OK";

const int enA = 23;
const int in1 = 22;
const int in2 = 21;
const int enB = 19;
const int in3 = 18;
const int in4 = 5;

const int leftEncoderPin = 27;
const int rightEncoderPin = 14;

int fwdLeftSpeed = 230;
int fwdRightSpeed = 230;
int turnLeftSpeed = 230;
int turnRightSpeed = 230;

const int freq = 5000;
const int resolution = 8;
const int MIN_MOTOR_SPEED = 100;
const int MAX_MOTOR_SPEED = 240;

const double WHEEL_DIAMETER_CM = 6.8;
const double WHEELBASE_CM = 15.5;
const double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159;

const int TICKS_PER_REVOLUTION = 40;

const double TICKS_PER_CM = (double)TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

long leftEncoderTicks = 0;
long rightEncoderTicks = 0;
bool lastLeftState = false;
bool lastRightState = false;
unsigned long lastLeftDebounceTime = 0;
unsigned long lastRightDebounceTime = 0;
const unsigned long debounceDelay = 2;

int leftMotorOffset = 0;
int rightMotorOffset = 0;
unsigned long lastPidTime = 0;
const int PID_INTERVAL_MS = 30;
enum RobotState { STOPPED, MOVING_FORWARD, MOVING_BACKWARD, TURNING_LEFT, TURNING_RIGHT };
RobotState currentState = STOPPED;
long targetTicks = 0;
bool isMovingToTarget = false;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int SMOOTHING_FACTOR = 3;

double currentYaw = 0.0;
double gyroZOffset = 0.0;
unsigned long lastGyroTime = 0;
unsigned long lastGyroReadTime = 0;
const int GYRO_INTERVAL_MS = 50;

double targetYaw = 0.0;

const double TURN_SCALE = 1.0;

float normalizeYaw(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H + 4);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 16.4;
}

void checkLeftEncoder() {
  if ((millis() - lastLeftDebounceTime) > debounceDelay) {
    bool reading = digitalRead(leftEncoderPin);
    if (reading != lastLeftState) {
      lastLeftState = reading;
      leftEncoderTicks++;
      lastLeftDebounceTime = millis();
    }
  }
}

void checkRightEncoder() {
  if ((millis() - lastRightDebounceTime) > debounceDelay) {
    bool reading = digitalRead(rightEncoderPin);
    if (reading != lastRightState) {
      lastRightState = reading;
      rightEncoderTicks++;
      lastRightDebounceTime = millis();
    }
  }
}

void checkEncoders() {
  checkLeftEncoder();
  checkRightEncoder();
}

void setMotorSpeedBalanced(int leftSpeed, int rightSpeed) {
  leftSpeed += leftMotorOffset;
  rightSpeed += rightMotorOffset;

  leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  currentLeftSpeed = currentLeftSpeed + (leftSpeed - currentLeftSpeed) / SMOOTHING_FACTOR;
  currentRightSpeed = currentRightSpeed + (rightSpeed - currentRightSpeed) / SMOOTHING_FACTOR;

  currentLeftSpeed = constrain(currentLeftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  currentRightSpeed = constrain(currentRightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  ledcWrite(enA, currentLeftSpeed);
  ledcWrite(enB, currentRightSpeed);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
}

void resetEncoders() {
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    lastLeftState = digitalRead(leftEncoderPin);
    lastRightState = digitalRead(rightEncoderPin);
}

void moveForward() {
  currentState = MOVING_FORWARD;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  setMotorSpeedBalanced(fwdLeftSpeed, fwdRightSpeed);
}

void moveBackward() {
  currentState = MOVING_BACKWARD;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  setMotorSpeedBalanced(fwdLeftSpeed, fwdRightSpeed);
}

void stopMotors() {
  currentState = STOPPED;
  isMovingToTarget = false;
  targetTicks = 0;

  targetYaw = 0.0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  setMotorSpeed(0, 0);
  resetEncoders();
  Serial.println("Motors STOPPED.");
}

void turnLeft() {
  currentState = TURNING_LEFT;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  setMotorSpeed(turnLeftSpeed, turnRightSpeed);
}

void turnRight() {
  currentState = TURNING_RIGHT;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  setMotorSpeed(turnLeftSpeed, turnRightSpeed);
}

void autoCalibrateMotors() {
  Serial.println("=== AUTO CALIBRATION STARTED ===");
  resetEncoders();
  delay(500);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  setMotorSpeed(200, 200);

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    checkEncoders();
    delay(1);
  }

  long error = leftEncoderTicks - rightEncoderTicks;

  if (error > 10) {
    leftMotorOffset = -constrain(error/3, 8, 25);
  } else if (error > 5) {
    leftMotorOffset = -constrain(error/2, 3, 15);
  } else if (error < -10) {
    rightMotorOffset = -constrain(abs(error)/3, 8, 25);
  } else if (error < -5) {
    rightMotorOffset = -constrain(abs(error)/2, 3, 15);
  } else {
    Serial.println("Motors are well balanced!");
  }

  String calibMsg = "L_Offset: " + String(leftMotorOffset) + " | R_Offset: " + String(rightMotorOffset);
  Serial.println(calibMsg);

  stopMotors();
  Serial.println("=== AUTO CALIBRATION COMPLETE ===");
}

void parseAndExecuteCommand(char* command) {
  char* commandPart = strtok(command, ":");

  char* valuePart = strtok(NULL, ":");

  if (commandPart == NULL) return;

  double value = 0;
  if (valuePart != NULL) {
    value = atof(valuePart);
  }

  if (currentState != STOPPED) {
      if (strcmp(commandPart, "STOP") == 0) {
      } else if (strcmp(commandPart, "SETFWDLEFTSPEED") == 0) {
      } else if (strcmp(commandPart, "SETFWDRIGHTSPEED") == 0) {
      } else if (strcmp(commandPart, "SETTURNLEFTSPEED") == 0) {
      } else if (strcmp(commandPart, "SETTURNRIGHTSPEED") == 0) {
      } else if (strcmp(commandPart, "SETYAW") == 0) {
      } else if (strcmp(commandPart, "LEFTICK") == 0) {
      } else if (strcmp(commandPart, "RIGHTICK") == 0) {
      } else {
          Serial.println("Command ignored, robot is busy. Send 'STOP' first.");
          return;
      }
  }

  if (strcmp(commandPart, "FORWARD") == 0 && value > 0) {
    resetEncoders();
    targetTicks = (long)(value * TICKS_PER_CM);
    isMovingToTarget = true;
    moveForward();
    String msg = "Moving FORWARD " + String(value) + "cm (" + String(targetTicks) + " ticks)";
    Serial.println(msg);
  } else if (strcmp(commandPart, "BACK") == 0 && value > 0) {
    resetEncoders();
    targetTicks = (long)(value * TICKS_PER_CM);
    isMovingToTarget = true;
    moveBackward();
    String msg = "Moving BACKWARD " + String(value) + "cm (" + String(targetTicks) + " ticks)";
    Serial.println(msg);

  } else if (strcmp(commandPart, "LEFT") == 0) {
    resetEncoders();
    targetYaw = currentYaw + (70 * TURN_SCALE);
    isMovingToTarget = true;
    turnLeft();
    String msg = "Turning LEFT " + String(value) + " degrees (Target Yaw: " + String(normalizeYaw(targetYaw)) + ")";
    Serial.println(msg);
  } else if (strcmp(commandPart, "RIGHT") == 0) {
    resetEncoders();
    targetYaw = currentYaw - (70 * TURN_SCALE);
    isMovingToTarget = true;
    turnRight();
    String msg = "Turning RIGHT " + String(value) + " degrees (Target Yaw: " + String(normalizeYaw(targetYaw)) + ")";
    Serial.println(msg);

  } else if (strcmp(commandPart, "STOP") == 0) {
    stopMotors();

  } else if (strcmp(commandPart, "SETFWDLEFTSPEED") == 0) {
      long speed = (long)value;
      if (speed >= MIN_MOTOR_SPEED && speed <= MAX_MOTOR_SPEED) {
          fwdLeftSpeed = speed;
          Serial.println("Fwd Left Speed set to: " + String(fwdLeftSpeed));
      } else {
          Serial.println("Speed out of range (100-240)");
      }
  } else if (strcmp(commandPart, "SETFWDRIGHTSPEED") == 0) {
      long speed = (long)value;
      if (speed >= MIN_MOTOR_SPEED && speed <= MAX_MOTOR_SPEED) {
          fwdRightSpeed = speed;
          Serial.println("Fwd Right Speed set to: " + String(fwdRightSpeed));
      } else {
          Serial.println("Speed out of range (100-240)");
      }
  } else if (strcmp(commandPart, "SETTURNLEFTSPEED") == 0) {
      long speed = (long)value;
      if (speed >= MIN_MOTOR_SPEED && speed <= MAX_MOTOR_SPEED) {
          turnLeftSpeed = speed;
          Serial.println("Turn Left Speed set to: " + String(turnLeftSpeed));
      } else {
          Serial.println("Speed out of range (100-240)");
      }
  } else if (strcmp(commandPart, "SETTURNRIGHTSPEED") == 0) {
      long speed = (long)value;
      if (speed >= MIN_MOTOR_SPEED && speed <= MAX_MOTOR_SPEED) {
          turnRightSpeed = speed;
          Serial.println("Turn Right Speed set to: " + String(turnRightSpeed));
      } else {
          Serial.println("Speed out of range (100-240)");
      }
  }
  else if (strcmp(commandPart, "SETYAW") == 0) {
      long angle = (long)value;
      if (angle >= -360 && angle <= 360) {
          targetYaw = angle;
          Serial.println("YAW angle set to: " + String(angle));
      } else {
          Serial.println("Speed out of range (100-240)");
      }
  }
   else {
    Serial.println("Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Robot (GYRO + ENCODER) Starting...");
  Serial.print("Ticks/cm: "); Serial.println(TICKS_PER_CM);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  lastLeftState = digitalRead(leftEncoderPin);
  lastRightState = digitalRead(rightEncoderPin);

  ledcAttach(enA, freq, resolution);
  ledcAttach(enB, freq, resolution);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! ESP32 IP: " + WiFi.localIP().toString());

  udp.begin(localPort);
  Serial.println("UDP server listening on port " + String(localPort));

  Serial.println("--- MPU6050 SETUP ---");
  pinMode(I2C_SDA , INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("!!! I2C ERROR !!!");
    Serial.print("MPU6050 not found at 0x68. Error code: ");
    Serial.println(error);
    Serial.println("Check wiring (SDA, SCL, VCC, GND).");
    while (true);
  }
  else {
    Serial.println("MPU6050 WAKED UP");
  }

  delay(100);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission();
  Serial.println("MPU6050 CONFIGURED");
  delay(100);

  Serial.println("CALIBRATING GYRO... DO NOT MOVE!");
  double sum = 0;
  for (int i = 0; i < 2000; i++) {
    sum += readGyroZ();
    delay(1);
  }
  gyroZOffset = sum / 2000.0;
  Serial.printf("GYRO OFFSET: %.6f °/s\n", gyroZOffset);
  Serial.println("CALIBRATION DONE!");
  lastGyroTime = micros();

  autoCalibrateMotors();
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;

  float gz = readGyroZ() - gyroZOffset;

  if (abs(gz) > 0.05) {
    currentYaw += gz * dt;
  }
}

void loop() {
  if (millis() - lastGyroReadTime > GYRO_INTERVAL_MS) {
    lastGyroReadTime = millis();
    updateYaw();
  }

  checkEncoders();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    if (!pcIPSet) {
      pcIP = udp.remoteIP();
      pcIPSet = true;
      Serial.println("PC IP address set to: " + pcIP.toString());
      Serial.println("PC GUI connected. Logging enabled.");
    }

    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    Serial.println("\nReceived command: " + String(incomingPacket));

    parseAndExecuteCommand(incomingPacket);

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((uint8_t*)replyPacket, strlen(replyPacket));
    udp.endPacket();
  }

  if (isMovingToTarget) {

    if (currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD) {
      long currentTicks = (leftEncoderTicks + rightEncoderTicks) / 2;

      if (currentTicks >= targetTicks) {
        String msg = "Target distance reached! (" + String(currentTicks) + " ticks)";
        Serial.println(msg);
        stopMotors();
      }
    }

    else if (currentState == TURNING_LEFT) {
      if (currentYaw >= targetYaw) {
        String msg = "Target angle reached! (" + String(normalizeYaw(currentYaw), 2) + " degrees)";
        Serial.println(msg);
        stopMotors();
      }
    }
    else if (currentState == TURNING_RIGHT) {
      if (currentYaw <= targetYaw) {
        String msg = "Target angle reached! (" + String(normalizeYaw(currentYaw), 2) + " degrees)";
        Serial.println(msg);
        stopMotors();
      }
    }

  }

  if (millis() - lastPidTime > PID_INTERVAL_MS) {
    if (currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD) {
      if (isMovingToTarget) {
        setMotorSpeedBalanced(fwdLeftSpeed, fwdRightSpeed);

        String msg = "FWD/BWD | Ticks:" + String(targetTicks) +
                     " | L:" + String(leftEncoderTicks) +
                     " | R:" + String(rightEncoderTicks) +
                     " | YAW:" + String(normalizeYaw(currentYaw), 2);
        Serial.println(msg);
      }
    } else if (currentState == TURNING_LEFT || currentState == TURNING_RIGHT) {
      if (isMovingToTarget) {
        String msg = "TURN | Target Yaw:" + String(normalizeYaw(targetYaw)) +
                     " | Current Yaw:" + String(normalizeYaw(currentYaw), 2);
        Serial.println(msg);
      }
    }
    lastPidTime = millis();
  }
}