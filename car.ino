#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define enA 3
#define in1 4
#define in2 5
#define enB 8
#define in3 6
#define in4 7

RF24 radio(9, 10);
const byte address[6] = "00001";

int receivedData[2];
int xAxis = 0, yAxis = 0;
int motorSpeedA = 0;
int motorSpeedB = 0;

unsigned long lastReceivedTime = 0;
const unsigned long signalTimeout = 500;

int xIdle = 112;
int yIdle = 112;

bool connected = false;
bool calibrated = false;

void setup() {
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  Serial.begin(9600);
  radio.begin();
  radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial.println("Receiver waiting for controller signal...");
}

void loop() {
  unsigned long currentMillis = millis();

  // 🕒 1. Wait for connection if not connected
  if (!connected) {
    if (radio.available()) {
      radio.read(&receivedData, sizeof(receivedData));
      Serial.println("Controller detected. Starting calibration...");
      calibrateJoystick();
      connected = true;
      calibrated = true;
      lastReceivedTime = currentMillis;
      Serial.println("Calibration complete. RC car ready!");
    }
    return; // do nothing while waiting
  }

  // 🕹 2. Normal operation once connected
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    xAxis = receivedData[0];
    yAxis = receivedData[1];
    lastReceivedTime = currentMillis;
  }

  // 🧠 3. Check for disconnection
  if (currentMillis - lastReceivedTime > signalTimeout) {
    stopMotors();
    if (connected) {
      Serial.println("Controller disconnected. Waiting for reconnection...");
      connected = false;
      calibrated = false;
    }
    return;
  }

  // 🚗 4. Drive logic (only runs if connected)
  // Forward/Backward control with calibrated idle
  if (yAxis < yIdle - 5) {  // Backward
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);

    motorSpeedA = map(yAxis, yIdle, 0, 0, 255);
    motorSpeedB = map(yAxis, yIdle, 0, 0, 255);
  }
  else if (yAxis > yIdle + 5) {  // Forward
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); 
    digitalWrite(in4, HIGH);

    motorSpeedA = map(yAxis, yIdle, 228, 0, 255);
    motorSpeedB = map(yAxis, yIdle, 228, 0, 255);
  }
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  // Steering with calibrated idle
  if (xAxis < xIdle - 15) {  // Left
    int xMapped = map(xAxis, xIdle, 0, 0, 255);
    motorSpeedA -= xMapped;
    motorSpeedB += xMapped;
  }
  else if (xAxis > xIdle + 15) {  // Right
    int xMapped = map(xAxis, xIdle, 228, 0, 255);
    motorSpeedA += xMapped;
    motorSpeedB -= xMapped;
  }

  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  Serial.print("X: "); Serial.print(xAxis);
  Serial.print(" | Y: "); Serial.print(yAxis);
  Serial.print(" | Xidle: "); Serial.print(xIdle);
  Serial.print(" | Yidle: "); Serial.print(yIdle);
  Serial.print(" | A: "); Serial.print(motorSpeedA);
  Serial.print(" | B: "); Serial.println(motorSpeedB);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Calibration after connection or reconnection
void calibrateJoystick() {
  Serial.println("Calibrating... keep joystick centered for 1 second.");

  unsigned long startTime = millis();
  long xTotal = 0;
  long yTotal = 0;
  int sampleCount = 0;

  while (millis() - startTime < 1000) {
    if (radio.available()) {
      radio.read(&receivedData, sizeof(receivedData));
      xTotal += receivedData[0];
      yTotal += receivedData[1];
      sampleCount++;
    }
  }

  if (sampleCount > 0) {
    xIdle = xTotal / sampleCount;
    yIdle = yTotal / sampleCount;
    Serial.print("New calibration: Xidle = ");
    Serial.print(xIdle);
    Serial.print(" | Yidle = ");
    Serial.println(yIdle);
  } else {
    Serial.println("Calibration failed. Using last known idle values.");
  }
}
