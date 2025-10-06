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

  Serial.println("Receiver Ready...");
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    xAxis = receivedData[0];
    yAxis = receivedData[1];
    lastReceivedTime = millis();
  }

  if (millis() - lastReceivedTime > signalTimeout) {
    stopMotors();
    return;
  }

  // Forward/Backward control
  if (yAxis < 112) {  // Backward
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW);

    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);
    motorSpeedA = map(yAxis, 112, 0, 0, 255);
    motorSpeedB = map(yAxis, 112, 0, 0, 255);
  }
  else if (yAxis > 112) {  // Forward
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    motorSpeedA = map(yAxis, 112, 228, 0, 255);
    motorSpeedB = map(yAxis, 112, 228, 0, 255);
  }
 else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  // Steering
  if (xAxis < 50) {  // Move left
    int xMapped = map(xAxis, 115, 0, 0, 255);
    motorSpeedA -= xMapped;
    motorSpeedB += xMapped;
  }
  else if (xAxis > 200) {  // Move right
    int xMapped = map(xAxis, 115, 228, 0, 255);
    motorSpeedA += xMapped;
    motorSpeedB -= xMapped;
  }

  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  Serial.print("X: "); Serial.print(xAxis);
  Serial.print(" | Y: "); Serial.print(yAxis);
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
