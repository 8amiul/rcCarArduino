/*

  RC CAR

*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Motor pins
#define enA 3  
#define in1 4
#define in2 5
#define enB 8   
#define in3 6
#define in4 7

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";


int receivedData[2];
int xAxis = 0, yAxis = 0;
int motorSpeedA = 0;
int motorSpeedB = 0;

int radio_found = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial.println("Receiver Ready...");
}

void loop() {
  // Receive joystick values

  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    xAxis = receivedData[0];
    delay(5);
    yAxis = receivedData[1];
    delay(5);
  }

  // --- Y-axis: Forward/Backward ---
  if (yAxis < 450) {  // Backward
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW);

    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);
    motorSpeedA = map(yAxis, 470, 0, 0, 255);
    motorSpeedB = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 825) {  // Forward
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);
  }
  else {  // Deadzone
    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  // --- X-axis: Left/Right ---
  if (xAxis < 460) {  // Move left
    int xMapped = map(xAxis, 470, 0, 0, 255);
    motorSpeedA -= xMapped;
    motorSpeedB += xMapped;
  }
  else if (xAxis > 900) {  // Move right
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    motorSpeedA += xMapped;
    motorSpeedB -= xMapped;
  }

  // Constrain motor speeds 0–255
  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  // Prevent buzzing at low speeds
  if (motorSpeedA < 70) motorSpeedA = 0;
  if (motorSpeedB < 70) motorSpeedB = 0;

  // Apply PWM
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  // --- Serial Debug ---
  Serial.print("X: "); Serial.print(xAxis);
  Serial.print(" | Y: "); Serial.print(yAxis);
  Serial.print(" | Motor A: "); Serial.print(motorSpeedA);
  Serial.print(" | Motor B: "); Serial.println(motorSpeedB);

  delay(100); // slow down serial output
}
