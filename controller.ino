/*
  Controller
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

int xAxis, yAxis;
int serialX = -1, serialY = -1;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Serial.println("Transmitter Ready...");
  Serial.println("Transmitter Ready...");
  Serial.println("Type X Y (0–255) to override joystick, e.g.: 120 200");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      int spaceIndex = input.indexOf(' ');
      if (spaceIndex > 0) {
        int xVal = input.substring(0, spaceIndex).toInt();
        int yVal = input.substring(spaceIndex + 1).toInt();
        xVal = constrain(xVal, 0, 255);
        yVal = constrain(yVal, 0, 255);

        serialX = xVal;
        serialY = yVal;

        Serial.print("Serial Override Set → X: ");
        Serial.print(serialX);
        Serial.print(" | Y: ");
        Serial.println(serialY);
      }
    }
  }



  if (serialX >= 0 && serialY >= 0) {
    // Use serial override values
    xAxis = serialX;
    yAxis = serialY;
  } else {
    // Normal joystick read

  xAxis = map(analogRead(A1), 0, 1023, 0, 255);
  yAxis = map(analogRead(A0), 0, 1023, 0, 255);
  }

  int data[2] = {xAxis, yAxis};

  bool ok = radio.write(&data, sizeof(data));
  if (!ok) {
    Serial.println("Failed to send");
    Serial.print("X: ");
    Serial.print(xAxis);
    Serial.print(" | Y: ");Serial.print(yAxis);
  } else {
    Serial.print("Sent X: ");
    Serial.print(xAxis);
    Serial.print(" | Y: ");Serial.println(yAxis);
  delay(20);
  }
}
