/*

  Controller

*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

char xyData[32] = "";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Serial.println("Transmitter Ready...");
}

int xAxis, yAxis;

void loop() {
  xAxis = analogRead(A1);
  yAxis = analogRead(A0);

  int data[2] = {xAxis, yAxis};  // put both in an array
  radio.write(&data, sizeof(data));

  Serial.print("Sent X: "); Serial.print(xAxis);
  Serial.print(" | Sent Y: "); Serial.println(yAxis);

  delay(20);
}


