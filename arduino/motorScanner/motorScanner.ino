/*
 * DRS-0101 smart motor tester
 * ZETA7, zeta0707@gmail.com
*/
#include <Herkulex.h>

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

#define ALL_MOTOR 0xFE

// Time interval for measurements in milliseconds
const int INTERVAL = 100;
long previousMillis = 0;
long currentMillis = 0;

bool blinkStatus = false;

int motorID;

void setup() {
  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_F, LOW);
  digitalWrite(BUZZER, LOW);

  Serial.begin(115200);  // Open serial communications for debug
  Serial.println("Begin Herkulex ID scanner");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  for (motorID = 0; motorID < 16; motorID++) {
    Herkulex.reboot(motorID);
    delay(200);
  }
  Herkulex.initialize();  //initialize motors
  for (motorID = 0; motorID < 16; motorID++) {
    Herkulex.torqueOFF(motorID);
  }
  for (motorID = 0; motorID < 16; motorID++) {
      Serial.print("ID:");
      Serial.print(motorID);
      Serial.print(",Angle:");
      Serial.println(Herkulex.getAngle(motorID));
  }

  Serial.println("Type motorID:");
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, receive command input
  // LED blink on ESP32, change color on Herkulex
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    if (blinkStatus == false) {
      Herkulex.setLed(motorID, LED_GREEN);
      blinkStatus = true;
    } else {
      Herkulex.setLed(motorID, 0);
      blinkStatus = false;
    }

    // blink LED to indicate activity
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    //input ID angle
    if (Serial.available() > 0) {
      motorID = Serial.parseInt();
      Serial.println("Watch Green LED blink");
    }
  }
}
