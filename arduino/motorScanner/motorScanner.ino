/*
 * DRS-0101 motor ID scan and ID changer
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
#define ADDR_ACCEL_RATIO 8
#define DFT_ACCEL_RATIO 25

// Time interval for measurements in milliseconds
const int INTERVAL = 100;
long previousMillis = 0;
long currentMillis = 0;

bool blinkStatus = false;

int motorID = 0, oldID = 0, newID = 0;

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
  Serial.println("Herkulex ID scanner");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  for (motorID = 0; motorID < ALL_MOTOR; motorID++) {
    Herkulex.reboot(motorID);
  }
  Herkulex.initialize();  //initialize motors

  for (motorID = 0; motorID < ALL_MOTOR; motorID++) {
    if (Herkulex.readRegistryRAM(motorID, 8) == DFT_ACCEL_RATIO) {
      Serial.print("ID:");
      Serial.println(motorID);
    }
  }

  Serial.println("Type oldID, newID:");
  Serial.println("or Type checkID:");
}

void loop() {
  // Record the time
  currentMillis = millis();

  //input ID angle
  if (Serial.available() > 0) {
    oldID = Serial.parseInt();
    newID = Serial.parseInt();
    if (newID != 0) {
      Herkulex.set_ID(oldID, newID);
      Serial.print(oldID);
      Serial.print("->");
      Serial.print(newID);
    } else {
      newID = oldID;
      Serial.print("Watch green LED blink");
    }
  }

  // If the time interval has passed, receive command input
  // LED blink on ESP32, change color on Herkulex
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    if (blinkStatus == false) {
      Herkulex.setLed(newID, LED_GREEN);
      blinkStatus = true;
    } else {
      Herkulex.setLed(newID, 0);
      blinkStatus = false;
    }

    // blink LED to indicate activity
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
