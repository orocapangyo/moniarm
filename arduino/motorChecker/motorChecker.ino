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
#define PUMP_PIN 4

#define M0_ID 6
#define M1_ID 7
#define M2_ID 8
#define M3_ID 15

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

// Time interval for measurements in milliseconds
const int INTERVAL = 100;
long previousMillis = 0;
long currentMillis = 0;

bool blinkStatus = false;

int motorID, targetAngle;

void setup() {
  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_F, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(PUMP_PIN, LOW);

  delay(2000);           //a delay to have time for serial monitor opening
  Serial.begin(115200);  // Open serial communications for debug
  Serial.println("Begin Herkulex Tester");
  Serial.println("Type motorID anggle, then enter");
  Serial.println("Air Pump, ID is 0, angle is 0/1");
  Serial.println("Exampe: 12 48");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
  Herkulex.reboot(M2_ID);
  delay(200);
  Herkulex.reboot(M3_ID);
  delay(200);
  Herkulex.initialize();  //initialize motors
  delay(500);
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, receive command input
  // LED blink on ESP32, change color on Herkulex
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    if (blinkStatus == false) {
      Herkulex.setLed(M0_ID, LED_GREEN);
      Herkulex.setLed(M1_ID, LED_BLUE);
      Herkulex.setLed(M2_ID, LED_GREEN);
      Herkulex.setLed(M3_ID, LED_BLUE);
      blinkStatus = true;
    } else {
      Herkulex.setLed(M0_ID, 0);
      Herkulex.setLed(M1_ID, 0);
      Herkulex.setLed(M2_ID, 0);
      Herkulex.setLed(M3_ID, 0);
      blinkStatus = false;
    }

    // blink LED to indicate activity
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    //input ID angle
    if (Serial.available() > 0) {
      motorID = Serial.parseInt();
      targetAngle = Serial.parseInt();

      //prints the received float number
      Serial.print("ID:");
      Serial.print(motorID);
      Serial.print(", Angle:");
      Serial.println(targetAngle);


      if (motorID == 0) {
        Serial.println("Air Pump on/off");
        digitalWrite(PUMP_PIN, targetAngle);
      } else {
        Serial.println("Moving motor...");
        Herkulex.moveOneAngle(motorID, targetAngle, 500, LED_RED);  //move motor with 500 speed
        delay(1200);
        Serial.print("Get servo Angle:");
        Serial.println(Herkulex.getAngle(motorID));
      }
    }
  }
}
