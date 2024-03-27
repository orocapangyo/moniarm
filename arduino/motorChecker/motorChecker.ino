#include <Herkulex.h>

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13

#define M0_ID 2
#define M1_ID 1
#define M2_ID 6

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

  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_F, LOW);
  digitalWrite(BUZZER, LOW);

  delay(2000);           //a delay to have time for serial monitor opening
  Serial.begin(115200);  // Open serial communications for debug
  Serial.println("Begin Herkulex Tester");
  Serial.println("Type motorID anggle, then enter");
  Serial.println("Exampe: 12 48");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
  Herkulex.reboot(M2_ID);
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
      Herkulex.setLed(M0_ID, LED_RED);
      Herkulex.setLed(M1_ID, LED_GREEN);
      Herkulex.setLed(M2_ID, LED_BLUE);
      blinkStatus = true;
    } else {
      Herkulex.setLed(M0_ID, 0);
      Herkulex.setLed(M1_ID, 0);
      Herkulex.setLed(M2_ID, 0);
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

      Serial.println("Moving motor...");
      Herkulex.moveOneAngle(motorID, targetAngle, 500, LED_RED);  //move motor with 500 speed
      delay(1200);
      Serial.print("Get servo Angle:");
      Serial.println(Herkulex.getAngle(motorID));
    }
  }
}
