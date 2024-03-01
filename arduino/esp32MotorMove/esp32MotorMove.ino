#include <Herkulex.h>

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13

#define M0_ID 6
#define M1_ID 7
#define M2_ID 8

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

// Time interval for measurements in milliseconds
const int INTERVAL = 100;
long previousMillis = 0;
long currentMillis = 0;

bool blinkState = false;

int motorID, targetAngle;

void setup() {
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
  Herkulex.reboot(M1_ID);
  Herkulex.reboot(M2_ID);
  delay(500);
  Herkulex.initialize();  //initialize motors
  delay(200);
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

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
