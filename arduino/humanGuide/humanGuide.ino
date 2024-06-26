/*
 * Human guide, capture each motor's angle after human move robot arm
 * ZETA7, zeta0707@gmail.com
*/
#include <Herkulex.h>

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13

#define M0_ID 6
#define M1_ID 7
#define M2_ID 8
#define M3_ID 15

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

// Time interval for measurements in milliseconds
const int INTERVAL = 300;
long previousMillis = 0;
long currentMillis = 0;
long previousPress = 0;

bool blinkStatus = false;

int motorID, targetAngle;

int captureMode = 0;
int keyinput;

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
  delay(1000);

  Herkulex.torqueOFF(BROADCAST_ID);

  Serial.println("Begin Herkulex Angle Capture");
  Serial.println("Press 'p': periodic capture, ' ': capture only ' ' key input");

  while (1) {
    if (Serial.available()) {
      keyinput = Serial.read();
      if (keyinput == 'p') {
        captureMode = 1;
        //Serial.println(captureMode);
        break;
      } else if (keyinput == ' ') {
        captureMode = 2;
        keyinput = 0;
        //Serial.println(captureMode);
        break;
      } else Serial.println('Input differnent key');
    }
  }

  previousPress = millis();
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, receive command input
  // LED blink on ESP32, change color on Herkulex
  if (currentMillis - previousMillis > INTERVAL) {
    //blink motor LED
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

    if (Serial.available()) {
      keyinput = Serial.read();
    }

    if ((captureMode == 1) || ((captureMode == 2) && (keyinput == ' '))) {
      keyinput = 0;
      Serial.print(int(Herkulex.getAngle(M0_ID)));
      Serial.print(':');
      Serial.print(int(Herkulex.getAngle(M1_ID)));
      Serial.print(':');
      Serial.print(int(Herkulex.getAngle(M2_ID)));
      Serial.print(':');
      Serial.print(int(Herkulex.getAngle(M3_ID)));
      Serial.print(':');
      Serial.println(float(currentMillis - previousPress) / 1000.0, 3);
      previousPress = currentMillis;
    }
    previousMillis = currentMillis;
  }
}
