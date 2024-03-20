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
  Serial.println("Begin Herkulex Angle Capture");
  Serial.println("Press c: continous capture, s: capture only 's'");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
  Herkulex.reboot(M2_ID);
  delay(200);
  Herkulex.initialize();  //initialize motors
  Herkulex.torqueOFF(M0_ID);
  Herkulex.torqueOFF(M1_ID);
  Herkulex.torqueOFF(M2_ID);
  delay(500);

  while (1) {
    if (Serial.available()) {
      keyinput = Serial.read();
      if (keyinput == 'c') {
        captureMode = 1;
        Serial.println(captureMode);
        break;
      } else if (keyinput == 's') {
        captureMode = 2; keyinput=0;
        Serial.println(captureMode);
        break;
      } else Serial.println('Input differnent key');
    }
  }
}

void loop() {
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, receive command input
  // LED blink on ESP32, change color on Herkulex
  if (currentMillis - previousMillis > INTERVAL) {
    //blink motor LED
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

    if (Serial.available()) {
      keyinput = Serial.read();
    }

    if ((captureMode == 1) || ((captureMode == 2) && (keyinput == ' '))) {
      keyinput = 0;
      Serial.print(Herkulex.getAngle(M0_ID));
      Serial.print(':');
      Serial.print(Herkulex.getAngle(M1_ID));
      Serial.print(':');
      Serial.print(Herkulex.getAngle(M2_ID));
      Serial.print(':');
      Serial.println(0.101);
    }
  }
}
