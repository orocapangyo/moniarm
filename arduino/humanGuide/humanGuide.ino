/*
# Human guide, capture each motor's angle after human move robot arm
# Copyright (c) 2024, ChangWhan Lee
# ZETA7, zeta0707@gmail.com
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/
#include <Herkulex.h>

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13

#define M0_ID 1
#define M1_ID 2
#define M2_ID 3
#define M3_ID 4

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
