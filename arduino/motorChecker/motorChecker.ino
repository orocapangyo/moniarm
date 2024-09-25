/*
# DRS-0101 smart motor tester
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

#define DUAL_SHOULDER 0

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_F 5
#define BUZZER 13
#define PUMP_PIN 4

#define M0_ID 1
#define M1_ID 2
#define M2_ID 3
#define M3_ID 4
#define M1M_ID 0

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

// Time interval for measurements in milliseconds
const int INTERVAL = 1000;
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
  Serial.println("Air Pump, ID is 256, angle is 0/1");
  Serial.println("Exampe: 12 48");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
#if (DUAL_SHOULDER == 1)
  Herkulex.reboot(M1M_ID);
  delay(200);
#endif
  Herkulex.reboot(M2_ID);
  delay(200);
  Herkulex.reboot(M3_ID);
  delay(200);
  Herkulex.initialize();  //initialize motors
  delay(500);

#if 0
  Serial.print("AccelRatio:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 8));
  Serial.print("MaxAccel:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 9));
  Serial.print("DeadZone:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 10));

  Herkulex.writeRegistryRAM(M0_ID, 8, 5);
  Herkulex.writeRegistryRAM(M0_ID, 9, 45);
  Herkulex.writeRegistryRAM(M0_ID, 10, 0);

  Serial.print("AccelRatio:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 8));
  Serial.print("MaxAccel:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 9));
  Serial.print("DeadZone:");
  Serial.println(Herkulex.readRegistryRAM(M0_ID, 10));
#endif
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
#if (DUAL_SHOULDER == 1)
      Herkulex.setLed(M1M_ID, LED_BLUE);
#endif
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_R, HIGH);
      digitalWrite(PUMP_PIN, HIGH);
      blinkStatus = true;
    } else {
      Herkulex.setLed(M0_ID, 0);
      Herkulex.setLed(M1_ID, 0);
      Herkulex.setLed(M2_ID, 0);
      Herkulex.setLed(M3_ID, 0);
#if (DUAL_SHOULDER == 1)
      Herkulex.setLed(M1M_ID, 0);
#endif
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_R, LOW);
      digitalWrite(PUMP_PIN, LOW);
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

      if (motorID == 256) {
        Serial.println("Air Pump on/off");
        digitalWrite(PUMP_PIN, targetAngle);
      } else {
        Serial.println("Moving motor...");
#if (DUAL_SHOULDER == 1)
        if (motorID == M1_ID)
          Herkulex.moveOneAngle(M1M_ID, -targetAngle, 500, LED_RED);  //move motor with 500 speed
        if (motorID == M1M_ID)
          Herkulex.moveOneAngle(M1_ID, -targetAngle, 500, LED_RED);  //move motor with 500 speed
#endif
        Herkulex.moveOneAngle(motorID, targetAngle, 500, LED_RED);  //move motor with 500 speed
        delay(1200);
        Serial.print("Get servo Angle:");
        Serial.print(Herkulex.getAngle(M0_ID));
        Serial.print(":");
        Serial.print(Herkulex.getAngle(M1_ID));
        Serial.print(":");
#if (DUAL_SHOULDER == 1)
        Serial.print(Herkulex.getAngle(M1M_ID));
        Serial.print(":");
#endif
        Serial.print(Herkulex.getAngle(M2_ID));
        Serial.print(":");
        Serial.println(Herkulex.getAngle(M3_ID));
      }
    }
  }
}
