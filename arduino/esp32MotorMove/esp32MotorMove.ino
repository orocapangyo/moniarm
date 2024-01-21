#include <Herkulex.h>
int n=1;  //motor ID - verify your ID !!!!
          //this code is for motor ID 1,2,3

void setup() {
  delay(2000);           //a delay to have time for serial monitor opening
  Serial.begin(115200);  // Open serial communications
  Serial.println("Begin");
#if 0
  Herkulex.beginSerial1(115200);  //open serial port 1
#else
  Herkulex.beginSerial1(115200, 16, 17);  //open serial port 1
#endif
  Herkulex.reboot(1);
  Herkulex.reboot(2);
  Herkulex.reboot(3);
  delay(500);
  Herkulex.initialize();  //initialize motors
  delay(200);
}

void loop() {

  Serial.println("Move Angle: -20 degrees");
  Herkulex.moveOneAngle(n, -20, 500, LED_RED);  //move motor with 500 speed
  delay(1200);
  Serial.print("Get servo Angle:");
  Serial.println(Herkulex.getAngle(n));
  Serial.println("Move Angle: 20 degrees");
  Herkulex.moveOneAngle(n, 20, 500, LED_RED);  //move motor with 500 speed
  delay(1200);
  Serial.print("Get servo Angle:");
  Serial.println(Herkulex.getAngle(n));

  if (++n> 3) n = 1;
}
