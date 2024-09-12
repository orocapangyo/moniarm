/*
# Main ROS2 control,DIY Robot ARM with DRS0101 smart motor
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
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <moniarm_interfaces/msg/cmd_motor.h>
#include <moniarm_interfaces/srv/init.h>
#include <moniarm_interfaces/srv/play_ani.h>
#include <moniarm_interfaces/srv/play_song.h>
#include <moniarm_interfaces/srv/set_led.h>

#include <Herkulex.h>

#include "songlcdled.h"

#define ESP_API_3 1
#define DOMAINID 11
#define DUAL_SHOULDER 0

moniarm_interfaces__msg__CmdMotor motorMsg, angleMsg;
moniarm_interfaces__srv__SetLED_Request req_led;
moniarm_interfaces__srv__SetLED_Response res_led;
moniarm_interfaces__srv__PlaySong_Request req_song;
moniarm_interfaces__srv__PlaySong_Response res_song;
moniarm_interfaces__srv__PlayAni_Request req_ani;
moniarm_interfaces__srv__PlayAni_Response res_ani;
moniarm_interfaces__srv__Init_Request req_init;
moniarm_interfaces__srv__Init_Response res_init;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_service_t service_led, service_song, service_ani, service_init;
rcl_subscription_t uros_subscriber;
rcl_publisher_t uros_publisher;
rcl_timer_t timer;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define MOTOR_NOMOVE 360
#define MOTOR_TOQOFF 720
#define MOTOR_TOQON 540
#define MAX_MOVE_TIME 1500

#define M0_ID 1
#define M1_ID 2
#define M2_ID 3
#define M3_ID 4
#define M1M_ID 0

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

#define AIRPUMP 0
#define GRIPPER 1
#define END_EFFCTOR AIRPUMP

#define PUMP_PIN 4

#define DEBUG 1
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define MOTOR0_MIN -60
#define MOTOR0_MAX 60

#define PRINT_ANGLE 1
#define INTERVAL 100

const unsigned int timer_timeout = 200;

bool blinkStatus = false;
long previousMillis = 0;
long currentMillis = 0;

// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)


void motorMoving(int mid, int tarAngle) {
  int moveAngle = 0, moveTime = 0, curAngle = 0;
  int timeFactor = 30;

  //filter out not moving case at first
  if (tarAngle == MOTOR_NOMOVE)
    return;
  else if (tarAngle == MOTOR_TOQOFF) {
    Herkulex.torqueOFF(mid);
#if (DUAL_SHOULDER == 1)
    if (mid == M1_ID)
      Herkulex.torqueOFF(M1M_ID);
#endif
    return;
  } else if (tarAngle == MOTOR_TOQON) {
    Herkulex.torqueON(mid);
#if (DUAL_SHOULDER == 1)
    if (mid == M1_ID)
      Herkulex.torqueON(M1M_ID);
#endif
    return;
  }
  //calculate moving time at first, should be enough for smooth operation
  else {
    //Herkulex.torqueON(mid);
    curAngle = int(Herkulex.getAngle(mid));

    // 180<tarAngle or -180>tarAngle, relative move
    if (tarAngle > 180) {
      tarAngle -= 180;
      moveAngle = tarAngle;
      tarAngle = curAngle + moveAngle;
      timeFactor = 15;
    } else if (tarAngle < -180) {
      tarAngle += 180;
      moveAngle = tarAngle;
      tarAngle = curAngle + moveAngle;
      timeFactor = 15;
    } else {
      moveAngle = tarAngle - curAngle;
    }

    //dont' need to move
    if (moveAngle == 0)
      return;
    //limit to MOTOR0_MIN/MOTOR0_MAX, since it can move relative angle
    else if (mid == M0_ID) {
      if (tarAngle > MOTOR0_MAX)
        tarAngle = MOTOR0_MAX;
      else if (tarAngle < MOTOR0_MIN)
        tarAngle = MOTOR0_MIN;
    }

    moveTime = abs(moveAngle) * timeFactor;

    if (moveTime > MAX_MOVE_TIME)
      moveTime = MAX_MOVE_TIME;

#if (PRINT_ANGLE == 1)
    DEBUG_PRINT("M:");
    DEBUG_PRINT(mid);
    DEBUG_PRINT(", Cur:");
    DEBUG_PRINT(curAngle);
    DEBUG_PRINT(" ,Move:");
    DEBUG_PRINT(moveAngle);
    DEBUG_PRINT(" ,Time:");
    DEBUG_PRINTLN(moveTime);
#endif

    Herkulex.moveOneAngle(mid, tarAngle, moveTime, LED_GREEN);
    //time for command sending, 10ms
    delay(10);
  }
}

// Take the angle array, then move each motor
void motor_callback(const void *msgin) {
  const moniarm_interfaces__msg__CmdMotor *msg = (const moniarm_interfaces__msg__CmdMotor *)msgin;
  int angle0, angle1, angle2, angle3, grip;

  angle0 = msg->angle0;
  angle1 = msg->angle1;
  angle2 = msg->angle2;
  angle3 = msg->angle3;
  grip = msg->grip;

  motorMoving(M0_ID, angle0);
  motorMoving(M1_ID, angle1);
  motorMoving(M2_ID, angle2);
  motorMoving(M3_ID, angle3);

#if (DUAL_SHOULDER == 1)
  motorMoving(M1M_ID, -angle1);
#endif

#if END_EFFCTOR == AIRPUMP
  digitalWrite(PUMP_PIN, grip);
#endif
}

void initMotors(void) {
  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
  Herkulex.reboot(M2_ID);
  delay(200);
  Herkulex.reboot(M3_ID);
  delay(200);
#if (DUAL_SHOULDER == 1)
  Herkulex.reboot(M1M_ID);
  delay(200);
#endif
  Herkulex.initialize();  //initialize motors
  delay(200);

#if 0
  //change accel profile, default 25 -> 5
  Herkulex.writeRegistryRAM(M0_ID, 8 , 5);
  Herkulex.writeRegistryRAM(M1_ID, 8 , 5);
  Herkulex.writeRegistryRAM(M2_ID, 8 , 5);
  Herkulex.writeRegistryRAM(M3_ID, 8 , 5);
  //change accel profile, default 45 -> 0
  Herkulex.writeRegistryRAM(M0_ID, 9 , 0);
  Herkulex.writeRegistryRAM(M1_ID, 9 , 0);
  Herkulex.writeRegistryRAM(M2_ID, 9 , 0);
  Herkulex.writeRegistryRAM(M3_ID, 9 , 0);
#endif
}

void init_callback(const void *req, void *res) {
  int index;
  moniarm_interfaces__srv__PlaySong_Request *req_in = (moniarm_interfaces__srv__PlaySong_Request *)req;
  moniarm_interfaces__srv__PlaySong_Response *res_in = (moniarm_interfaces__srv__PlaySong_Response *)res;

  index = (int)(req_in->index);
  //initialize
  if (index == 1) {
    Herkulex.torqueOFF(BROADCAST_ID);
  } else if (index == 2) {
    Herkulex.torqueON(BROADCAST_ID);
  } else {
    initMotors();
  }

  res_in->success = true;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

  angleMsg.angle0 = int(Herkulex.getAngle(M0_ID));
  angleMsg.angle1 = int(Herkulex.getAngle(M1_ID));
  angleMsg.angle2 = int(Herkulex.getAngle(M2_ID));
  angleMsg.angle3 = int(Herkulex.getAngle(M3_ID));

  RCLC_UNUSED(last_call_time);
  RCSOFTCHECK(rcl_publish(&uros_publisher, &angleMsg, NULL));
  return;
}

void setup() {
  int i;

#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("Micro ROS Starts");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  RGB(ALL_OFF);  // RGB LED all off

#if (ESP_API_3 == 1)
  ledcAttachChannel(BUZZER, 5000, 8, BUZZER_CH);
#else
  ledcSetup(BUZZER, 5000, 8);  //BUZZER, channel: 2, 5000Hz, 8bits = 256(0 ~ 255)
  ledcAttachPin(BUZZER, BUZZER_CH);
#endif
  ledcWrite(BUZZER_CH, 0);

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  beginLcd();

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");
  set_microros_transports();

  initMotors();
  Herkulex.torqueOFF(BROADCAST_ID);

#if END_EFFCTOR == AIRPUMP
  // configure AIRPUMP
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, 0);
#endif

  //wait agent comes up
  do {
    EXECUTE_EVERY_N_MS(300, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(300);

    if ((i++ % 32) == 0) DEBUG_PRINT("\n");
    else DEBUG_PRINT(".");
    if (state == AGENT_AVAILABLE)
      break;
  } while (1);

  Herkulex.torqueON(M0_ID);
  Herkulex.torqueON(M1_ID);
  Herkulex.torqueON(M2_ID);
  Herkulex.torqueON(M3_ID);

#if (DUAL_SHOULDER == 1)
  Herkulex.torqueON(M1M_ID);
#endif

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, DOMAINID);
  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  DEBUG_PRINTLN("rclc_support_init done");

  RCCHECK(rclc_node_init_default(&node, "uros_arduino_node", "", &support));
  DEBUG_PRINTLN("rclc_node_init done");

  // create service
  RCCHECK(rclc_service_init_default(&service_led, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, SetLED), "/SetLED"));
  // create service
  RCCHECK(rclc_service_init_default(&service_song, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, PlaySong), "/PlaySong"));
  // create service
  RCCHECK(rclc_service_init_default(&service_ani, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, PlayAni), "/PlayAni"));
  // create service
  RCCHECK(rclc_service_init_default(&service_init, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, Init), "/Init"));

  // create timer service
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create motor control subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &uros_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(moniarm_interfaces, msg, CmdMotor), "cmd_motor"));

  // create motor angle publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &uros_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(moniarm_interfaces, msg, CmdMotor), "angle_motor"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  // add subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &uros_subscriber, &motorMsg, &motor_callback, ON_NEW_DATA));
  // add timer
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // add services
  RCCHECK(rclc_executor_add_service(&executor, &service_led, &req_led, &res_led, led_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_song, &req_song, &res_song, song_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_ani, &req_ani, &res_ani, ani_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_init, &req_init, &res_init, init_callback));

  DEBUG_PRINTLN("ROS established");
  DEBUG_PRINTLN("Done setup");
}

void loop() {
  switch (state) {
    case AGENT_AVAILABLE:
      //if setup done, always here. then go to next step
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      //if ping doesn't work, then reset board(=wait agent comes up)
      ESP.restart();
      break;
    default:
      break;
  }

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
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
      blinkStatus = true;
    } else {
      Herkulex.setLed(M0_ID, 0);
      Herkulex.setLed(M1_ID, 0);
      Herkulex.setLed(M2_ID, 0);
      Herkulex.setLed(M3_ID, 0);
#if (DUAL_SHOULDER == 1)
    Herkulex.setLed(M1M_ID, 0);
#endif
      blinkStatus = false;
    }

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
