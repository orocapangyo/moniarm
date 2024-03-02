/*
 *
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
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <Herkulex.h>

#include "songlcdled.h"

#define DOMAINID 108

rcl_subscription_t motorSub, ledSub, songSub, lcdSub;
std_msgs__msg__Int32 ledMsg, songMsg, lcdMsg;
std_msgs__msg__Int32MultiArray motorMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define M0_ID 6
#define M1_ID 7
#define M2_ID 8

#define RXD1 15
#define TXD1 23

#define RXD2 16
#define TXD2 17

#define DEBUG 1
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define INTERVAL 50
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


// Take the velocity command as input and calculate the PWM values.
void motor_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  int angle0, angle1, angle2, angle3;

  angle0 = msg->data.data[0];
  angle1 = msg->data.data[1];
  angle2 = msg->data.data[2];

  Herkulex.moveOneAngle(M0_ID, angle0, 500, LED_RED);
  delay(10);
  Herkulex.moveOneAngle(M1_ID, angle1, 500, LED_RED);
  delay(10);
  Herkulex.moveOneAngle(M2_ID, angle2, 500, LED_RED);
  //DEBUG_PRINT("M0:");
  //DEBUG_PRINT(angle0);
  //DEBUG_PRINT(" ,M1:");
  //DEBUG_PRINT(angle1);
  //DEBUG_PRINT(" ,M2:");
  //DEBUG_PRINTLN(angle2);
}

void subled_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  RGB(int(msg->data));
}

void subsong_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  playsong(int(msg->data));
}

void sublcd_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  showAnimation(int(msg->data));
}

void setup() {
  int i;

#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("Enc/Motor/MPU6050/Other Starts");

  Herkulex.beginSerial1(115200, RXD1, TXD1);  //open serial1 for motor communication

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  RGB(ALL_OFF);  // RGB LED all off

  ledcSetup(BUZZER, 5000, 8);  //enB, channel: 1, 5000Hz, 8bits = 256(0 ~ 255)
  ledcAttachPin(BUZZER, BUZZER_CH);
  ledcWrite(BUZZER, 0);

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  beginLcd();

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");
  set_microros_transports();

  Herkulex.reboot(M0_ID);
  Herkulex.reboot(M1_ID);
  Herkulex.reboot(M2_ID);
  delay(500);
  Herkulex.initialize();  //initialize motors
  delay(1500);

  DEBUG_PRINT("M0:");
  DEBUG_PRINT(Herkulex.getAngle(M0_ID));
  DEBUG_PRINT(" ,M1:");
  DEBUG_PRINT(Herkulex.getAngle(M1_ID));
  DEBUG_PRINT(" ,M2:");
  DEBUG_PRINTLN(Herkulex.getAngle(M2_ID));

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

  // Init the memory of your array in order to provide it to the executor.
  // If a message from ROS comes and it is bigger than this, it will be ignored, so ensure that capacities here are big enought.
  motorMsg.data.capacity = 4;
  motorMsg.data.size = 0;
  motorMsg.data.data = (int32_t *)malloc(motorMsg.data.capacity * sizeof(int32_t));

  motorMsg.layout.dim.capacity = 4;
  motorMsg.layout.dim.size = 0;
  motorMsg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(motorMsg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < motorMsg.layout.dim.capacity; i++) {
    motorMsg.layout.dim.data[i].label.capacity = 4;
    motorMsg.layout.dim.data[i].label.size = 0;
    motorMsg.layout.dim.data[i].label.data = (char *)malloc(motorMsg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, DOMAINID);
  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  DEBUG_PRINTLN("rclc_support_init done");

  RCCHECK(rclc_node_init_default(&node, "uros_arduino_node", "", &support));
  DEBUG_PRINTLN("rclc_node_init done");

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &ledSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ledSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &songSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "songSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &lcdSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lcdSub"));

  // create motor control subscriber
  RCCHECK(rclc_subscription_init_default(
    &motorSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motorSub"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &ledSub, &ledMsg, &subled_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &songSub, &songMsg, &subsong_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lcdSub, &lcdMsg, &sublcd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motorSub, &motorMsg, &motor_callback, ON_NEW_DATA));

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
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
