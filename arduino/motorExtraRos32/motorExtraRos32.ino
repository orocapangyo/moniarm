/*
 * Main ROS2 control,DIY Robot ARM with DRS0101 smart motor
 * ZETA7, zeta0707@gmail.com
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

rcl_subscription_t motorSub;
std_msgs__msg__Int32MultiArray motorMsg;

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

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define MOTOR_NOMOVE 360
#define MOTOR_TOQOFF 720
#define MOTOR_TOQON  540

#define MAX_MOVE_TIME 1500

#define M0_ID 6
#define M1_ID 7
#define M2_ID 8
#define M3_ID 15

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

#define PRINT_ANGLE 1
#define INTERVAL 100

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

  //don't move angle input
  if (tarAngle == MOTOR_NOMOVE)
    return;
  else if (tarAngle == MOTOR_TOQOFF) {
    Herkulex.torqueOFF(mid);
    return;
  }
  else if (tarAngle == MOTOR_TOQON) {
    Herkulex.torqueON(mid);
    return;
  }
  //calculate moving time at first, should be enough for smooth operation
  else {
    Herkulex.torqueON(mid);
    curAngle = int(Herkulex.getAngle(mid) + 0.5);
    moveAngle = abs(tarAngle - curAngle);
    //0, 1 degree, don't moave
    if (moveAngle < 2)
      return;

    moveTime = moveAngle * 30;

    if (moveTime > MAX_MOVE_TIME)
      moveTime = MAX_MOVE_TIME;

#if (PRINT_ANGLE == 1)
    DEBUG_PRINT("M:");
    DEBUG_PRINT(mid);
    DEBUG_PRINT(" ,Angle:");
    DEBUG_PRINT(tarAngle);
    DEBUG_PRINT(" ,MovingTime:");
    DEBUG_PRINTLN(moveTime);
#endif
    Herkulex.moveOneAngle(mid, tarAngle, moveTime, LED_RED);
  }
}

// Take the angle array, then move each motor
void motor_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  int angle0, angle1, angle2, angle3;

  angle0 = msg->data.data[0];
  angle1 = msg->data.data[1];
  angle2 = msg->data.data[2];
  angle3 = msg->data.data[3];

  motorMoving(M0_ID, angle0);
  motorMoving(M1_ID, angle1);
  motorMoving(M2_ID, angle2);
  motorMoving(M3_ID, angle3);
}

void init_callback(const void *req, void *res) {
  moniarm_interfaces__srv__Init_Response *res_in = (moniarm_interfaces__srv__Init_Response *)res;

  Herkulex.reboot(M0_ID);
  delay(200);
  Herkulex.reboot(M1_ID);
  delay(200);
  Herkulex.reboot(M2_ID);
  delay(200);
  Herkulex.reboot(M3_ID);
  delay(200);
  Herkulex.initialize();  //initialize motors
  delay(200);

  res_in->success = true;
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

  ledcSetup(BUZZER, 5000, 8);  //BUZZER, channel: 2, 5000Hz, 8bits = 256(0 ~ 255)
  ledcAttachPin(BUZZER, BUZZER_CH);
  ledcWrite(BUZZER_CH, 0);

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  beginLcd();

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");
  set_microros_transports();

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

  Herkulex.torqueON(BROADCAST_ID);
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

  // create service
  RCCHECK(rclc_service_init_default(&service_led, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, SetLED), "/SetLED"));
  // create service
  RCCHECK(rclc_service_init_default(&service_song, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, PlaySong), "/PlaySong"));
  // create service
  RCCHECK(rclc_service_init_default(&service_ani, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, PlayAni), "/PlayAni"));
  // create service
  RCCHECK(rclc_service_init_default(&service_init, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, Init), "/Init"));

  // create motor control subscriber
  RCCHECK(rclc_subscription_init_default(
    &motorSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "cmd_motor"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  // topic subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &motorSub, &motorMsg, &motor_callback, ON_NEW_DATA));
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
      blinkStatus = true;
    } else {
      Herkulex.setLed(M0_ID, 0);
      Herkulex.setLed(M1_ID, 0);
      Herkulex.setLed(M2_ID, 0);
      Herkulex.setLed(M3_ID, 0);
      blinkStatus = false;
    }

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
