#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <moniarm_interfaces/msg/arm3dof.h>
#include <moniarm_interfaces/srv/get_status.h>

rcl_subscription_t subscriber;
//std_msgs__msg__Int32 msg;
moniarm_interfaces__msg__Arm3dof msg;
rclc_executor_t executor;
rclc_executor_t executor2;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;



rcl_service_t service;
//rcl_wait_set_t wait_t


moniarm_interfaces__srv__GetStatus_Request req;
moniarm_interfaces__srv__GetStatus_Response res;

int flag;


#define DOMAINID 103

int led_status;
int led_success;

#define LED_PIN LED_BUILTIN //NodeMCU

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void service_callback(const void * req, void * res) {


  moniarm_interfaces__srv__GetStatus_Request * req_in = (moniarm_interfaces__srv__GetStatus_Request*) req;
  moniarm_interfaces__srv__GetStatus_Response * res_in = (moniarm_interfaces__srv__GetStatus_Response*) res;


  res_in->status = led_status;
  res_in->success = led_success;
}

void subscription_callback(const void * msgin)
{  
   
  const moniarm_interfaces__msg__Arm3dof * arm3dof_msg = ( const moniarm_interfaces__msg__Arm3dof *) msgin;

  led_status = arm3dof_msg->ang0;
  led_success =  arm3dof_msg->ang1;
  digitalWrite(LED_PIN, (led_status== 0) ? LOW : HIGH);  


}

void setup() {
  delay(1000);
  Serial.begin(115200);


  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  
  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, DOMAINID);
  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));



  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_moniarm_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(moniarm_interfaces, msg, Arm3dof),
    "arm3dof"));


  // create service
  RCCHECK(rclc_service_init_default(
    &service, 
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(moniarm_interfaces, srv, GetStatus),
    "get_status"))

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&executor2, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor2, &service, &req, &res, service_callback));

  led_status = 0;
  led_success = 0;

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor2, RCL_MS_TO_NS(100)));
}
