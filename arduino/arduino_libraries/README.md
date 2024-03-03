본 문서를 moniarm 프로젝트에서 ESP32에서 동작한 MoniArm Custom Message, Service을 포함하는 micro_ros_arudino library을 작성하는 방법에서 대해서 설명한다.


# micro-ROS micro_ros_arudino

 아래  micro_ros_arudino  의 아래를 참고해서 작성하였다.

- [micro-ROS/micro_ros_arduino: micro-ROS library for Arduino (github.com)](https://github.com/micro-ROS/micro_ros_arduino?tab=readme-ov-file#how-to-use-the-precompiled-library)

## micro-ROS micro_ros_arduino library 생성하기

1. Arduino IDE 의 library을 빌드하기 위해  micro_ros_arduino 소스 코스를 다운로드한다.
    1. 아래 git command을 수행하여 galactic  용 micro_ros_arduino을 소스코드 다운로드 한다.
    
    ```bash
    cd ~/ros2_ws/src/moniarm/arduino_libraries
    git clone -b galactic https://github.com/micro-ROS/micro_ros_arduino
    ```
    
2. micro_ros_arudino에  사용할 Custom Message 용 Package와 Message, Service 정의한다.
    1.   package 생성 없이 jetson의 moniarm_interfaces directory을 “micro_ros_arduino/extras/library_generation/extra_packages” 에 복사한다
        1. *별도의 interface package 정의 시 Jetson Node에 다른 package name으로 message, service 참고할 수 없음*
            
    ```bash
    cd ~/ros2_ws/src/moniarm/arduino_libraries/micro_ros_arduino/extras/library_generation/extra_packages
    cp -r ~/ros2_ws/src/moniarm/moniarm_interfaces .
    ```
        
3. docker 을 이용하여 micro_ros_arudino 을 빌드한다.
         
    ```bash
    cd ~/ros2_ws/src/moniarm/arduino_libraries
    docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:galactic -p esp32
     ```
        
4. 빌드된 micro_ros_arudino을 Arudino IDE의 Library로 복사한다.
    1. 방안#) Arudino IDE libraries에 바로 복사합니다.
        
    ```bash
    cd ~/ros2_ws/src/moniarm/arduino_libraries  
    cp -r micro_ros_arduino ~/Arduino/libraries
    ```
        
    2. 방안#2) 또는 arduino_libraries  을 zip로 압축하여  Arudino IDE 에 library에 등록합니다.
        1. zip 압축
        
        ```bash
        cd ~/ros2_ws/src/moniarm/arduino_libraries  
        zip micro_ros_arduino.zip micro_ros_arduino
        ```
        
        1. 압축된 micro_ros_arduino.zip 을   Arudino IDE에서  Sketch —> Include Library —> Add .ZIP Library 클릭하여 library 추가한다.
        
    
    ## MoniArm micro_ros_arudino 시험
    
    1. Arudino IDE 수행
    2. 파일 ~/ros2_ws/src/moni_arm/arduino/micro_ros_publisher/**[micro_ros_arm3dof_publisher.ino](https://github.com/orocapangyo/moniarm/blob/main/arduino/micro_ros_publisher/micro_ros_publisher.ino) 연다.**
    3.  “Upload”을 클릭합니다.
        1. micro_ros_publisher.ino 파일 변경된 내용은 아래 주황색 코드와 같습니다.
            1. moni arm용 message, service 헤더 파일 추가
            2. message Arm3dof 객체 선언
            3. "moniarm_arm3dof_publisher” publisher 생성
            
        ```bash
          ...

            #include <std_msgs/msg/int32.h>
            
            #include <moniarm_interfaces/msg/arm3dof.h>
            #include <moniarm_interfaces/srv/get_status.h>
            
            rcl_publisher_t publisher;

            ..
            }
            
            void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
            {  
              RCLC_UNUSED(last_call_time);
              if (timer != NULL) {
                RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                msg.data++;
                arm3dof.ang0++;
                arm3dof.ang1++;
                arm3dof.ang2++;
              }
            }
            
            void setup() {
             ...
            
              // create publisher
              // RCCHECK(rclc_publisher_init_default(
              //   &publisher,
              //   &node,
              //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
              //   "micro_ros_arduino_node_publisher"));
            
              RCCHECK(rclc_publisher_init_default(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(moniarm_interfaces, msg, Arm3dof),
                "moniarm_arm3dof_publisher"));
            
        ....
            
              msg.data = 0;
              arm3dof.ang0 = 0;
              arm3dof.ang1 = 0;
              arm3dof.ang2 = 0;
            }
            
            void loop() {
              delay(100);
              RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
            }
            
            ```
            
            1. 
            
            ![Untitled](https://prod-files-secure.s3.us-west-2.amazonaws.com/f6b68902-bc62-4ee6-a46f-32ed3421cddf/88ed4753-6aac-4ed3-8f38-85978e7fa4ad/Untitled.png)
            
        2. arm3dof publisher 을 생성한다.
    4. ros2 command로 node publisher을 확인합니다
            
        ```bash
        jhmoon0224@jhmoonnotebook:~$ ros2 topic list
        /moniarm_arm3dof_publisher
        /parameter_events
        /rosout
        jhmoon0224@jhmoonnotebook: ~$ ros2 topic echo /moniarm_arm3dof_publisher
        ang0: 32
        ang1: 1073460504
        ang2: 1061159836
        ---
        ang0: 33
        ang1: 1073460504
        ang2: 1061159836
        ---
        ang0: 34
        ang1: 1073460504
        ang2: 1061159836
        ---
        ang0: 35
        ang1: 1073460504
        ang2: 1061159836
        ---
        ang0: 36
        ang1: 1073460504
        ang2: 1061159836
        ---
        ang0: 37
        ang1: 1073460504
        ang2: 1061159836
        
        ```