// ================================================================
// ===               MPU MODULE                ===
// ================================================================
/*Inclusión de librerpias y declaración de variables necesarias para MPU6050*/

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define LED_PIN 2 


bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    

uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container

// ================================================================
// ===               ROS MODULE                ===
// ================================================================
/*Inclusión de librerpias y declaración de variables necesarias para microROS*/
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_raw;
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_raw, NULL));
  }
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

// ================================================================
// ===                      MPU SETUP                       ===
// ================================================================
/*Configuracion e inicialización del mpu*/


    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(115200);

    mpu.initialize();

    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();


    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1788);


    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);

// ================================================================
// ===                      ros SETUP                       ===
// ================================================================

/*Esta parte del código configura el publicador, timer y ejecutor*/

  set_microros_transports();

  

  allocator = rcl_get_default_allocator();


  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));


  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));


  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //Si no hay datos en el mpu, no hace nada
    if (!dmpReady) return;


    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            // Se obtienen los quaternions desde el mpu
            mpu.dmpGetQuaternion(&q, fifoBuffer);

            //Se arma el mensaje para enviar los cuaternios
            imu_raw.header.frame_id.data = const_cast<char*>("imu_link");
            imu_raw.header.stamp.sec = millis() / 1000;
            imu_raw.header.stamp.nanosec = millis() * 1000000;
            imu_raw.orientation.w = q.w;
            imu_raw.orientation.x = q.x;
            imu_raw.orientation.y = q.y;
            imu_raw.orientation.z = q.z;
            //Se envia el mensaje
            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

    }
}
