
#include <SimpleFOC.h>

#include <string>
#include <ros.h>

#include <std_msgs/Float64.h>

#include "mpu.h"
#include "motors.h"

ros::NodeHandle nh;

TaskHandle_t doloopFOC;
void doloopFOCfun( void * pvParameters);
TaskHandle_t getAng;
void getAngfun( void * pvParameters);
TaskHandle_t mpuGetAng;
void mpuGetAngfun( void * pvParameters);
TaskHandle_t moveMotors;
void moveMotorsfun( void * pvParameters);
// TaskHandle_t serialRecieveUsr;
// void serialRecieveUsrfun( void * pvParameters);


//PID
PIDController balancePid(20,750,0.2,1000000,35);
PIDController posPID(.50,1,0.,10000,0.075);
PIDController velPID(.0072,0.018,0.,10000,0.075);



float pitch, target_motor_velocity,target_pitch,target_pos,target_velocity;

void usrcmd_cb( const std_msgs::Float64& cmd_msg){
  target_velocity = cmd_msg.data;  
}
ros::Subscriber<std_msgs::Float64> sub("usrcmd", usrcmd_cb);

void setup() {
    Serial.begin(115200);
    mpuSetup();
    focSetup();
    //Ros Setup
    nh.initNode();
    nh.subscribe(sub);
    //0.022;
    target_pos = 0;
    target_velocity = -5;
    disableCore0WDT();
    disableCore1WDT();

    xTaskCreatePinnedToCore(
                        doloopFOCfun,   /* Task function. */
                        "LoopFOC",     /* name of task. */
                        100000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        7,           /* priority of the task */
                        &doloopFOC,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */                  
    delay(500); 


    xTaskCreatePinnedToCore(
                        mpuGetAngfun,   /* Task function. */
                        "get mpu angle",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        10,           /* priority of the task */
                        &mpuGetAng,      /* Task handle to keep track of created task */
                        1);          /* pin task to core 1 */                  
    delay(500); 

    xTaskCreatePinnedToCore(
                        moveMotorsfun,   /* Task function. */
                        "move motors",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        5,           /* priority of the task */
                        &moveMotors,      /* Task handle to keep track of created task */
                        1);          /* pin task to core 1 */                  
    delay(500); 

    // xTaskCreatePinnedToCore(
    //                     serialRecieveUsrfun,   /* Task function. */
    //                     "recieve usr input",     /* name of task. */
    //                     10000,       /* Stack size of task */
    //                     NULL,        /* parameter of the task */
    //                     2,           /* priority of the task */
    //                     &serialRecieveUsr,      /* Task handle to keep track of created task */
    //                     1);          /* pin task to core 1 */                  
    // delay(500); 
}





void doloopFOCfun( void * pvParameters) {
    for(;;){
        //This function keeps motors spinning and must be run as fast as possible
        motor1.loopFOC();
        motor2.loopFOC();
    }
}


void mpuGetAngfun( void * pvParameters) {
    for(;;){
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch = ypr[2];
        }
        nh.spinOnce();
    }
}

void moveMotorsfun( void * pvParameters) {
    for(;;){
        target_pitch = velPID(target_velocity-(motor1.shaft_velocity+motor2.shaft_velocity)/2);
        target_motor_velocity = balancePid(pitch-target_pitch);
        //Serial.println(pitch);
        //motor2.monitor();
        motor1.move(target_motor_velocity);
        motor2.move(target_motor_velocity);
    }
}
/* For Testing 
void serialRecieveUsrfun(void * pvParameters){
    for(;;) {
  // a string to hold incoming data
  static String received_chars;
  
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      
      // change the motor target
      target_velocity = received_chars.toFloat();
      Serial.print("Target: ");
      Serial.println(target_velocity);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
  //motor2.monitor();
}
}
*/
 

void loop() {}