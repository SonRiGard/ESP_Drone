#include "MPU9250.h"
#include "SoftwareSerial.h"
#include <Wire.h>
//#include "mpu9250_gy271.h"
#include "math.h"
#include <esp32-hal-timer.h>
#include <esp32-hal-gpio.h>


#define pi 3.141592653589793238462
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float a12, a22, a31, a32, a33;
extern float roll,yaw,pitch;
extern float yaw1, yaw2, yaw3;

uint32_t newTime{0}, oldTime{0};
extern float deltaT;
//Init for timer
hw_timer_t * timer = NULL;
//#include "SoftwareSerial.h"
MPU9250 mpu;


void print_roll_pitch_yaw();

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(5000);

    // Set up and calibration MPU6050 and mag GY-271

    if (!mpu.mysetup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    //setup timer0 to interrupt T=0.1S to receiver request status of drone
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true); // 1 second timer
    timerAlarmEnable(timer);
}

void loop()
{
  //time of loop
  newTime = micros();
  deltaT = newTime - oldTime;
  oldTime = newTime;
  deltaT = deltaT * 0.001 * 0.001;

  //update value ax,ay,az,gx,gy,gz,mx,my,mz
  mpu.Process_IMU();

  //update angle yall pitch roll
  update_quaternion();

}      

//Interrupt funsion on timer 0 
void IRAM_ATTR onTimer() {
  // Your code here
  
}

void print_roll_pitch_yaw() {
    //Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);
}

void update_quaternion (void)
{
  q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        float sinp = a32;
        if (fabsf(sinp) >= 1)
        	pitch = copysign(M_PI/2,sinp);
        else
        	pitch =  asin(sinp);
        //pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        pitch *= 180.0f / pi;
        yaw = atan2f(sinf(yaw),cosf(yaw));
        yaw   *= 180.0f / pi;
       	roll  *= 180.0f / pi;
}