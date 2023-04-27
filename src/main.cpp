#include "MPU9250.h"
#include "SoftwareSerial.h"
#include <Wire.h>
//#include "mpu9250_gy271.h"
#include "math.h"
#include <esp32-hal-timer.h>
#include <esp32-hal-gpio.h>
#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include "PID.h"
#include "BluetoothSerial.h"
#include "string.h"
//----Define devices and function for drone-----
#define Due_core_MCU
//#define NRF24 // dung nrf 24 de truyen nhan du lieu rada
//#define TEST_REAL use blutooth serial
//#define Auto_calib_IMU
//-------------MPU9250----------------------
#define pi 3.141592653589793238462
// create timer interrpt to receive request may be angle roll pitch yaw
#define TIMER1_INTERVAL_US 200000 // 0.2 seconds
// Timer interrup interval to PID 
#define TIMER2_INTERVAL_US 10000 // 0.01 seconds
//Configuration Low Pass Filte for mpu6050 in header file mpu9250.h
//------drone's parameters---
double L = 0.16;//length from motor center to center of gravity
double KmT = 0.02;//propeller parameters
//-------------INit use in madgwick ---------
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float a12, a22, a31, a32, a33;
extern volatile float roll,yaw,pitch;
//------------INIT time to caculate angle -------------
uint32_t newTime{0}, oldTime{0};
extern float deltaT;

//test time program
uint32_t newTime1{0}, oldTime1{0};
float deltaT1;
// ----------- Using the Due Core -----------
TaskHandle_t Task1;
TaskHandle_t Task2;
//---------------end config-----------------
//-------------Init for interrupt timer--------------------
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool Flag_PID_Interrupted = false;
volatile bool Flag_NRF_Interrupted = false;
//struct MPU9250
MPU9250 mpu;
//-------------NRF-----------------
//-------------PWM----------------------
// Define the PWM channels and pins
// Set the frequency and resolution of the PWM signal
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16


#define PWM_CHANNEL_1 0
#define PWM_PIN_1 32
#define PWM_CHANNEL_2 1
#define PWM_PIN_2 33
#define PWM_CHANNEL_3 2
#define PWM_PIN_3 25
#define PWM_CHANNEL_4 3
#define PWM_PIN_4 26

volatile float pwm1,pwm2,pwm3,pwm4;
const int pwm_min = 3276;
const int pwm_max = 6552;
//------------END init PWM-------
#ifdef NRF24
//------------INIT nrf24----------
// instantiate an object for the nRF24L01 receiver
RF24 radio(19, 18);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role
float payload[4];
//------------END init NRF -----------
#endif
//------------Init for blutooth---------
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//------------END Init for blutooth---------
//------------PID Init-------------
float deltaT_PID = (float)TIMER2_INTERVAL_US/1000000.0;
PID_t pid_pitch, pid_roll, pid_yaw, pid_altitude;
//END---------PID-------------/
//-------define function ------- 
void print_roll_pitch_yaw();
void update_quaternion (void);
void IRAM_ATTR onTimer1();
void IRAM_ATTR onTimer2();
void setup_pwm(void);
void set_pwm_to_motor(void);
void pwm_caculate (int8_t pitch, int8_t roll,int8_t yaw, int8_t altitude);
void cali_motor(void);
#ifdef Due_core_MCU
  void Task1code( void * pvParameters );
  void Task2code( void * pvParameters );
#endif
//----end define function ------- 
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Serial.print(SCL);
    //---------PID-------------
  setK(01,0.21,-0.18,&pid_pitch);
	setK(0,0,0,&pid_roll);
	setK(0,0,0,&pid_yaw);
	setK(0,0,0,&pid_altitude);
    //END---------PID-------------
    //start Bluetooth
     SerialBT.begin("ESP32test"); //Bluetooth device name
     Serial.println("The device started, now you can pair it with bluetooth!");
    //---end pwm init ------
    pinMode(2,OUTPUT);
    // --------Set up and calibration MPU6050 and mag GY-271---------
    if (!mpu.mysetup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    
    //----pwm setup -------
    ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION); 
    ledcAttachPin(PWM_PIN_1, PWM_CHANNEL_1);
    ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION); 
    ledcAttachPin(PWM_PIN_2, PWM_CHANNEL_2);
    ledcSetup(PWM_CHANNEL_3, PWM_FREQUENCY, PWM_RESOLUTION); 
    ledcAttachPin(PWM_PIN_3, PWM_CHANNEL_3);
    ledcSetup(PWM_CHANNEL_4, PWM_FREQUENCY, PWM_RESOLUTION); 
    ledcAttachPin(PWM_PIN_4, PWM_CHANNEL_4);

#ifdef AUTO_calib_IMU // need edit again 
    //cali_motor();
#endif
    // setup_pwm();
//
#ifdef NRF24
    //---------setup for radio operation----------------
    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
      while (1) {}  // hold in infinite loop
    }
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
    // additional setup specific to the node's role
    if (role) {
      radio.stopListening();  // put radio in TX mode
    } else {
      radio.startListening();  // put radio in RX mode
    }
    //--------END setup for radio----------------

    //-----setup timer0 to interrupt T=S to receiver request status of drone--------------
    timer1 = timerBegin(0, 256, true); // timer 0, prescaler 80, count up
    timerAttachInterrupt(timer1, &onTimer1, true);
    timerAlarmWrite(timer1, TIMER1_INTERVAL_US, true);
    timerAlarmEnable(timer1);
#endif
    //-----setup timer1 to interrupt T=S for sample time of PID --------------
    timer2 = timerBegin(1, 160, true); // timer 1, prescaler 160, count up
    timerAttachInterrupt(timer2, &onTimer2, true);
    timerAlarmWrite(timer2, TIMER2_INTERVAL_US, true);
    timerAlarmEnable(timer2); 
#ifdef Due_core_MCU
    //----Config due core
    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                      Task1code,   /* Task function. */
                      "Task1",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      1,           /* priority of the task */
                      &Task1,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */                  
    delay(500); 
    //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
                      Task2code,   /* Task function. */
                      "Task2",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      1,           /* priority of the task */
                      &Task2,      /* Task handle to keep track of created task */
                      1);          /* pin task to core 1 */
      delay(500);
    //-----END cf due core---
#endif
}

#ifdef Due_core_MCU
//Task1code: use to caculate angle and use blutooth to send data to pc
void Task1code( void * pvParameters ){
  //time of loop
  for(;;){
    newTime = micros();
    deltaT = newTime - oldTime;
    oldTime = newTime;
    deltaT = deltaT * 0.001 * 0.001;
    //update value ax,ay,az,gx,gy,gz,mx,my,mz
    mpu.Process_IMU();

    //update angle yall pitch roll
    update_quaternion();
    //Print yaw pitch roll to pc via bluetooth
    char str[30]={};
    sprintf(str, "%.2f,%.2f,%.2f,%.5f\n",yaw,pitch,roll,deltaT );
    SerialBT.write((uint8_t*)str, strlen(str));
  }
}

//Task2code: us to caculate PID and receiver data Blutooth from PC
void Task2code( void * pvParameters ){
  for(;;){
    if(Flag_PID_Interrupted == true){
      get_out(&pid_pitch,pid_pitch.set_point,pitch);
      get_out(&pid_roll,pid_roll.set_point,roll);
      pwm_caculate (pid_pitch.u, 0,0,0);//pid_roll.u,pid_yaw.u, pid_altitude.u);
      set_pwm_to_motor(); 
      Flag_PID_Interrupted = false;
    }
  }
}
void loop(){}
#else
void loop()
{
    newTime = micros();
    deltaT = newTime - oldTime;
    oldTime = newTime;
    deltaT = deltaT * 0.001 * 0.001;
    //update value ax,ay,az,gx,gy,gz,mx,my,mz
    mpu.Process_IMU();

    //update angle yall pitch roll
    update_quaternion();
    //Print yaw pitch roll to pc via bluetooth
    char str[30]={};
    sprintf(str, "%.2f,%.2f,%.2f,%.5f\n",yaw,pitch,roll,deltaT );
    SerialBT.write((uint8_t*)str, strlen(str));

    if(Flag_PID_Interrupted == true){
      get_out(&pid_pitch,pid_pitch.set_point,pitch);
      get_out(&pid_roll,pid_roll.set_point,roll);
      pwm_caculate (pid_pitch.u, 0,0,0);//pid_roll.u,pid_yaw.u, pid_altitude.u);
      set_pwm_to_motor(); 
      Flag_PID_Interrupted = false;
  }
}      
#endif
//Interrupt function on timer 0 
void IRAM_ATTR onTimer1() {
// receiver data throught by NRF24
//payload[0] -> setpitch 
//payload[1] -> setroll
//payload[2] -> setyall 
//payload[3] -> setaltitude
#ifdef NRF24
      uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, sizeof(payload));             // fetch payload from FIFO
      Serial.print(payload[0]);  // print the payload's value
      Serial.print(payload[1]);
      Serial.println(payload[2]);
    }
    
    pid_pitch.set_point = payload[0];
    pid_pitch.set_point = payload[1];
    pid_pitch.set_point = payload[2];
    pid_pitch.set_point = payload[3];
#endif
  pid_pitch.set_point = 0;
  pid_roll.set_point = 0;
  pid_yaw.set_point = 0;
  pid_altitude.set_point = 0;

    // This device is a RX node
  //Serial.println("interrupted");
//
}
//interrup timer 2 to PID and write pwm to motor
void IRAM_ATTR onTimer2() {
  Flag_PID_Interrupted = true;
}

void pwm_caculate (int8_t pitch, int8_t roll,int8_t yaw, int8_t altitude){
  pwm1 = pwm_min+(+ pitch/(4*L) - roll/(4*L) - yaw/(4*KmT) + altitude/4)*0.063;
	pwm2 = pwm_min+(+ pitch/(4*L) + roll/(4*L) + yaw/(4*KmT) + altitude/4)*0.063;
	pwm3 = pwm_min+(- pitch/(4*L) + roll/(4*L) - yaw/(4*KmT) + altitude/4)*0.063;
	pwm4 = pwm_min+(- pitch/(4*L) - roll/(4*L) + yaw/(4*KmT) + altitude/4)*0.063;
}

void set_pwm_to_motor(void){
    // Set the duty cycle of each PWM signal (0-255) if define 8 bit solution
  ledcWrite(PWM_CHANNEL_1, pwm1); 
  ledcWrite(PWM_CHANNEL_2, pwm2); 
  ledcWrite(PWM_CHANNEL_3, pwm3); 
  ledcWrite(PWM_CHANNEL_4, pwm4); 
}
void setup_pwm(void){
    // Configure the timers for PWM generation
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN_1, PWM_CHANNEL_1);
  
  ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN_2, PWM_CHANNEL_2);
  
  ledcSetup(PWM_CHANNEL_3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN_3, PWM_CHANNEL_3);
  
  ledcSetup(PWM_CHANNEL_4, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN_4, PWM_CHANNEL_4);
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

void cali_motor(void){
  ledcWrite(PWM_CHANNEL_1, pwm_max);
  ledcWrite(PWM_CHANNEL_2, pwm_max);
  ledcWrite(PWM_CHANNEL_3, pwm_max);
  ledcWrite(PWM_CHANNEL_4, pwm_max);

  delay(4000);

  ledcWrite(PWM_CHANNEL_1, pwm_min);
  ledcWrite(PWM_CHANNEL_2, pwm_min);
  ledcWrite(PWM_CHANNEL_3, pwm_min);
  ledcWrite(PWM_CHANNEL_4, pwm_min);
  delay(10000);
}