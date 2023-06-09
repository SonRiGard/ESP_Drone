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
// --- Индексы  для углов ----
# define YAW         0
# define PITCH       1
# define ROLL        2

#define Due_core_MCU
//#define NRF24 // dung nrf 24 de truyen nhan du lieu rada
//#define TEST_REAL use blutooth serial
//#define Auto_calib_IMU
//-------------MPU9250----------------------
#define pi 3.141592653589793238462
// create timer interrpt to receive request may be angle roll pitch yaw
#define TIMER1_INTERVAL_US 200000 // 0.2 seconds
// Timer interrup interval to PID 
#define TIMER2_INTERVAL_US 20000 // 0.01 seconds
//пока тут пишем только для тангажа
float gyroz=0, gyroz_prev=0, omgz=0, omgz_prev=0;
//Configuration Low Pass Filte for mpu6050 in header file mpu9250.h
//------drone's parameters---
extern float Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
// заданное значение углов Эйлера [yaw, pitch, roll]
float thetaDesired[3]={0,0,0}, omgDesired[3]= {0,0,0};
// double L = 0.16;//length from motor center to center of gravity
// double KmT = 0.02;//propeller parameters
volatile int8_t flag_ON_OFF = 0;
volatile float Myaw, Mpitch=0, Mroll=0;
float Lq=0.18, kM=7.4E-7, kT=3.13E-5;
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


const int THROLLTE0 = 4000;
//const int THROTTLE_MAXIMUM = 6552;
const int THROTTLE_MAXIMUM = 4500;
const int THROTTLE_MINIMUM = 3276;

int PWM1, PWM2, PWM3, PWM4;

float a=1.18E-6, b=1.51E-3, c=-14.17E-3;
float TH1=0.0,TH2=0.0,TH3=0.0,TH4=0.0;
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

  volatile  uint32_t pretime_PID;
  volatile  uint32_t endtime_PID;
//END---------PID-------------/
//-------define function ------- 
void print_roll_pitch_yaw();
void update_quaternion (void);
void IRAM_ATTR onTimer1();
void IRAM_ATTR onTimer2();
void setup_pwm(void);
void cali_motor(void);
void printBT_roll_pitch_yaw();
volatile int U2PWM(float T);
float SATURATION(float x, float xmax, float xmin);

void GetErrs_Angle(PID_t *DataStruct);
void FindAngleRateDesired(PID_t *DataStruct,uint8_t ki);
float DigLowPassFil(float x_prev, float u, float u_prev);
float CONTROL(uint32_t tmr1);
float LMT(float x, float thrsd);

#ifdef Due_core_MCU
  void Task1code( void * pvParameters );
  void Task2code( void * pvParameters );
#endif
//----end define function ------- 
void setup() {

    Serial.begin(115200);
        Wire.begin();
    //start Bluetooth
    // while(Serial.read() != '1');
    // flag_ON_OFF=1;

    SerialBT.begin("ESP32test"); //Bluetooth device name
    char str[30]={"the device started"};
    SerialBT.write((uint8_t*)str, strlen(str));




    //---------PID-------------
    delay(2000);
    setK(0.1,0.2,0.02,&pid_pitch);
    parameter_calculation(&pid_pitch);
    // setK(0,0,0,&pid_roll);
    // parameter_calculation(&pid_roll);
    // setK(0,0,0,&pid_yaw);
    // parameter_calculation(&pid_yaw);
    // setK(0,0,0,&pid_altitude);
    // parameter_calculation(&pid_altitude);
    //END---------PID-------------

    //---end pwm init ------
    pinMode(2,OUTPUT);
    // --------Set up and calibration MPU6050 and mag GY-271---------
    if (!mpu.mysetup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    Serial.println("Waiting for 30s ...");
    while(millis()<50000){
      newTime = micros();
      deltaT = newTime - oldTime;
      oldTime = newTime;
      deltaT = deltaT * 0.001 * 0.001;
      //update value ax,ay,az,gx,gy,gz,mx,my,mz
      mpu.Process_IMU();

      //update angle yall pitch roll
      update_quaternion();

      printBT_roll_pitch_yaw();
      //Print yaw pitch roll to pc via bluetooth
    }
    Serial.println("Setup done");

    //----motors setup ----------
    setup_pwm();
    //cali_motor();
    ledcWrite(PWM_CHANNEL_1, 3276);
    ledcWrite(PWM_CHANNEL_2, 3276);
    ledcWrite(PWM_CHANNEL_3, 3276);
    ledcWrite(PWM_CHANNEL_4, 3276);
    delay(5000);
    // ledcWrite(PWM_CHANNEL_1, 3476);
    // delay(3000);
    //  ledcWrite(PWM_CHANNEL_2, 3476);
    // delay(3000);
    //  ledcWrite(PWM_CHANNEL_3, 3476);
    // delay(3000);
    //  ledcWrite(PWM_CHANNEL_4, 3476);
    // delay(3000);
    //----END motors setup -------

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

    //-----setup timer1 to interrupt T=S for sample time of PID--------------
    timer2 = timerBegin(1, 80, true); // timer 1, prescaler 160, count up
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
      Serial.println("Setup done");
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

    printBT_roll_pitch_yaw();
    //Print yaw pitch roll to pc via bluetooth

  }
}

//Task2code: us to caculate PID and receiver data Blutooth from PC
void Task2code( void * pvParameters ){
  uint8_t running_flag = 0;
  for(;;){
      char cmd = Serial.read();
      if (cmd == '1') {
        running_flag = 1;
      } else if (cmd == '2') {
        running_flag = 0;
        ledcWrite(PWM_CHANNEL_1, THROTTLE_MINIMUM);
        ledcWrite(PWM_CHANNEL_2, THROTTLE_MINIMUM);
        ledcWrite(PWM_CHANNEL_3, THROTTLE_MINIMUM);
        ledcWrite(PWM_CHANNEL_4, THROTTLE_MINIMUM);
      }
      if (running_flag == 0) {
        continue;
      }
//    if(flag_ON_OFF == 1){
      if(Flag_PID_Interrupted == true)
      {/*       
        pretime_PID = micros();
        // Serial.println(millis());
        // Serial.print(pitch);Serial.print(",");
        // Serial.print(pid_pitch.set_point);Serial.print(",");
        // Serial.print(pid_pitch.u);Serial.print(",");
        // Serial.print(pid_pitch.e);Serial.print(",");
        //get_out(&pid_pitch,pid_pitch.set_point,pitch);
//        if(pitch>45){flag_ON_OFF=0;};
        //Serial.print(pid_pitch.u);Serial.print(",");
        // get_out(&pid_roll,pid_roll.set_point,roll);  
        // get_out(&pid_yaw,pid_yaw.set_point,yaw);      

        
        //Mpitch = SATURATION(pid_pitch.u, 3.0, -3.0);
        //Mroll  = SATURATION(pid_roll.u, 3.0, -3.0);
        //Myaw = SATURATION(pid_yaw.u, 3.0, -3.0);
        pid_yaw.u=0;

        TH1=0.25*pid_yaw.u+0.25*Mpitch/Lq-0.25*Mroll/Lq-0.25*Myaw*kM/kT;
        TH2=0.25*pid_yaw.u-0.25*Mpitch/Lq+0.25*Mroll/Lq+0.25*Myaw*kM/kT;
        TH3=0.25*pid_yaw.u-0.25*Mpitch/Lq+0.25*Mroll/Lq-0.25*Myaw*kM/kT;
        TH4=0.25*pid_yaw.u+0.25*Mpitch/Lq-0.25*Mroll/Lq+0.25*Myaw*kM/kT;

        Serial.print(TH1);Serial.print(",");
        Serial.print(TH2);Serial.print(",");
        Serial.print(TH3);Serial.print(",");
        Serial.print(TH4);Serial.print(",");
        PWM1=U2PWM(TH1);
        PWM2=U2PWM(TH2);
        PWM3=U2PWM(TH3);
        PWM4=U2PWM(TH4);

        ledcWrite(PWM_CHANNEL_1, PWM1);
        ledcWrite(PWM_CHANNEL_2, PWM2);
        ledcWrite(PWM_CHANNEL_3, PWM3);
        ledcWrite(PWM_CHANNEL_4, PWM4);
        
        endtime_PID=micros();
        
        Serial.print(PWM1);Serial.print(",");
        Serial.print(PWM2);Serial.print(",");
        Serial.print(PWM3);Serial.print(",");
        Serial.println(PWM4);
        //Flag_PID_Interrupted = false;
*/
         // фильтрация показаний гироскопов
 //Gyro_X,Gyro_Y,Gyro_Z
  //float omg[3]={Gyro_X,Gyro_Y,Gyro_Z};

  // фильтрация показаний гироскопов
  //gyroz = Gyro_Y;
  //omgz=DigLowPassFil(omgz_prev, gyroz, gyroz_prev);
  //gyroz_prev=gyroz;omgz_prev=omgz;
  //omg[1]=omgz;

  //thetaDesired[YAW]=0.0;thetaDesired[PITCH]=0.0;thetaDesired[ROLL]=0.0;
  //omgDesired[YAW]=0.0;omgDesired[PITCH]=0.0;omgDesired[ROLL]=0.0;
    // pid_pitch.set_point_angle=0.0;
    // pid_pitch.set_point=0.0;

  //TM=millis()-startMillis;
  pid_pitch.set_point_angle=CONTROL(millis()-30000);

  GetErrs_Angle(&pid_pitch);
  
  FindAngleRateDesired(&pid_pitch, PITCH);

  if (abs(pid_pitch.set_point)<=M_PI){
    get_out(&pid_pitch,pid_pitch.set_point,-Gyro_Y);
  }else{
    if(pid_pitch.set_point>0){
          get_out(&pid_pitch,M_PI,-Gyro_Y);
    }else{
          get_out(&pid_pitch,-M_PI,-Gyro_Y);
    }
  }

  // Пересчет требуемого значения моментов (с учетом физическими ограничениями)
  //Myaw   = SATURATION(pid_pitch.u, 1.0, -1.0);        //рыскание
  Mpitch = SATURATION(pid_pitch.u, 1.0, -1.0);        //тангаж
  //Mroll  = SATURATION(pid_pitch.u, 1.0, -1.0);       //крен


  pid_yaw.u=0;

        TH1=0.25*pid_yaw.u+0.25*Mpitch/Lq-0.25*Mroll/Lq-0.25*Myaw*kM/kT;
        TH2=0.25*pid_yaw.u-0.25*Mpitch/Lq+0.25*Mroll/Lq+0.25*Myaw*kM/kT;
        TH3=0.25*pid_yaw.u-0.25*Mpitch/Lq+0.25*Mroll/Lq-0.25*Myaw*kM/kT;
        TH4=0.25*pid_yaw.u+0.25*Mpitch/Lq-0.25*Mroll/Lq+0.25*Myaw*kM/kT;

        // Serial.print(TH1);Serial.print(",");
        // Serial.print(TH2);Serial.print(",");
        // Serial.print(TH3);Serial.print(",");
        // Serial.print(TH4);Serial.print(",");
        PWM1=U2PWM(TH1);
        PWM2=U2PWM(TH2);
        PWM3=U2PWM(TH3);
        PWM4=U2PWM(TH4);

        ledcWrite(PWM_CHANNEL_1, PWM1);
        ledcWrite(PWM_CHANNEL_2, PWM2);
        ledcWrite(PWM_CHANNEL_3, PWM3);
        ledcWrite(PWM_CHANNEL_4, PWM4);

        Serial.print(pid_pitch.set_point_angle);Serial.print(",");
        Serial.print(pid_pitch.set_point);Serial.print(",");
        Serial.print(pitch);Serial.print(",");
        Serial.print(pid_pitch.err_angle);Serial.print(",");
        Serial.println(-Gyro_Y);

        // Serial.print(pid_pitch.set_point);Serial.print(",");
        //Serial.println(Gyro_Y);
        // Serial.print(PWM1);Serial.print(",");
        // Serial.print(PWM2);Serial.print(",");
        // Serial.print(PWM3);Serial.print(",");
        // Serial.println(PWM4);
      }
    // }
    // else{
    //     ledcWrite(PWM_CHANNEL_1, THROTTLE_MINIMUM);
    //     ledcWrite(PWM_CHANNEL_2, THROTTLE_MINIMUM);
    //     ledcWrite(PWM_CHANNEL_3, THROTTLE_MINIMUM);
    //     ledcWrite(PWM_CHANNEL_4, THROTTLE_MINIMUM);
    // }
    Flag_PID_Interrupted = false;
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

void setup_pwm(void){
  // Configure the timers for PWM generation
  // configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_4, PWM_FREQUENCY, PWM_RESOLUTION);
  
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_PIN_1, PWM_CHANNEL_1);
  ledcAttachPin(PWM_PIN_2, PWM_CHANNEL_2);
  ledcAttachPin(PWM_PIN_3, PWM_CHANNEL_3);
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

void printBT_roll_pitch_yaw() {
    //Print yaw pitch roll to pc via bluetooth
    char str[30]={};
    sprintf(str, "%.2f,%.2f,%.2f,%.5f\n",yaw,pitch,roll,deltaT );
    SerialBT.write((uint8_t*)str, strlen(str));
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
  ledcWrite(PWM_CHANNEL_1, THROTTLE_MAXIMUM);
  ledcWrite(PWM_CHANNEL_2, THROTTLE_MAXIMUM);
  ledcWrite(PWM_CHANNEL_3, THROTTLE_MAXIMUM);
  ledcWrite(PWM_CHANNEL_4, THROTTLE_MAXIMUM);

  delay(4000);

  ledcWrite(PWM_CHANNEL_1, THROTTLE_MINIMUM);
  ledcWrite(PWM_CHANNEL_2, THROTTLE_MINIMUM);
  ledcWrite(PWM_CHANNEL_3, THROTTLE_MINIMUM);
  ledcWrite(PWM_CHANNEL_4, THROTTLE_MINIMUM);
  delay(10000);
}

volatile int U2PWM(float T)
{
  float PWM;
  PWM=((-b+sqrt(b*b-4*a*(c-abs(T))))*0.5/a/2)*3,276;
  if (T<0)
  {
    PWM=-PWM;
  }
  PWM=PWM+THROLLTE0;
  
  if (PWM<THROTTLE_MINIMUM)
  {
    PWM=THROTTLE_MINIMUM;
  }

  if (PWM>THROTTLE_MAXIMUM)
  {
    PWM=THROTTLE_MAXIMUM;
  }
  return PWM;
}
float SATURATION(float x, float xmax, float xmin)
{  
  if (x>xmax){
    x=xmax; 
  }

  if (x<xmin){
    x= xmin; 
  }
  return x;
}

// Вычисление ошибок управления 
void GetErrs_Angle(PID_t *DataStruct) {
    DataStruct->err_angle  = DataStruct->set_point_angle   - pitch;//*180/M_PI;
}
void FindAngleRateDesired(PID_t *DataStruct,uint8_t ki)
{

    float k[3]={0.1,3,0.1};
  /*  if (abs(angleErr[1])>20/57.3)
    {
      k[1]=5;
    }
    else if (abs(angleErr[1])>10/57.3)
         {
            k[1]=4.0;           
         } 
        else  if (abs(angleErr[1])>5/57.3)
              {
                  k[1]=2.0;           
              }
              else
              {
                  k[1]=2*abs(angleErr[1])/(5/57.3);
              }  
   */
    if (abs(DataStruct->err_angle)>10/57.3)
    {
        k[ki]=sqrt(abs(DataStruct->err_angle)*57.3);
    }               
    else
    {
        k[ki]=0.5*abs(DataStruct->err_angle)*57.3;
    }
    DataStruct->set_point=k[ki]*DataStruct->err_angle;
}
// Низкочастотный фильтр
float DigLowPassFil(float x_prev, float u, float u_prev)
{
  float DT=0.01,w0=30;
  float a0=DT*w0+2, a1=DT*w0-2,b=DT*w0;
  return (-a1*x_prev+b*(u+u_prev))/a0;
}

float CONTROL(uint32_t tmr1)
{
  float thetaDesired=0.0;
  if (tmr1>10000)
  {
    thetaDesired=20.0*M_PI/180;  
  }

  if (tmr1>20000)
  {
    thetaDesired=0.0*M_PI/180;  
  }

  if (tmr1>30000)
  {
    thetaDesired=-20.0*M_PI/180;  
  }

  if (tmr1>40000)
  {
    thetaDesired=0.0;
  }
  return thetaDesired;
}



// Ограничение по thresold
float LMT(float x, float thrsd)
{
  if (abs(x)>thrsd)
  {
    x=thrsd*abs(x)/x;
  }
  return x;
}

