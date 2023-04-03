This code is written for controlling a drone using ESP32 as a microcontroller. The drone is equipped with MPU9250 sensor and nRF24 module.\n\n### Dependencies
- MPU9250.h
- SoftwareSerial.h
- Wire.h
- math.h
- esp32-hal-timer.h
- esp32-hal-gpio.h
- Arduino.h
- RF24.h
- SPI.h
- PID.h\n\n### Functionality\n- Setup the PID controller for roll,pitch,yaw and altitude.
- Setup PWM channels and target pins.
- Initiate nRF24 module for communication between nodes.
- Initiate Timer1 and Timer2 for interrupting T=S to receive requests of drone and interrupting T=S to run PID algorithm.
- Initiate MPU6050 and GY-271 sensor and calibrate them.
- Update roll,pitch,yaw angles of the drone.

### Functions & ISR
- `void print_roll_pitch_yaw()` : To print roll,pitch and yaw values.
- `void update_quaternion()` : To update angles of drone using sin, cos and tan functions.
- `void IRAM_ATTR onTimer1()` : ISR to receive data through nRF24.
- `void IRAM_ATTR onTimer2()` : ISR for the pid algorithm and to write pwm to motor.
- `void setup_pwm(void)` : To setup PWM channels and pins.
- `void set_pwm_to_motor(void)` : To set the duty cycle of each PWM signal. 
- `void pwm_caculate (int8_t pitch,int8_t roll,int8_t yaw,int8_t altitude)` : To calculate pwm values from roll,pitch and yaw values.

### Variables
- `MPU9250 mpu` : Object of MPU9250 sensor.
- `float q[4]` : Quaternion used in update_quaternion().
- `double L = 0.16` : length from motor center to center of gravity.
- `double KmT = 0.02` : propeller parameters.
- `float roll, yaw, pitch` : angles of drone.
- `float deltaT` : Time to calculate angle.
- `uint32_t newTime, oldTime` : Initiation of time variable.
- `RF24 radio(19, 18)` : Object of NRF24 module.
- `bool radioNumber` : To identify radio addresses.
- `bool role` : To manage paring and transmitting of NRF24.
- `float pwm_min, pwm_max, pwm1, pwm2, pwm3, pwm4` : pwm values for each motor.
- `float deltaT_PID` : Time to sample PID algorithm.
- `PID_t pid_pitch, pid_roll, pid_yaw, pid_altitude` : PID controller.

### Note
This code can be optimized further for better performance.