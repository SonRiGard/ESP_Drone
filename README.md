## MPU9250 Code Readme

This code implements a program that uses the MPU9250 sensor along with the Mag GY-271 using Arduino. The program reads data from the sensors and updates the yaw, pitch, and roll angles. The yaw, pitch, and roll values are printed to the serial monitor.

The program includes an interrupt function for Timer 0 that is to be executed every second.

The required libraries are:
- MPU9250.h
- SoftwareSerial.h
- Wire.h
- math.h
- esp32-hal-timer.h
- esp32-hal-gpio.h

The program initializes the serial communication with the baud rate of 9600, sets up and calibrates the MPU6050 and Mag GY-271 sensors.

The `setup()` function configures the timer and sets the address of the MPU9250. 

The `loop()` function runs in a loop, reads data from the MPU9250 and calculates the yaw, pitch, and roll angles using the `update_quaternion()` function.

The `update_quaternion()` function updates the angles using the quaternions obtained from the MPU9250 sensor. 

The `onTimer()` function is executed every second.

The `print_roll_pitch_yaw()` function prints the yaw, pitch, and roll values to the serial monitor.