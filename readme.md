
Component list for the autoleveling drone below.

 **HGLRC 60A 6S V1 ESC BLS 2-6S 4in1 M3, Supports DShot600/300/150 Digital Protocols x 1**

 **Readytosky RS2205 2300KV Brushless Motor CW/CCW 3-4S RC Motors for FPV Racing Drone FPV Multicopter x 4**
 
 **PJRC Teensy 4.0 x 1**

 **HiLetgo 3pcs GY-521 MPU-6050 MPU6050 3 Axis Accelerometer Gyroscope Module 6 DOF 6-axis Accelerometer x 1**

 **Usmile 250 mm 3K Carbon Fiber FPV Quadcopter Frame x 1**

 **LiPo Battery Pack 35C 2200mAh 3S 11.1V x 1**

 **FLYSKY FS-i6X 10CH 2.4GHz RC Transmitter Controller with FS-iA6B Receiver x 1**

 **MP1584EN 5pcs Mini 5V Buck Converter Board - 5-30V to 5V, 1.8A x 1**

Software Circuit Diagram:ToDo

**Project Discussion:**

Building a quadcopter requires certain basic elements.  Naturally it requires motors and propellers for thrust, a battery supply to power said motors.  Since the motors used for this project are brushless DC motors, an electronic speed controller(ESC) is also required.  A microcontroller or flight controller is required to provide data signals to the speed controller.  In this case, the microcontroller is a Teensy 4.0.  The Teensy 4.0 has to interface with a sensor/sensors and also a receiver so that it can receive remote control commands from the drone pilot.  

The sensor in this case is an MPU6050 6 axis IMU, while the receiver is an fsi6 receiver.

The Teensy 4.0 communicates with the IMU via SPI protocol at 1000Khz.  It also receives information from the remote control using a serial interface and PPM modulation. Pulse-position modulation (PPM) is a technique where analog message values are encoded by varying the position of fixed-width pulses within specific time slots relative to a reference clock.)

What signal will the Teensy send to the speed controller?  The ESC used is a 4 in 1 controller where there are four ESC's mounted on the same printed circuit board.  Each esc has its own serial input from the microcontroller.  The protocol used to transmit the information is OneShot125.  This protocol sends pwm signals at a frequency of 2000hz (500 microsecond period), with a duty cycle ranging from 125 microseconds to 250 microseconds representing the minimum and maximum speeds respectively.

![Software components](drone_software_components.png)
