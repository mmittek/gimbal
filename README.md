# 3-AXIS GIMBAL PROJECT
Poorly documented attempt to build an inexpensive 3-axis gimbal using STM32FX uC and MPU6050/9250 IMU. Main goal is to learn how to use STM32CubeMX and maintain such project (it's my first real-world use of STM32FX-series uC).

![alt text](https://github.com/mmittek/gimbal/blob/master/images/1-axis.jpg?raw=true "Single axis assembly")

Demo available here: https://www.youtube.com/watch?v=dBaz5YtMMdg

## Features
- STM32F100 running at 24Mhz
- Eclipse Makefile project
- Pins defined and configuration managed using STM32CubeMX (http://www.st.com/en/development-tools/stm32cubemx.html)
- IMU sampled at 100Hz using Invense's API
- Complementary filter for 3-axis (relative) orientation estimation
- PID loops for motor speed control with PWM-controlled enable pins
 

## Components
- STM32F100 discovery board (http://www.st.com/en/evaluation-tools/stm32vldiscovery.html)
- GY-521 MPU6050 module board (https://playground.arduino.cc/Main/MPU-6050)
- JLink JTAG EDU (https://www.digikey.com/products/en?mpart=8.08.90%20J-LINK%20EDU&v=899)
- Inexpensive JGY-371 motors readily available on eBay (https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/108990007_Web.pdf)
- Some 3D-printed parts for the assembly
- L298N motor control boards readily available online (https://www.amazon.com/DROK-Controller-H-Bridge-Mega2560-Duemilanove/dp/B00CAG6GX2)

## References
- L298N Solid Works model taken from https://grabcad.com/library/driver-l298n-1
- GY6050/GY9250 Solid Works model taken from https://grabcad.com/library/mpu-92-65-1