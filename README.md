# Smart Room Monitor

### CSE 321 - Real Time and Embedded Operating Systems

This project involves building a Smart Room Monitor using the STM32 NUCLEO-F072RB and FreeRTOS.

The system monitors temperature, humidity, light levels, and air quality using different sensors. The readings are shown on an OLED display, and an RGB LED and buzzer are used to give alerts when certain readings go above their limits. A 12V fan is also included to turn on when the temperature gets too high or the air quality gets too poor.

We are also using a PIR motion sensor to detect movement in the room. If no movement is detected for a certain amount of time, the system will go into Sleep mode.

## Hardware Used

- STM32 NUCLEO-F072RB
- BME280 (Temperature/Humidity)
- BH1750 (Light Sensor)
- MQ-135 (Air Quality)
- PIR Motion Sensor
- GME12864 OLED Display
- RGB LED and Buzzer
- 12V DC Fan and MOSFET
- Push Buttons
- Buck Converter
- 12V 2A Power Adapter

## Software Used

- STM32CubeIDE
- Embedded C
- FreeRTOS

## FreeRTOS Tasks

We are using FreeRTOS to run different tasks for the sensors, display, alerts, and fan.

- Sensor Task - Reads the temperature, humidity, light, and air quality sensors.
- Monitoring Task - Checks if the sensor readings are within the set limits.
- Alert Task - Controls the LED and buzzer when readings get too high.
- Fan Control - Turns the fan on or off depending on the sensor readings.
- Display Task - Shows sensor values on the OLED and uses a button to switch screens.
- Motion Detection - Detects movement and puts the system into Sleep mode after some time without activity.

We are also using queues to send sensor readings between tasks, a mutex for the I2C bus, and semaphores for button presses and motion detection.

## Project Status

Still working on testing the sensors and getting everything to work together.
