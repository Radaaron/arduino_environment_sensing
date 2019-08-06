# arduino_environment_sensing
Arduino implementation of various sensors to determine enviromental factors such as wind velocity, attitude angles (pitch, roll, yaw), magnetic north reference, and GPS coordinates with all data peripherals using the I²C protocol for communication. Wireless communications can be implemented through a serial RF transciever with a wireless working frequency band of 433.4 MHz to 473.0 MHz. For use in transient applications such as hot air balloons or drones and other AUV's.

The peripherals used and corresponding resources are as follows (Not including part datasheets): 

Wind Velocity --> Davis Anemometer 07911: http://cactus.io/hookups/weather/anemometer/davis/hookup-arduino-to-davis-anemometer

Attitude Angles --> InvenSense MPU-6050: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/, https://playground.arduino.cc/Main/MPU-6050/

Magnetic North Compass --> GY-271 Magnetometer: http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/arduino-gy-273-hmc5883l-magnetometer-compass-tutorial/

GPS --> GPS Arduino Shield using the TinyGPS library: http://arduiniana.org/libraries/tinygps/

Wireless Communication with other microcontrollers --> HC-12 Wireless Transceiver: https://howtomechatronics.com/tutorials/arduino/arduino-and-hc-12-long-range-wireless-communication-module/, https://www.allaboutcircuits.com/projects/understanding-and-implementing-the-hc-12-wireless-transceiver-module/


