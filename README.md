# ESP32 Custom Flight Controller 🚁

A from-scratch, bare-metal flight controller firmware written in C++ for the ESP32 microcontroller. This project implements custom PID control loops, IMU sensor fusion, and radio communication to stabilize and fly a Quad-X drone.

## Hardware Architecture
* **Microcontroller:** ESP-32E (NodeMCU style)
* **IMU:** MPU6050 (GY-521) - 6-DOF Accelerometer and Gyroscope
* **Radio:** NRF24L01 (2.4GHz Transceiver)
* **Motors:** 4x 930KV Brushless Motors
* **ESCs:** 4x 30A Electronic Speed Controllers
* **Battery:** LiPo Battery
* *(Note: Magnetometer/Compass HW-127 was intentionally omitted in favor of Yaw-Rate control to eliminate I2C bus locking and magnetic interference).*

## Key Features
* **Custom PID Controller:** Independent PI(D) tuning for Roll, Pitch, and Yaw.
* **Hybrid Flight Modes:** * **Angle Mode (Roll/Pitch):** The drone self-levels based on stick input (0 to max angle).
  * **Rate/Acro Mode (Yaw):** The drone spins at a targeted degrees-per-second, utilizing raw Gyro-Z data.
* **Hardware Failsafes:** Immediate motor kill on NRF radio signal loss (>500ms) or throttle drop.
* **Precise Loop Timing:** Hardware-timed `micros()` loop to ensure a rock-solid control frequency (e.g., 250Hz).
* **Deadband Filtering:** Software deadbands on joystick inputs to prevent motor jitter and drift when sticks are centered.

## Pin Mapping (ESP32)
| Component | Pin Function | ESP32 Pin |
| :--- | :--- | :--- |
| **MPU6050** | I2C SDA | GPIO 21 |
| **MPU6050** | I2C SCL | GPIO 22 |
| **NRF24L01**| CE | *(Your CE Pin)* |
| **NRF24L01**| CSN | *(Your CSN Pin)* |
| **NRF24L01**| SCK | GPIO 18 (VSPI SCK) |
| **NRF24L01**| MISO | GPIO 19 (VSPI MISO)|
| **NRF24L01**| MOSI | GPIO 23 (VSPI MOSI)|
| **ESC 1 (FR)**| PWM | *(Your PWM Pin)* |
| **ESC 2 (FL)**| PWM | *(Your PWM Pin)* |
| **ESC 3 (RL)**| PWM | *(Your PWM Pin)* |
| **ESC 4 (RR)**| PWM | *(Your PWM Pin)* |

## Setup & Installation
1. Clone this repository and open it in **VS Code with the PlatformIO extension**.
2. Configure your specific pins and PID tuning values in `config.h`.
3. **Important:** Remove propellers before testing!
4. Compile and upload via USB.
5. Open the Serial Monitor (115200 baud) to monitor IMU calibration and NRF connection status.