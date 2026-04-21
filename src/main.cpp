#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors/IMU.h"
#include "control/PID.h"
#include "control/NRF.h"
#include "actuators/MotorController.h"

const byte nrfAddress[6] = NRF_ADDR;
NRFReceiver nrf(NRF_CE, NRF_CSN, nrfAddress);

IMU imu;
MotorController motors;

// PID Tuning for Yaw Rate (This usually needs a higher P than angle)
PID pidRoll(KP_ROLL, KI_ROLL, KD_ROLL, -PID_I_MAX, PID_I_MAX, FILTER_ALPHA_D);
PID pidPitch(KP_PITCH, KI_PITCH, KD_PITCH, -PID_I_MAX, PID_I_MAX, FILTER_ALPHA_D);
PID pidYaw(KP_YAW, KI_YAW, KD_YAW, -PID_I_MAX, PID_I_MAX, FILTER_ALPHA_D);

// --- Task Handles ---
TaskHandle_t FlightTaskHandle;

// --- Shared Data (Thread-Safeish) ---
struct ControllData
{
    float throttle, rollTarget, pitchTarget, yawTarget;
    bool connected;
} sharedData;

// --- Flight Control (Core 1 - High Priority) ---
void flightTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // 4ms period = 250Hz. Professional FCs usually run at 1kHz+,
    // but 250Hz is a great stable start for I2C.
    const TickType_t xFrequency = pdMS_TO_TICKS(4);
    const float dt = 0.004f; // Fixed dt for the PID math

    for (;;)
    {
        // Wait until exactly 4ms has passed since the last start
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Update the Sensors
        imu.update(dt);

        // Get local copies of targets (Minimize time spent accessing shared memory)
        float t = sharedData.throttle;
        float rt = sharedData.rollTarget;
        float pt = sharedData.pitchTarget;
        float yt = sharedData.yawTarget;
        bool isConnected = sharedData.connected;

        // Debug output
        // Serial.printf("Conn: %d, Thr: %.0f, R: %.1f, P: %.1f, Y: %.1f\n", isConnected, t, imu.getRoll(), imu.getPitch(), imu.getYaw());

        // Failsafe logic
        if (!isConnected || t < 1000)
        {
            motors.update(PWM_MIN, 0, 0, 0);
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            continue;
        }

        // PID Calculations
        float rPID = pidRoll.compute(rt, imu.getRoll(), dt);
        float pPID = pidPitch.compute(pt, imu.getPitch(), dt);
        float yPID = pidYaw.compute(yt, imu.getYaw(), dt);

        // Motor Output
        motors.update(t, rPID, pPID, yPID);
    }
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    nrf.begin();
    imu.begin();
    motors.begin();
    motors.arm();

    
    xTaskCreatePinnedToCore(
        flightTask,
        "FlightLoop",
        8192,
        NULL,
        24,
        &FlightTaskHandle,
        1);
    
    Serial.println("Flight Task Started on Core 1");
    Serial.println("Send 'C' via Serial Monitor to calibrate IMU.");

    // Initialize sharedData to prevent garbage values
    sharedData.throttle = 1000.0f;
    sharedData.rollTarget = 0.0f;
    sharedData.pitchTarget = 0.0f;
    sharedData.yawTarget = 0.0f;
    sharedData.connected = false;
}

void loop()
{
    // --- Task 2: Radio & Telemetry (Core 0 - Default) ---
    // The loop() function on ESP32 runs on Core 1 by default, but we can 
    // use it for the Radio since the Flight Task will preempt it.
    // --- Listen for Calibration Command ---

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            Serial.println("Suspending flight loop for calibration...");
            
            // 1. Pause the Flight Task to prevent I2C bus collisions
            vTaskSuspend(FlightTaskHandle);
            
            // 2. Ensure motors are forced off before we stop calculating PID
            motors.update(PWM_MIN, 0, 0, 0); 
            
            // 3. Run the 2-second calibration and save to flash
            imu.calibrate();
            
            // 4. Resume the flight loop!
            vTaskResume(FlightTaskHandle);
            Serial.println("Flight loop resumed. Ready to fly!");
        }
    }

    // --- Read Radio ---
    if (nrf.available())
    {
        nrf.read();

        // Update the shared struct for Core 1 to see
        sharedData.throttle = nrf.data.throttle;
        sharedData.rollTarget = (abs(nrf.data.rollTarget) < 2) ? 0 : nrf.data.rollTarget;
        sharedData.pitchTarget = (abs(nrf.data.pitchTarget) < 2) ? 0 : nrf.data.pitchTarget;
        sharedData.yawTarget = (abs(nrf.data.yawTarget) < 5) ? 0 : nrf.data.yawTarget;
        sharedData.connected = true;
    }

    // Simple Failsafe timeout check
    if (millis() - nrf.lastReceiveTime > 1000) {
        sharedData.connected = false;
    }

    // Small delay to let the background Watchdog timer breathe
    delay(1);
}