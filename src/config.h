#pragma once

// --- I2C & Sensors ---
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define NRF_ADDR "DRONE"

// --- ESC Pins --- 
#define ESC_FL 13
#define ESC_FR 12
#define ESC_BL 14
#define ESC_BR 27

// --- NRF Pins ---
#define NRF_CE 5
#define NRF_CSN 17

// --- Timing ---
// Core 1 Flight Task Frequency
#define LOOP_FREQ 250 // 250Hz = 4ms per loop

// --- Filtering ---
// D-Term Low Pass Filter. Lower = smoother but more delay.
// 0.2 is a good starting point for wooden frames.
#define FILTER_ALPHA_D 0.2f 

// --- PID Tuning ---
// Wood frame baseline (Lower P on Roll/Pitch to prevent flex oscillation)
#define KP_ROLL 3.0   
#define KI_ROLL 0.01
#define KD_ROLL 0.05

#define KP_PITCH 3.0
#define KI_PITCH 0.01
#define KD_PITCH 0.05

// Yaw requires more P-gain because it relies on torque, not direct thrust
#define KP_YAW 5.0
#define KI_YAW 0.05
#define KD_YAW 0.05

// --- Limits ---
#define PWM_MIN 1000
#define PWM_MAX 2000

// I-term limit to prevent windup on the ground (200 = max 20% motor power adjustment)
#define PID_I_MAX 200.0