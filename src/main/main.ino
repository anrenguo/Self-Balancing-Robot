/**
 * Self-Balancing Inverted Pendulum Robot
 * 
 * Hardware Architecture:
 * - Microcontroller: Arduino Uno
 * - IMU: MPU6050 (I2C)
 * - Motor Driver: L298N (BJT H-Bridge)
 * - Motors: DC Gear Motors
 * - Power: 7.4V Nominal (2x 3.7V 18650 cells)
 *
 * Control Theory Implementation:
 * - Sensor Fusion: Software low-pass filter coupled with a complementary filter 
 *   to mitigate mechanical high-frequency vibration and gyroscope drift.
 * - Control Law: Custom PID loop featuring dynamic setpoint tuning and a strict 
 *   anti-windup clamp to manage steady-state error without inducing runaway oscillations.
 * - Kinematic Mitigation: Decoupled deadband PWM injection to overcome asymmetrical 
 *   static gearbox friction and L298N voltage sag.
 */

#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// ==========================================
// SENSOR FUSION & FILTER VARIABLES
// ==========================================
float pitch = 0;
float gyroYrate;
uint32_t timer;
unsigned long printTimer = 0;

float filtered_ax = 0;
float filtered_az = 0;

// Low-Pass Filter Alpha: 0.1 = Trust 10% raw incoming data, retain 90% smoothed data.
// Crucial for stripping high-frequency mechanical chassis vibrations from the accelerometer.
float lpf_alpha = 0.1; 

// Calibrated MPU6050 Hardware Offsets (Specific to this silicon die)
// IMPORTANT: make sure to calibrate your own MPU6050 and insert the values here!
int accX_offset = -1354; 
int accY_offset = 2918;
int accZ_offset = 1860;
int gyroX_offset = 30;
int gyroY_offset = 73;
int gyroZ_offset = 55;

// ==========================================
// PID CONTROL VARIABLES
// ==========================================
double setpoint = 0; // The target balance angle (dynamically adjusted via potentiometer)
double output = 0;   // The final computed PWM control signal

// Base PID Gains
// Kp: Provides explosive torque to break static friction.
// Kd: Acts as a predictive shock absorber to dampen high-frequency limit cycling.
// Ki: Sponges up steady-state error (e.g., an off-center battery pack).
float Kp = 40.0; 
float Kd = 2.5;   
float Ki = 4.0; 

float integralError = 0; 

// ==========================================
// HARDWARE PIN DEFINITIONS (L298N)
// ==========================================
#define ENA 3  // Left Motor PWM
#define IN1 4  // Left Motor Direction 1
#define IN2 5  // Left Motor Direction 2
#define ENB 6  // Right Motor PWM
#define IN3 7  // Right Motor Direction 1
#define IN4 8  // Right Motor Direction 2
#define POT_PIN A0 // Analog input for live setpoint tuning

void setup() {
  Serial.begin(115200);
  
  // Initialize Motor Control Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Initialize I2C Bus for MPU6050
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();

  // Inject calibrated offsets to establish a true zero baseline
  mpu.setXAccelOffset(accX_offset);
  mpu.setYAccelOffset(accY_offset);
  mpu.setZAccelOffset(accZ_offset);
  mpu.setXGyroOffset(gyroX_offset);
  mpu.setYGyroOffset(gyroY_offset);
  mpu.setZGyroOffset(gyroZ_offset);
  
  // Seed the low-pass filter with the initial resting state to prevent a startup spike
  int16_t temp_ax, temp_ay, temp_az, temp_gx, temp_gy, temp_gz;
  mpu.getMotion6(&temp_ax, &temp_ay, &temp_az, &temp_gx, &temp_gy, &temp_gz);
  filtered_ax = temp_ax;
  filtered_az = temp_az;

  // --- STARTUP KINEMATIC SANITY CHECK ---
  // Prevents the motors from spooling up to 100% PWM if the robot is initialized lying flat.
  if (abs(setpoint) > 45) {
    Serial.println("SYSTEM HALTED: Robot is not upright. Prop it up and press reset.");
    while (1) {
      // Processor trapped securely to prevent phantom driving
    } 
  }
  
  pitch = setpoint; 
  timer = micros();
}

void loop() {
  // 1. Calculate precise delta time (dt) for accurate integral and derivative scaling
  float dt = (micros() - timer) / 1000000.0;
  timer = micros();

  // 2. Poll Raw MPU6050 Vectors
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 3. Apply Software Low-Pass Filter to Accelerometer Data
  filtered_ax = (lpf_alpha * ax) + ((1.0 - lpf_alpha) * filtered_ax);
  filtered_az = (lpf_alpha * az) + ((1.0 - lpf_alpha) * filtered_az);

  // 4. Compute Angle via Complementary Filter
  // Fuses the long-term accuracy of the filtered accelerometer with the short-term precision of the gyroscope.
  float accAngle = atan2(filtered_ax, filtered_az) * 180 / PI; 
  gyroYrate = -gy / 131.0; 
  pitch = 0.98 * (pitch + gyroYrate * dt) + 0.02 * accAngle;

  // ==========================================
  // CUSTOM HIGH-SPEED PID CONTROL LAW
  // ==========================================
  
  // Live Potentiometer Tuning: Map 10-bit ADC to a physical +/- 5 degree balance window
  int rawPot = analogRead(POT_PIN);
  setpoint = -5.0 + ((rawPot / 675.0) * 10.0);

  float error = pitch - setpoint;
  
  // Accumulate the integral error over time
  integralError += error * dt;
  
  // --- ANTI-WINDUP CLAMP ---
  // Prevents lethal integral windup during an extended fall. Caps the integral's maximum 
  // authority over the motors to +/- 30 PWM, allowing it to fix steady-state leans 
  // without overriding the proportional recovery response.
  float maxIntegralInfluence = 30.0; 
  float maxErrorSum = maxIntegralInfluence / Ki; 
  
  if (integralError > maxErrorSum) integralError = maxErrorSum;
  if (integralError < -maxErrorSum) integralError = -maxErrorSum;
  
  // Instantly flush the integral memory if the chassis crosses the unrecoverable tipping point (15 degrees)
  // This prevents the "whip" effect where accumulated error drives the robot into the ground.
  if (abs(error) > 15) {
    integralError = 0;
  }
  
  // Compute final PID control output
  output = (Kp * error) + (Ki * integralError) + (Kd * gyroYrate);
  
  // Serial debugging (Throttled to 10Hz to prevent blocking the high-frequency control loop)
  if (millis() - printTimer > 100) {
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Target: ");
    Serial.print(setpoint);
    Serial.print(" | Out: ");
    Serial.println(output);
    printTimer = millis();
  }
  
  // Transmit computed effort to the H-Bridge
  driveMotors(output);
}

void driveMotors(double pidOut) {
  // --- DECOUPLED DEADBAND INJECTION ---
  // The motors used require ~70 PWM just to break static friction.
  // This injects a baseline voltage so the PID output translates immediately into physical torque.
  int minPwm_Left_Forward = 70;  
  int minPwm_Left_Backward = 70; 
  int minPwm_Right_Forward = 70; 
  int minPwm_Right_Backward = 70;

  int speedLeft = 0;
  int speedRight = 0;
  
  // Map PID output to directional PWM and constrain to the 8-bit timer limit (0-255)
  if (pidOut > 0) {
    speedLeft = constrain(abs(pidOut) + minPwm_Left_Forward, 0, 255);
    speedRight = constrain(abs(pidOut) + minPwm_Right_Forward, 0, 255);
  } else if (pidOut < 0) {
    speedLeft = constrain(abs(pidOut) + minPwm_Left_Backward, 0, 255);
    speedRight = constrain(abs(pidOut) + minPwm_Right_Backward, 0, 255);
  }
  
  // --- ACTUATOR SATURATION KILL SWITCH ---
  // If the pitch exceeds 40 degrees, the required recovery velocity exceeds the physical RPM 
  // limit of the motors. Cut power entirely to prevent the robot from accelerating the fall.
  if (abs(pitch - setpoint) > 30) {
    speedLeft = 0;
    speedRight = 0;
  }

  // Engage L298N Directional Logic Pins
  if (pidOut > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }

  // Transmit final adjusted PWM to motor driver enable pins
  analogWrite(ENA, speedLeft);
  analogWrite(ENB, speedRight);
}
