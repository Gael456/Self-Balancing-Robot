#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>

// ----- IMU (MPU6050) -----
MPU6050 mpu(Wire);

// ----- Servos -----
Servo servoPitch;  // top servo (front/back tilt)
Servo servoRoll;   // bottom servo (left/right tilt)

// ====== USER SETTINGS (EDIT THESE IF NEEDED) ======

// Arduino pins
const int SERVO_PITCH_PIN = 9;   // signal wire of top servo
const int SERVO_ROLL_PIN  = 10;  // signal wire of bottom servo

// Center positions for your servos.
// After you mount the platform, change these so the tray is level when at rest.
int pitchCenter = 90;  // try 90 first, then adjust (e.g. 85 or 95)
int rollCenter  = 90;

// Servo limits: keep a safe range so linkage does not hit mechanical stops.
const int SERVO_MIN = 40;   // was 30; 40 is safer for HS-485HB
const int SERVO_MAX = 140;  // was 150

// PID gains: these control how aggressively the platform moves.
float Kp = 6.0;   // proportional gain
float Ki = 0.0;   // integral gain (start at 0)
float Kd = 2.0;   // derivative gain

// Target angles (we want level = 0 degrees)
float setPitch = 0.0;
float setRoll  = 0.0;

// PID state
float iPitch = 0.0, iRoll = 0.0;
float lastErrP = 0.0, lastErrR = 0.0;
unsigned long lastMs = 0;

// ===================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // --- IMU init ---
  Wire.begin();
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);

  // If status != 0, the sensor is not found; stay here.
  while (status != 0) {
    Serial.println("MPU6050 not found, check wiring!");
    delay(1000);
  }

  Serial.println("Calibrating... keep the platform still and level.");
  delay(1000);
  // This measures offsets while the sensor is not moving
  mpu.calcOffsets(true, true);  // gyro + accel
  Serial.println("Calibration done.");

  // --- Servo init ---
  servoPitch.attach(SERVO_PITCH_PIN);
  servoRoll.attach(SERVO_ROLL_PIN);

  // Move to center positions
  servoPitch.write(pitchCenter);
  servoRoll.write(rollCenter);

  lastMs = millis();
}

void loop() {
  // Update IMU readings
  mpu.update();

  // Read angles from MPU6050 (degrees)
  // Depending on board orientation:
  //   angleX is often pitch (front/back)
  //   angleY is often roll  (left/right)
  float pitch = mpu.getAngleX();
  float roll  = mpu.getAngleY();

  // Time step for PID
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastMs = now;

  // ----- PITCH PID -----
  float eP = setPitch - pitch;  // error (target - measured)

  // Small deadband to reduce jitter
  if (fabs(eP) < 0.2) eP = 0.0;

  iPitch += eP * dt;
  float dP = (eP - lastErrP) / dt;
  lastErrP = eP;

  float uP = Kp * eP + Ki * iPitch + Kd * dP;  // control output

  // If pitch moves the wrong direction, uncomment the next line:
  // uP = -uP;

  int pitchCmd = pitchCenter + (int)uP;
  pitchCmd = constrain(pitchCmd, SERVO_MIN, SERVO_MAX);

  // ----- ROLL PID -----
  float eR = setRoll - roll;

  if (fabs(eR) < 0.2) eR = 0.0;

  iRoll += eR * dt;
  float dR = (eR - lastErrR) / dt;
  lastErrR = eR;

  float uR = Kp * eR + Ki * iRoll + Kd * dR;


  int rollCmd = rollCenter + (int)uR;
  rollCmd = constrain(rollCmd, SERVO_MIN, SERVO_MAX);

  // ----- Write to servos -----
  servoPitch.write(pitchCmd);
  servoRoll.write(rollCmd);

  */

  delay(5);  // controls loop speed (~200 Hz)
}
